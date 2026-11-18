// Single-axis diagnostic and manual compensation evaluation; other axes hold.
#include <array>
#include <atomic>
#include <cmath>
#include <fstream>
#include <memory>
#include <iomanip>
#include <limits>
#include <sstream>
#include <vector>
#include "controller_interface/controller_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "friction_compensation_controller/position_velocity_observer.hpp"
#include "friction_compensation_controller/elastoplastic_replay.hpp"
#include "friction_compensation_controller/sine_probe_sequence.hpp"
#include "friction_compensation_controller/loaded_rest_pose.hpp"
#include "friction_compensation_controller/probe_abort_gravity.hpp"
#include "friction_compensation_controller/stateful_friction.hpp"
#include "friction_compensation_controller/common_release_sequence.hpp"
#include "friction_compensation_controller/current_tracking_sequence.hpp"

namespace friction_compensation_controller {
class CurrentProbeController : public controller_interface::ControllerInterface {
  using Callback = controller_interface::CallbackReturn;
  struct Row { double t, dt, command; int phase; std::array<double,18> state; std::array<double,6> drive_errors{}; double observed_velocity, velocity_age; double test_input=0., assistance=0., elastic=0., rate=0., damping=0., dither=0., dither_envelope=0., gravity=0.; };
  PositionVelocityObserver observer_{.02,.05};
  int target_ = 5;
  std::array<size_t,6> ci_{};
  std::array<size_t,24> si_{};
  bool monitor_drive_errors_=false;
  std::array<double,6> hold_{};
  std::vector<double> expected_, ends_, values_;
  std::vector<int64_t> directions_, assistance_mask_;
  std::unique_ptr<ElastoplasticReplay> transition_;
  std::unique_ptr<SineProbeSequence> sine_probe_;
  std::unique_ptr<StatefulFriction> combined_;
  std::unique_ptr<CommonReleaseSequence> common_release_;
  std::unique_ptr<CurrentTrackingSequence> tracking_;
  std::vector<double> gravity_angles_,gravity_values_;
  bool stationary_gravity_=false;
  double sine_elapsed_=0.;
  bool manual_hand_=false;
  bool prepared_combined_=false;
  bool is_ab() const {return kind_=="transition_ab" || kind_=="combined_ab";}
  bool active_assistance_=false;
  double fp_=0., fn_=0., bp_=0., bn_=0.;
  double excursion_=0.03, speed_=0.1, hold_error_=0.03, elapsed_=0.;
  std::string file_, kind_;
  int ramp_trial_=0;
  bool ramp_active_=false;
  double stage_elapsed_=0., ramp_start_q_=0.;
  double rest_min_=0.,rest_max_=0.; bool rest_observed_=false;
  double ab_distance_=0.0005, ab_speed_=0.015;
  double ramp_max_=3.,ramp_rate_=.2;
  std::vector<Row> rows_;
  size_t count_=0, saved_count_=0;
  Row last_saved_row_{};
  std::atomic<size_t> consumed_count_{0};
  std::ofstream journal_;
  bool used_=false;
  std::atomic<int> status_{0}; // 0 configured, 1 running, 2 complete, 3 fault, 4 priming, 5 stopping
  bool deferred_start_=false;
  std::atomic<bool> start_requested_{false};
  std::atomic<bool> stop_requested_{false};
  std::atomic<int> fault_{0};
  std::atomic<size_t> published_count_{0};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  std::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>> command_publisher_;
  double last_command_=0., command_publish_elapsed_=0.;
  double gravity_at(double q) const {
    // Explicit neutral reporting diagnostics prime zero, without inventing a
    // gravity table or changing the measured sine input.
    if(kind_=="rest_response" && !stationary_gravity_)return std::isfinite(q)?0.:NAN;
    return probe_gravity_at(gravity_angles_,gravity_values_,q,manual_hand_);
  }
  bool write(double command) {
    bool ok=true;
    for(size_t j=0;j<6;++j) ok=command_interfaces_[ci_[j]].set_value(
      j==static_cast<size_t>(target_) ? command : hold_[j]) && ok;
    last_command_=command;
    return ok;
  }
  void publish_command(const rclcpp::Time &time, const rclcpp::Duration &period) {
    command_publish_elapsed_+=period.seconds();
    if(command_publish_elapsed_<.01 || !command_publisher_ || !command_publisher_->trylock())return;
    auto &message=command_publisher_->msg_;
    message.header.stamp=time;
    for(size_t j=0;j<6;++j) {
      message.position[j]=j==static_cast<size_t>(target_)?std::numeric_limits<double>::quiet_NaN():hold_[j];
      message.effort[j]=j==static_cast<size_t>(target_)?last_command_:std::numeric_limits<double>::quiet_NaN();
    }
    command_publish_elapsed_=0.;
    command_publisher_->unlockAndPublish();
  }
public:
  Callback on_init() override {
    auto_declare<bool>("defer_probe_start",false);
    auto_declare<int>("target_joint",6);
    auto_declare<bool>("monitor_drive_errors",false);
    auto_declare<std::string>("probe_kind","step");
    auto_declare<std::vector<double>>("sine_amplitudes_nm",{});
    auto_declare<std::vector<double>>("sine_frequencies_hz",{5.,10.});
    auto_declare<int>("sine_repeats",2);
    auto_declare<bool>("stationary_gravity_compensation",false);
    auto_declare<bool>("neutral_current_response",false);
    auto_declare<bool>("rest_selection",false);
    auto_declare<double>("common_start_angle",0.);
    auto_declare<std::vector<double>>("stationary_reviewed_pose",{});
    auto_declare<bool>("common_rest_dither",false);
    auto_declare<bool>("common_soft_start_rest",false);
    auto_declare<bool>("common_dither_only",false);
    auto_declare<double>("common_hand_seconds",0.);
    auto_declare<double>("common_zero_input_seconds",0.);
    auto_declare<double>("common_output_fraction",1.);
    auto_declare<double>("common_release_speed",.02);
    auto_declare<double>("common_minimum_travel",0.);
    for(const char *p:{"tracking_times","tracking_positions","tracking_velocities","tracking_feedforward"})
      auto_declare<std::vector<double>>(p,{});
    for(const char *p:{"common_speeds","common_positive","common_negative","common_gravity_knots","gravity_angles","gravity_values"})
      auto_declare<std::vector<double>>(p,{});
    auto_declare<double>("common_load_width",0.);
    auto_declare<double>("common_load_scale",0.);
    auto_declare<double>("common_transition_distance",0.);
    auto_declare<double>("common_periodic_frequency",0.);
    auto_declare<bool>("common_load_hat_mean_extension",false);
    auto_declare<std::vector<int64_t>>("common_periodic_orders",{1,2});
    for(const char *p:{"combined_speed","combined_positive","combined_negative"}) auto_declare<std::vector<double>>(p,{});
    for(const char *p:{"combined_scale","dither_amplitude","dither_frequency","dither_full_velocity","dither_zero_velocity","dither_ramp"}) auto_declare<double>(p,0.);
    auto_declare<std::vector<int64_t>>("ramp_directions",{1,-1,1,-1,1,-1});
    auto_declare<std::vector<int64_t>>("assistance_mask",{});
    for(const char *p:{"positive_yield_nm","negative_yield_nm","positive_viscous","negative_viscous","transition_stiffness","transition_decay","additional_damping"}) auto_declare<double>(p,0.);
    auto_declare<std::vector<double>>("expected_pose",{});
    auto_declare<std::vector<double>>("step_ends_s",{});
    auto_declare<std::vector<double>>("step_effort_nm",{});
    auto_declare<double>("max_excursion_rad",0.03);
    auto_declare<double>("max_speed_rad_s",0.1);
    auto_declare<double>("held_joint_error_rad",0.03);
    auto_declare<double>("ramp_max_nm",3.);
    auto_declare<double>("ramp_rate_nm_s",.2);
    auto_declare<double>("ab_release_distance_rad",0.0005);
    auto_declare<double>("ab_release_speed_rad_s",0.015);
    auto_declare<std::string>("output_csv","");
    return Callback::SUCCESS;
  }
  Callback on_configure(const rclcpp_lifecycle::State &) override {
    sine_probe_.reset(); transition_.reset(); combined_.reset();common_release_.reset();tracking_.reset();
    target_=get_node()->get_parameter("target_joint").as_int()-1;
    monitor_drive_errors_=get_node()->get_parameter("monitor_drive_errors").as_bool();
    kind_=get_node()->get_parameter("probe_kind").as_string();
    manual_hand_=get_node()->get_parameter("common_hand_seconds").as_double()!=0.;
    if(manual_hand_ && (kind_!="common_ab" || target_<0 || target_>=6))return Callback::ERROR;
    stationary_gravity_=get_node()->get_parameter("stationary_gravity_compensation").as_bool();
    const bool neutral_response=get_node()->get_parameter("neutral_current_response").as_bool();
    const bool rest_selection=get_node()->get_parameter("rest_selection").as_bool();
    if(rest_selection && (kind_!="rest_response" || stationary_gravity_ || neutral_response))return Callback::ERROR;
    if(neutral_response && (kind_!="rest_response" || stationary_gravity_))return Callback::ERROR;
    if(stationary_gravity_ && (kind_!="rest_response" || target_!=1))return Callback::ERROR;
    const bool common_rest=get_node()->get_parameter("common_rest_dither").as_bool();
    const bool soft_start_rest=get_node()->get_parameter("common_soft_start_rest").as_bool();
    if(soft_start_rest && (!common_rest || kind_!="common_ab"))return Callback::ERROR;
    const bool dither_only=get_node()->get_parameter("common_dither_only").as_bool();
    if(common_rest && (kind_!="common_ab" || target_<0 || target_>=6))return Callback::ERROR;
    if(dither_only && !common_rest)return Callback::ERROR;
    ramp_max_=get_node()->get_parameter("ramp_max_nm").as_double();
    ramp_rate_=get_node()->get_parameter("ramp_rate_nm_s").as_double();
    if(!(ramp_max_>0. && ramp_max_<=(target_==0?10.:4.) && ramp_rate_>=.2 && ramp_rate_<=.5 && ramp_max_/ramp_rate_<=20.)) return Callback::ERROR;
    if(is_ab() && target_!=5) return Callback::ERROR;
    if(kind_!="step" && kind_!="breakaway" && kind_!="traverse" && kind_!="transition_ab" && kind_!="rest_response" && kind_!="combined_ab" && kind_!="common_ab") return Callback::ERROR;
    if(kind_=="rest_response") {
      if(target_!=0 && target_!=1 && target_!=5 && !neutral_response && !rest_selection) return Callback::ERROR;
      try {
        sine_probe_=std::make_unique<SineProbeSequence>(
          get_node()->get_parameter("sine_amplitudes_nm").as_double_array(),
          get_node()->get_parameter("sine_frequencies_hz").as_double_array(),
          get_node()->get_parameter("sine_repeats").as_int());
      } catch(const std::exception &) {return Callback::ERROR;}
    }
    directions_=get_node()->get_parameter("ramp_directions").as_integer_array();
    if(directions_.empty() || directions_.size()>8) return Callback::ERROR;
    for(auto d:directions_) if(d!=1 && d!=-1) return Callback::ERROR;
    if(kind_!="common_ab" && get_node()->get_parameter("common_zero_input_seconds").as_double()!=0.)return Callback::ERROR;
    if(kind_=="common_ab") {
      try {
        const auto get=[this](const char *p){return get_node()->get_parameter(p).as_double();};
        const auto array=[this](const char *p){return get_node()->get_parameter(p).as_double_array();};
        if(target_<0 || target_>=6 || get("additional_damping")!=0.)return Callback::ERROR;
        const auto knots=array("common_gravity_knots");
        DirectionalLoadLaw law=get("common_periodic_frequency")>0. ?
          DirectionalLoadLaw(DirectionalLoadLaw::PeriodicLoad{get("common_load_width"),get("common_load_scale"),
            get("common_periodic_frequency"),get_node()->get_parameter("common_periodic_orders").as_integer_array(),
            array("common_speeds"),array("common_positive"),array("common_negative"),knots,
            get_node()->get_parameter("common_load_hat_mean_extension").as_bool()}) : !knots.empty() ?
          DirectionalLoadLaw(knots,array("common_speeds"),array("common_positive"),array("common_negative")) :
          DirectionalLoadLaw(DirectionalLoadLaw::SmoothLoad{get("common_load_width"),get("common_load_scale"),
            array("common_speeds"),array("common_positive"),array("common_negative")});
        std::optional<RestDither> rest;
        if(common_rest)rest.emplace(RestDitherParameters{get("dither_amplitude"),get("dither_frequency"),
          get("dither_full_velocity"),get("dither_zero_velocity"),get("dither_ramp")});
        combined_=std::make_unique<StatefulFriction>(make_common_probe_friction(
          StatefulParameters{law.yield(true,0.),law.yield(false,0.),0.,0.,get("transition_stiffness"),get("transition_decay"),0.,0.,get("common_transition_distance")},
          law,rest,dither_only,get("common_output_fraction")));
        common_release_=std::make_unique<CommonReleaseSequence>(directions_,
          get_node()->get_parameter("assistance_mask").as_integer_array(),get("common_hand_seconds"),
          target_==5?.2:.5,target_==5?3.:12.,get("common_release_speed"),get("common_minimum_travel"),get("common_zero_input_seconds"),
          soft_start_rest?get("dither_frequency"):0.);
        if(!array("tracking_times").empty()) {
          if(target_!=1 || manual_hand_ || common_rest || get("common_zero_input_seconds")!=0. || directions_.size()!=1 ||
            get("common_release_speed")!=.3 || get("common_minimum_travel")!=0.)return Callback::ERROR;
          tracking_=std::make_unique<CurrentTrackingSequence>(array("tracking_times"),
            array("tracking_positions"),array("tracking_velocities"),array("tracking_feedforward"));
        }
        if(get("common_minimum_travel")!=0. && target_!=1)return Callback::ERROR;
        if(!common_release_speed_allowed(target_,get("common_release_speed"),manual_hand_))return Callback::ERROR;
        if(manual_hand_ && dither_only)return Callback::ERROR;
        gravity_angles_=array("gravity_angles");gravity_values_=array("gravity_values");
        if(gravity_angles_.size()<100 || gravity_angles_.size()!=gravity_values_.size() ||
          gravity_angles_.front()>-.18 || gravity_angles_.back()<.18)return Callback::ERROR;
        for(size_t i=0;i<gravity_angles_.size();++i)
          if(!std::isfinite(gravity_angles_[i]) || !std::isfinite(gravity_values_[i]) ||
            (i && gravity_angles_[i]<=gravity_angles_[i-1]))return Callback::ERROR;
        if(manual_hand_ && (std::abs(gravity_angles_.back()-gravity_angles_.front()-2.*std::acos(-1.))>1e-10 ||
          std::abs(gravity_values_.back()-gravity_values_.front())>1e-8))return Callback::ERROR;
      } catch(const std::exception &) {return Callback::ERROR;}
    }
    if(is_ab()) {
      ab_distance_=get_node()->get_parameter("ab_release_distance_rad").as_double();
      ab_speed_=get_node()->get_parameter("ab_release_speed_rad_s").as_double();
      if(!(ab_distance_>=.0005 && ab_distance_<=.003 && ab_speed_>=.015 && ab_speed_<=.03)) return Callback::ERROR;
      assistance_mask_=get_node()->get_parameter("assistance_mask").as_integer_array();
      if(assistance_mask_.size()!=directions_.size()) return Callback::ERROR;
      for(auto m:assistance_mask_) if(m!=0 && m!=1) return Callback::ERROR;
      fp_=get_node()->get_parameter("positive_yield_nm").as_double();
      fn_=get_node()->get_parameter("negative_yield_nm").as_double();
      bp_=get_node()->get_parameter("positive_viscous").as_double();
      bn_=get_node()->get_parameter("negative_viscous").as_double();
      if(!(fp_>0. && fp_<5. && fn_>0. && fn_<5. && bp_>=0. && bp_<20. && bn_>=0. && bn_<20.)) return Callback::ERROR;
      try {
        const auto get=[this](const char *p){return get_node()->get_parameter(p).as_double();};
        if(kind_=="combined_ab") {
          if(get("additional_damping")!=0. || bp_!=0. || bn_!=0.) return Callback::ERROR;
          combined_=std::make_unique<StatefulFriction>(
            StatefulParameters{fp_,fn_,0.,0.,get("transition_stiffness"),get("transition_decay"),0.,0.},
            DirectionalKineticCurve(get_node()->get_parameter("combined_speed").as_double_array(),
              get_node()->get_parameter("combined_positive").as_double_array(),
              get_node()->get_parameter("combined_negative").as_double_array(),true),get("combined_scale"),
            RestDither({get("dither_amplitude"),get("dither_frequency"),get("dither_full_velocity"),
              get("dither_zero_velocity"),get("dither_ramp")}),true);
        } else {
          transition_=std::make_unique<ElastoplasticReplay>(get("transition_stiffness"),
            get("transition_decay"),get("additional_damping"));
        }
      } catch(const std::exception &) {return Callback::ERROR;}
    }
    deferred_start_=get_node()->get_parameter("defer_probe_start").as_bool();
    if(deferred_start_ && !common_release_ && !stationary_gravity_ && !neutral_response && !rest_selection)return Callback::ERROR;
    if(neutral_response && !deferred_start_)return Callback::ERROR;
    expected_=get_node()->get_parameter("expected_pose").as_double_array();
    const auto reviewed_pose=get_node()->get_parameter("stationary_reviewed_pose").as_double_array();
    const double start_angle=get_node()->get_parameter("common_start_angle").as_double();
    if(!common_release_start_allowed(target_,get_node()->get_parameter("common_release_speed").as_double(),start_angle,manual_hand_) ||
       (start_angle!=0. && (!common_release_ || !deferred_start_ || stationary_gravity_ ||
        expected_.size()!=6 || std::abs(expected_[1]-start_angle)>=.001 || std::abs(expected_[2])>=.001)))return Callback::ERROR;
    if(!reviewed_pose.empty() && !valid_loaded_rest_pose(
        reviewed_pose,expected_,stationary_gravity_,deferred_start_,target_))return Callback::ERROR;
    ends_=get_node()->get_parameter("step_ends_s").as_double_array();
    values_=get_node()->get_parameter("step_effort_nm").as_double_array();
    excursion_=get_node()->get_parameter("max_excursion_rad").as_double();
    speed_=get_node()->get_parameter("max_speed_rad_s").as_double();
    hold_error_=get_node()->get_parameter("held_joint_error_rad").as_double();
    file_=get_node()->get_parameter("output_csv").as_string();
    // J2 is restricted separately to a small stationary response experiment.
    if(target_<0 || target_>=6 || (!common_release_ && !neutral_response && !rest_selection && target_!=0 && target_!=1 && target_!=4 && target_!=5) || expected_.size()!=6 || ends_.empty() || ends_.size()!=values_.size() ||
       ends_.size()>100 || file_.empty() || values_.front()!=0. || values_.back()!=0. ||
       (manual_hand_ ? (excursion_!=0. || speed_!=0. || hold_error_!=0.) :
       (!(excursion_>0. && excursion_<=(common_release_ ?
          (get_node()->get_parameter("common_release_speed").as_double()!=.02?(target_==1?(tracking_?1.1:.9):1.4):.18):.03)) ||
        !(speed_>0. && speed_<=(common_release_ ?
          (get_node()->get_parameter("common_release_speed").as_double()!=.02?(target_==1?.6:.9):.3):.1)) ||
        !(hold_error_>0. && hold_error_<=0.03)))) return Callback::ERROR;
    for(double q:expected_) if(!std::isfinite(q)) return Callback::ERROR;
    if(rest_selection) {
      if(!deferred_start_ || excursion_>.002 || hold_error_>.001)return Callback::ERROR;
      const auto amplitudes=get_node()->get_parameter("sine_amplitudes_nm").as_double_array();
      for(double a:amplitudes)if(a>3.)return Callback::ERROR;
    }
    if(neutral_response) {
      if(excursion_>.002 || hold_error_>.001)return Callback::ERROR;
      for(double q:expected_)if(std::abs(q)>=.001)return Callback::ERROR;
      const auto amplitudes=get_node()->get_parameter("sine_amplitudes_nm").as_double_array();
      for(double a:amplitudes)
        if(a>1.28)return Callback::ERROR;
    }
    if(target_==1) {
      if((kind_!="rest_response" && !common_release_) || (!common_release_ && excursion_>.002) || hold_error_>.001) return Callback::ERROR;
      if(!manual_hand_ && !rest_selection && reviewed_pose.empty())for(size_t j=0;j<expected_.size();++j)
        if(std::abs(expected_[j]-(j==1?start_angle:0.))>=(stationary_gravity_ && j==1?.041:common_release_ && j==2?.251:.001))return Callback::ERROR;
      const auto sine_amplitudes=get_node()->get_parameter("sine_amplitudes_nm").as_double_array();
      for(double a:sine_amplitudes)
        if(a>((stationary_gravity_ || rest_selection)?3.:1.28)) return Callback::ERROR;
    }
    if(stationary_gravity_) {
      gravity_angles_=get_node()->get_parameter("gravity_angles").as_double_array();
      gravity_values_=get_node()->get_parameter("gravity_values").as_double_array();
      if(gravity_angles_.size()<100 || gravity_angles_.size()!=gravity_values_.size() ||
         gravity_angles_.front()>expected_[target_]-excursion_ ||
         gravity_angles_.back()<expected_[target_]+excursion_)return Callback::ERROR;
      for(size_t i=0;i<gravity_angles_.size();++i)
        if(!std::isfinite(gravity_angles_[i]) || !std::isfinite(gravity_values_[i]) ||
           (i && gravity_angles_[i]<=gravity_angles_[i-1]))return Callback::ERROR;
    }
    for(size_t k=0;k<ends_.size();++k) if(!std::isfinite(ends_[k]) ||
      ends_[k]<=(k?ends_[k-1]:0.) || ends_[k]>120. || !std::isfinite(values_[k]) ||
      std::abs(values_[k])>0.641) return Callback::ERROR;
    // SPSC journal ring: the status callback flushes completed rows, then
    // releases those slots. Duration alone cannot exhaust the recording buffer.
    rows_.resize(16384);
    status_.store(0); fault_.store(0); published_count_.store(0);
    used_=false;saved_count_=0;consumed_count_.store(0);last_saved_row_={};
    journal_.open(file_);journal_ << std::setprecision(17) << "time_s,dt_s,command_nm,phase";
    for(size_t j=0;j<6;++j) for(const char * name:{"q","v","effort"}) journal_<<",j"<<j+1<<"_"<<name;
    journal_<<",j"<<target_+1<<"_position_velocity,j"<<target_+1<<"_velocity_age_s,test_input_nm,assistance_nm,elastic_nm,rate_term_nm,damping_nm,dither_nm,dither_envelope,gravity_nm";
    for(size_t j=0;j<6;++j)journal_<<",j"<<j+1<<"_drive_error";
    journal_<<"\n";
    journal_.flush();if(!journal_.good()) return Callback::ERROR;
    service_=get_node()->create_service<std_srvs::srv::Trigger>("~/save_status",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        int s=status_.load(std::memory_order_acquire);
        // Read only completed rows published by RT. Release slots after copying
        // the last status row and flushing; the producer never overwrites them early.
        const size_t available=published_count_.load(std::memory_order_acquire);
        while(saved_count_<available) {
          const auto &r=rows_[saved_count_%rows_.size()];journal_<<r.t<<","<<r.dt<<","<<r.command<<","<<r.phase;
          for(double v:r.state) journal_<<","<<v;
          journal_<<","<<r.observed_velocity<<","<<r.velocity_age<<","<<r.test_input<<","<<r.assistance<<","<<r.elastic<<","<<r.rate<<","<<r.damping<<","<<r.dither<<","<<r.dither_envelope<<","<<r.gravity;
          for(double e:r.drive_errors)journal_<<","<<e;
          journal_<<"\n";
          last_saved_row_=r;++saved_count_;
        }
        journal_.flush();
        consumed_count_.store(saved_count_,std::memory_order_release);
        const bool saved=journal_.good() && (s==2 || s==3 || s==5);
        std::ostringstream message;
        message<<"{\"state\":"<<s<<",\"fault\":"<<fault_.load()
          // The receipt describes the complete snapshot flushed above. RT may
          // publish more gravity-only rows while this callback writes the CSV.
          <<",\"rows\":"<<available<<",\"saved_rows\":"<<saved_count_
          <<",\"published_rows_at_response\":"<<published_count_.load()
          <<",\"journal_ok\":"<<(journal_.good()?"true":"false")<<",\"saved\":"<<(saved?"true":"false");
        if(available) {
          const auto &last=last_saved_row_;
          message<<std::setprecision(17)<<",\"last_phase\":"<<last.phase
            <<",\"last_command_nm\":"<<last.command<<",\"last_gravity_nm\":"<<last.gravity
            <<",\"last_test_input_nm\":"<<last.test_input<<",\"last_assistance_nm\":"<<last.assistance
            <<",\"last_target_position_rad\":"<<last.state[3*target_];
        }
        message<<"}";
        res->success=(s==2 && saved); res->message=message.str();
      });
    start_service_=get_node()->create_service<std_srvs::srv::Trigger>("~/start_probe",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        res->success=deferred_start_ && status_.load()==4 && !start_requested_.exchange(true);
        res->message=res->success?"Start requested after supervisor motor verification":"Probe not awaiting start";
      });
    stop_service_=get_node()->create_service<std_srvs::srv::Trigger>("~/stop_probe",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        const int state=status_.load();
        res->success=deferred_start_ && state>=1 && state<=5;
        if(res->success && (state==1 || state==4))stop_requested_.store(true);
        res->message=res->success?"Stimulus stop requested; gravity/position holds remain active":"Probe not active";
      });
    auto publisher=get_node()->create_publisher<sensor_msgs::msg::JointState>("~/command_state",rclcpp::SensorDataQoS());
    command_publisher_=std::make_unique<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(publisher);
    auto &message=command_publisher_->msg_;
    for(size_t j=0;j<6;++j)message.name.push_back("joint"+std::to_string(j+1));
    message.position.resize(6);message.effort.resize(6);
    return Callback::SUCCESS;
  }
  controller_interface::InterfaceConfiguration command_interface_configuration() const override {
    controller_interface::InterfaceConfiguration c{controller_interface::interface_configuration_type::INDIVIDUAL,{}};
    for(size_t j=0;j<6;++j) c.names.push_back("joint"+std::to_string(j+1)+(j==static_cast<size_t>(target_)?"/effort":"/position"));
    return c;
  }
  controller_interface::InterfaceConfiguration state_interface_configuration() const override {
    controller_interface::InterfaceConfiguration c{controller_interface::interface_configuration_type::INDIVIDUAL,{}};
    for(size_t j=0;j<6;++j) for(const char *type:{"position","velocity","effort"})
      c.names.push_back("joint"+std::to_string(j+1)+"/"+type);
    if(monitor_drive_errors_)for(size_t j=0;j<6;++j)
      c.names.push_back("dxl"+std::to_string(j+1)+"/Error Code");
    return c;
  }
  Callback on_activate(const rclcpp_lifecycle::State &) override {
    if(used_) return Callback::ERROR;
    used_=true;
    auto cn=command_interface_configuration().names, sn=state_interface_configuration().names;
    for(size_t k=0;k<cn.size();++k) {
      size_t i=0; for(;i<command_interfaces_.size();++i) if(command_interfaces_[i].get_name()==cn[k]) break;
      if(i==command_interfaces_.size()) return Callback::ERROR;
      ci_[k]=i;
    }
    for(size_t k=0;k<sn.size();++k) {
      size_t i=0; for(;i<state_interfaces_.size();++i) if(state_interfaces_[i].get_name()==sn[k]) break;
      if(i==state_interfaces_.size()) return Callback::ERROR;
      si_[k]=i;
    }
    for(size_t j=0;j<6;++j) {
      auto q=state_interfaces_[si_[3*j]].get_optional();
      if(!q || !std::isfinite(*q) || std::abs(*q-expected_[j])>(target_==1?.0005:.03)) {
        RCLCPP_ERROR(get_node()->get_logger(),"Activation pose mismatch J%zu: expected %.9f, read %.9f",j+1,expected_[j],q?*q:NAN);
        return Callback::ERROR;
      }
      hold_[j]=*q;
    }
    observer_.reset();active_assistance_=false;
    if(transition_) transition_->reset();
    if(combined_) combined_->reset();
    prepared_combined_=false;
    count_=0;elapsed_=0.;sine_elapsed_=0.;stage_elapsed_=0.;ramp_trial_=0;ramp_active_=false;rest_observed_=false;
    fault_.store(0);published_count_.store(0);
    start_requested_.store(false);
    stop_requested_.store(false);command_publish_elapsed_=0.;
    const double initial=deferred_start_?gravity_at(hold_[target_]):0.;
    if(!std::isfinite(initial) || !write(initial)) return Callback::ERROR;
    status_.store(deferred_start_?4:1,std::memory_order_release);
    return Callback::SUCCESS;
  }
  Callback on_deactivate(const rclcpp_lifecycle::State &) override {
    // A successful finite test keeps valid gravity until the hardware's OFF
    // transition. Faults retain the original zero-command shutdown behavior.
    const auto q=state_interfaces_[si_[3*target_]].get_optional();
    const double g=deferred_start_ && status_.load()==2 && q?gravity_at(*q):0.;
    write(std::isfinite(g)?g:0.);
    if(status_.load()==1 || status_.load()==4) {fault_.store(6);status_.store(3,std::memory_order_release);}
    return Callback::SUCCESS;
  }
  controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override {
    int state=status_.load();
    if(deferred_start_ && stop_requested_.exchange(false) && (state==1 || state==4)) {
      start_requested_.store(false);status_.store(5,std::memory_order_release);state=5;
    }
    if(deferred_start_ && state==4 && start_requested_.exchange(false)) {
      status_.store(1,std::memory_order_release);state=1;
    }
    const bool finite_gravity_diagnostic=stationary_gravity_ || (common_release_ && !manual_hand_);
    const bool abort_gravity=state==3 && probe_abort_keeps_gravity(
      finite_gravity_diagnostic,deferred_start_,fault_.load());
    const bool gravity_only=deferred_start_ && (state==4 || state==2 || state==5 || abort_gravity);
    if(state!=1 && !gravity_only) {write(0.);publish_command(time,period);return controller_interface::return_type::OK;}
    Row row{};row.t=elapsed_;row.dt=period.seconds();
    int error=0;
    if(state==4 && elapsed_>30.)error=13;
    if(!(row.dt>0. && row.dt<=0.02)) error=1;
    for(size_t k=0;k<18;++k) {
      auto v=state_interfaces_[si_[k]].get_optional();
      if(!v || !std::isfinite(*v)) {error=2;row.state[k]=NAN;} else row.state[k]=*v;
    }
    const int motion_error=probe_motion_fault(row.state,hold_,static_cast<size_t>(target_),
      excursion_,state==4?std::max(hold_error_,.01):hold_error_,speed_,manual_hand_);
    if(motion_error && !error)error=motion_error;
    if(monitor_drive_errors_)for(size_t j=0;j<6;++j) {
      const auto e=state_interfaces_[si_[18+j]].get_optional();
      row.drive_errors[j]=e?*e:NAN;
      if(!e || !std::isfinite(*e) || *e!=0.)error=12;
    }
    if(abort_gravity || probe_abort_keeps_gravity(finite_gravity_diagnostic,deferred_start_,error)) {
      // The gravity table assumes the other five axes hold their pose. Losing
      // that premise is a different abort and cannot keep this gravity lookup.
      for(size_t j=0;j<6;++j) if(j!=static_cast<size_t>(target_) &&
        (std::abs(row.state[3*j]-hold_[j])>(state==4?.01:hold_error_) ||
         std::abs(row.state[3*j+1])>speed_))error=14;
    }
    std::array<double,6> positions{};
    for(size_t j=0;j<6;++j) positions[j]=row.state[3*j];
    const auto observed=observer_.update(elapsed_,positions);
    row.observed_velocity=observed.valid?observed.velocity[target_]:NAN;
    row.velocity_age=observed.age;
    if(elapsed_>.1 && !observed.valid) error=8;
    bool done=false;
    if(gravity_only) {
      row.phase=abort_gravity?-4:state==4?-2:state==5?-5:-3;
    } else if(tracking_ && !error) {
      try {
        const auto sample=tracking_->step(row.state[3*target_],
          observed.valid?observed.velocity[target_]:0.,row.dt);
        // Stored feedforward includes nominal gravity at the reference pose.
        // The shared command path adds actual-pose gravity below. Subtract it
        // here to avoid counting gravity twice; abort retains actual gravity.
        row.command=sample.command-gravity_at(row.state[3*target_]);
        row.phase=60;active_assistance_=false;done=sample.done;
        if(sample.failed)error=11;
      } catch(const std::exception &) {error=9;}
    } else if(common_release_ && !error) {
      const auto sample=common_release_->step(row.state[3*target_],observed.valid?observed.velocity[target_]:0.,row.dt);
      row.command=sample.input;row.phase=10*sample.trial+sample.phase;
      active_assistance_=sample.assistance;done=sample.done;
      if(sample.failed)error=11;
    } else if(sine_probe_) {
      // Priming/settling time must not consume any part of the first sine block.
      const auto sample=sine_probe_->sample(sine_elapsed_);
      sine_elapsed_+=row.dt;
      row.command=sample.command;row.phase=sample.phase;done=sample.done;
    } else if(kind_=="step") {
      size_t phase=0;while(phase<ends_.size() && elapsed_>=ends_[phase]) ++phase;
      done=phase==ends_.size();row.phase=static_cast<int>(phase);
      row.command=done?0.:values_[phase];
    } else {
      // Slow torque ramps measure operational breakaway without position PI.
      // Stop each ramp after 1 mrad displacement OR 0.01 rad/s; rest 3 seconds.
      // Precondition the exact combined controller during the known zero-input
      // dwell. Preserve its state across same-ON reversals, as in freedrive.
      if(combined_ && !ramp_active_ && !prepared_combined_ && stage_elapsed_>=2. &&
        ramp_trial_<static_cast<int>(directions_.size())) {
        const bool next=assistance_mask_[ramp_trial_]!=0;
        if(next!=active_assistance_) combined_->reset();
        active_assistance_=next; prepared_combined_=true;
      }
      if(is_ab() && !ramp_active_ && ramp_trial_>0 && stage_elapsed_>=3.) {
        if(!rest_observed_) {rest_min_=rest_max_=row.state[3*target_];rest_observed_=true;}
        rest_min_=std::min(rest_min_,row.state[3*target_]);rest_max_=std::max(rest_max_,row.state[3*target_]);
      }
      if(!ramp_active_ && stage_elapsed_ >= (combined_?5.:(ramp_trial_==0?2.:(is_ab()?5.:3.))) && ramp_trial_<static_cast<int>(directions_.size())) {
        if(is_ab()) {
          if(ramp_trial_>0 && (!rest_observed_ || rest_max_-rest_min_>(combined_?.0002:.00006) ||
            (observed.valid && std::abs(observed.velocity[target_])>(combined_?.01:.002)))) error=10;
          if(transition_) {active_assistance_=assistance_mask_[ramp_trial_]!=0;transition_->reset();}
          rest_observed_=false;
        }
        ramp_active_=true;stage_elapsed_=0.;ramp_start_q_=row.state[3*target_];
      }
      row.phase=ramp_trial_*2+(ramp_active_?1:0);
      if(ramp_active_) {
        const bool released=is_ab() ?
          (std::abs(row.state[3*target_]-ramp_start_q_)>=ab_distance_ ||
           (observed.valid && std::abs(observed.velocity[target_])>=ab_speed_)) : kind_=="traverse" ?
          (std::abs(row.state[3*target_]-ramp_start_q_)>=0.003 ||
           (observed.valid && std::abs(observed.velocity[target_])>=0.03)) :
          (std::abs(row.state[3*target_]-ramp_start_q_)>=0.001 || std::abs(row.state[3*target_+1])>=0.01);
        row.command=directions_[ramp_trial_]*std::min(ramp_max_,ramp_rate_*stage_elapsed_);
        if(released || stage_elapsed_>=(kind_=="traverse"?20.:15.)) {
          row.command=0.;row.phase=100+ramp_trial_;
          ++ramp_trial_;ramp_active_=false;stage_elapsed_=0.;rest_observed_=false;prepared_combined_=false;
        }
      } else row.command=0.;
      done=(ramp_trial_==static_cast<int>(directions_.size()) && stage_elapsed_>=(is_ab()?5.:3.)) || elapsed_>=195.;
      stage_elapsed_+=row.dt;
    }
    if((common_release_ || stationary_gravity_) && (!error ||
      probe_abort_keeps_gravity(finite_gravity_diagnostic,deferred_start_,error))) {
      row.gravity=gravity_at(row.state[3*target_]);
      if(!std::isfinite(row.gravity)) {row.gravity=0.;error=9;}
    }
    row.test_input=row.command;
    if(kind_=="transition_ab" && observed.valid && !error && !done) {
      try {
        const auto assist=transition_->step(row.state[3*target_],observed.velocity[target_],row.dt,fp_,fn_,
          [this](double vp){return (vp>=0.?bp_:bn_)*vp;},active_assistance_);
        row.assistance=assist.total;row.elastic=assist.elastic;row.rate=assist.rate_term;row.damping=assist.damping;
        row.command+=assist.total;
        if(!std::isfinite(row.command) || std::abs(row.command)>5.) error=9;
      } catch(const std::exception &) {error=9;}
    }
    if(combined_ && observed.valid && !error && !done && !gravity_only) {
      row.assistance=combined_->step(row.state[3*target_],observed.velocity[target_],row.dt,true,active_assistance_,row.gravity);
      const auto &parts=combined_->components();
      row.elastic=parts.elastic;row.rate=parts.rate_term;row.damping=parts.damping;
      row.dither=combined_->dither_components().torque;row.dither_envelope=combined_->dither_components().envelope;
      row.command+=row.assistance;
      if(combined_->fault() || !std::isfinite(row.command) || (!common_release_ && std::abs(row.command)>5.)) error=9;
    }
    row.command+=row.gravity;
    if(!std::isfinite(row.command))error=9;
    if(done && is_ab() && (!rest_observed_ || rest_max_-rest_min_>(combined_?.0002:.00006) ||
      (observed.valid && std::abs(observed.velocity[target_])>(combined_?.01:.002)))) error=10;
    if(error || done || abort_gravity) {
      const bool retain=probe_abort_keeps_gravity(finite_gravity_diagnostic,deferred_start_,error) ||
        (!error && deferred_start_ && (done || abort_gravity));
      row.command=retain?row.gravity:0.;
      if(retain && (error || abort_gravity)) {
        row.phase=-4;row.test_input=0.;row.assistance=0.;row.elastic=0.;row.rate=0.;
        row.damping=0.;row.dither=0.;row.dither_envelope=0.;
      }
      if(transition_) transition_->reset();
      if(combined_) combined_->reset();
    }
    if(!write(row.command)) {error=5;row.command=0.;write(0.);}
    if(count_-consumed_count_.load(std::memory_order_acquire)>=rows_.size()) {error=7;write(0.);}
    else {rows_[count_%rows_.size()]=row;++count_;}
    published_count_.store(count_,std::memory_order_release);
    elapsed_+=row.dt;
    if(error || done) {fault_.store(error);status_.store(error?3:2,std::memory_order_release);}
    publish_command(time,period);
    return controller_interface::return_type::OK;
  }
};
}
PLUGINLIB_EXPORT_CLASS(friction_compensation_controller::CurrentProbeController,controller_interface::ControllerInterface)
