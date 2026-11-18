#include "contact_responsive_controller/contact_responsive_controller.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <chrono>
#include <limits>

namespace omy_contact
{
namespace
{
int64_t steady_ns() {return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();}
std::vector<double> values(const Vector & v) {return {v.data(),v.data()+6};}
}

controller_interface::CallbackReturn ContactResponsiveController::on_init()
{
  const auto result=Base::on_init();
  if(result!=controller_interface::CallbackReturn::SUCCESS) {return result;}
  rcl_interfaces::msg::ParameterDescriptor fixed;fixed.read_only=true;
  const auto declare=[&](const std::string & key,const Vector & v) {
      get_node()->declare_parameter<std::vector<double>>(key,values(v),fixed);};
  declare("nominal.stiffness",settings_.stiffness);declare("nominal.damping",settings_.damping);
  declare("inner.stiffness",settings_.inner_stiffness);declare("inner.damping",settings_.inner_damping);
  declare("contact.task_limit_nm",settings_.task_limit);
  declare("contact.correction_limit_nm",settings_.correction_limit);
  declare("contact.motion_limit_nm",settings_.motion_limit);
  declare("contact.nominal_error_rad",settings_.nominal_error);
  declare("model.rotor_inertia",Vector::Constant(.01));
  get_node()->declare_parameter<double>("qp.error_slack_weight",settings_.error_slack_weight,fixed);
  get_node()->declare_parameter<double>("qp.tolerance",settings_.solver_tolerance,fixed);
  get_node()->declare_parameter<int>("qp.max_iterations",settings_.solver_iterations,fixed);
  get_node()->declare_parameter<double>("qp.max_period_s",settings_.max_period,fixed);
  get_node()->declare_parameter<double>("fallback.ramp_s",fallback_ramp_,fixed);
  get_node()->declare_parameter<std::string>("model.root_link","link0",fixed);
  get_node()->declare_parameter<std::string>("model.tip_link","end_effector_link",fixed);
  get_node()->declare_parameter<bool>("gravity_compensation_enabled",true,fixed);
  get_node()->declare_parameter<bool>("friction_compensation_enabled",true,fixed);
  return result;
}

controller_interface::CallbackReturn ContactResponsiveController::on_configure(const rclcpp_lifecycle::State & state)
{
  auto states=get_node()->get_parameter("state_interfaces").as_string_array();
  std::sort(states.begin(),states.end());
  const auto commands=get_node()->get_parameter("command_interfaces").as_string_array();
  const auto joints=get_node()->get_parameter("joints").as_string_array();
  const auto command_joints=get_node()->get_parameter("command_joints").as_string_array();
  if(states!=std::vector<std::string>({"position","velocity"}) ||
    commands!=std::vector<std::string>({"effort"}) || joints.size()!=6 ||
    (!command_joints.empty() && command_joints!=joints)) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "Require six matching joints, position+velocity states and effort-only commands");
    return controller_interface::CallbackReturn::ERROR;
  }
  const auto load=[&](const std::string & key,Vector & v) {
      const auto data=get_node()->get_parameter(key).as_double_array();
      if(data.size()!=6) {return false;}
      for(int i=0;i<6;++i) {v[i]=data[i];}
      return v.allFinite();};
  if(!load("nominal.stiffness",settings_.stiffness) || !load("nominal.damping",settings_.damping) ||
    !load("inner.stiffness",settings_.inner_stiffness) || !load("inner.damping",settings_.inner_damping) ||
    !load("contact.task_limit_nm",settings_.task_limit) || !load("contact.correction_limit_nm",settings_.correction_limit) ||
    !load("contact.motion_limit_nm",settings_.motion_limit) || !load("contact.nominal_error_rad",settings_.nominal_error) ||
    !load("model.rotor_inertia",rotor_inertia_)) {
    RCLCPP_ERROR(get_node()->get_logger(),"All joint parameters require six finite values");
    return controller_interface::CallbackReturn::ERROR;
  }
  settings_.error_slack_weight=get_node()->get_parameter("qp.error_slack_weight").as_double();
  settings_.solver_tolerance=get_node()->get_parameter("qp.tolerance").as_double();
  settings_.solver_iterations=static_cast<int>(get_node()->get_parameter("qp.max_iterations").as_int());
  settings_.max_period=get_node()->get_parameter("qp.max_period_s").as_double();
  fallback_ramp_=get_node()->get_parameter("fallback.ramp_s").as_double();
  gravity_enabled_=get_node()->get_parameter("gravity_compensation_enabled").as_bool();
  friction_enabled_policy_=get_node()->get_parameter("friction_compensation_enabled").as_bool();
  if((rotor_inertia_.array()<0.).any() || !std::isfinite(fallback_ramp_) || fallback_ramp_<=0. ||
    !law_.configure(settings_)) {
    RCLCPP_ERROR(get_node()->get_logger(),"Invalid QP/controller parameters");
    return controller_interface::CallbackReturn::ERROR;
  }
  const auto result=Base::on_configure(state);
  if(result!=controller_interface::CallbackReturn::SUCCESS) {return result;}
  if(!tree_.getChain(get_node()->get_parameter("model.root_link").as_string(),
      get_node()->get_parameter("model.tip_link").as_string(),chain_) || chain_.getNrOfJoints()!=6) {
    RCLCPP_ERROR(get_node()->get_logger(),"Nominal chain must have six ordered joints");
    return controller_interface::CallbackReturn::ERROR;
  }
  size_t i=0;
  for(const auto & segment:chain_.segments) {
    if(segment.getJoint().getType()!=KDL::Joint::Fixed && segment.getJoint().getName()!=joint_names_[i++]) {
      RCLCPP_ERROR(get_node()->get_logger(),"Nominal joint order differs from hardware");
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  dynamics_=std::make_unique<KDL::ChainDynParam>(chain_,KDL::Vector::Zero());
  target_subscription_=get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "~/reference",rclcpp::QoS(1),[this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      if(!controller_active_.load() || !motor_ready_.load() || msg->data.size()!=6) {return;}
      Reference ref;
      for(size_t k=0;k<6;++k) {if(!std::isfinite(msg->data[k])) {return;} ref.values[k]=msg->data[k];}
      ref.stamp_ns=steady_ns();ref.sequence=target_sequence_.fetch_add(1)+1;
      target_input_.writeFromNonRT(ref);
    });
  effort_subscription_.reset();
  effort_subscription_=get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "~/commands",rclcpp::QoS(1),[this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      if(!controller_active_.load() || !motor_ready_.load()) {return;}
      Reference ref;
      if(msg->data.size()!=6) {effort_input_.writeFromNonRT(ref);return;}
      std::copy(msg->data.begin(),msg->data.end(),ref.values.begin());ref.stamp_ns=steady_ns();
      if(!friction_compensation_controller::valid_effort_reference(ref)) {effort_input_.writeFromNonRT(Reference{});return;}
      ref.sequence=effort_sequence_.fetch_add(1)+1;effort_input_.writeFromNonRT(ref);
    });
  reset_service_=get_node()->create_service<std_srvs::srv::Trigger>("~/reset_contact_controller",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      response->success=controller_active_.load() && motor_ready_.load();
      if(response->success) {reset_requested_.store(true);}
      response->message=response->success?"Reset queued: capture current pose in next update":"Controller not ready";
    });
  telemetry_=std::make_unique<realtime_tools::RealtimePublisher<control_msgs::msg::DynamicJointState>>(
    get_node()->create_publisher<control_msgs::msg::DynamicJointState>("~/impedance_state",rclcpp::SensorDataQoS()));
  telemetry_->msg_.joint_names=joint_names_;telemetry_->msg_.interface_values.resize(6);
  for(auto & item:telemetry_->msg_.interface_values) {
    item.interface_names={"reference_position","position_error","enabled","update_sequence",
      "nominal_position","nominal_velocity","nominal_acceleration","task_nm","correction_nm",
      "motion_nm","external_effort_nm","error_slack_rad","mode","failure_reason","qp_status",
      "qp_iterations","qp_time_s"};
    item.values.resize(item.interface_names.size());
  }
  return result;
}

controller_interface::CallbackReturn ContactResponsiveController::on_activate(const rclcpp_lifecycle::State & state)
{
  const auto result=Base::on_activate(state);
  if(result!=controller_interface::CallbackReturn::SUCCESS) {return result;}
  if(joint_state_interface_.size()!=2 || joint_state_interface_[0].size()!=6 ||
    joint_state_interface_[1].size()!=6 || joint_command_interface_.size()!=1 ||
    joint_command_interface_[0].size()!=6) {
    controller_active_.store(false);motor_ready_.store(false);
    RCLCPP_ERROR(get_node()->get_logger(),"Incomplete resolved hardware interfaces");
    return controller_interface::CallbackReturn::ERROR;
  }
  Vector q,v;
  for(size_t i=0;i<6;++i) {
    q[i]=joint_state_interface_[0][i].get().get_optional().value_or(std::numeric_limits<double>::quiet_NaN());
    v[i]=joint_state_interface_[1][i].get().get_optional().value_or(std::numeric_limits<double>::quiet_NaN());
  }
  if(!law_.activate(q,v)) {
    controller_active_.store(false);motor_ready_.store(false);return controller_interface::CallbackReturn::ERROR;
  }
  goal_=hold_=q;last_motion_.setZero();fault_start_.setZero();output_=Result{};
  was_ready_=false;fault_latched_=false;reset_requested_.store(false);
  target_input_.writeFromNonRT(Reference{});effort_input_.writeFromNonRT(Reference{});effort_gate_={};
  applied_reference_=0;updates_=0;state_elapsed_=0.;fault_elapsed_=0.;
  return result;
}

controller_interface::return_type ContactResponsiveController::update(const rclcpp::Time & time,const rclcpp::Duration & period)
{
  Vector q,v;
  for(size_t i=0;i<6;++i) {
    q[i]=joint_state_interface_[0][i].get().get_optional().value_or(std::numeric_limits<double>::quiet_NaN());
    v[i]=joint_state_interface_[1][i].get().get_optional().value_or(std::numeric_limits<double>::quiet_NaN());
  }
  if(!q.allFinite() || !v.allFinite()) {return controller_interface::return_type::ERROR;}
  const double dt=period.seconds();
  if(!std::isfinite(dt) || dt<=0.) {return controller_interface::return_type::ERROR;}
  const bool ready=motor_ready_.load();
  const bool reset=reset_requested_.exchange(false);
  if(ready && (!was_ready_ || reset)) {
    if(!law_.activate(q,v)) {return controller_interface::return_type::ERROR;}
    goal_=hold_=q;fault_latched_=false;fault_elapsed_=0.;output_=Result{};last_motion_.setZero();
    // Discard commands predating re-enable/reset; new targets must follow this capture.
    applied_reference_=target_input_.readFromRT()->sequence;
  }
  was_ready_=ready;
  const auto target=*target_input_.readFromRT();
  if(target.sequence!=applied_reference_) {
    applied_reference_=target.sequence;
    const auto age=steady_ns()-target.stamp_ns;
    if(ready && !fault_latched_ && target.sequence && age>=0 && age<300000000) {
      for(int i=0;i<6;++i) {goal_[i]=target.values[i];}
    }
  }
  const auto external=effort_gate_.consume(*effort_input_.readFromRT(),steady_ns(),period.nanoseconds(),ready);
  Vector feedforward,motion=Vector::Zero();
  for(int i=0;i<6;++i) {feedforward[i]=external[i];}
  if(ready && !fault_latched_) {
    qn_.data=law_.position();vn_.data=law_.velocity();
    bool success=dynamics_->JntToMass(qn_,inertia_)>=0 && dynamics_->JntToCoriolis(qn_,vn_,coriolis_)>=0;
    if(success) {
      Matrix mass=inertia_.data;mass.diagonal()+=rotor_inertia_;
      success=law_.step(q,v,goal_,mass,coriolis_.data,dt,output_,feedforward);
    } else {output_=Result{};output_.failure=1;}
    if(success) {motion=output_.effort;}
    else {fault_latched_=true;hold_=q;fault_start_=last_motion_;fault_elapsed_=0.;}
  }
  if(ready && fault_latched_) {
    // Latch a low-gain hold at the fault pose; never keep replaying a failed QP result.
    const double elapsed=std::isfinite(dt) && dt>0.?std::min(dt,settings_.max_period):0.;
    fault_elapsed_+=elapsed;
    const double blend=std::min(1.,fault_elapsed_/fallback_ramp_);
    Vector hold_effort=settings_.stiffness.cwiseProduct(hold_-q)-settings_.damping.cwiseProduct(v)+feedforward;
    hold_effort=hold_effort.cwiseMax(-settings_.motion_limit).cwiseMin(settings_.motion_limit);
    motion=(1.-blend)*fault_start_+blend*hold_effort;
  }
  if(!ready) {last_motion_.setZero();}
  else {last_motion_=motion;}
  Reference combined;
  for(int i=0;i<6;++i) {combined.values[i]=motion[i];}
  combined.stamp_ns=steady_ns();combined.sequence=external_effort_sequence_.fetch_add(1)+1;
  *external_effort_buffer_.readFromRT()=combined;
  const auto result=Base::update(time,period);
  ++updates_;
  if(std::isfinite(dt) && dt>0.) {state_elapsed_+=dt;}
  if(state_elapsed_>=.02 && telemetry_ && telemetry_->trylock()) {
    auto & msg=telemetry_->msg_;msg.header.stamp=time;
    const Vector reference=fault_latched_?hold_:goal_;
    for(int i=0;i<6;++i) {
      auto & a=msg.interface_values[i].values;
      a[0]=reference[i];a[1]=reference[i]-q[i];
      a[2]=motor_ready_applied_.load() && !fault_latched_?1.:0.;a[3]=static_cast<double>(updates_);
      a[4]=law_.position()[i];a[5]=law_.velocity()[i];a[6]=output_.acceleration[i];
      a[7]=output_.task[i];a[8]=output_.correction[i];a[9]=motion[i];a[10]=feedforward[i];
      a[11]=output_.error_slack[i];a[12]=!ready?0.:(fault_latched_?2.:1.);
      a[13]=output_.failure;a[14]=output_.solver_status;a[15]=output_.iterations;a[16]=output_.solve_time_s;
    }
    telemetry_->unlockAndPublish();state_elapsed_=0.;
  }
  return result;
}

controller_interface::CallbackReturn ContactResponsiveController::on_deactivate(const rclcpp_lifecycle::State & state)
{
  was_ready_=false;last_motion_.setZero();reset_requested_.store(false);
  return Base::on_deactivate(state);
}
}  // namespace omy_contact
PLUGINLIB_EXPORT_CLASS(omy_contact::ContactResponsiveController,controller_interface::ControllerInterface)
