// Copyright 2024 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Sungho Woo

#include "open_manipulator_collision/self_collision_node.hpp"
#include <kdl_parser/kdl_parser.hpp>

SelfCollisionNode::SelfCollisionNode()
: Node("self_collision_node")
{
  this->declare_parameter("robot_description", "");
  this->declare_parameter("base_link", "link0");
  this->declare_parameter("tip_link", "rh_p12_rn_base");
  this->declare_parameter("enable_marker", true);

  this->get_parameter("robot_description", urdf_string_);
  this->get_parameter("base_link", base_link_);
  this->get_parameter("tip_link", tip_link_);
  this->get_parameter("enable_marker", enable_marker_);

  collision_pairs_ = {
    {"link6", "link0"},
    {"link6", "link1"},
    {"link6", "link2"},
    {"link6", "link3"},
    {"end_effector_flange_link", "link1"},
    {"end_effector_flange_link", "link2"},
    {"end_effector_flange_link", "link3"},
    {"end_effector_flange_link", "link4"},
    {"link5", "link1"},
    {"link5", "link2"},
    {"link5", "link3"},
    {"link4", "link1"},
    {"link3", "link1"},
  };

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/leader/joint_states", 10,
    std::bind(&SelfCollisionNode::joint_callback, this, std::placeholders::_1));

  collision_pub_ = this->create_publisher<std_msgs::msg::Bool>("/collision_flag", 10);
  if (enable_marker_) {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/collision_marker",
      10);
  }

  load_model();
  RCLCPP_INFO(this->get_logger(), "Self Collision Node Initialized.");
}

void SelfCollisionNode::load_model()
{
  urdf::Model model;
  if (!model.initString(urdf_string_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF.");
    return;
  }

  if (!kdl_parser::treeFromUrdfModel(model, kdl_tree_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to convert URDF to KDL Tree.");
    return;
  }

  if (!kdl_tree_.getChain(base_link_, tip_link_, kdl_chain_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to extract KDL chain.");
    return;
  }

  fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(kdl_chain_);

  std::set<std::string> all_links;
  for (const auto & seg : kdl_chain_.segments) {
    all_links.insert(seg.getName());
  }
  all_links.insert(base_link_);
  all_links.insert(tip_link_);

  for (const auto & link : all_links) {
    process_collision_from_URDF(model, link);
  }
}

void SelfCollisionNode::process_collision_from_URDF(
  const urdf::Model & model,
  const std::string & link_name)
{
  auto urdf_link = model.getLink(link_name);
  if (!urdf_link || !urdf_link->collision || !urdf_link->collision->geometry) {return;}

  double radius = 0.02, length = 0.2;
  fcl::Transform3d offset = fcl::Transform3d::Identity();

  if (urdf_link->collision->geometry->type == urdf::Geometry::CYLINDER) {
    auto cyl = std::dynamic_pointer_cast<urdf::Cylinder>(urdf_link->collision->geometry);
    if (cyl) {
      radius = cyl->radius;
      length = cyl->length;
    }
  }

  const auto & o = urdf_link->collision->origin;
  offset.translation() = fcl::Vector3d(o.position.x, o.position.y, o.position.z);
  double r, p, y;
  o.rotation.getRPY(r, p, y);
  offset.linear() = (Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ())).toRotationMatrix();

  collision_geometries_[link_name] = std::make_shared<fcl::Capsuled>(radius, length);
  collision_offsets_[link_name] = offset;
}

void SelfCollisionNode::joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  KDL::JntArray joint_positions(kdl_chain_.getNrOfJoints());
  size_t joint_idx = 0;
  for (const auto & seg : kdl_chain_.segments) {
    const auto & joint = seg.getJoint();
    if (joint.getType() == KDL::Joint::None) {continue;}
    auto it = std::find(msg->name.begin(), msg->name.end(), joint.getName());
    if (it != msg->name.end()) {
      joint_positions(joint_idx++) = msg->position[std::distance(msg->name.begin(), it)];
    }
  }

  std::map<std::string, fcl::Transform3d> link_transforms;
  for (size_t i = 0; i < kdl_chain_.getNrOfSegments(); ++i) {
    KDL::Frame frame;
    fk_solver_->JntToCart(joint_positions, frame, i);
    std::string link_name = (i == 0) ? base_link_ : kdl_chain_.getSegment(i - 1).getName();
    fcl::Transform3d tf = fcl::Transform3d::Identity();
    tf.translation() = fcl::Vector3d(frame.p.x(), frame.p.y(), frame.p.z());
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        tf.linear()(r, c) = frame.M(r, c);
      }
    }
    link_transforms[link_name] = tf * collision_offsets_[link_name];
  }

  const auto & last_seg = kdl_chain_.getSegment(kdl_chain_.getNrOfSegments() - 1).getName();
  if (!link_transforms.count(tip_link_) && link_transforms.count(last_seg)) {
    link_transforms[tip_link_] = collision_offsets_[tip_link_] * link_transforms[last_seg];
  }

  bool collision = false;
  std::map<std::string, bool> link_collision_flags;
  visualization_msgs::msg::MarkerArray marker_array;
  int marker_id = 0;

  for (const auto & [name1, name2] : collision_pairs_) {
    if (!link_transforms.count(name1) || !link_transforms.count(name2)) {continue;}
    fcl::CollisionObjectd obj1(collision_geometries_[name1], link_transforms[name1]);
    fcl::CollisionObjectd obj2(collision_geometries_[name2], link_transforms[name2]);
    fcl::CollisionResultd res;
    fcl::collide(&obj1, &obj2, fcl::CollisionRequestd(), res);
    if (res.isCollision()) {
      collision = true;
      link_collision_flags[name1] = link_collision_flags[name2] = true;
    }
  }

  if (enable_marker_) {
    std::set<std::string> all_links;
    for (const auto & [l1, l2] : collision_pairs_) {
      all_links.insert(l1);
      all_links.insert(l2);
    }
    all_links.insert(tip_link_);

    for (const auto & link : all_links) {
      if (!link_transforms.count(link)) {continue;}
      bool colliding = link_collision_flags.count(link) > 0;
      auto markers = create_capsule_markers(marker_id++, link, link_transforms[link], colliding);
      marker_array.markers.insert(
        marker_array.markers.end(), markers.markers.begin(),
        markers.markers.end());
      marker_array.markers.push_back(create_text_marker(marker_id++, link, link_transforms[link]));
    }
    marker_pub_->publish(marker_array);
  }

  std_msgs::msg::Bool out;
  out.data = collision;
  collision_pub_->publish(out);
}
visualization_msgs::msg::MarkerArray SelfCollisionNode::create_capsule_markers(
  int id_start, const std::string & link_name, const fcl::Transform3d & transform,
  bool is_collision)
{
  visualization_msgs::msg::MarkerArray array;
  auto capsule = std::dynamic_pointer_cast<fcl::Capsuled>(collision_geometries_[link_name]);
  if (!capsule) {return array;}

  std_msgs::msg::ColorRGBA color;
  color.r = is_collision ? 1.0f : 0.0f;
  color.g = is_collision ? 0.0f : 1.0f;
  color.b = 0.0f;
  color.a = 0.2f;

  Eigen::Vector3d pos = transform.translation();
  Eigen::Quaterniond q(transform.linear());

  visualization_msgs::msg::Marker cyl;
  cyl.header.frame_id = base_link_;
  cyl.header.stamp = this->now();
  cyl.ns = "capsule_" + link_name;
  cyl.id = id_start;
  cyl.type = visualization_msgs::msg::Marker::CYLINDER;
  cyl.action = visualization_msgs::msg::Marker::ADD;
  cyl.scale.x = cyl.scale.y = 2.0 * capsule->radius;
  cyl.scale.z = capsule->lz;
  cyl.pose.position.x = pos.x();
  cyl.pose.position.y = pos.y();
  cyl.pose.position.z = pos.z();
  cyl.pose.orientation.x = q.x();
  cyl.pose.orientation.y = q.y();
  cyl.pose.orientation.z = q.z();
  cyl.pose.orientation.w = q.w();
  cyl.color = color;
  cyl.lifetime = rclcpp::Duration::from_seconds(0.3);
  array.markers.push_back(cyl);

  // TOP SPHERE
  Eigen::Vector3d top_offset = transform.linear() * Eigen::Vector3d(0, 0, capsule->lz / 2.0);
  visualization_msgs::msg::Marker top;
  top = cyl;
  top.id = id_start + 1;
  top.type = visualization_msgs::msg::Marker::SPHERE;
  top.scale.z = top.scale.y = top.scale.x = 2.0 * capsule->radius;
  top.pose.position.x += top_offset.x();
  top.pose.position.y += top_offset.y();
  top.pose.position.z += top_offset.z();
  array.markers.push_back(top);

  // BOTTOM SPHERE
  Eigen::Vector3d bottom_offset = transform.linear() * Eigen::Vector3d(0, 0, -capsule->lz / 2.0);
  visualization_msgs::msg::Marker bottom;
  bottom = cyl;
  bottom.id = id_start + 2;
  bottom.type = visualization_msgs::msg::Marker::SPHERE;
  bottom.scale.z = bottom.scale.y = bottom.scale.x = 2.0 * capsule->radius;
  bottom.pose.position.x += bottom_offset.x();
  bottom.pose.position.y += bottom_offset.y();
  bottom.pose.position.z += bottom_offset.z();
  array.markers.push_back(bottom);

  return array;
}


visualization_msgs::msg::Marker SelfCollisionNode::create_text_marker(
  int id, const std::string & link_name, const fcl::Transform3d & transform)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = base_link_;
  marker.header.stamp = this->now();
  marker.ns = "label_" + link_name;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = transform.translation().x();
  marker.pose.position.y = transform.translation().y();
  marker.pose.position.z = transform.translation().z() + 0.05;
  marker.scale.z = 0.05;
  marker.color.r = 1.0f;
  marker.color.g = 1.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0f;
  marker.text = link_name;
  marker.lifetime = rclcpp::Duration::from_seconds(0.3);
  return marker;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SelfCollisionNode>());
  rclcpp::shutdown();
  return 0;
}
