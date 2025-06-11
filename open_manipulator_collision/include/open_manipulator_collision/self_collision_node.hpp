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

#ifndef OPEN_MANIPULATOR_COLLISION__SELF_COLLISION_NODE_HPP_
#define OPEN_MANIPULATOR_COLLISION__SELF_COLLISION_NODE_HPP_

#include <urdf/model.h>
#include <fcl/fcl.h>

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>


#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>


namespace fcl
{
using Capsuled = fcl::Capsule<double>;
using CollisionObjectd = fcl::CollisionObject<double>;
using CollisionRequestd = fcl::CollisionRequest<double>;
using CollisionResultd = fcl::CollisionResult<double>;
using DistanceRequestd = fcl::DistanceRequest<double>;
using DistanceResultd = fcl::DistanceResult<double>;
using Transform3d = Eigen::Transform<double, 3, Eigen::Isometry>;
using Vector3d = Eigen::Matrix<double, 3, 1>;
}  // namespace fcl

class SelfCollisionNode : public rclcpp::Node
{
public:
  SelfCollisionNode();

private:
  // Core logic
  void load_model();
  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void process_collision_from_URDF(const urdf::Model & model, const std::string & link_name);

  visualization_msgs::msg::MarkerArray create_capsule_markers(
    int id, const std::string & link_name, const fcl::Transform3d & transform, bool is_collision);

  visualization_msgs::msg::Marker create_text_marker(
    int id, const std::string & link_name, const fcl::Transform3d & transform);

  // Parameters
  std::string urdf_string_;
  std::string base_link_;
  std::string tip_link_;
  bool enable_marker_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr collision_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // KDL
  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;

  // FCL collision models
  std::map<std::string, std::shared_ptr<fcl::Capsuled>> collision_geometries_;
  std::map<std::string, fcl::Transform3d> collision_offsets_;
  std::vector<std::pair<std::string, std::string>> collision_pairs_;
};

#endif  // OPEN_MANIPULATOR_COLLISION__SELF_COLLISION_NODE_HPP_
