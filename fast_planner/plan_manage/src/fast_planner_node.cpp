/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/



#include <rclcpp/rclcpp.hpp>

#include <plan_manage/kino_replan_fsm.h>
#include <plan_manage/topo_replan_fsm.h>

#include <plan_manage/backward.hpp>
namespace backward {
backward::SignalHandling sh;
}

using namespace fast_planner;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>
    ("fast_planner_node", rclcpp::NodeOptions());
  rclcpp::executors::SingleThreadedExecutor executor;

  int planner = 1;
  node->declare_parameter("planner_node/planner", -1);
  planner = node->get_parameter("planner_node/planner").as_int();
  RCLCPP_INFO(node->get_logger(), " Planner:[%d]", planner);

  TopoReplanFSM topo_replan;
  KinoReplanFSM kino_replan;

  if (planner == 1) {
    RCLCPP_INFO(node->get_logger(), " Starting kinodynamic planner");
    kino_replan.init(node);
  } else if (planner == 2) {
    RCLCPP_INFO(node->get_logger(), " Starting topological planner");
    topo_replan.init(node);
  }

  std::this_thread::sleep_for(std::chrono::seconds(1));
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
