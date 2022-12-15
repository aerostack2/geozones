/*!*******************************************************************************************
 *  \file       geofencing.hpp
 *  \brief      Geofencing for Aerostack2
 *  \authors    Javier Melero Deza
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#ifndef GEOFENCING_HPP_
#define GEOFENCING_HPP_

#include "geofence.hpp"
#include "json.hpp"

#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <iterator>
#include <vector>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "as2_core/names/services.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/node.hpp"
#include "as2_msgs/srv/set_geofence.hpp"
#include "as2_msgs/srv/get_geofence.hpp"
#include "as2_msgs/msg/alert.hpp"

class Geofencing : public as2::Node
{
public:
  Geofencing();

  void setupNode();
  void cleanupNode();
  void run();
  void loadGeofences(const std::string path);

private:

  bool start_run_;

  float self_latitude_;
  float self_longitude_;
  float self_x_;
  float self_y_;
  int max_priority;
  bool geofence_detected;

  std::string config_path_;
  std::string mode_;
  
  std::vector<std::vector<std::array<float,2>>> polygons;
  std::array<float,2> point_;
  std::vector<int>ids;
  std::vector<int>alerts;
  std::vector<bool>geofences_in;
  
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<as2_msgs::msg::Alert>::SharedPtr alert_pub_;
  
  rclcpp::Service<as2_msgs::srv::SetGeofence>::SharedPtr set_geofence_srv_;
  rclcpp::Service<as2_msgs::srv::GetGeofence>::SharedPtr get_geofence_srv_;
  
  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr _msg);
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr _msg);
  void setGeofenceCb(const std::shared_ptr<as2_msgs::srv::SetGeofence::Request> request, std::shared_ptr<as2_msgs::srv::SetGeofence::Response> response);
  void getGeofenceCb(const std::shared_ptr<as2_msgs::srv::GetGeofence::Request> request, std::shared_ptr<as2_msgs::srv::GetGeofence::Response> response);
  std::tuple<std::array<float,2>, std::vector<std::array<float,2>>> translatePolygonWithPoint(const std::vector<std::array<float,2>> polygon, const std::array<float,2> point);

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
};

#endif // BASIC_STATE_ESTIMATOR_HPP_
