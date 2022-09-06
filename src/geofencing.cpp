/*!*******************************************************************************************
 *  \file       geofencing.cpp
 *  \brief      Geofencing for AeroStack2
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

#include "geofencing.hpp"

Geofencing::Geofencing() : as2::Node("geofencing")
{
  this->declare_parameter<std::string>("config_file", "config/geofences.json");
}

void Geofencing::run()
{
  if (!start_run_)
  {
    return;
  }
  if (polygons.size() == 0){

  }
  else {
    std::array<float,2> point{self_latitude_, self_longitude_};
    as2_msgs::msg::Alert alert;
    for (std::vector<std::vector<std::array<float,2>>>::iterator ptr = polygons.begin(); ptr < polygons.end(); ptr++){
      int index = ptr - polygons.begin();
      std::vector<bool>::iterator ptr_in = geofences_in.begin() + index;
      if (!geofence::isIn<float>((*ptr), point)){
        if ((*ptr_in) == true){
          std::vector<int>::iterator ptr_alert = alerts.begin() + index;
          alert.alert_code = (*ptr_alert);
          alert_pub_->publish(alert);
          (*ptr_in) = false;
        }
      }
      else{
        if ((*ptr_in)==false){
          (*ptr_in) = true;
        }
      }
    } 
  }
}
// TODO: METHODS
void Geofencing::setupNode()
{
  gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      this->generate_global_name(as2_names::topics::sensor_measurements::gps),
      as2_names::topics::sensor_measurements::qos,
      std::bind(&Geofencing::gpsCallback, this, std::placeholders::_1));

  set_geofence_srv_ = this->create_service<as2_msgs::srv::SetGeofence>(
      this->generate_global_name("set_geofence"),
      std::bind(&Geofencing::setGeofenceCb, this, std::placeholders::_1, std::placeholders::_2));

  get_geofence_srv_ = this->create_service<as2_msgs::srv::GetGeofence>(
      this->generate_global_name("get_geofences"),
      std::bind(&Geofencing::getGeofenceCb, this, std::placeholders::_1, std::placeholders::_2));

  alert_pub_ = this->create_publisher<as2_msgs::msg::Alert>(
      this->generate_global_name("alert"), 10);

  std::string full_path_ = ament_index_cpp::get_package_share_directory("geofencing");
  //config_path_ = "config/geofences.json";
  loadGeofences(full_path_ + "/" + config_path_);
}

void Geofencing::loadGeofences(const std::string path){
  std::ifstream fJson(path);
  std::stringstream buffer;
  buffer << fJson.rdbuf();
  auto json = nlohmann::json::parse(buffer.str());
  for (auto geofence : json["geofences"])
  {
      if (std::find(ids.begin(), ids.end(), geofence["id"]) != ids.end()){
        RCLCPP_WARN(this->get_logger(), "Id already exist.");
        return;
      }
      if (std::size(geofence["polygon"]) < 3){
        RCLCPP_WARN(this->get_logger(), "Invalid Geofence.");
        return;
      }
      else{
        std::vector<std::array<float,2>> polygon;
        for (std::array<float,2> point : geofence["polygon"])
        {
            polygon.push_back(point);
        }
        alerts.push_back(geofence["alert"]);
        ids.push_back(geofence["id"]);
        geofences_in.push_back(false);
        polygons.push_back(polygon);
        RCLCPP_INFO(this->get_logger(), "Geofence Succesfully loaded from JSON file");
      }
  }
}
// CALLBACKS //

void Geofencing::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr _msg)
{
  // rclcpp::Time timestamp = this->get_clock()->now();
  // odom2baselink_tf_.header.stamp = timestamp;
  self_latitude_ = _msg->latitude;
  self_longitude_ = _msg->longitude;

  start_run_ = true;
}

void Geofencing::setGeofenceCb(const std::shared_ptr<as2_msgs::srv::SetGeofence::Request> request, 
                                std::shared_ptr<as2_msgs::srv::SetGeofence::Response> response) {
  if (std::find(ids.begin(), ids.end(), request->geofence.id) != ids.end()){
      RCLCPP_WARN(this->get_logger(), "Id already exist.");
      response->success = false;
      return;
  }
  if (std::size(request->geofence.polygon.points) < 3){
      RCLCPP_WARN(this->get_logger(), "Invalid Geofence.");
      response->success = false;
  }
  else{
      std::vector<std::array<float,2>> polygon;
      for (int i = 0; i < std::size(request->geofence.polygon.points); i++){
        std::array<float,2> point{request->geofence.polygon.points[i].x, request->geofence.polygon.points[i].y};
        polygon.push_back(point);
      }
      alerts.push_back(request->geofence.alert);
      ids.push_back(request->geofence.id);
      geofences_in.push_back(false);
      polygons.push_back(polygon);
      RCLCPP_INFO(this->get_logger(), "Geofence added.");
      response->success = true;
  }
}

void Geofencing::getGeofenceCb(const std::shared_ptr<as2_msgs::srv::GetGeofence::Request> request, 
                                std::shared_ptr<as2_msgs::srv::GetGeofence::Response> response) {
  if (polygons.size() == 0){
      RCLCPP_WARN(this->get_logger(), "No geofence has been set yet.");
      response->success = false;
  }
  else{
      std::vector<as2_msgs::msg::Geofence> geofence_list;
      for (std::vector<std::vector<std::array<float,2>>>::iterator ptr = polygons.begin(); ptr < polygons.end(); ptr++){
        as2_msgs::msg::Geofence geofence;

        for (std::vector<std::array<float,2>>::iterator ptr2 = (*ptr).begin(); ptr2 < (*ptr).end(); ptr2++){
          geometry_msgs::msg::Point32 point;
          point.x = (*ptr2)[0];
          point.y = (*ptr2)[1];
          geofence.polygon.points.push_back(point);
        }
        
        int index = ptr - polygons.begin();
        std::vector<int>::iterator ptr_alert = alerts.begin() + index;
        std::vector<int>::iterator ptr_id = ids.begin() + index;
        geofence.alert = (*ptr_alert);
        geofence.id = (*ptr_id);
        geofence_list.push_back(geofence);
      }
      response->geofences = geofence_list;
      response->success = true;
  }
}

void Geofencing::cleanupNode(){
    // TODO: CLeanup Node
};

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn Geofencing::on_configure(const rclcpp_lifecycle::State &_state)
{
  // Set subscriptions, publishers, services, actions, etc. here.
  this->get_parameter("config_file", config_path_);
  setupNode();

  return CallbackReturn::SUCCESS;
};

CallbackReturn Geofencing::on_activate(const rclcpp_lifecycle::State &_state)
{
  // Set parameters?

  return CallbackReturn::SUCCESS;
};

CallbackReturn Geofencing::on_deactivate(const rclcpp_lifecycle::State &_state)
{
  // Clean up subscriptions, publishers, services, actions, etc. here.
  cleanupNode();

  return CallbackReturn::SUCCESS;
};

CallbackReturn Geofencing::on_shutdown(const rclcpp_lifecycle::State &_state)
{
  // Clean other resources here.

  return CallbackReturn::SUCCESS;
};
