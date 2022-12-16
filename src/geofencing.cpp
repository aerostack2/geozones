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
  this->declare_parameter<std::string>("config_file", "geofencing/geofences.json");
  this->declare_parameter<std::string>("mode", "gps");
}

void Geofencing::run()
{
  if (mode_ == "gps"){
    point_ = {self_latitude_, self_longitude_};
  }

  else if (mode_ == "cartesian"){
    point_ = {self_x_, self_y_};
  }

  else{
    RCLCPP_ERROR(this->get_logger(), "Invalid mode: %s" , mode_.c_str());
    return;
  }

  if (!start_run_){
    return;
  }

  if (geofences.size() == 0){
    return;
  }

  else {
    checkGeofences();
  }
}
// TODO: METHODS
void Geofencing::setupNode()
{

  RCLCPP_INFO(this->get_logger(), "Geofence in mode: %s" , mode_.c_str());

  if (mode_ == "gps"){
    
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        this->generate_global_name(as2_names::topics::sensor_measurements::gps),
        as2_names::topics::sensor_measurements::qos,
        std::bind(&Geofencing::gpsCallback, this, std::placeholders::_1));
  }

  else if (mode_ == "cartesian"){

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        this->generate_global_name(as2_names::topics::self_localization::pose),
        as2_names::topics::self_localization::qos,
        std::bind(&Geofencing::poseCallback, this, std::placeholders::_1));  
  }

  set_geofence_srv_ = this->create_service<as2_msgs::srv::SetGeofence>(
      this->generate_local_name("set_geofence"),
      std::bind(&Geofencing::setGeofenceCb, this, std::placeholders::_1, std::placeholders::_2));

  get_geofence_srv_ = this->create_service<as2_msgs::srv::GetGeofence>(
      this->generate_local_name("get_geofences"),
      std::bind(&Geofencing::getGeofenceCb, this, std::placeholders::_1, std::placeholders::_2));

  alert_pub_ = this->create_publisher<as2_msgs::msg::AlertEvent>(
      this->generate_global_name("alert_event"), 1);

  //std::string full_path_ = ament_index_cpp::get_package_share_directory("geofencing");
  //config_path_ = "config/geofences.json";
  loadGeofences(config_path_);
}

void Geofencing::loadGeofences(const std::string path){
  std::ifstream fJson(path);
  std::stringstream buffer;
  buffer << fJson.rdbuf();
  auto json = nlohmann::json::parse(buffer.str());
  geofence geofence_to_load;
  for (auto json_geofence : json["geofences"])
  {
    if (!checkValidity(std::size(json_geofence["polygon"]), json_geofence["id"], json_geofence["type"] )){
      return;
    }
    else{
      std::vector<std::array<float,2>> polygon;
      for (std::array<float,2> point : json_geofence["polygon"])
      {
          polygon.push_back(point);
      }
      geofence_to_load.id = json_geofence["id"];
      geofence_to_load.alert = json_geofence["alert"];
      geofence_to_load.in = false;
      geofence_to_load.polygon = polygon;
      geofences.push_back(geofence_to_load);

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

void Geofencing::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr _msg)
{
  self_x_ = _msg->pose.position.x;
  self_y_ = _msg->pose.position.y;

  start_run_ = true;
}

void Geofencing::setGeofenceCb(const std::shared_ptr<as2_msgs::srv::SetGeofence::Request> request, 
                                std::shared_ptr<as2_msgs::srv::SetGeofence::Response> response) {
  if (!checkValidity(std::size(request->geofence.polygon.points), request->geofence.id, request->geofence.type)){
    response->success = false;
  }
  
  else{
      geofence geofence_to_load;
      std::vector<std::array<float,2>> polygon;
      for (int i = 0; i < std::size(request->geofence.polygon.points); i++){
        std::array<float,2> point{request->geofence.polygon.points[i].x, request->geofence.polygon.points[i].y};
        polygon.push_back(point);
      }
      geofence_to_load.id = request->geofence.id;
      geofence_to_load.alert = request->geofence.alert;
      geofence_to_load.type = request->geofence.type;
      geofence_to_load.in = false;
      geofence_to_load.polygon = polygon;
      geofences.push_back(geofence_to_load);

      RCLCPP_INFO(this->get_logger(), "Geofence added.");
      response->success = true;
  }
}

void Geofencing::getGeofenceCb(const std::shared_ptr<as2_msgs::srv::GetGeofence::Request> request, 
                                std::shared_ptr<as2_msgs::srv::GetGeofence::Response> response) {
  if (geofences.size() == 0){
      RCLCPP_WARN(this->get_logger(), "No geofence has been set yet.");
      response->success = false;
  }
  else{
      std::vector<as2_msgs::msg::Geofence> geofence_list;
      for (std::vector<geofence>::iterator ptr = geofences.begin(); ptr < geofences.end(); ptr++){
        as2_msgs::msg::Geofence geofence;
        for (std::vector<std::array<float,2>>::iterator ptr2 = ptr->polygon.begin(); ptr2 < ptr->polygon.end(); ptr2++){
          geometry_msgs::msg::Point32 point;
          point.x = (*ptr2)[0];
          point.y = (*ptr2)[1];
          geofence.polygon.points.push_back(point);
        }
        
        geofence.alert = ptr->alert;
        geofence.id = ptr->id;
        geofence_list.push_back(geofence);
      }
      response->geofences = geofence_list;
      response->success = true;
  }
}

std::tuple<std::array<float,2>, std::vector<std::array<float,2>>> Geofencing::translatePolygonWithPoint(const std::vector<std::array<float,2>> polygon, const std::array<float,2> point){

  float most_negative_number_x = 0.0;
  float most_negative_number_y = 0.0;

  for (std::vector<std::array<float,2>>::const_iterator ptr = polygon.begin(); ptr < polygon.end(); ptr++){
    if ((*ptr)[0] < most_negative_number_x){
      most_negative_number_x = (*ptr)[0];
    }
    if ((*ptr)[1] < most_negative_number_y){
      most_negative_number_y = (*ptr)[1];
    }
  }

  if (point[0] < most_negative_number_x){
    most_negative_number_x = point[0];
  }

  if (point[1] < most_negative_number_y){
    most_negative_number_y = point[1];
  }

  std::vector<std::array<float,2>> ret_polygon = polygon;
  std::array<float,2> ret_point = point;

  for (std::vector<std::array<float,2>>::iterator ptr = ret_polygon.begin(); ptr < ret_polygon.end(); ptr++){

    (*ptr)[0] += fabsf(most_negative_number_x);
    (*ptr)[1] += fabsf(most_negative_number_y);
  }

  ret_point[0] += fabsf(most_negative_number_x);
  ret_point[1] += fabsf(most_negative_number_y);

  return {ret_point, ret_polygon};
}

void Geofencing::checkGeofences(){

  as2_msgs::msg::AlertEvent alert;
  for (std::vector<geofence>::iterator ptr = geofences.begin(); ptr < geofences.end(); ptr++){

    auto [point, polygon] = translatePolygonWithPoint(ptr->polygon, point_);

    if (!Geofence::isIn<float>(polygon, point)){
      alert.alert = ptr->alert;
      //alert.id = (*ptr_id);
      alert_pub_->publish(alert);
      if (ptr->in == true){
        ptr->in = false;
        RCLCPP_INFO(this->get_logger(), "Exited geofence: %s" , std::to_string(ptr->id).c_str());
      }
    }
    else{
      if (ptr->in==false){
        ptr->in = true;
        RCLCPP_INFO(this->get_logger(), "Entered Geofence: %s" , std::to_string(ptr->id).c_str());
      }
    }
  }
}

bool Geofencing::findGeofenceId(int id){

  for (std::vector<geofence>::iterator ptr = geofences.begin(); ptr < geofences.end(); ptr++){
    if (id == ptr->id){ 
      return false;
    }
  }
  return true;
}

bool Geofencing::checkValidity(int size, int id, std::string type){

  if (!findGeofenceId(id)){
    RCLCPP_WARN(this->get_logger(), "Id already exist.");
    return false;
  }

  if (type != "inclusion" && type != "exclusion"){
    RCLCPP_WARN(this->get_logger(), "Invalid type. Allowed values: 'exclusion', 'inclusion'.");
    return false;
  }

  if (size < 3){
    RCLCPP_WARN(this->get_logger(), "Invalid Geofence. Polygon must contain at least 3 points");
    return false;
  }

  return true;
}

void Geofencing::cleanupNode(){
    // TODO: CLeanup Node
};

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn Geofencing::on_configure(const rclcpp_lifecycle::State &_state)
{
  // Set subscriptions, publishers, services, actions, etc. here.
  this->get_parameter("config_file", config_path_);
  this->get_parameter("mode", mode_);

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