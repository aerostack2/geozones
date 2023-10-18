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

Geofencing::Geofencing() : as2::Node("geofencing") {
  this->declare_parameter<std::string>("config_file",
                                       "geofencing/geofences.json");
}

void Geofencing::run() {

  point_ = {self_x_, self_y_};

  if (!start_run_) {
    return;
  }

  if (geofences.size() == 0) {
    return;
  }

  else {
    checkGeofences();
  }
}
// TODO: METHODS
void Geofencing::setupNode() {

  RCLCPP_INFO(this->get_logger(), "Geofence in mode: %s", mode_.c_str());

  // if (mode_ == "gps") {

  //   gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
  //       this->generate_global_name(as2_names::topics::sensor_measurements::gps),
  //       as2_names::topics::sensor_measurements::qos,
  //       std::bind(&Geofencing::gpsCallback, this, std::placeholders::_1));
  // }

  // else if (mode_ == "cartesian") {

  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      this->generate_global_name(as2_names::topics::self_localization::pose),
      as2_names::topics::self_localization::qos,
      std::bind(&Geofencing::poseCallback, this, std::placeholders::_1));

  set_geofence_srv_ = this->create_service<geofencing::srv::SetGeofence>(
      this->generate_local_name("set_geofence"),
      std::bind(&Geofencing::setGeofenceCb, this, std::placeholders::_1,
                std::placeholders::_2));

  get_geofence_srv_ = this->create_service<geofencing::srv::GetGeofence>(
      this->generate_local_name("get_geofences"),
      std::bind(&Geofencing::getGeofenceCb, this, std::placeholders::_1,
                std::placeholders::_2));

  alert_pub_ = this->create_publisher<as2_msgs::msg::AlertEvent>(
      this->generate_global_name("alert_event"), 1);

  // std::string full_path_ =
  // ament_index_cpp::get_package_share_directory("geofencing"); config_path_ =
  // "config/geofences.json";
  loadGeofences(config_path_);
}

void Geofencing::loadGeofences(const std::string path) {
  std::ifstream fJson(path);
  std::stringstream buffer;
  buffer << fJson.rdbuf();
  auto json = nlohmann::json::parse(buffer.str());

  for (auto json_geofence : json["geofences"]) {
    geofence geofence_to_load;
    if (!checkValidity(std::size(json_geofence["polygon"]), json_geofence["id"],
                       json_geofence["type"])) {
      continue;
    } else {
      geofence_to_load.data_type = json_geofence["data_type"];
      std::vector<std::array<double, 2>> polygon;
      if (geofence_to_load.data_type == "gps") {
        if (!origin_set_) {
          setupGPS();
          origin_set_ = true;
        }
        for (std::array<double, 2> point : json_geofence["polygon"]) {
          double z;
          gps_handler->LatLon2Local(point[0], point[1], 0.0, point[0], point[1],
                                    z);
          polygon.push_back(point);
        }
      } else {
        for (std::array<double, 2> point : json_geofence["polygon"]) {
          polygon.push_back(point);
        }
      }

      geofence_to_load.id = json_geofence["id"];
      geofence_to_load.alert = json_geofence["alert"];

      geofence_to_load.type = json_geofence["type"];
      geofence_to_load.in = false;
      geofence_to_load.polygon = polygon;
      geofences.push_back(geofence_to_load);

      RCLCPP_INFO(this->get_logger(),
                  "Geofence Succesfully loaded from JSON file");
    }
  }
}
// CALLBACKS //

void Geofencing::poseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr _msg) {
  self_x_ = _msg->pose.position.x;
  self_y_ = _msg->pose.position.y;
  self_z_ = _msg->pose.position.z;

  start_run_ = true;
}

void Geofencing::setGeofenceCb(
    const std::shared_ptr<geofencing::srv::SetGeofence::Request> request,
    std::shared_ptr<geofencing::srv::SetGeofence::Response> response) {
  if (!checkValidity(std::size(request->geofence.polygon.points),
                     request->geofence.id, request->geofence.type)) {
    response->success = false;
  }

  else {
    geofence geofence_to_load;
    std::vector<std::array<double, 2>> polygon;
    for (int i = 0; i < std::size(request->geofence.polygon.points); i++) {
      std::array<double, 2> point{request->geofence.polygon.points[i].x,
                                  request->geofence.polygon.points[i].y};
      polygon.push_back(point);
    }
    geofence_to_load.id = request->geofence.id;
    geofence_to_load.alert = request->geofence.alert;
    geofence_to_load.type = request->geofence.type;
    geofence_to_load.data_type = request->geofence.data_type;
    geofence_to_load.z_up = request->geofence.z_up;
    geofence_to_load.z_down = request->geofence.z_down;
    geofence_to_load.in = false;
    geofence_to_load.polygon = polygon;
    geofences.push_back(geofence_to_load);

    RCLCPP_INFO(this->get_logger(), "Geofence added.");
    response->success = true;
  }
}

void Geofencing::getGeofenceCb(
    const std::shared_ptr<geofencing::srv::GetGeofence::Request> request,
    std::shared_ptr<geofencing::srv::GetGeofence::Response> response) {
  if (geofences.size() == 0) {
    RCLCPP_WARN(this->get_logger(), "No geofence has been set yet.");
    response->success = false;
  } else {
    std::vector<geofencing::msg::Geofence> geofence_list;
    for (std::vector<geofence>::iterator ptr = geofences.begin();
         ptr < geofences.end(); ptr++) {
      geofencing::msg::Geofence geofence;
      for (std::vector<std::array<double, 2>>::iterator ptr2 =
               ptr->polygon.begin();
           ptr2 < ptr->polygon.end(); ptr2++) {
        geometry_msgs::msg::Point32 point;
        double x, y, z;
        if (ptr->data_type == "gps") {
          gps_handler->Local2LatLon((*ptr2)[0], (*ptr2)[1], 0.0, x, y, z);
          point.x, point.y, point.z = x, y, z;
        } else {
          point.x = (*ptr2)[0];
          point.y = (*ptr2)[1];
        }

        geofence.polygon.points.push_back(point);
      }
      geofence.z_up = ptr->z_up;
      geofence.z_down = ptr->z_down;
      geofence.type = ptr->type;
      geofence.data_type = ptr->data_type;
      geofence.alert = ptr->alert;
      geofence.id = ptr->id;
      geofence_list.push_back(geofence);
    }
    response->geofences = geofence_list;
    response->success = true;
  }
}

void Geofencing::checkGeofences() {

  as2_msgs::msg::AlertEvent alert;
  for (std::vector<geofence>::iterator ptr = geofences.begin();
       ptr < geofences.end(); ptr++) {

    // auto [point, polygon] = translatePolygonWithPoint(ptr->polygon, point_);

    if (!Geofence::isIn<double>(ptr->polygon, point_) ||
        self_z_ < ptr->z_down || self_z_ > ptr->z_up) {

      if (ptr->type == "geocage") {
        alert.alert = ptr->alert;
        alert_pub_->publish(alert);
      }
      if (ptr->in == true) {
        ptr->in = false;
        RCLCPP_INFO(this->get_logger(), "Exited geofence: %s",
                    std::to_string(ptr->id).c_str());
      }
    } else {
      if (ptr->type == "geofence") {
        alert.alert = ptr->alert;
        alert_pub_->publish(alert);
      }
      if (ptr->in == false) {
        ptr->in = true;
        RCLCPP_INFO(this->get_logger(), "Entered Geofence: %s",
                    std::to_string(ptr->id).c_str());
      }
    }
  }
}

bool Geofencing::findGeofenceId(int id) {

  for (std::vector<geofence>::iterator ptr = geofences.begin();
       ptr < geofences.end(); ptr++) {
    if (id == ptr->id) {
      return false;
    }
  }
  return true;
}

bool Geofencing::checkValidity(int size, int id, std::string type) {

  if (!findGeofenceId(id)) {
    RCLCPP_WARN(this->get_logger(), "Id already exist.");
    return false;
  }

  if (type != "geofence" && type != "geocage") {
    RCLCPP_WARN(this->get_logger(),
                "Invalid type. Allowed values: 'geofence', 'geocage'.");
    return false;
  }

  if (size < 3) {
    RCLCPP_WARN(this->get_logger(),
                "Invalid Geofence. Polygon must contain at least 3 points");
    return false;
  }

  return true;
}

void Geofencing::setupGPS() {
  get_origin_srv_ = this->create_client<as2_msgs::srv::GetOrigin>(
      as2_names::services::gps::get_origin); // Should be same origin for every
                                             // drone ?

  while (!get_origin_srv_->wait_for_service(std::chrono::seconds(3))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Interrupted while waiting for the service. Exiting.");
      break;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }

  auto request = std::make_shared<as2_msgs::srv::GetOrigin::Request>();

  request->structure_needs_at_least_one_member = 0;

  bool success = false; // TO-DO: Improve this
  // Wait for the result.
  while (!success) {
    auto result = get_origin_srv_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           result, std::chrono::seconds(1)) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      // ;

    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service get origin");
      return;
    }
    auto result_obj = *result.get();
    success = result_obj.success;
    if (success) {
      origin_ =
          std::make_unique<geographic_msgs::msg::GeoPoint>(result_obj.origin);
      RCLCPP_INFO(this->get_logger(), "Origin in: lat: %f, lon %f, alt: %f",
                  origin_->latitude, origin_->longitude, origin_->altitude);
      gps_handler = std::make_unique<as2::gps::GpsHandler>(
          origin_->latitude, origin_->longitude, origin_->altitude);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Get origin request not successful, trying again...");
    }
  }
}

void Geofencing::cleanupNode(){
    // TODO: CLeanup Node
};

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn Geofencing::on_configure(const rclcpp_lifecycle::State &_state) {
  // Set subscriptions, publishers, services, actions, etc. here.
  this->get_parameter("config_file", config_path_);

  setupNode();

  return CallbackReturn::SUCCESS;
};

CallbackReturn Geofencing::on_activate(const rclcpp_lifecycle::State &_state) {
  // Set parameters?

  return CallbackReturn::SUCCESS;
};

CallbackReturn
Geofencing::on_deactivate(const rclcpp_lifecycle::State &_state) {
  // Clean up subscriptions, publishers, services, actions, etc. here.
  cleanupNode();

  return CallbackReturn::SUCCESS;
};

CallbackReturn Geofencing::on_shutdown(const rclcpp_lifecycle::State &_state) {
  // Clean other resources here.

  return CallbackReturn::SUCCESS;
};