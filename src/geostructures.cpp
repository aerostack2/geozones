/*!*******************************************************************************************
 *  \file       geostructures.cpp
 *  \brief      Geostructures for AeroStack2
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

#include "geostructures.hpp"

Geostructures::Geostructures()
: as2::Node("geostructures")
{
  this->declare_parameter<std::string>(
    "config_file",
    "geostructures/geofences.json");
}

void Geostructures::run()
{
  point_ = {self_x_, self_y_};

  if (!start_run_) {
    return;
  }

  if (geostructures_.size() == 0) {
    return;
  } else {
    checkGeostructures();
  }
}

void Geostructures::setupNode()
{
  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    this->generate_global_name(as2_names::topics::self_localization::pose),
    as2_names::topics::self_localization::qos,
    std::bind(&Geostructures::poseCallback, this, std::placeholders::_1));

  set_geostructure_srv_ =
    this->create_service<geostructures::srv::SetGeostructure>(
    this->generate_local_name("set_geostructure"),
    std::bind(
      &Geostructures::setGeoStructureCb, this, std::placeholders::_1,
      std::placeholders::_2));

  get_geostructure_srv_ =
    this->create_service<geostructures::srv::GetGeostructure>(
    this->generate_local_name("get_geostructure"),
    std::bind(
      &Geostructures::getGeoStructureCb, this, std::placeholders::_1,
      std::placeholders::_2));

  alert_pub_ = this->create_publisher<as2_msgs::msg::AlertEvent>(
    this->generate_global_name("alert_event"), 1);

  loadGeostructures(config_path_);
}

void Geostructures::loadGeostructures(const std::string path)
{
  std::ifstream fJson(path);
  std::stringstream buffer;
  buffer << fJson.rdbuf();
  auto json = nlohmann::json::parse(buffer.str());

  for (auto json_geostructure : json["geostructures"]) {
    geoStructure geostructure_to_load;
    if (!checkValidity(
        std::size(json_geostructure["polygon"]),
        json_geostructure["id"], json_geostructure["type"],
        json_geostructure["data_type"]))
    {
      continue;
    } else {
      geostructure_to_load.data_type = json_geostructure["data_type"];
      std::vector<std::array<double, 2>> polygon;
      if (geostructure_to_load.data_type == "gps") {
        if (!origin_set_) {
          setupGPS();
          origin_set_ = true;
        }
        for (std::array<double, 2> point : json_geostructure["polygon"]) {
          double z;
          gps_handler->LatLon2Local(
            point[0], point[1], 0.0, point[0], point[1],
            z);
          polygon.push_back(point);
        }
      } else {
        for (std::array<double, 2> point : json_geostructure["polygon"]) {
          polygon.push_back(point);
        }
      }

      geostructure_to_load.id = json_geostructure["id"];
      geostructure_to_load.alert = json_geostructure["alert"];

      geostructure_to_load.type = json_geostructure["type"];
      geostructure_to_load.data_type = json_geostructure["data_type"];
      geostructure_to_load.z_up =
        json.contains("z_up") ? static_cast<float>(json_geostructure["z_up"]) :
        100000.0;
      geostructure_to_load.z_down =
        json.contains("z_down") ?
        static_cast<float>(json_geostructure["z_down"]) :
        -100000.0;
      geostructure_to_load.in = false;
      geostructure_to_load.polygon = polygon;
      geostructures_.push_back(geostructure_to_load);

      RCLCPP_INFO(
        this->get_logger(),
        "Geostructure Succesfully loaded from JSON file");
    }
  }
}
// CALLBACKS //

void Geostructures::poseCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr _msg)
{
  self_x_ = _msg->pose.position.x;
  self_y_ = _msg->pose.position.y;
  self_z_ = _msg->pose.position.z;

  start_run_ = true;
}

void Geostructures::setGeoStructureCb(
  const std::shared_ptr<geostructures::srv::SetGeostructure::Request> request,
  std::shared_ptr<geostructures::srv::SetGeostructure::Response> response)
{
  if (!checkValidity(
      std::size(request->geostructure.polygon.points),
      request->geostructure.id, request->geostructure.type,
      request->geostructure.data_type))
  {
    response->success = false;
  } else {
    geoStructure geostructure_to_load;
    std::vector<std::array<double, 2>> polygon;
    for (int i = 0; i < std::size(request->geostructure.polygon.points); i++) {
      std::array<double, 2> point{request->geostructure.polygon.points[i].x,
        request->geostructure.polygon.points[i].y};
      polygon.push_back(point);
    }
    geostructure_to_load.id = request->geostructure.id;
    geostructure_to_load.alert = request->geostructure.alert;
    geostructure_to_load.type = request->geostructure.type;
    geostructure_to_load.data_type = request->geostructure.data_type;
    geostructure_to_load.z_up = request->geostructure.z_up;
    geostructure_to_load.z_down = request->geostructure.z_down;
    geostructure_to_load.in = false;
    geostructure_to_load.polygon = polygon;
    geostructures_.push_back(geostructure_to_load);

    RCLCPP_INFO(this->get_logger(), "Geostructure added.");
    response->success = true;
  }
}

void Geostructures::getGeoStructureCb(
  const std::shared_ptr<geostructures::srv::GetGeostructure::Request> request,
  std::shared_ptr<geostructures::srv::GetGeostructure::Response> response)
{
  if (geostructures_.size() == 0) {
    RCLCPP_WARN(this->get_logger(), "No geostructure has been set yet.");
    response->success = false;
  } else {
    std::vector<geostructures::msg::Geostructure> geofence_list;
    for (std::vector<geoStructure>::iterator ptr = geostructures_.begin();
      ptr < geostructures_.end(); ptr++)
    {
      geostructures::msg::Geostructure geostructure;
      for (std::vector<std::array<double, 2>>::iterator ptr2 =
        ptr->polygon.begin();
        ptr2 < ptr->polygon.end(); ptr2++)
      {
        geometry_msgs::msg::Point32 point;
        double x, y, z;
        if (ptr->data_type == "gps") {
          gps_handler->Local2LatLon((*ptr2)[0], (*ptr2)[1], 0.0, x, y, z);
          point.x, point.y, point.z = x, y, z;
        } else {
          point.x = (*ptr2)[0];
          point.y = (*ptr2)[1];
        }

        geostructure.polygon.points.push_back(point);
      }
      geostructure.z_up = ptr->z_up != ptr->z_up;
      geostructure.z_down = ptr->z_down != ptr->z_down;
      geostructure.type = ptr->type;
      geostructure.data_type = ptr->data_type;
      geostructure.alert = ptr->alert;
      geostructure.id = ptr->id;
      geofence_list.push_back(geostructure);
    }
    response->geostructure_list = geofence_list;
    response->success = true;
  }
}

void Geostructures::checkGeostructures()
{

  as2_msgs::msg::AlertEvent alert;
  for (std::vector<geoStructure>::iterator ptr = geostructures_.begin();
    ptr < geostructures_.end(); ptr++)
  {
    // auto [point, polygon] = translatePolygonWithPoint(ptr->polygon, point_);

    if (!Pnpoly::isIn<double>(ptr->polygon, point_) ||
      self_z_ < ptr->z_down || self_z_ > ptr->z_up)
    {
      if (ptr->type == "geocage") {
        alert.alert = ptr->alert;
        alert_pub_->publish(alert);
      }
      if (ptr->in == true) {
        ptr->in = false;
        RCLCPP_INFO(
          this->get_logger(), "Exited area: %s",
          std::to_string(ptr->id).c_str());
      }
    } else {
      if (ptr->type == "geofence") {
        alert.alert = ptr->alert;
        alert_pub_->publish(alert);
      }
      if (ptr->in == false) {
        ptr->in = true;
        RCLCPP_INFO(
          this->get_logger(), "Entered area: %s",
          std::to_string(ptr->id).c_str());
      }
    }
  }
}

bool Geostructures::findGeostructureId(int id)
{
  for (std::vector<geoStructure>::iterator ptr = geostructures_.begin();
    ptr < geostructures_.end(); ptr++)
  {
    if (id == ptr->id) {
      return false;
    }
  }
  return true;
}

bool Geostructures::checkValidity(
  int size, int id, std::string type,
  std::string data_type)
{
  if (!findGeostructureId(id)) {
    RCLCPP_WARN(this->get_logger(), "Id already exist.");
    return false;
  }

  if (type != "geofence" && type != "geocage") {
    RCLCPP_WARN(
      this->get_logger(),
      "Invalid type: %s. Allowed values: 'geofence', 'geocage'.",
      type.c_str());
    return false;
  }

  if (data_type != "gps" && data_type != "cartesian") {
    RCLCPP_WARN(
      this->get_logger(),
      "Invalid data type: %s. Allowed values: 'gps', 'cartesian'.",
      type.c_str());
    return false;
  }

  if (size < 3) {
    RCLCPP_WARN(
      this->get_logger(),
      "Invalid Geofence. Polygon must contain at least 3 points");
    return false;
  }

  return true;
}

void Geostructures::setupGPS()
{
  get_origin_srv_ = this->create_client<as2_msgs::srv::GetOrigin>(
    as2_names::services::gps::get_origin);   // Should be same origin for every
                                             // drone ?
  while (!get_origin_srv_->wait_for_service(std::chrono::seconds(3))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        this->get_logger(),
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
    if (rclcpp::spin_until_future_complete(
        this->get_node_base_interface(),
        result, std::chrono::seconds(1)) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
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
      RCLCPP_INFO(
        this->get_logger(), "Origin in: lat: %f, lon %f, alt: %f",
        origin_->latitude, origin_->longitude, origin_->altitude);
      gps_handler = std::make_unique<as2::gps::GpsHandler>(
        origin_->latitude, origin_->longitude, origin_->altitude);
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "Get origin request not successful, trying again...");
    }
  }
}

void Geostructures::cleanupNode()
{
  // TODO: CLeanup Node
}

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn Geostructures::on_configure(const rclcpp_lifecycle::State & _state)
{
  // Set subscriptions, publishers, services, actions, etc. here.
  this->get_parameter("config_file", config_path_);

  setupNode();

  return CallbackReturn::SUCCESS;
}

CallbackReturn Geostructures::on_activate(const rclcpp_lifecycle::State & _state)
{
  // Set parameters?

  return CallbackReturn::SUCCESS;
}

CallbackReturn
Geostructures::on_deactivate(const rclcpp_lifecycle::State & _state)
{
  // Clean up subscriptions, publishers, services, actions, etc. here.
  cleanupNode();

  return CallbackReturn::SUCCESS;
}

CallbackReturn Geostructures::on_shutdown(const rclcpp_lifecycle::State & _state)
{
  // Clean other resources here.

  return CallbackReturn::SUCCESS;
}
