#pragma once

#include <rclcpp/rclcpp.hpp>
#include <genicam/msg/camera_device.hpp>
#include <genicam/msg/camera_device_array.hpp>
#include <genicam/srv/create_camera.hpp>


#include <ArenaApi.h>


class CameraArray : public rclcpp::Node {
    rclcpp::TimerBase::SharedPtr publish_camera_discovered_timer;
    genicam::msg::CameraDeviceArray::SharedPtr camera_discovered;
    rclcpp::Publisher<genicam::msg::CameraDeviceArray>::SharedPtr array_pub_;
    rclcpp::Service<genicam::srv::CreateCamera>::SharedPtr service;
    public:
        CameraArray(genicam::msg::CameraDeviceArray::SharedPtr camera_discovered);
    private:
        void publish_camera_discovered();
        void service_trigger(const std::shared_ptr<genicam::srv::CreateCamera::Request> request, std::shared_ptr<genicam::srv::CreateCamera::Response> response);
};

class CameraInfo : public rclcpp::Node {
    rclcpp::TimerBase::SharedPtr update_camera_discovered_timer;
    genicam::msg::CameraDeviceArray::SharedPtr camera_discovered;
    public:
        CameraInfo(genicam::msg::CameraDeviceArray::SharedPtr camera_discovered);
    private:
        void update_camera_discovered();
};