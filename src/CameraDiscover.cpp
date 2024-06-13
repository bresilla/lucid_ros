#include <rclcpp/rclcpp.hpp>
#include <genicam/msg/camera_device.hpp>
#include <genicam/msg/camera_device_array.hpp>

#include <ArenaApi.h>
#include <genicam/CameraSets.hpp>
#include <genicam/CameraDiscover.hpp>


inline uint64_t convert_mac(std::string mac) {
    mac.erase(std::remove(mac.begin(), mac.end(), ':'), mac.end());
    return strtoul(mac.c_str(), NULL, 16);
}

CameraArray::CameraArray(genicam::msg::CameraDeviceArray::SharedPtr camera_discovered) : Node("camera_array"), camera_discovered (camera_discovered) {
    publish_camera_discovered_timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&CameraArray::publish_camera_discovered, this));
    array_pub_ = this->create_publisher<genicam::msg::CameraDeviceArray>("/cam_discovered", 10);
    service = this->create_service<genicam::srv::CreateCamera>("cam_discovered", std::bind(&CameraArray::service_trigger, this, std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "Camer info service started and publishing camera info");
}

void CameraArray::service_trigger(const std::shared_ptr<genicam::srv::CreateCamera::Request> request, std::shared_ptr<genicam::srv::CreateCamera::Response> response) {
    if (camera_discovered->cameras.empty()) {
        response->success = false;
        response->message = "No cameras found";
        return;
    }
    std::string camera_name = request->camera_name;
    std::string mac_address = request->mac_address;
    genicam::msg::CameraDevice camera_device = genicam::msg::CameraDevice();
    RCLCPP_INFO(this->get_logger(), "Camera info requested for %s", camera_name.c_str());
    if (!mac_address.empty()) {
        uint64_t hex_mac = convert_mac(mac_address);
        for (const auto &camera : camera_discovered->cameras) {
            if (convert_mac(camera.mac) == hex_mac) {
                response->camera_device = camera;
                response->success = true;
                break;
            }
        }
    } else if (!camera_name.empty()) {
        for (const auto &camera : camera_discovered->cameras) {
            if (camera.id == camera_name) {
                response->camera_device = camera;
                response->success = true;
                break;
            }
        }
    } else {
        response->success = false;
        response->message = "Missing camera name or mac address";
    }
}

void CameraArray::publish_camera_discovered() {
    if (camera_discovered->cameras.empty()) {
        return;
    }
    array_pub_->publish(*camera_discovered);
    std::vector<std::string> ids;
    for (const auto &camera : camera_discovered->cameras) {
        ids.push_back(camera.id);
    }
}


CameraInfo::CameraInfo(genicam::msg::CameraDeviceArray::SharedPtr camera_discovered) : Node("camera_info"), camera_discovered (camera_discovered) {
    update_camera_discovered_timer = this->create_wall_timer(std::chrono::minutes(1), std::bind(&CameraInfo::update_camera_discovered, this));
    update_camera_discovered();
}

void CameraInfo::update_camera_discovered() {
    Arena::ISystem* pSystem = nullptr;
    std::vector<std::pair<Arena::IDevice*, uint64_t>> vDevices = std::vector<std::pair<Arena::IDevice*, uint64_t>>();
    genicam::msg::CameraDeviceArray camera_array = genicam::msg::CameraDeviceArray();
    RCLCPP_INFO(this->get_logger(), "Updating camera info");

    try {
        pSystem = Arena::OpenSystem();
        pSystem->UpdateDevices(1000);
        std::vector<Arena::DeviceInfo> deviceInfos = pSystem->GetDevices();
        for (auto& deviceInfo : deviceInfos){
            genicam::msg::CameraDevice camera_device = genicam::msg::CameraDevice();
            camera_device.model = deviceInfo.ModelName();
            camera_device.vendor = deviceInfo.VendorName();
            camera_device.serial = deviceInfo.SerialNumber();
            camera_device.ip = deviceInfo.IpAddressStr();
            camera_device.subnetmask = deviceInfo.SubnetMaskStr();
            camera_device.defaultgateway = deviceInfo.DefaultGatewayStr();
            camera_device.mac = deviceInfo.MacAddressStr();
            camera_device.name = deviceInfo.UserDefinedName();
            camera_device.dhcp = deviceInfo.IsDHCPConfigurationEnabled();
            camera_device.presistentip = deviceInfo.IsPersistentIpConfigurationEnabled();
            camera_device.lla = deviceInfo.IsLLAConfigurationEnabled();
            uint64_t hex_mac = convert_mac(deviceInfo.MacAddressStr().c_str());
            if (camset::by_mac.find(hex_mac) != camset::by_mac.end()) {
                camera_device.id = camset::by_mac.at(hex_mac).name;
                camera_device.name = camset::by_mac.at(hex_mac).name;
            }
            camera_array.cameras.push_back(camera_device);   
        }
        Arena::CloseSystem(pSystem);
        *camera_discovered = camera_array;

    } catch(const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error: %s", e.what());
    } catch (GenICam::GenericException& ge) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error: %s", ge.what());
    }
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    std::cout << "... GETTING CAMERAS ..." << std::endl;

    rclcpp::executors::MultiThreadedExecutor executor;
    genicam::msg::CameraDeviceArray::SharedPtr camera_discovered = std::make_shared<genicam::msg::CameraDeviceArray>();

    auto camera_info_node = std::make_shared<CameraInfo>(camera_discovered);
    auto camera_array_node = std::make_shared<CameraArray>(camera_discovered);

    executor.add_node(camera_info_node);
    executor.add_node(camera_array_node);

    try {
        executor.spin();
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "An error occurred: %s", e.what());
    }

    executor.remove_node(camera_info_node);
    executor.remove_node(camera_array_node);

    rclcpp::shutdown();
    return 0;
}