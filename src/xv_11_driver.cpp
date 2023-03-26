

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Eric Perko, Chad Rockey
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Case Western Reserve University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*********************************************************************
 *
 * This xv-11 lidar driver started from a popular XV-11 or 'Neato' Lidar driver.
 *
 * https://github.com/ros2/demos/blob/master/dummy_robot/dummy_sensors/src/dummy_laser.cpp
 * And xv11 code:  https://github.com/rohbotics/xv_11_laser_driver
 * Also refer to conversion from ROS to ROS2  at https://index.ros.org/doc/ros2/Contributing/Migration-Guide/
 *
 * Launch:  ros2 run xv_11_driver xv_11_driver
 *********************************************************************/

#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"

// Make files need to change to support project include in a clean way
#include "../include/xv_11_driver/xv11_laser.h"

#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/u_int16.hpp>

#define DEG2RAD (M_PI / 180.0)

// default xv11 dev name.  Suggest you use symbolic link for an easier to ricognize name on your system
// example:  sudo ln -s /dev/yourTtyDevName /dev/ttyXV11
// Here I assume you have setup symbolic link to your actual serial port tty driver to the lidar
constexpr int kBaudRate = 115200;        // Serial baud rate
constexpr int kFirmWareVersion = 2;      // XV-11 firmware rev. 1 is oldest
constexpr auto kPort = "/dev/ttyXV11";   // Serial device driver name (sym link to real dev)
constexpr auto kFrameId = "neato_laser"; // frame_id in LaserScan messages
constexpr bool kPublishRpmFlag = true;   // whether to publish rpms 

class XV11LaserNode : public rclcpp::Node
{
public:
  XV11LaserNode() : Node("xv11_laser")
  {
    this->declare_parameter("port", kPort);
    this->declare_parameter("baud_rate", kBaudRate);
    this->declare_parameter("frame_id", kFrameId);
    this->declare_parameter("firmware_version", kFirmWareVersion);
    this->declare_parameter("publish_rmp", kPublishRpmFlag);
  }

  void dump()
  {
    std::cout << port_param() << "\n";
    std::cout << baud_rate_param() << "\n";
    std::cout << frame_id_param() << "\n";
    std::cout << firmware_number_param() << "\n";
    std::cout << publish_rpm_param() << "\n";
  }

  bool publish_rpm_param()
  {
    if (this->get_param("publish_rmp", m_publish_rpm))
    {
      return m_publish_rpm;
    }
    return kPublishRpmFlag;
  }


  std::string port_param()
  {
    if (this->get_param("port", m_port))
    {
      return m_port;
    }
    return kPort;
  }

  int baud_rate_param()
  {
    if (this->get_param("baud_rate", m_baud_rate))
    {
      return m_baud_rate;
    }
    return kBaudRate;
  }

  std::string frame_id_param()
  {
    if (this->get_param("frame_id", m_frame_id))
    {
      return m_frame_id;
    }
    return kFrameId;
  }

  int firmware_number_param()
  {
    if (get_param("firmware_version", m_firmware_number))
    {
      return m_firmware_number;
    }
    return kFirmWareVersion;
  }

private:
  std::string m_port{};
  int m_baud_rate{};
  std::string m_frame_id{};
  int m_firmware_number{};
  bool m_publish_rpm{kPublishRpmFlag};

  template <typename T>
  bool get_param(std::string param_name_str, T &param)
  {
    if (this->get_parameter(param_name_str, param))
    {
      return true;
    }
    return false;
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<XV11LaserNode>();

  auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
  auto motor_pub = node->create_publisher<std_msgs::msg::UInt16>("rpms",1000);
  
  node->dump();

  // std_msgs::msg::UInt16 rpms;
  boost::asio::io_service io;
  try
  {
    xv_11_driver::XV11Laser laser(node->port_param(), node->baud_rate_param(), node->firmware_number_param(), io);

    while (rclcpp::ok())
    {
      sensor_msgs::msg::LaserScan *scan;
      scan = new sensor_msgs::msg::LaserScan;

      scan->header.frame_id = node->frame_id_param();
      scan->header.stamp = rclcpp::Clock().now(); //  ROS was  Time::now();
      laser.poll(scan);
      laser_pub->publish(*scan);
      
      if(node->publish_rpm_param())
      {
        std_msgs::msg::UInt16 rpms;
        rpms.data=laser.rpms;
        motor_pub->publish(rpms);
      }
    }
    laser.close();
    rclcpp::shutdown();
  }
  catch (const std::runtime_error &re)
  {
    std::stringstream ss;
    ss << "Runtime error: " << re.what();
    ss << ". Possible error: no permision to read from the specified port." << std::endl;
    RCLCPP_ERROR(node->get_logger(), ss.str().c_str());
  }
  catch (const std::exception &ex)
  {
    std::stringstream ss;
    ss << "Error occurred: " << ex.what() << std::endl;
    RCLCPP_ERROR(node->get_logger(), ss.str().c_str());
  }
  catch (...)
  {
    RCLCPP_ERROR(node->get_logger(), "Error instantiating laser object. Check correct port and baud rate!");
    return -1;
  }

    return 0;
}