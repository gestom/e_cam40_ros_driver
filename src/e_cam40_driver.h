/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (C) 2016 Thomas Fischer
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
*   * Neither the name of the author nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
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
#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>
#include <camera_info_manager/camera_info_manager.h>

#include <opencv2/opencv.hpp>

#include "CCamera.h"

#include "std_msgs/Int32.h"

#include <dynamic_reconfigure/server.h>
#include <e_cam40_ros_driver/cam40Config.h>

namespace e_cam40_ros_driver
{

class CameraDriver
{
  public:

    CameraDriver(const std::string& device, ros::NodeHandle nh, ros::NodeHandle nhp);

    /**
     * @brief Run the data acquisition polling loop. Blocking!
     */
    void run();

  private:

    ros::NodeHandle nh_, nhp_;

    CCamera camera;
    //Cameraproperty::Cameraproperty cam_property;
    //Videostreaming::Videostreaming video_cam;

    image_transport::ImageTransport it_;
    image_transport::CameraPublisher cam_pub_ir_, cam_pub_right_;

    camera_info_manager::CameraInfoManager cinfo_manager_ir_, cinfo_manager_right_;

    size_t next_seq_;
    std::string frame_id_;

    dynamic_reconfigure::Server<e_cam40_ros_driver::cam40Config> dyn_srv_;
    int exposure;
    int brightness;
    bool autoExposure;
    float exposureGain;
    int targetBrightness;

    // image prefix name
    std::string img_name;

    // dynamic reconfiguration of parameters
    void configCallback(e_cam40_ros_driver::cam40Config &config, uint32_t level);

};

};
