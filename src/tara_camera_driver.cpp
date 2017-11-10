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

#include "tara_camera_driver.h"


#include "std_msgs/String.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace e_cam40_ros_driver;

CameraDriver::CameraDriver(const std::string& device, ros::NodeHandle nh, ros::NodeHandle nhp)
  : nh_( nh ), nhp_( nhp )
 , tara_cam_( device ), it_( nh )
//	,
  , cinfo_manager_left_( ros::NodeHandle(nhp, "left") )
  , cinfo_manager_right_( ros::NodeHandle(nhp, "right") )
  , next_seq_( 0 )
  , exposure(1000)
  , brightness(1)
  , autoExposure(true)
  , exposureGain(1.0)
  , targetBrightness(128)
{
    /* server for dynamic reconfiguration of camera parameters */
	dynamic_reconfigure::Server<e_cam40_ros_driver::cam40Config>::CallbackType cb = boost::bind(&CameraDriver::configCallback, this, _1, _2);
	dyn_srv_.setCallback(cb);

    /* publishers of camera images */
	cam_pub_left_ = it_.advertiseCamera("left/image_raw", 1, false);
	cam_pub_right_ = it_.advertiseCamera("right/image_raw", 1, false);

    /* load and set parameters */
	nhp.param<std::string>("frame_id", frame_id_, "tara_camera");

	std::string left_camera_info_url, right_camera_info_url;
	if (nhp.hasParam("left/camera_info_url"))
		nhp.param<std::string>("left/camera_info_url", left_camera_info_url, "");

	if (nhp.hasParam("right/camera_info_url"))
		nhp.param<std::string>("right/camera_info_url", right_camera_info_url, "");

    cinfo_manager_left_.loadCameraInfo( left_camera_info_url );
    cinfo_manager_right_.loadCameraInfo( right_camera_info_url );

    std::string left_camera_name, right_camera_name;
    nhp.param<std::string>("left/camera_name", left_camera_name, "tara_left");
    nhp.param<std::string>("right/camera_name", right_camera_name, "tara_right");

	cinfo_manager_left_.setCameraName(left_camera_name);
	cinfo_manager_right_.setCameraName(right_camera_name);

	ros::NodeHandle pnh("~");
	pnh.param("exposure", exposure, exposure);
	pnh.param("brightness", brightness, brightness);
}

/*receive parameters from dynamic reconfiguration and set them*/
void CameraDriver::configCallback(e_cam40_ros_driver::cam40Config &config, uint32_t level)
{
  autoExposure = config.autoExposure;
  if (autoExposure == false) exposure = config.exposure;
  targetBrightness = config.targetBrightness;
  brightness = config.brightness;
  exposureGain = config.exposureGain;

  tara_cam_.setExposure(exposure);
  tara_cam_.setBrightness(brightness);

  ROS_INFO("reconfigure: exp[%i], bri[%i],  des[%i]", exposure, brightness,targetBrightness);
}

void CameraDriver::run()
{
	cv::Mat left_image(FRAME_HEIGHT/2, FRAME_WIDTH/2, CV_8UC1);
	cv::Mat right_image(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC1);

	while( ros::ok() )
	{
		tara_cam_.grabNextFrame(left_image, right_image);
		//video_cam.updateFrame(left_image);

		ros::Time timestamp = ros::Time::now();

		//Encoding information in the image
		std_msgs::Header header;
		header.seq = next_seq_;
		header.stamp = timestamp;
		header.frame_id = frame_id_;

		// Convert OpenCV image to ROS image msg.
		cv_bridge::CvImage bridge_image_left(header, sensor_msgs::image_encodings::MONO8, left_image);
		cv_bridge::CvImage bridge_image_right(header, sensor_msgs::image_encodings::MONO8, right_image);

		sensor_msgs::CameraInfo::Ptr camera_info_left(new sensor_msgs::CameraInfo(cinfo_manager_left_.getCameraInfo()));
		sensor_msgs::CameraInfo::Ptr camera_info_right(new sensor_msgs::CameraInfo(cinfo_manager_right_.getCameraInfo()));

		camera_info_left->header = header;
		camera_info_right->header = header;

		cam_pub_left_.publish(bridge_image_left.toImageMsg(), camera_info_left);
		cam_pub_right_.publish(bridge_image_right.toImageMsg(), camera_info_right);

		//automatic exposure control - trying to target a given mean brightness of the captured images
		if (autoExposure){
			if (next_seq_++%5 == 0){
				cv::Mat tmp = left_image(cv::Rect(0,0,left_image.cols,left_image.rows/2));
				float sum = cv::sum(tmp).val[0]/tmp.rows/tmp.cols;
				ROS_INFO("Image brightness %.3f %i",sum,exposure);
				exposure += exposureGain*(targetBrightness/sum*exposure-exposure);   //adaptive step for exposure setting
				//tara_cam_.setExposure(exposure);
			}
		}
		ros::spinOnce();
	}
}

