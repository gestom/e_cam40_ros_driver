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

#include <ros/ros.h>
#include "tara_camera_driver.h"


#include "std_msgs/String.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <iostream>
#include <iomanip>

using namespace e_cam40_ros_driver;
using namespace std;

void encode_brightness_exposure(cv::Mat& image, int brightness, int exposure);

CameraDriver::CameraDriver(const std::string& device, ros::NodeHandle nh, ros::NodeHandle nhp)
  : nh_( nh ), nhp_( nhp )
  , tara_cam_( device ), it_( nh )
  , cinfo_manager_left_( ros::NodeHandle(nhp, "left") )
  , cinfo_manager_right_( ros::NodeHandle(nhp, "right") )
  , next_seq_( 0 )
{
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

  nhp.param<std::string>("img_prefix",img_name,"image");
}

void CameraDriver::run()
{
	cv::Mat left_image(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC1);
	cv::Mat right_image(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC1);

	float sum = 0; 
	float lastSum = 10; 
	int exposure = 10;
	int exposureArray[255];
	int minGray = 20;
	int maxGray = 255-minGray;
	for (int i = minGray;i<maxGray;i++) exposureArray[i] = 0;
	int increment = 1;
	char filename[100];
	vector<string> filenames;
	const char *prefix = img_name.c_str();
	int iter=0;

	for (int brightness = 1;brightness<=7;brightness++){
		tara_cam_.setBrightness(brightness);
		/*let the camera settle*/
		for (int j=0;j<5;j++) {
            tara_cam_.grabNextFrame(left_image, right_image);
        }

		/* set up variables */
		increment = 1;
		sum = 0;
		lastSum = 10;
		exposure=10;
		for (int i = minGray;i<maxGray;i++) exposureArray[i] = 0;

		/*initial walkthrough with adaptive exposure step (increment)*/
		for (int i = 0;i<2000 && (sum < maxGray || lastSum < maxGray || exposure < 255);i++)
		{
			if(!tara_cam_.setExposure(exposure)){
				ROS_INFO("FALSE: Exposure setting finished in initial walkthrough");
				break;
			}
			/*let the camera settle*/
			for (int j=0;j<5;j++) {
				tara_cam_.grabNextFrame(left_image, right_image);	
			}
			/*calculate image brightness*/
			lastSum = sum;
			sum = cv::sum(left_image).val[0]/left_image.rows/left_image.cols;
            /*save images*/
			if (exposureArray[(int)(sum+0.5)]==0){
				sprintf(filename,"%s_%01i_%07i_%03i_%04i_%04i_r.png",prefix,brightness,exposure,(int)(round(sum)),i,iter);
				encode_brightness_exposure(right_image,brightness,exposure);
                cv::imwrite(filename,right_image);
				filenames.push_back(filename);
				sprintf(filename,"%s_%01i_%07i_%03i_%04i_%04i_l.png",prefix,brightness,exposure,(int)(round(sum)),i,iter);
				encode_brightness_exposure(left_image,brightness,exposure);
				cv::imwrite(filename,left_image);
				filenames.push_back(filename);
				exposureArray[(int)(sum+0.5)] = exposure;
				iter++;
            }
			if (lastSum < sum)  increment = (int)fmax(fmin((round(sum+1))/sum*exposure-exposure,exposure/5.0),1);   //adaptive step for exposure increment
			ROS_INFO("Exposure %i and brightness %i setting generated image with average brightness %.3f. Adaptive step suggested %i",exposure,brightness,sum,increment);
			exposure+=increment;
			ros::spinOnce();
		}

		/*detect missing average brightnesses and try to generate them */
		for (int i = minGray;i<maxGray;i++){
			if (exposureArray[i] ==0)
			{
				int j = i+1;
				while (j<maxGray && exposureArray[j] ==0) j++;
				float minExposure = exposureArray[i-1];
				float maxExposure = exposureArray[j];
					//linear interpolation
					exposure = (maxExposure-minExposure)/(j-i+1)+minExposure;
					ROS_INFO("Gap detected in %i, trying to fill with %i by linear interpolation from %i:%.0f  %i:%.0f",i,exposure,i-1,minExposure,j,maxExposure);
					tara_cam_.setExposure(exposure);
					for (int l=0;l<8;l++)  tara_cam_.grabNextFrame(left_image, right_image);

                    /*calculate image brightness*/
					sum = cv::sum(left_image).val[0]/left_image.rows/left_image.cols;
					int index = (int)(round(sum));
					if (exposureArray[index]==0){
						//save images
						sprintf(filename,"%s_%01i_%07i_%03i_%04i_%04i_r.png",prefix,brightness,exposure,(int)(round(sum)),i,iter);
						encode_brightness_exposure(right_image,brightness,exposure);
						cv::imwrite(filename,right_image);
						filenames.push_back(filename);
						sprintf(filename,"%s_%01i_%07i_%03i_%04i_%04i_l.png",prefix,brightness,exposure,(int)(round(sum)),i,iter);
						encode_brightness_exposure(left_image,brightness,exposure);
						cv::imwrite(filename,left_image);
						filenames.push_back(filename);
						exposureArray[index]=exposure; 
						ROS_INFO("Tried %i, got %.3f, filled gap at %i.",exposure,sum,index);
						ROS_INFO("Image %i, exp %i, bri %i, sum %i",iter,exposure,brightness,(int)(sum+0.5));
						iter++;
					}else{
						ROS_INFO("Tried %i, got %.3f, gap %i not filled.",exposure,sum,index);
					}
					ros::spinOnce();
			}
		}
	}

    ROS_INFO("Publishing images.");
	sort(filenames.begin(),filenames.end());
	for (auto it = filenames.begin(); it!=filenames.end(); it++)
	{
		ROS_INFO("Loading and publishing %s",it->c_str());
		left_image = cv::imread(*it);
		it++;	
		ROS_INFO("Loading and publishing %s",it->c_str());
		right_image = cv::imread(*it);
		int imExposure = atoi(&(it->c_str()[strlen(prefix)+1]));
		int imBrightness = atoi(&(it->c_str()[strlen(prefix)+5]));
		ROS_INFO("Publishing image with exposure %s and brightness %i %i",&it->c_str()[strlen(prefix)+5],imExposure,imBrightness);


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


		next_seq_++;

		ros::spinOnce();
	}
}

/*
 * 	Encode brightness setting (1-7) to the first pixel
 * 	and exposure setting (10-1000000) converted to hex (A-F4240) to the next five pixels.
 */
void encode_brightness_exposure(cv::Mat& image, int brightness, int exposure)
{
	image.at<uint8_t>(0, 0) = (uint8_t) brightness;

	// exposure is converted to hex string and filled with '0' with the max size of 5, which is the hex length of the max of exposure
	int hex_length = 5;
	std::stringstream stream;
	stream << std::setfill ('0') << std::setw(hex_length) << std::hex << exposure;
	std::string hex_exposure( stream.str() );

	for (int m = 0; m < hex_length; ++m) {
		image.at<uint8_t>(0, m+1) = (uint8_t) hex_exposure[m];
	}
}


int main (int argc, char** argv)
{
  ros::init(argc, argv, "tara_camera");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  std::string device;
  nhp.param<std::string>("device", device, "/dev/video0");

  e_cam40_ros_driver::CameraDriver driver(device, nh, nhp);

  driver.run();

  return EXIT_SUCCESS;
}
