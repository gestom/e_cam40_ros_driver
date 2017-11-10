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
#include "tara_imu_driver.h"
#include <sensor_msgs/Imu.h>

#define RADIANS( x ) ( x * M_PI / 180.0 )

using namespace tara_camera_driver;

ImuDriver::ImuDriver(ros::NodeHandle nh, ros::NodeHandle nhp)
: nh_( nh ), nhp_( nhp )
{
  imu_pub_ = nh.advertise<sensor_msgs::Imu> ("/imu/data_raw", 1);
}

void ImuDriver::run()
{
  // TODO why doesn't this work?
  //~ ImuDevice::Callback_t callback = std::bind(&ImuDriver::imu_callback, this, _1, _2);

  ImuDevice::Callback_t callback = [=](float dt, const IMUDATAOUTPUT_TypeDef& measurement) {
    this->imu_callback(dt, measurement);
  };

  imu_.run( callback );
}

void ImuDriver::imu_callback(float dt, const IMUDATAOUTPUT_TypeDef& measurement)
{
  //~ ROS_INFO_STREAM(measurement.gyroX << " " << measurement.gyroY << " " << measurement.gyroZ);

  sensor_msgs::Imu imu_msg;
  imu_msg.header.frame_id = "imu";
  imu_msg.header.stamp = ros::Time::now();

  // fill orientation
  /*{
    imu_msg.orientation.x = quaternion.x();
    imu_msg.orientation.y = quaternion.y();
    imu_msg.orientation.z = quaternion.z();
    imu_msg.orientation.w = quaternion.w();

    imu_msg.orientation_covariance.fill( 0. );
    imu_msg.orientation_covariance[ 0*3 + 0 ] = RADIANS(1.);
    imu_msg.orientation_covariance[ 1*3 + 1 ] = RADIANS(1.);
    imu_msg.orientation_covariance[ 2*3 + 2 ] = RADIANS(9.);
  }*/

  // fill angular velocity
  {
    // imu ouput comes as dps, we need to transform it back to rps.
    imu_msg.angular_velocity.x = RADIANS( measurement.gyroX );
    imu_msg.angular_velocity.y = RADIANS( measurement.gyroY );
    imu_msg.angular_velocity.z = RADIANS( measurement.gyroZ );

    // values taken from ethzasl_xsens_driver, who took it from datashhet.
    imu_msg.angular_velocity_covariance.fill( 0. );
    imu_msg.angular_velocity_covariance[ 0*3 + 0 ] = RADIANS(0.025);
    imu_msg.angular_velocity_covariance[ 1*3 + 1 ] = RADIANS(0.025);
    imu_msg.angular_velocity_covariance[ 2*3 + 2 ] = RADIANS(0.025);
  }

  // fill linear acceleration
  {
    imu_msg.linear_acceleration.x = measurement.accX;
    imu_msg.linear_acceleration.y = measurement.accY;
    imu_msg.linear_acceleration.z = measurement.accZ;

    // values taken from ethzasl_xsens_driver, who took it from datashhet.
    imu_msg.linear_acceleration_covariance.fill( 0. );
    imu_msg.linear_acceleration_covariance[ 0*3 + 0 ] = 0.0004;
    imu_msg.linear_acceleration_covariance[ 1*3 + 1 ] = 0.0004;
    imu_msg.linear_acceleration_covariance[ 2*3 + 2 ] = 0.0004;
  }

  // Publish ROS message
  imu_pub_.publish( imu_msg );

  ros::spinOnce();
}
