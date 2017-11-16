# ROS driver for E-CONs IR camera

 Start the driver with the launch file camera.launch:

    roslaunch e_cam40_ros_driver camera.launch

 Change camera settings (e.g. brightness, exposure, automatic exposure) through dynamic reconfigure:

    rosrun rqt_reconfigure rqt_reconfigure

 Published topics:
    - rgb and infrared image:
        /ir/image_raw
        /rgb/image_raw

