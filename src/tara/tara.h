#ifndef _TARA_H
#define _TARA_H

#include "opencv2/opencv.hpp"

#define DEFAULT_WIDTH 752
#define DEFAULT_HEIGHT 480
#define DEFAULT_EXPOSURE 15000

namespace tara {

class TaraCamera {
private:
  // camera device
  cv::VideoCapture _CameraDevice;

  // variables
  cv::Mat InputFrame10bit, InterleavedFrame;
  
public:
  // Constructor
  TaraCamera(void);

  // Destructor
  ~TaraCamera(void);

  // Inits the camera
  int cameraInit(int deviceID, cv::Size ImageSize);

  // Grabs left and right image
  int grabFrame(cv::Mat *LeftImage, cv::Mat *RightImage);

  // set exposure
  int setExposure(int exposure);

};
}
#endif // _TARA_H
