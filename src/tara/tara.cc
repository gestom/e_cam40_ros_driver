#include "tara.h"

#include <iostream>
#include <sys/ioctl.h>

namespace tara {
// Constructor
TaraCamera::TaraCamera(void) {}

// Destructor
TaraCamera::~TaraCamera(void) { _CameraDevice.release(); }

// Inits the camera
int TaraCamera::cameraInit(int deviceID, cv::Size ImageSize) {

  /* open the device */
  _CameraDevice.open(deviceID);

  if (!_CameraDevice.isOpened()) {
#ifdef DEBUG
    printf("Device opening failed\n");
#endif
    return 101;
  }

  /* set resolution */
  _CameraDevice.set(CV_CAP_PROP_FRAME_WIDTH, ImageSize.width);
  _CameraDevice.set(CV_CAP_PROP_FRAME_HEIGHT, ImageSize.height);

  /* setcolor format */
  _CameraDevice.set(CV_CAP_PROP_FOURCC, CV_FOURCC('Y', '1', '6', ' '));

  /* set up the interleaved frame container */
  InterleavedFrame.create(ImageSize.height, ImageSize.width, CV_8UC2);

  return 0;
}

// Grabs left and right image
int TaraCamera::grabFrame(cv::Mat *LeftImage, cv::Mat *RightImage) {

  /* read one frame */
  _CameraDevice.read(InputFrame10bit);

  /* Check for the valid image */
  if (InputFrame10bit.empty()) {
#ifdef DEBUG
    printf("Empty frame");
#endif
    return 1;
  }

  /* split the interleaved data to left and right */
  InterleavedFrame.data = InputFrame10bit.data;

  std::vector<cv::Mat> StereoFrames;
  split(InterleavedFrame, StereoFrames);

  /* Copy the Frames */
  *LeftImage = StereoFrames[0].clone();
  *RightImage = StereoFrames[1].clone();

  return 0;
}

// set exposure
int TaraCamera::setExposure(int exposure) { return 0; }
}
