/*
 * File name: CCamera.cpp
 * Date:      2010
 * Author:    Tom Krajnik 
 */
#include "CCamera.h"
#include <stdio.h>
#include <stdlib.h>


extern "C" {
//#include "utils.h"
//#include "color.h"
}

//-----------------------------------------------------------------------------
CCamera::CCamera()
{
	cameraType = CT_UNKNOWN;
	aviBuffer1  = NULL;
	aviBuffer2 = NULL;
	autoexposure = true;
	gain = exposition = 0;
	width = height = 0;
	exposition = 0;
	fileNum = 0;
	videoIn = (struct vdIn*) calloc(1,sizeof(struct vdIn));
	fileNameList = NULL;
	numImages = 0;
	return;
}

CCamera::~CCamera()
{
	for (int i = 0;i<numImages;i++) free(fileNameList[i]);
	free(fileNameList);
	close_v4l2(videoIn);
	free(videoIn);
	free(aviBuffer1);
	free(aviBuffer2);
	videoIn = NULL;
}

int CCamera::init(const char *deviceName,int *wi,int *he,bool saveI)
{
	save = saveI;
	format = V4L2_PIX_FMT_Y16;

	if (save) videoIn->toggleAvi = 1; else videoIn->toggleAvi = 0;
	const float fps = 30.0;
	const int grabemethod = 1; 

	time_t timeNow;
	time(&timeNow);
		int ret = init_videoIn(videoIn,(char *)deviceName, wi,he,fps,format,grabemethod);
		if (ret < 0) {
			fprintf(stderr,"Cannot init camera device %s.\n",deviceName);
			return -1;
		} else {
			fprintf(stderr,"Camera %s initialized.\n",deviceName);
		}

		// gets camera parameters
		gain = getDeviceGain();
		exposition = getDeviceExposition();
		brightness = getDeviceBrightness();

		int min,max,def;
		min=max=def=0;
		width = *wi;
		height = *he;

		getParamInfo(V4L2_CID_GAIN,min,max,def);
		fprintf(stderr,"Gain parameter: min=%d max=%d default=%d\n",min,max,def);
		getParamInfo(V4L2_CID_BRIGHTNESS,min,max,def);
		fprintf(stderr,"Brightness parameter: min=%d max=%d default=%d\n",min,max,def);
		getParamInfo(V4L2_CID_EXPOSURE_ABSOLUTE,min,max,def);
		fprintf(stderr,"Exposure parameter: min=%d max=%d default=%d\n",min,max,def);
		getParamInfo(V4L2_CID_EXPOSURE_AUTO,min,max,def);
		fprintf(stderr,"Exposure auto parameter: min=%d max=%d default=%d\n",min,max,def);

		getParamInfo(V4L2_CID_WHITE_BALANCE_TEMPERATURE,min,max,def);
		fprintf(stderr,"White temperature balance: min=%d max=%d default=%d\n",min,max,def);
		getParamInfo(V4L2_CID_AUTO_WHITE_BALANCE,min,max,def);
		fprintf(stderr,"White auto balance: min=%d max=%d default=%d\n",min,max,def);

		setDeviceAutoExposure(0);
		setDeviceWhiteBalanceAuto(1); 
		fprintf(stderr,"Camera gain = %d\n",gain);
		fprintf(stderr,"Camera exposition = %d\n",exposition);
		fprintf(stderr,"Camera brightness = %d\n",brightness);
	return -1;
}

int CCamera::loadConfig(const char* filename)
{
	FILE* file = fopen(filename,"r");
	if (file == NULL) return -1;
	int exp,cntr,gain,brt;
	fscanf(file,"%i %i %i %i\n",&exp,&cntr,&gain,&brt);	
	setDeviceExposition(exp); 
	setDeviceContrast(cntr); 
	setDeviceGain(gain);
	setDeviceBrightness(brt);
        fclose(file);
	return 0;	
}

int CCamera::saveConfig(const char* filename)
{
	if (cameraType == CT_WEBCAM){
		FILE* file = fopen(filename,"w");
		if (file == NULL) return -1;
		int exp,cntr,gain,brt;
		exp = getDeviceExposition(); 
		cntr = getDeviceContrast(); 
		gain = getDeviceGain();
		brt = getDeviceBrightness();
		fprintf(file,"%i %i %i %i\n",exp,cntr,gain,brt);	
		fclose(file);
	}
	return 0;	
}

int CCamera::renewImage(cv::Mat& ir, cv::Mat& rgb)
{
	int ret = uvcGrab(videoIn);
	if (ret < 0) {
		fprintf(stderr,"Cannot grab a frame from a camera!\n"); 
		return ret;
	}
	unsigned char *buffer = videoIn->framebuffer;
	int blue, red, green,pos,irv;
    int maxVal = 255;
	for( int j=0;j< videoIn->height-1;j++)
	{

		if (j % 2 == 0) {
			pos = 2*j*videoIn->width;
		}else{
			pos = 2*(j+1)*videoIn->width;
		}
		red = buffer[pos] + 256 * buffer[pos + 1];
		if (red>maxVal)red = maxVal;
		for( int i=0;i< videoIn->width;i++) {
			if (j % 2 == 0) {
				if (i % 2 == 0) {
					pos = 2 * (i + j * videoIn->width);
					green = buffer[pos + 2] + 256 * buffer[pos + 3];
					blue = buffer[pos + videoIn->width * 2 + 2] + 256 * buffer[pos + videoIn->width * 2 + 3];
					if (green>maxVal)green = maxVal;
					if (blue>maxVal)blue = maxVal;
					irv = buffer[pos+videoIn->width * 2] + 256 * buffer[pos + videoIn->width * 2+ 1];
					if (irv > maxVal) irv = maxVal;
				} else {
					red = buffer[pos + 2] + 256 * buffer[pos + 3];
					if (red>maxVal)red = maxVal;
				}
			} else {
				if (i % 2 == 0) {
					pos = 2 * (i + (j + 1) * videoIn->width);
					green = buffer[pos + 2] + 256 * buffer[pos + 3];
					blue = buffer[pos - videoIn->width * 2 + 2] + 256 * buffer[pos - videoIn->width * 2 + 3];
					if (green > maxVal)green = maxVal;
					if (blue > maxVal)blue = maxVal;
					irv = buffer[pos-videoIn->width * 2] + 256 * buffer[pos - videoIn->width * 2+ 1];
					if (irv > maxVal) irv = maxVal;
				}else {
					red = buffer[pos + 2] + 256 * buffer[pos + 3];
					if (red > maxVal)red = maxVal;
				}
			}


			cv::Vec3b color(red,green,blue);
			rgb.at<cv::Vec3b>(cv::Point(i,j)) = color;
			ir.at<uint8_t>(j, i) = irv;
		}	
	}
	return -1;
}


int CCamera::getGain()
{
	gain = getDeviceGain();
	return gain;
}

int CCamera::getContrast()
{
	contrast = getDeviceGain();
	return contrast;
}

int CCamera::getExposition()
{
	exposition = getDeviceExposition();
	return exposition;
}

int CCamera::getBrightness()
{
	brightness = getDeviceBrightness();
	return brightness;
}

void CCamera::setGain(int value)
{
	gain = value;
	setDeviceGain(value);
}

void CCamera::setContrast(int value)
{
	contrast = value;
	setDeviceContrast(contrast);
}

void CCamera::changeBrightness(int brt)
{
	setBrightness(brightness+brt);
}

void CCamera::changeExposition(int exp)
{
	setExposition(exposition+exp);
}

void CCamera::changeGain(int exp)
{
	setGain(gain+exp);
}

void CCamera::changeContrast(int expc)
{
	setContrast(contrast+expc);
}

void CCamera::setExposition(int exp)
{
	exposition = exp;
	setDeviceExposition(exp);	
	fprintf(stdout,"Exposure is set to %i\n",v4l2GetControl(videoIn,V4L2_CID_EXPOSURE_ABSOLUTE));
}

void CCamera::setBrightness(int val)
{
	brightness = val;
	setDeviceBrightness(val);
}

int CCamera::getDeviceGain() {
	return v4l2GetControl(videoIn,V4L2_CID_GAIN);
}

int CCamera::getDeviceContrast() {
	return v4l2GetControl(videoIn,V4L2_CID_CONTRAST);
}

int CCamera::getDeviceBrightness() {
	fprintf(stdout,"Brightness is set to %i\n",v4l2GetControl(videoIn,V4L2_CID_BRIGHTNESS));
	return v4l2GetControl(videoIn,V4L2_CID_BRIGHTNESS);
}

int CCamera::getDeviceExposition() {
	int rawValue = v4l2GetControl(videoIn,V4L2_CID_EXPOSURE_ABSOLUTE);
	int value = rawValue;
	//sometimes, the cameras have the value in powers of 2, if this is the case, uncomment the following line
	//value = (int)log2(rawValue);
	fprintf(stdout,"Exposition is set to %i-%i\n",value,rawValue);
	return value;
}

int CCamera::setDeviceExposition(int value) {
	if (autoexposure){
		autoexposure = false;
		setDeviceAutoExposure(1);
	}
	//sometimes, the cameras have the value in powers of 2, if this is the case, uncomment the following line
	//value = exp2(value);
	return v4l2SetControl(videoIn,V4L2_CID_EXPOSURE_ABSOLUTE,value);
}

int CCamera::getDeviceSharpness() {
	return v4l2GetControl(videoIn,V4L2_CID_BASE+27);
}

int CCamera::setDeviceSharpness(const int value) {
	return v4l2SetControl(videoIn,V4L2_CID_BASE+27,value);
}


int CCamera::setDeviceGain(const int value) {
	fprintf(stdout,"Gain is set to %i\n",v4l2GetControl(videoIn,V4L2_CID_GAIN));
	return v4l2SetControl(videoIn,V4L2_CID_GAIN,value);
}

int CCamera::setDeviceContrast(const int value) {
	fprintf(stdout,"Contrast is set to %i\n",v4l2GetControl(videoIn,V4L2_CID_CONTRAST));
	return v4l2SetControl(videoIn,V4L2_CID_CONTRAST,value);
}

int CCamera::setDeviceBrightness(const int value) {
	fprintf(stdout,"Brightness is set to %i\n",v4l2GetControl(videoIn,V4L2_CID_BRIGHTNESS));
	return v4l2SetControl(videoIn,V4L2_CID_BRIGHTNESS,value);
}

int CCamera::setDeviceWhiteTemperature(const int value) {
	return v4l2SetControl(videoIn,V4L2_CID_WHITE_BALANCE_TEMPERATURE,value);
}

int CCamera::getDeviceWhiteTemperature() {
	fprintf(stderr,"Camera white temperature = %d\n",v4l2GetControl(videoIn,V4L2_CID_WHITE_BALANCE_TEMPERATURE));
	return v4l2GetControl(videoIn,V4L2_CID_WHITE_BALANCE_TEMPERATURE);
}

int CCamera::getParamInfo(const int paramType, int &min, int &max, int &default_val) 
{
	struct v4l2_queryctrl queryctrl;
	if (isv4l2Control(videoIn,paramType,&queryctrl) < 0) {
		fprintf(stderr,"Error getting camera info.\n");
		return -1;
	}
	min = queryctrl.minimum;
	max = queryctrl.maximum;
	default_val = queryctrl.default_value;
	return 0;
}

int CCamera::setDeviceAutoExposure(const int val) {
	struct v4l2_control control;
	control.id = V4L2_CID_EXPOSURE_AUTO;
	control.value =(int)val;
	int ret;
	if ((ret = ioctl(videoIn->fd, VIDIOC_S_CTRL, &control)) < 0) {
		printf("Set Auto Exposure on error\n");
	} else {
		printf("Auto Exposure set to %d\n", control.value);
	}
	return 0;
}

int CCamera::setDeviceWhiteBalanceAuto(const int val) {
	struct v4l2_control control;
	control.id = V4L2_CID_AUTO_WHITE_BALANCE;
	control.value =(int)val;
	int ret;
	if ((ret = ioctl(videoIn->fd, VIDIOC_S_CTRL, &control)) < 0) {
		printf("Set Auto white balance on error\n");
	} else {
		printf("Auto white balance set to %d\n", control.value);
	}
	return 0;
}

