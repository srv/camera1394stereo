///////////////////////////////////////////////////////////////////////////
// Joan Pau Beltran
//  Modificat per donar suport a les cameres Bumblebee2 de Pointgrey.
//
// Copyright (C) 2009, 2010 Patrick Beeson
//  ROS port of the Player 1394 camera driver.
//
// Copyright (C) 2004 Nate Koenig, Andrew Howard
//  Player driver for IEEE 1394 digital camera capture
//
// Copyright (C) 2000-2003 Damien Douxchamps, Dan Dennedy
//  Bayer filtering from libdc1394
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of the
// License, or (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
//
///////////////////////////////////////////////////////////////////////////

// $Id: dev_camera1394.cpp 33358 2010-10-19 13:59:00Z joq $

#include <stdint.h>

/** @file

    @brief IEEE 1394 digital camera library interface implementation

 */

#include "yuv.h"
#include <sensor_msgs/image_encodings.h>
#include "dev_camera1394stereo.h"


#define NUM_DMA_BUFFERS 8   // Pointgrey programs use 8

const timespec NSLEEP_TIME = { 0, 10000 }; // (0s, 10ms) => max 100Hz


//! Macro for throwing an exception with a message
#define CAM_EXCEPT(except, msg)					\
  {								\
    char buf[100];						\
    snprintf(buf, 100, "[Camera1394Stereo::%s]: " msg, __FUNCTION__); \
    throw except(buf);						\
  }

//! Macro for throwing an exception with a message, passing args
#define CAM_EXCEPT_ARGS(except, msg, ...)				\
  {									\
    char buf[100];							\
    snprintf(buf, 100, "[Camera1394Stereo::%s]: " msg, __FUNCTION__, __VA_ARGS__); \
    throw except(buf);							\
  }


namespace camera1394stereo
{

////////////////////////////////////////////////////////////////////////////////
// Constructor
Camera1394Stereo::Camera1394Stereo():
  features_(NULL), camera_(NULL)
{}

Camera1394Stereo::~Camera1394Stereo() 
{
  SafeCleanup();
}


void Camera1394Stereo::findFrameRate(float fps)
{
  if (fps < 3.75)
    frameRate_ = DC1394_FRAMERATE_1_875;
  else if (fps < 7.5)
    frameRate_ = DC1394_FRAMERATE_3_75;
  else if (fps < 15)
    frameRate_ = DC1394_FRAMERATE_7_5;
  else if (fps < 30)
    frameRate_ = DC1394_FRAMERATE_15;
  else if (fps < 60)
    frameRate_ = DC1394_FRAMERATE_30;
  else if (fps < 120)
    frameRate_ = DC1394_FRAMERATE_60;
  else if (fps < 240)
    frameRate_ = DC1394_FRAMERATE_120;
  else
    frameRate_ = DC1394_FRAMERATE_240;
}

void Camera1394Stereo::findVideoMode(const char* mode)
{
  if (0 == strcmp(mode, "160x120_yuv444"))
    videoMode_ = DC1394_VIDEO_MODE_160x120_YUV444;
  else if (0 == strcmp(mode, "320x240_yuv422"))
    videoMode_ = DC1394_VIDEO_MODE_320x240_YUV422;
  else if (0 == strcmp(mode, "640x480_yuv411"))
    videoMode_ = DC1394_VIDEO_MODE_640x480_YUV411;
  else if (0 == strcmp(mode, "640x480_yuv422"))
    videoMode_ = DC1394_VIDEO_MODE_640x480_YUV422;
  else if (0 == strcmp(mode, "640x480_mono8"))
    videoMode_ = DC1394_VIDEO_MODE_640x480_MONO8;
  else if (0 == strcmp(mode, "640x480_mono16"))
    videoMode_ = DC1394_VIDEO_MODE_640x480_MONO16;
  else if (0 == strcmp(mode, "640x480_rgb8"))
    videoMode_ = DC1394_VIDEO_MODE_640x480_RGB8;
  else if (0 == strcmp(mode, "800x600_yuv422"))
    videoMode_ = DC1394_VIDEO_MODE_800x600_YUV422;
  else if (0 == strcmp(mode, "800x600_rgb8"))
    videoMode_ = DC1394_VIDEO_MODE_800x600_RGB8;
  else if (0 == strcmp(mode, "800x600_mono8"))
    videoMode_ = DC1394_VIDEO_MODE_800x600_MONO8;
  else if (0 == strcmp(mode, "1024x768_yuv422"))
    videoMode_ = DC1394_VIDEO_MODE_1024x768_YUV422;
  else if (0 == strcmp(mode, "1024x768_rgb8"))
    videoMode_ = DC1394_VIDEO_MODE_1024x768_RGB8;
  else if (0 == strcmp(mode, "1024x768_mono8"))
    videoMode_ = DC1394_VIDEO_MODE_1024x768_MONO8;
  else if (0 == strcmp(mode, "800x600_mono16"))
    videoMode_ = DC1394_VIDEO_MODE_800x600_MONO16;
  else if (0 == strcmp(mode, "1024x768_mono16"))
    videoMode_ = DC1394_VIDEO_MODE_1024x768_MONO16;
  else if (0 == strcmp(mode, "1280x960_yuv422"))
    videoMode_ = DC1394_VIDEO_MODE_1280x960_YUV422;
  else if (0 == strcmp(mode, "1280x960_rgb8"))
    videoMode_ = DC1394_VIDEO_MODE_1280x960_RGB8;
  else if (0 == strcmp(mode, "1280x960_mono8"))
    videoMode_ = DC1394_VIDEO_MODE_1280x960_MONO8;
  else if (0 == strcmp(mode, "1600x1200_yuv422"))
    videoMode_ = DC1394_VIDEO_MODE_1600x1200_YUV422;
  else if (0 == strcmp(mode, "1600x1200_rgb8"))
    videoMode_ = DC1394_VIDEO_MODE_1600x1200_RGB8;
  else if (0 == strcmp(mode, "1600x1200_mono8"))
    videoMode_ = DC1394_VIDEO_MODE_1600x1200_MONO8;
  else if (0 == strcmp(mode, "1280x960_mono16"))
    videoMode_ = DC1394_VIDEO_MODE_1280x960_MONO16;
  else if (0 == strcmp(mode, "1600x1200_mono16"))
    videoMode_ = DC1394_VIDEO_MODE_1600x1200_MONO16;
  else if (0 == strcmp(mode, "format7_0"))
    videoMode_ = DC1394_VIDEO_MODE_FORMAT7_0;
  else if (0 == strcmp(mode, "format7_3"))
    videoMode_ = DC1394_VIDEO_MODE_FORMAT7_3;
  else
    {
      // raise exception
      CAM_EXCEPT(camera1394stereo::Exception, "Unsupported video mode");
    }
}

void Camera1394Stereo::findFormat7ColorCoding(const char* coding)
{
  if (0 == strcmp(coding, "mono8"))
    colorCoding_ = DC1394_COLOR_CODING_MONO8;
  else if (0 == strcmp(coding, "yuv411"))
    colorCoding_ = DC1394_COLOR_CODING_YUV411;
  else if (0 == strcmp(coding, "yuv422"))
    colorCoding_ = DC1394_COLOR_CODING_YUV422;
  else if (0 == strcmp(coding, "yuv444"))
    colorCoding_ = DC1394_COLOR_CODING_YUV444;
  else if (0 == strcmp(coding, "rgb8"))
    colorCoding_ = DC1394_COLOR_CODING_RGB8;
  else if (0 == strcmp(coding, "mono16"))
    colorCoding_ = DC1394_COLOR_CODING_MONO16;
  else if (0 == strcmp(coding, "rgb16"))
    colorCoding_ = DC1394_COLOR_CODING_RGB16;
  else if (0 == strcmp(coding, "mono16s"))
    colorCoding_ = DC1394_COLOR_CODING_MONO16S;
  else if (0 == strcmp(coding, "rgb16s"))
    colorCoding_ = DC1394_COLOR_CODING_RGB16S;
  else if (0 == strcmp(coding, "raw8"))
    colorCoding_ = DC1394_COLOR_CODING_RAW8;
  else if (0 == strcmp(coding, "raw16"))
    colorCoding_ = DC1394_COLOR_CODING_RAW16;
  else
    {
      colorCoding_ = DC1394_COLOR_CODING_RAW8;
      ROS_ERROR("Unknown color coding [%s]. Defaulting to raw8.",
                coding);
    }
}

void Camera1394Stereo::findFormat7ROI(int left, int top, int width, int height)
{
  if ( left < 0 || top < 0 || width < 0 || height < 0 )
  {
    ROS_WARN("ROI for format 7 with origin at (%d,%d) and size %dx%d",
              left, top, width, height);
  }
  roi_left_ = left;
  roi_top_ = top;
  roi_width_ = width;
  roi_height_ = height;
}

void Camera1394Stereo::findStereoMethod(const char* method)
{
  stereoMethod_ = (dc1394stereo_method_t) DC1394_STEREO_METHOD_NUM;
  if (0 == strcmp(method, "Interlaced"))
    {
      stereoMethod_ = DC1394_STEREO_METHOD_INTERLACED;
    }
  else if (0 == strcmp(method, "Field"))
    {
      stereoMethod_ = DC1394_STEREO_METHOD_FIELD;
    }
    
  // Do stereo conversion in the driver node?
  DoStereoExtract_ = false;
  if (stereoMethod_ != DC1394_STEREO_METHOD_NUM)
      DoStereoExtract_ = true;        // decoding in driver

}

void Camera1394Stereo::findBayerFilter(const char* bayer, const char* method) 
{
  // determine Bayer color encoding pattern
  // (default is different from any color filter provided by DC1394)
  BayerPattern_ = (dc1394color_filter_t) DC1394_COLOR_FILTER_NUM;
  if (0 == strcmp(bayer, "bggr"))
    {
      BayerPattern_ = DC1394_COLOR_FILTER_BGGR;
    }
  else if (0 == strcmp(bayer, "grbg"))
    {
      BayerPattern_ = DC1394_COLOR_FILTER_GRBG;
    }
  else if (0 == strcmp(bayer, "rggb"))
    {
      BayerPattern_ = DC1394_COLOR_FILTER_RGGB;
    }
  else if (0 == strcmp(bayer, "gbrg"))
    {
      BayerPattern_ = DC1394_COLOR_FILTER_GBRG;
    }
  else if (0 != strcmp(bayer, ""))
    {
      ROS_ERROR("unknown bayer pattern [%s]", bayer);
    }

  // Do Bayer conversion in the driver node?
  DoBayerConversion_ = false;
  if (0 != strcmp(method, "")
      && BayerPattern_ != DC1394_COLOR_FILTER_NUM)
    {
      DoBayerConversion_ = true;        // decoding in driver

      // Set decoding method
      if (!strcmp(method, "Nearest"))
        BayerMethod_ = DC1394_BAYER_METHOD_NEAREST;
      else if (!strcmp(method, "DownSample"))
        BayerMethod_ = DC1394_BAYER_METHOD_DOWNSAMPLE;
      else if (!strcmp(method, "Simple"))
        BayerMethod_ = DC1394_BAYER_METHOD_SIMPLE;
#if 0 // removed from libdc1394 due to patent restrictions
      else if (!strcmp(method, "Edge"))
        BayerMethod_ = DC1394_BAYER_METHOD_EDGESENSE;
#endif
      else if (!strcmp(method, "Bilinear"))
        BayerMethod_ = DC1394_BAYER_METHOD_BILINEAR;
      else if (!strcmp(method, "HQ"))
        BayerMethod_ = DC1394_BAYER_METHOD_HQLINEAR;
      else if (!strcmp(method, "VNG"))
        BayerMethod_ = DC1394_BAYER_METHOD_VNG;
      else if (!strcmp(method, "AHD"))
        BayerMethod_ = DC1394_BAYER_METHOD_AHD;
      else
        {
          BayerMethod_ = DC1394_BAYER_METHOD_SIMPLE;
          ROS_ERROR("unknown Bayer method [%s].  Defaulting to Simple.",
                    method);
        }
    }
}

void Camera1394Stereo::findIsoSpeed(int iso_speed) 
{
  switch (iso_speed)
    {
    case 100:
      isoSpeed_ = DC1394_ISO_SPEED_100;
      break;
    case 200:
      isoSpeed_ = DC1394_ISO_SPEED_200;
      break;
    case 400:
      isoSpeed_ = DC1394_ISO_SPEED_400;
      break;
    case 800:
      isoSpeed_ = DC1394_ISO_SPEED_800;
      break;
    case 1600:
      isoSpeed_ = DC1394_ISO_SPEED_1600;
      break;
    case 3200:
      isoSpeed_ = DC1394_ISO_SPEED_3200;
      break;
    default:
      ROS_ERROR("Unsupported iso_speed. Defaulting to 400.");
      isoSpeed_ = DC1394_ISO_SPEED_400;
      break;
    }
}

/** Open the 1394 device */
int Camera1394Stereo::open(const char* guid,
        const char* video_mode,
        const char* color_coding, // only used with format 7 video modes
        int roi_left,       // only used with format 7 video modes
        int roi_top,        // only used with format 7 video modes
        int roi_width,      // only used with format 7 video modes
        int roi_height,     // only used with format 7 video modes
        float fps,
        int iso_speed,
        const char* bayer_pattern,
        const char* bayer_method,
        const char* stereo_method,
        bool reset_on_open )
{

  findFrameRate(fps);
  findVideoMode(video_mode);
  findFormat7ColorCoding(color_coding);
  findFormat7ROI(roi_left, roi_top, roi_width, roi_height);
  findBayerFilter(bayer_pattern, bayer_method);
  findStereoMethod(stereo_method);
  findIsoSpeed(iso_speed);

  // First we try to find a camera
  int err;
  dc1394_t *d;
  dc1394camera_list_t *list;

  // TODO: make error exit paths clean up resources properly
  d = dc1394_new ();
  if (d == NULL)
    {
      CAM_EXCEPT(camera1394stereo::Exception,
                 "Could not initialize dc1394_context.\n"
                 "Make sure /dev/raw1394 exists, you have access permission,\n"
                 "and libraw1394 development package is installed.");
    }

  err = dc1394_camera_enumerate(d, &list);
  if (err != DC1394_SUCCESS)
    {
      CAM_EXCEPT(camera1394stereo::Exception, "Could not get camera list");
      return -1;
    }
  
  if (list->num == 0)
    {
      CAM_EXCEPT(camera1394stereo::Exception, "No cameras found");
      return -1;
    }
  
  char* temp=(char*)malloc(1024*sizeof(char));
  for (unsigned i=0; i < list->num; i++)
    {
      // Create a camera
      camera_ = dc1394_camera_new (d, list->ids[i].guid);
      if (!camera_)
        ROS_WARN_STREAM("Failed to initialize camera with GUID "
                          << std::hex << list->ids[i].guid);
      else
        ROS_INFO_STREAM("Found camera with GUID "
                          << std::hex << list->ids[i].guid);

      uint32_t value[3];
      
      value[0]= camera_->guid & 0xffffffff;
      value[1]= (camera_->guid >>32) & 0x000000ff;
      value[2]= (camera_->guid >>40) & 0xfffff;
      
      sprintf(temp,"%06x%02x%08x", value[2], value[1], value[0]);

      if (strcmp(guid,"")==0)
        {
          ROS_INFO_STREAM("No guid specified, using first camera found, GUID: "
                          << std::hex << camera_->guid);
          device_id_ = std::string(temp);
          break;
        }

      ROS_DEBUG("Comparing %s to %s",guid,temp);
      if (strcmp(temp,guid)==0)
        {
          device_id_=guid;
          break;
        }
      SafeCleanup();
    }
  free (temp);
  dc1394_camera_free_list (list);
  
  if (!camera_)
    {
      if (strcmp(guid,"")==0)
        { 
          CAM_EXCEPT(camera1394stereo::Exception, "Could not find camera");
        }
      else
        {
          CAM_EXCEPT_ARGS(camera1394stereo::Exception,
                          "Could not find camera with guid %s", guid);
        }
      return -1;
    }

  ROS_INFO_STREAM("camera model: " << camera_->vendor << " "
                  << camera_->model);

  // resetting some cameras is not a good idea
  if (reset_on_open && DC1394_SUCCESS != dc1394_camera_reset(camera_))
    {
      // reset failed: log a warning, but continue
      ROS_WARN("Unable to reset camera (continuing).");
    }

  // Enable IEEE1394b mode if the camera and bus support it
  if (camera_->bmode_capable)
    {
      if (DC1394_SUCCESS !=
          dc1394_video_set_operation_mode(camera_, DC1394_OPERATION_MODE_1394B))
        {
          SafeCleanup();
          CAM_EXCEPT(camera1394stereo::Exception,
                     "Unable to enable B mode for IEEE1394-capable camera.");
          return -1;
        }
    }

  // Get the ISO channel and speed of the video
  dc1394speed_t speed;
  if (DC1394_SUCCESS != dc1394_video_get_iso_speed(camera_, &speed))
    {
      SafeCleanup();
      CAM_EXCEPT(camera1394stereo::Exception,
                 "Unable to get iso data; is the camera plugged in?");
      return -1;
    }
  
  // Set camera to use DMA, improves performance.
  bool DMA_Success = true;

  // first set parameters that are common between format 7 and other modes
  if (DC1394_SUCCESS != dc1394_video_set_framerate(camera_,frameRate_))
    {
      ROS_WARN("Failed to set frameRate");
      DMA_Success = false;
    }
  if (DC1394_SUCCESS != dc1394_video_set_iso_speed(camera_,isoSpeed_))
  {
    ROS_WARN("Failed to set iso speed");
    DMA_Success = false;
  }
  // added for format 7 video modes
  if ( (videoMode_>=DC1394_VIDEO_MODE_FORMAT7_0) &&
       (DC1394_SUCCESS != dc1394_format7_set_roi(
                          camera_,
                          videoMode_, colorCoding_,
                            // bytes per packet - sets frame rate
                          DC1394_USE_MAX_AVAIL, 
                          roi_left_, roi_top_,
                          roi_width_, roi_height_) ) )
    {
       ROS_WARN("Failed to set ROI and coding for format 7 mode");
    }
  if (DC1394_SUCCESS != dc1394_video_set_mode(camera_,videoMode_))
    {
      ROS_WARN("Failed to set mode");
      DMA_Success = false;
    }

  // when video mode is one of the format 7 modes
  // set the ROI and the color coding
  // TODO: check if frame_rate is the actual rate in this modes
  
  // now start capture
  if (DC1394_SUCCESS != dc1394_capture_setup(camera_, NUM_DMA_BUFFERS,
                                             DC1394_CAPTURE_FLAGS_DEFAULT))
    DMA_Success = false;
  
  if (!DMA_Success)
    {
      SafeCleanup();
      CAM_EXCEPT(camera1394stereo::Exception, "Failed to open device!");
      return -1;
    }
    
  // Start transmitting camera data
  if (DC1394_SUCCESS != dc1394_video_set_transmission(camera_, DC1394_ON))
    {
      SafeCleanup();
      CAM_EXCEPT(camera1394stereo::Exception, "Failed to start device!");
      return -1;
    }

  features_ = new Features(camera_);
 
  return 0;
  
  
}


/** Safe Cleanup */
void Camera1394Stereo::SafeCleanup()
{
  if (features_)
    {
      delete features_;
      features_ = NULL;
    }

  if (camera_)
    {
      dc1394_capture_stop(camera_);
      dc1394_camera_free(camera_);
    }
  camera_ = NULL;

  // Also clean up data if memory is allocated later.
}


/** close the 1394 device */
int Camera1394Stereo::close()
{
  if (camera_)
    {
      if (DC1394_SUCCESS != dc1394_video_set_transmission(camera_, DC1394_OFF)
          || DC1394_SUCCESS != dc1394_capture_stop(camera_))
        ROS_WARN("unable to stop camera");
    }

  // Free resources
  SafeCleanup();

  return 0;
}


/** Return an image frame */
void Camera1394Stereo::readData(
    sensor_msgs::Image& image, 
    sensor_msgs::Image& image2)
{

  if (camera_ == NULL) {
    CAM_EXCEPT(camera1394stereo::Exception, "Read attempted on NULL camera port!");
    return;
  }

  //  CAM_EXCEPT(camera1394stereo::Exception, "Read not implemented.");

  dc1394video_frame_t * frame0 = NULL;
  dc1394_capture_dequeue (camera_, DC1394_CAPTURE_POLICY_WAIT, &frame0);
  if (!frame0)
    {
      CAM_EXCEPT(camera1394stereo::Exception, "Unable to capture frame");
      return;
    }

  dc1394video_frame_t frame1 = *frame0;

  if (DoStereoExtract_)
    {
      // deinterlace frame into two images one on top the other
      size_t frame1_size = frame0->total_bytes;
      frame1.image = (unsigned char *) malloc(frame1_size);
      frame1.allocated_image_bytes = frame1_size;
      switch (frame0->color_coding)
        {
        case DC1394_COLOR_CODING_MONO16:
          frame1.color_coding = DC1394_COLOR_CODING_MONO8;
          break;
        case DC1394_COLOR_CODING_RAW16:
          frame1.color_coding = DC1394_COLOR_CODING_RAW8;
          break;
        default:
          ROS_WARN("Stereo extract in a non 16bit video mode");
        };
        int err = dc1394_deinterlace_stereo_frames(frame0, &frame1, stereoMethod_);
        if (err != DC1394_SUCCESS)
        {
          free(frame1.image);
          dc1394_capture_enqueue(camera_, frame0);
          CAM_EXCEPT(camera1394stereo::Exception, "Could not extract stereo frames");
          return;
        }
    }
  
  uint8_t* capture_buffer = reinterpret_cast<uint8_t *>(frame1.image);
  
  if (DoBayerConversion_)
    {
      // debayer frame into RGB8
      dc1394video_frame_t frame2;
      size_t frame2_size = (frame1.size[0] * frame1.size[1]
                            * 3 * sizeof(unsigned char));
      frame2.image = (unsigned char *) malloc(frame2_size);
      frame2.allocated_image_bytes = frame2_size;
      frame2.color_coding = DC1394_COLOR_CODING_RGB8;
      frame1.color_filter = BayerPattern_;
      int err = dc1394_debayer_frames(&frame1, &frame2, BayerMethod_);
      if (err != DC1394_SUCCESS)
        {
          free(frame2.image);
          dc1394_capture_enqueue(camera_, frame0);
          CAM_EXCEPT(camera1394stereo::Exception, "Could not convert/debayer frames");
          return;
        }
      capture_buffer = reinterpret_cast<uint8_t *>(frame2.image);
    }

  assert(capture_buffer);

  image.header.stamp = ros::Time( double(frame0->timestamp) * 1.e-6 );
  image.width = frame0->size[0];
  image.height = frame0->size[1];
  image2.header.stamp = image.header.stamp;
  image2.width = 0;
  image2.height = 0;
  image2.step = 0;

  int image_size;  
  switch (videoMode_)
    {
    case DC1394_VIDEO_MODE_160x120_YUV444:
      image.step=image.width*3;
      image_size = image.height*image.step;
      image.encoding = sensor_msgs::image_encodings::RGB8;
      image.data.resize(image_size);
      yuv::uyv2rgb(reinterpret_cast<unsigned char *> (capture_buffer),
                   reinterpret_cast<unsigned char *> (&image.data[0]),
                   image.width * image.height);
      break;
    case DC1394_VIDEO_MODE_640x480_YUV411:
      image.step=image.width*3;
      image_size = image.height*image.step;
      image.encoding = sensor_msgs::image_encodings::RGB8;
      image.data.resize(image_size);
      yuv::uyyvyy2rgb(reinterpret_cast<unsigned char *> (capture_buffer),
                      reinterpret_cast<unsigned char *> (&image.data[0]),
                      image.width * image.height);
      break;
    case DC1394_VIDEO_MODE_320x240_YUV422:
    case DC1394_VIDEO_MODE_640x480_YUV422:
    case DC1394_VIDEO_MODE_800x600_YUV422:
    case DC1394_VIDEO_MODE_1024x768_YUV422:
    case DC1394_VIDEO_MODE_1280x960_YUV422:
    case DC1394_VIDEO_MODE_1600x1200_YUV422:
      image.step=image.width*3;
      image_size = image.height*image.step;
      image.encoding = sensor_msgs::image_encodings::RGB8;
      image.data.resize(image_size);
      yuv::uyvy2rgb(reinterpret_cast<unsigned char *> (capture_buffer),
                    reinterpret_cast<unsigned char *> (&image.data[0]),
                    image.width * image.height);
      break;
    case DC1394_VIDEO_MODE_640x480_RGB8:
    case DC1394_VIDEO_MODE_800x600_RGB8:
    case DC1394_VIDEO_MODE_1024x768_RGB8:
    case DC1394_VIDEO_MODE_1280x960_RGB8:
    case DC1394_VIDEO_MODE_1600x1200_RGB8:
      image.step=image.width*3;
      image_size = image.height*image.step;
      image.encoding = sensor_msgs::image_encodings::RGB8;
      image.data.resize(image_size);
      memcpy(&image.data[0], capture_buffer, image_size);
      break;
    case DC1394_VIDEO_MODE_640x480_MONO8:
    case DC1394_VIDEO_MODE_800x600_MONO8:
    case DC1394_VIDEO_MODE_1024x768_MONO8:
    case DC1394_VIDEO_MODE_1280x960_MONO8:
    case DC1394_VIDEO_MODE_1600x1200_MONO8:
      if (!DoBayerConversion_)
        {
          image.step=image.width;
          image_size = image.height*image.step;
          // set Bayer encoding in ROS Image message
          switch (BayerPattern_)
            {
            case DC1394_COLOR_FILTER_RGGB:
              image.encoding = sensor_msgs::image_encodings::BAYER_RGGB8;
              break;
            case DC1394_COLOR_FILTER_GBRG:
              image.encoding = sensor_msgs::image_encodings::BAYER_GBRG8;
              break;
            case DC1394_COLOR_FILTER_GRBG:
              image.encoding = sensor_msgs::image_encodings::BAYER_GRBG8;
              break;
            case DC1394_COLOR_FILTER_BGGR:
              image.encoding = sensor_msgs::image_encodings::BAYER_BGGR8;
              break;
            default:
              image.encoding = sensor_msgs::image_encodings::MONO8;
            }
          image.data.resize(image_size);
          memcpy(&image.data[0], capture_buffer, image_size);
        }
      else
        {
          image.step=image.width*3;
          image_size = image.height*image.step;
          image.encoding = sensor_msgs::image_encodings::RGB8;
          image.data.resize(image_size);
          memcpy(&image.data[0], capture_buffer, image_size);
        } 
      break;
    case DC1394_VIDEO_MODE_640x480_MONO16:
    case DC1394_VIDEO_MODE_800x600_MONO16:
    case DC1394_VIDEO_MODE_1024x768_MONO16:
    case DC1394_VIDEO_MODE_1280x960_MONO16:
    case DC1394_VIDEO_MODE_1600x1200_MONO16:
    // check if these modes are handled properly
    case DC1394_VIDEO_MODE_FORMAT7_0:
    case DC1394_VIDEO_MODE_FORMAT7_3:
      if (DoStereoExtract_ && DoBayerConversion_)
        {
          image2.width = image.width;
          image2.height = image.height;
          image2.step = image.step = image.width*3;
          image_size = image.height*image.step;
          image2.encoding = image.encoding = sensor_msgs::image_encodings::RGB8;
          image.data.resize(image_size);
          image2.data.resize(image_size);
          memcpy(&image.data[0], capture_buffer, image_size);
          memcpy(&image2.data[0], capture_buffer+image_size, image_size);
        }
      else if (DoStereoExtract_)
        {
          image2.width = image.width;
          image2.height = image.height;
          image.step = image.width;
          image2.step = image.step;
          image_size = image.height*image.step;
          switch (BayerPattern_)
            {
            case DC1394_COLOR_FILTER_RGGB:
              image.encoding = sensor_msgs::image_encodings::BAYER_RGGB8;
              break;
            case DC1394_COLOR_FILTER_GBRG:
              image.encoding = sensor_msgs::image_encodings::BAYER_GBRG8;
              break;
            case DC1394_COLOR_FILTER_GRBG:
              image.encoding = sensor_msgs::image_encodings::BAYER_GRBG8;
              break;
            case DC1394_COLOR_FILTER_BGGR:
              image.encoding = sensor_msgs::image_encodings::BAYER_BGGR8;
              break;
            default:
              image.encoding = sensor_msgs::image_encodings::MONO8;
            }
          image2.encoding = image.encoding;
          image.data.resize(image_size);
          image2.data.resize(image_size);
          memcpy(&image.data[0], capture_buffer, image_size);
          memcpy(&image2.data[0], capture_buffer+image_size, image_size);
        }
      else if (DoBayerConversion_)
        {
          // @todo test Bayer conversions for mono16
          image.step=image.width*3;
          image_size = image.height*image.step;
          image.encoding = sensor_msgs::image_encodings::RGB8;
          image.data.resize(image_size);
          memcpy(&image.data[0], capture_buffer, image_size);
        }
      else
        {
          image.step=image.width*2;
          image_size = image.height*image.step;
          image.encoding = sensor_msgs::image_encodings::MONO16;
          image.is_bigendian = false;    // check Bumblebee2 endianness
          image.data.resize(image_size);
          memcpy(&image.data[0], capture_buffer, image_size);
        }
      break;
    default:
      CAM_EXCEPT(camera1394stereo::Exception, "Unknown image mode");
      return;
    }

  dc1394_capture_enqueue(camera_, frame0);

  if (DoStereoExtract_ || DoBayerConversion_) 
    { 
      free(capture_buffer);
      if (DoStereoExtract_ && DoBayerConversion_)
         free(frame1.image); 
    }
}


} // namespace camera1394stereo
