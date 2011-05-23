///////////////////////////////////////////////////////////////////////////
// Joan Pau Beltran
//  Modificat per donar suport a les cameres Bumblebee2 de Pointgrey.
//
// Copyright (C) 2009, 2010 Patrick Beeson, Jack O'Quin
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

// $Id: dev_camera1394.h 33358 2010-10-19 13:59:00Z joq $

/** @file

    @brief IEEE 1394 digital camera library interface

 */

#ifndef DEV_CAMERA1394_HH
#define DEV_CAMERA1394_HH

#include <dc1394/dc1394.h>

// ROS include
#include <sensor_msgs/Image.h>

#include "features.h"

namespace camera1394stereo
{
  //! Macro for defining an exception with a given parent
  //  (std::runtime_error should be top parent)
  // code borrowed from drivers/laser/hokuyo_driver/hokuyo.h
#define DEF_EXCEPTION(name, parent)		\
  class name  : public parent {			\
  public:					\
    name (const char* msg) : parent (msg) {}	\
  }

  //! A standard Camera1394 exception
  DEF_EXCEPTION(Exception, std::runtime_error);

  class Camera1394Stereo
  {
  public:
    Camera1394Stereo ();
    ~Camera1394Stereo ();

    int open (const char* guid,
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
              bool reset_on_open);
    int close();

    void readData (sensor_msgs::Image &image_left, 
        sensor_msgs::Image &image_right);

    std::string device_id_;
    Features *features_;

  private:
    // device identifier
    dc1394camera_t * camera_;
      
    dc1394framerate_t frameRate_;
    dc1394video_mode_t videoMode_;
    dc1394color_coding_t colorCoding_;   // needed for format 7 (stereo)
    int roi_left_;    // needed for format 7 ROI (stereo)
    int roi_top_;     // needed for format 7 ROI (stereo)
    int roi_width_;   // needed for format 7 ROI (stereo)
    int roi_height_;  // needed for format 7 ROI (stereo)
    dc1394speed_t isoSpeed_;
    dc1394color_filter_t BayerPattern_;
    dc1394bayer_method_t BayerMethod_;
    bool DoBayerConversion_;
    dc1394stereo_method_t stereoMethod_;
    bool DoStereoExtract_;

    void SafeCleanup();

    void findFrameRate(float);
    void findVideoMode(const char*);
    void findFormat7ColorCoding(const char* coding);
    void findFormat7ROI(int left, int top, int width, int height);
    void findIsoSpeed(int);
    void findBayerFilter(const char*, const char*);
    void findStereoMethod(const char* method);
    
  };
};

#endif // DEV_CAMERA1394_HH
