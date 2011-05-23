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

// $Id: camera1394.cpp 33358 2010-10-19 13:59:00Z joq $

#include <signal.h>
#include <stdlib.h>     // for system() call
#include <ros/ros.h>
#include <boost/format.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_listener.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <driver_base/SensorLevels.h>
#include <driver_base/driver.h>

// #include <opencv/cv.h>
// #include <opencv/highgui.h>
// #include <cv_bridge/CvBridge.h>

#include "dev_camera1394stereo.h"
#include "camera1394stereo/Camera1394StereoConfig.h"

/** @file

@brief A CANVIAR PER JOANPAU ROS driver node for IEEE 1394 digital cameras.

This is a ROS port of the Player driver for 1394 cameras, using
libdc1394.  It provides a reliable driver with minimal dependencies,
intended to fill a role in the ROS image pipeline similar to the other
ROS camera drivers.

The ROS image pipeline provides Bayer filtering at a higher level (in
image_proc).  In some cases it is useful to run the driver without the
entire image pipeline, so libdc1394 Bayer decoding is also provided.

@par Advertises

 - @b stereo_camera/left/image_raw topic (sensor_msgs/Image) raw 2D left camera images

 - @b stereo_camera/left/camera_info topic (sensor_msgs/CameraInfo) Calibration
   information for each left image
   
 - @b stereo_camera/right/image_raw topic (sensor_msgs/Image) raw 2D right camera images
 
 - @b stereo_camera/right/camera_info topic (sensor_msgs/CameraInfo) Calibration
   information for each right image


*/

typedef driver_base::Driver Driver;
typedef driver_base::SensorLevels Levels;

/** Segfault signal handler */
void sigsegv_handler(int sig)
{
  signal(SIGSEGV, SIG_DFL);
  fprintf(stderr, "Segmentation fault, stopping camera driver.\n");
  ROS_ERROR("Segmentation fault, stopping camera.");
  ros::shutdown();                      // stop the main loop
}

class Camera1394StereoNode
{
private:

  Driver::state_t state_;               // current driver state
  
  enum CameraSelector {LEFT=0,RIGHT=1};
  static const int NUM_CAMERAS = 2;
  static const std::string CameraSelectorString[NUM_CAMERAS]; // = {"left","right"};

  ros::NodeHandle privNH_;              // common private node handle
  ros::NodeHandle camera_nh_;           // common name space handle
  ros::NodeHandle single_camera_nh_[NUM_CAMERAS]; // left/right camera name space handle
  sensor_msgs::Image image_[NUM_CAMERAS];
  sensor_msgs::CameraInfo cam_info_[NUM_CAMERAS];
  std::string camera_name_;             // camera name

  /** 1394 camera device interface */
  camera1394stereo::Camera1394Stereo *dev_;

  /** dynamic parameter configuration */
  typedef camera1394stereo::Camera1394StereoConfig Config;
  Config config_;

  /** camera calibration information */
  CameraInfoManager *cinfo_[NUM_CAMERAS];
  bool calibration_matches_[NUM_CAMERAS]; // cam_info_(left|right) matches video mode

  /** image transport interfaces */
  image_transport::ImageTransport *it_;
  image_transport::CameraPublisher image_pub_[NUM_CAMERAS];
  
  /** saver members */  
//  sensor_msgs::CvBridge bridge_;
//  int count_;
//  boost::format format_;

public:

  Camera1394StereoNode(): it_(0)
  {
    state_ = Driver::CLOSED;
    privNH_ = ros::NodeHandle("~");
    camera_nh_ = ros::NodeHandle("stereo_camera");
    single_camera_nh_[LEFT] = ros::NodeHandle("stereo_camera/left");  // for left CameraInfoManager
    single_camera_nh_[RIGHT] = ros::NodeHandle("stereo_camera/right"); // for right CameraInfoManager
    camera_name_ = "stereo_camera";
    dev_ = new camera1394stereo::Camera1394Stereo();
    for (int i=0; i< NUM_CAMERAS; i++) 
      {
        cinfo_[i] = new CameraInfoManager(single_camera_nh_[i]);
        calibration_matches_[i] = true;
      }
//    // saver members initialization
//    format_.parse("%s%08i.%s");
//    count_=0;
  }

  ~Camera1394StereoNode()
  {
    if (it_)
      delete it_;
    delete dev_;
    for (int i=0; i<NUM_CAMERAS; i++)
      delete cinfo_[i];
  }

  /** Close camera device
   *
   *  postcondition: state_ is Driver::CLOSED
   */
  void closeCamera()
  {
    if (state_ != Driver::CLOSED)
      {
        ROS_INFO_STREAM("[" << camera_name_ << "] closing device");
        dev_->close();
        state_ = Driver::CLOSED;
      }
  }

  /** Open the camera device.
   *
   * @param newconfig configuration parameters
   * @return true, if successful
   *
   * if successful:
   *   state_ is Driver::OPENED
   *   camera_name_ set to GUID string
   */
  bool openCamera(Config &newconfig)
  {
    bool success = false;
    int retries = 2;                    // number of retries, if open fails
    do
      {
        try
          {
            if (dev_->open(newconfig.guid.c_str(),
                           newconfig.video_mode.c_str(),
                           newconfig.color_coding.c_str(),
                           newconfig.roi_left,
                           newconfig.roi_top,
                           newconfig.roi_width,
                           newconfig.roi_height,
                           newconfig.frame_rate,
                           newconfig.iso_speed, 
                           newconfig.bayer_pattern.c_str(),
                           newconfig.bayer_method.c_str(),
                           newconfig.stereo_method.c_str(),
                           newconfig.reset_on_open)
                == 0)
              {
                if (camera_name_ != dev_->device_id_)
                  {
                    camera_name_ = dev_->device_id_;
                    for (int i=0; i<NUM_CAMERAS; i++)
                      if ( ! cinfo_[i]->setCameraName(camera_name_ + "(" + 
                                                      CameraSelectorString[i] + ")") )
                        {
                          // GUID is 16 hex digits, which should be valid.
                          // If not, use it for log messages anyway.
                          ROS_WARN_STREAM("[" << camera_name_ 
                                          <<"("<<CameraSelectorString[i]<<")"
                                          << "] name not valid"
                                          << " for camera_info_manger");
                        }
                  }
                ROS_INFO_STREAM("[" << camera_name_ << "] opened: "
                                << newconfig.video_mode << " "
                                << "(format7 options: "
                                << newconfig.color_coding << " "
                                << "ROI with origin at ("
                                << newconfig.roi_left << ","
                                << newconfig.roi_top << ") "
                                << "and size "
                                << newconfig.roi_width << "x" 
                                << newconfig.roi_height << "), "
                                << newconfig.frame_rate << " fps "
                                << "(may be overwritten by frame rate feature),"
                                << newconfig.iso_speed << " Mb/s");
                state_ = Driver::OPENED;
                for (int i=0; i<NUM_CAMERAS; i++)
                  calibration_matches_[i] = true;
                success = true;
              }
          }
        catch (camera1394stereo::Exception& e)
          {
            if (retries > 0)
              {
              ROS_WARN_STREAM("[" << camera_name_
                              << "] exception opening device (retrying): "
                              << e.what());

                ROS_WARN_STREAM("Trying to reset bus with system call...");
                if ( system("dc1394_reset_bus") == 0 ) 
                  ROS_WARN_STREAM("Bus reset system call successful");
                else
                  ROS_WARN_STREAM("Bus reset system call failure");

              }
            else
              ROS_ERROR_STREAM("[" << camera_name_
                               << "] device open failed: " << e.what());
          }
      }
    while (!success && --retries >= 0);

    return success;
  }

  /** Publish camera stream topics
   *
   *  @pre image_ contains latest camera frame
   */
  void publish()
  {
    for (int i=0; i<NUM_CAMERAS; i++)
      {
        image_[i].header.frame_id = config_.frame_id;
        // get current CameraInfo data
        cam_info_[i] = cinfo_[i]->getCameraInfo();
        if ( cam_info_[i].height != image_[i].height ||
             cam_info_[i].width != image_[i].width )
          {
            // image size does not match: publish a matching uncalibrated
            // CameraInfo instead
            if (calibration_matches_[i])
              {
                // warn user once
                calibration_matches_[i] = false;
                ROS_WARN_STREAM("[" << camera_name_
                                << "(" << CameraSelectorString[i] << ")"
                                << "] calibration does not match video mode "
                                << "(publishing uncalibrated data)");
              }
            cam_info_[i] = sensor_msgs::CameraInfo();
            cam_info_[i].height = image_[i].height;
            cam_info_[i].width = image_[i].width;
          }
        else if (!calibration_matches_[i])
          {
            // calibration OK now
            calibration_matches_[i] = true;
            ROS_INFO_STREAM("[" << camera_name_
                            << "(" << CameraSelectorString[i] << ")"
                            << "] calibration matches video mode now");
          }

        cam_info_[i].header.frame_id = config_.frame_id;
        cam_info_[i].header.stamp = image_[i].header.stamp;

        // @todo log a warning if (filtered) time since last published
        // image is not reasonably close to configured frame_rate
        
        // Publish via image_transport
        image_pub_[i].publish(image_[i], cam_info_[i]);
      }
  }
  
//  /** Joan Pau - Save image to file
//   *
//   *  @param image image to save
//   *  @param name name prefix of the image file
//   *  @param count number suffix of the file name (before the extension)
//   */
//  void save(const sensor_msgs::Image& image, const std::string& name, int count)
//  {
//    if (bridge_.fromImage(image, "bgr8")) {
//      IplImage *ipl_image = bridge_.toIpl();
//      if (ipl_image) {
//        std::string filename = (format_ % name % count % "jpg").str();
//        cvSaveImage(filename.c_str(), ipl_image);
//        ROS_INFO("Saved image %s", filename.c_str());
//      } else {
//        ROS_WARN("Couldn't save image, no data!");
//      }
//    }
//    else
//      ROS_ERROR("Unable to convert %s image to bgr8", image.encoding.c_str());
//  }

  /** Read camera data.
   *
   * @return true if successful
   */
  bool read()
  {
    bool success = true;
    try
      {
        // Read data from the Camera
        ROS_DEBUG_STREAM("[" << camera_name_ << "] reading data");
        dev_->readData(image_[LEFT],image_[RIGHT]);
        ROS_DEBUG_STREAM("[" << camera_name_ << "] read returned");
      }
    catch (camera1394stereo::Exception& e)
      {
        ROS_WARN_STREAM("[" << camera_name_
                        << "] Exception reading data: " << e.what());
        success = false;
      }
    return success;
  }

  /** Dynamic reconfigure callback
   *
   *  Called immediately when callback first defined. Called again
   *  when dynamic reconfigure starts or changes a parameter value.
   *
   *  @param newconfig new Config values
   *  @param level bit-wise OR of reconfiguration levels for all
   *               changed parameters (0xffffffff on initial call)
   **/
  void reconfig(Config &newconfig, uint32_t level)
  {
    ROS_DEBUG("dynamic reconfigure level 0x%x", level);

    // resolve frame ID using tf_prefix parameter
    if (newconfig.frame_id == "")
      newconfig.frame_id = "stereo_camera";
    std::string tf_prefix = tf::getPrefixParam(privNH_);
    ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
    newconfig.frame_id = tf::resolve(tf_prefix, newconfig.frame_id);

    if (state_ != Driver::CLOSED && (level & Levels::RECONFIGURE_CLOSE))
      {
        // must close the device before updating these parameters
        closeCamera();                  // state_ --> CLOSED
      }

    if (state_ == Driver::CLOSED)
      {
        // open with new values
        if (openCamera(newconfig))
          {
            // update GUID string parameter
            newconfig.guid = camera_name_;
          }
      }
      
    std::string camera_info_url[NUM_CAMERAS];
    camera_info_url[LEFT] = config_.camera_info_url_left;
    camera_info_url[RIGHT] = config_.camera_info_url_right;
    std::string new_camera_info_url[NUM_CAMERAS];
    new_camera_info_url[LEFT] = newconfig.camera_info_url_left;
    new_camera_info_url[RIGHT] = newconfig.camera_info_url_right;

    for (int i=0; i<NUM_CAMERAS; i++)
      {
        if (camera_info_url[i] != new_camera_info_url[i])
          {
            // set the new URL and load CameraInfo (if any) from it
            if (cinfo_[i]->validateURL(new_camera_info_url[i]))
              {
                cinfo_[i]->loadCameraInfo(new_camera_info_url[i]);
              }
            else
              {
                // new URL not valid, use the old one
                ROS_WARN_STREAM("[" << camera_name_ 
                                    << "("<<CameraSelectorString[i]<<")" << "]"
                                    << "not updating calibration to invalid URL " 
                                    << new_camera_info_url[i] );
              }
          }
       }

    if (state_ != Driver::CLOSED)       // openCamera() succeeded?
      {
        // configure IIDC features
        if (level & Levels::RECONFIGURE_CLOSE)
          {
            // initialize all features for newly opened device
            if (false == dev_->features_->initialize(&newconfig))
              {
                ROS_ERROR_STREAM("[" << camera_name_
                                 << "] feature initialization failure");
                closeCamera();          // can't continue
              }
          }
        else
          {
            // update any features that changed
            dev_->features_->reconfigure(&newconfig);
          }
      }

    config_ = newconfig;                // save new parameters

    ROS_DEBUG_STREAM("[" << camera_name_
                     << "] reconfigured: frame_id " << newconfig.frame_id
                     << ", camera_info_url_left " << newconfig.camera_info_url_left
                     << ", camera_info_url_right " << newconfig.camera_info_url_right);
  }


  /** driver main spin loop */
  void spin(void)
  {
    // the bring up order is tricky
    ros::NodeHandle node;

    // define segmentation fault handler, sometimes libdc1394 craps out
    signal(SIGSEGV, &sigsegv_handler);

    // Define dynamic reconfigure callback, which gets called
    // immediately with level 0xffffffff.  The reconfig() method will
    // set initial parameter values, then open the device if it can.
    dynamic_reconfigure::Server<Config> srv;
    dynamic_reconfigure::Server<Config>::CallbackType f
      = boost::bind(&Camera1394StereoNode::reconfig, this, _1, _2);
    srv.setCallback(f);

    // set up ROS interfaces in camera namespace
    it_ = new image_transport::ImageTransport(camera_nh_);
    image_pub_[LEFT] = it_->advertiseCamera("left/image_raw", 1);
    image_pub_[RIGHT] = it_->advertiseCamera("right/image_raw", 1);
    
    while (node.ok())
      {
        if (state_ != Driver::CLOSED)
          {
            if (read())
              {
                publish();
                // count_++;
                // save(image_[LEFT],std::string("left"),count_);
                // save(image_[RIGHT],std::string("right"),count_);
              }
          }
      
        ros::spinOnce();
      }

    closeCamera();
  }

}; // end Camera1394StereoNode class definition

const std::string Camera1394StereoNode::CameraSelectorString[Camera1394StereoNode::NUM_CAMERAS] = {"left","right"};

/** Main entry point */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera1394stereo_node");
  ros::NodeHandle node;
  Camera1394StereoNode cm;

  cm.spin();

  return 0;
}
