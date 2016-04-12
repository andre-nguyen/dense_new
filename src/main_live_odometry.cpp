/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University
*of Munich)
* For more information see <http://vision.in.tum.de/lsdslam>
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include "LiveSLAMWrapper.h"
#include <iostream>
#include <fstream>
#include <boost/thread.hpp>
#include "util/settings.h"
#include "util/globalFuncs.h"
#include "SlamSystem.h"
#include "DataStructures/types.h"

#include "IOWrapper/ROS/rosReconfigure.h"

#include <X11/Xlib.h>
#include "dense_new/LSDParamsConfig.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "cv_bridge/cv_bridge.h"

#include "sensor_msgs/Imu.h"
//#include "visensor_node/visensor_calibration.h"

using namespace lsd_slam;
using namespace std;

CALIBRATION_PAR calib_par;
ros::Subscriber sub_image[2];
ros::Subscriber sub_imu;
LiveSLAMWrapper* globalLiveSLAM = NULL;
cv::Mat R0, R1, P0, P1, Q;
cv::Rect roi1, roi2;
cv::Mat map00_, map01_, map10_, map11_;

void readCalibrationExtrinsics(string caliFilePath) {
  cv::FileStorage fs(caliFilePath.c_str(), cv::FileStorage::READ);

  cv::Mat Ric_0, Tic_0;
  cv::Mat Ric_1, Tic_1;

  fs["Ric_0"] >> Ric_0;
  fs["Tic_0"] >> Tic_0;
  fs["Ric_1"] >> Ric_1;
  fs["Tic_1"] >> Tic_1;

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      calib_par.R_i_2_c(i, j) = Ric_1.at<double>(i, j);
    }
    calib_par.T_i_2_c(i) = Tic_1.at<double>(0, i);
  }
}

void readCalibrationIntrisics(string caliFilePath) {
  cv::FileStorage fs(caliFilePath.c_str(), cv::FileStorage::READ);

  cv::Mat D0, K0, D1, K1, R1, P1, R0, P0;

  fs["D0"] >> D0;
  fs["D1"] >> D1;
  fs["K0"] >> K0;
  fs["K1"] >> K1;
  fs["R0"] >> R0;
  fs["R1"] >> R1;
  fs["P0"] >> P0;
  fs["P1"] >> P1;

  int image_width = 752;
  int image_height = 480;
  cv::Size img_size(image_width, image_height);

  // cv::Mat K0_new = cv::getOptimalNewCameraMatrix(K0, D0, img_size, 0.0 ) ;
  // cv::Mat K1_new = cv::getOptimalNewCameraMatrix(K1, D1, img_size, 0.0 ) ;

  // cv::initUndistortRectifyMap
  cv::initUndistortRectifyMap(K0, D0, R0, P0, img_size, CV_16SC2, map00_,
                              map01_);
  cv::initUndistortRectifyMap(K1, D1, R1, P1, img_size, CV_16SC2, map10_,
                              map11_);

  calib_par.fx = K0.at<double>(0, 0) / 2.0;
  calib_par.fy = K0.at<double>(1, 1) / 2.0;
  calib_par.cx = (K0.at<double>(0, 2) + 0.5) / 2.0 - 0.5;
  calib_par.cy = (K0.at<double>(1, 2) + 0.5) / 2.0 - 0.5;
  for (int i = 0; i < 4; i++) {
    calib_par.d[i] = D0.at<double>(0, i);
  }

  calib_par.width = image_width / 2;
  calib_par.height = image_height / 2;

  printf("fx=%f fy=%f cx=%f cy=%f\n", calib_par.fx, calib_par.fy, calib_par.cx,
         calib_par.cy);
  printf("height=%d width=%d\n", calib_par.width, calib_par.height);
}

void image0CallBack(const sensor_msgs::ImageConstPtr& msg) {
  ros::Time tImage = msg->header.stamp;
  cv::Mat image = cv_bridge::toCvShare(msg, std::string("mono8"))->image;
  cv::Mat imgRect;

  // double t = (double)cvGetTickCount()  ;
  cv::remap(image, imgRect, map00_, map01_, cv::INTER_LINEAR);
  cv::pyrDown(imgRect, imgRect, cv::Size(imgRect.cols / 2, imgRect.rows / 2));
  // printf("rect time: %f\n", ((double)cvGetTickCount() - t) /
  // (cvGetTickFrequency() * 1000) );

  //    cv::imshow("image0", imgRect ) ;
  //    cv::waitKey(1) ;

  globalLiveSLAM->image0_queue_mtx.lock();
  globalLiveSLAM->image0Buf.push_back(ImageMeasurement(tImage, imgRect));
  globalLiveSLAM->image0_queue_mtx.unlock();
}

void image1CallBack(const sensor_msgs::ImageConstPtr& msg) {
  ros::Time tImage = msg->header.stamp;
  cv::Mat image = cv_bridge::toCvShare(msg, std::string("mono8"))->image;
  cv::Mat imgRect;
  cv::remap(image, imgRect, map10_, map11_, cv::INTER_LINEAR);
  cv::pyrDown(imgRect, imgRect, cv::Size(imgRect.cols / 2, imgRect.rows / 2));

  //    cv::imshow("image1", imgRect ) ;
  //    cv::waitKey(1) ;

  globalLiveSLAM->image1_queue_mtx.lock();
  globalLiveSLAM->image1Buf.push_back(ImageMeasurement(tImage, imgRect));
  globalLiveSLAM->image1_queue_mtx.unlock();
}

void imuCallBack(const sensor_msgs::Imu& imu_msg) {
  globalLiveSLAM->imu_queue_mtx.lock();
  globalLiveSLAM->imuQueue.push_back(imu_msg);
  globalLiveSLAM->imu_queue_mtx.unlock();
}

void process_image() { globalLiveSLAM->Loop(); }

void process_BA() { globalLiveSLAM->BALoop(); }

int main(int argc, char** argv) {
  XInitThreads();

  ros::init(argc, argv, "LSD_SLAM");
  ros::NodeHandle nh;

  dynamic_reconfigure::Server<dense_new::LSDParamsConfig> srv(
      ros::NodeHandle("~"));
  srv.setCallback(dynConfCb);

  string packagePath = ros::package::getPath("dense_new") + "/";
  // string caliFilePath = packagePath + "calib/LSD_calib.cfg" ;

  readCalibrationIntrisics(packagePath + "calib/pinky.yml");
  readCalibrationExtrinsics(packagePath + "calib/pinky_extrinsics.yml");
  //    if ( initCalibrationPar(caliFilePath) == false ){
  //        return 0 ;
  //    }

  sub_image[0] = nh.subscribe("/sync/cam1/image_raw", 100, &image0CallBack);
  sub_image[1] = nh.subscribe("/sync/cam0/image_raw", 100, &image1CallBack);
  sub_imu = nh.subscribe("/sync/imu/imu", 1000, &imuCallBack);

  // Output3DWrapper* outputWrapper = new ROSOutput3DWrapper(calib_par.width,
  // calib_par.height, nh);
  LiveSLAMWrapper slamNode(packagePath, nh, calib_par);
  globalLiveSLAM = &slamNode;
  globalLiveSLAM->popAndSetGravity();
  boost::thread ptrProcessImageThread = boost::thread(&process_image);
  boost::thread ptrProcessBAThread = boost::thread(&process_BA);

  ros::spin();
  ptrProcessImageThread.join();
  ptrProcessBAThread.join();

  return 0;
}
