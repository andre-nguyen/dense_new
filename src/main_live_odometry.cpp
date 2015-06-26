/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
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

#include "visensor_node/visensor_imu.h"
#include "visensor_node/visensor_calibration.h"

using namespace lsd_slam;
using namespace std ;

CALIBRATION_PAR calib_par ;
ros::Subscriber sub_image[2];
ros::Subscriber sub_imu;
LiveSLAMWrapper* globalLiveSLAM = NULL ;

bool initCalibrationPar(string caliFilePath)
{
    //read calibration parameters
    std::ifstream f(caliFilePath.c_str());
    if (!f.good())
    {
        f.close();
        printf(" %s not found!\n", caliFilePath.c_str());
        return false;
    }
    std::string l1, l2;
    std::getline(f,l1);
    std::getline(f,l2);
    f.close();

    if(std::sscanf(l1.c_str(), "%f %f %f %f %f %f %f %f",
                   &calib_par.fx, &calib_par.fy, &calib_par.cx, &calib_par.cy,
                   &calib_par.d[0], &calib_par.d[1], &calib_par.d[2], &calib_par.d[3]) != 8 )
    {
        puts("calibration file format error 1") ;
        return false ;
    }
    if(std::sscanf(l2.c_str(), "%d %d", &calib_par.width, &calib_par.height ) != 2)
    {
        puts("calibration file format error 2") ;
        return false ;
    }

    return true ;
}

void image0CallBack(const sensor_msgs::ImageConstPtr& msg)
{
    ros::Time tImage = msg->header.stamp;
    cv::Mat   image  = cv_bridge::toCvShare(msg, std::string("mono8"))->image;
    globalLiveSLAM->image0_queue_mtx.lock();
    globalLiveSLAM->image0Buf.push_back(ImageMeasurement(tImage, image));
    globalLiveSLAM->image0_queue_mtx.unlock();
}

void image1CallBack(const sensor_msgs::ImageConstPtr& msg)
{
    ros::Time tImage = msg->header.stamp;
    cv::Mat   image  = cv_bridge::toCvShare(msg, std::string("mono8"))->image;
    globalLiveSLAM->image1_queue_mtx.lock();
    globalLiveSLAM->image1Buf.push_back(ImageMeasurement(tImage, image));
    globalLiveSLAM->image1_queue_mtx.unlock();
}

void imuCallBack(const visensor_node::visensor_imu& imu_msg )
{
    globalLiveSLAM->imu_queue_mtx.lock();
    globalLiveSLAM->imuBuf.push_back( imu_msg );
    globalLiveSLAM->imu_queue_mtx.unlock();
}

void process_image()
{
    globalLiveSLAM->Loop();
}

int main( int argc, char** argv )
{
    XInitThreads();

	ros::init(argc, argv, "LSD_SLAM");
    ros::NodeHandle nh ;

    dynamic_reconfigure::Server<dense_new::LSDParamsConfig> srv(ros::NodeHandle("~"));
	srv.setCallback(dynConfCb);

    string packagePath = ros::package::getPath("dense_new")+"/";
    string caliFilePath = packagePath + "calib/LSD_calib.cfg" ;

    if ( initCalibrationPar(caliFilePath) == false ){
        return 0 ;
    }

    sub_image[0] = nh.subscribe("/cam0", 100, &image0CallBack );
    sub_image[1] = nh.subscribe("/cam1", 100, &image1CallBack );
    sub_imu = nh.subscribe("/cust_imu0", 1000, &imuCallBack ) ;

    //Output3DWrapper* outputWrapper = new ROSOutput3DWrapper(calib_par.width, calib_par.height, nh);
    LiveSLAMWrapper slamNode(packagePath, nh, calib_par );
    globalLiveSLAM = &slamNode ;
    globalLiveSLAM->popAndSetGravity();
    boost::thread ptrProcessImageThread = boost::thread(&process_image);

    ros::spin() ;
    ptrProcessImageThread.join();

	return 0;
}
