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
#include <vector>
#include <list>
#include <iostream>
#include "util/SophusUtil.h"
#include "util/globalFuncs.h"
#include "SlamSystem.h"
#include "IOWrapper/ImageDisplay.h"
#include "cv_bridge/cv_bridge.h"


namespace lsd_slam
{


LiveSLAMWrapper::LiveSLAMWrapper(std::string packagePath, ros::NodeHandle& _nh, const CALIBRATION_PAR &calib_par)
{
    fx = calib_par.fx;
    fy = calib_par.fy;
    cx = calib_par.cx;
    cy = calib_par.cy;
    width = calib_par.width;
    height = calib_par.height;
    nh = _nh ;

    isInitialized = false;
    Sophus::Matrix3f K_sophus;
    K_sophus << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;

	outFileName = packagePath+"estimated_poses.txt";
    outFile = nullptr;

	// make Odometry
    monoOdometry = new SlamSystem(width, height, K_sophus, _nh);

	imageSeqNumber = 0;
    image0Buf.clear();
    image1Buf.clear();
    imuBuf.clear();
}


LiveSLAMWrapper::~LiveSLAMWrapper()
{
	if(monoOdometry != 0)
		delete monoOdometry;
	if(outFile != 0)
	{
		outFile->flush();
		outFile->close();
		delete outFile;
	}
    image0Buf.clear();
    image1Buf.clear();
    imuBuf.clear();
}

void LiveSLAMWrapper::popAndSetGravity()
{
    unsigned int image0BufSize ;
    unsigned int image1BufSize ;
    std::list<ImageMeasurement>::iterator iter0 ;
    std::list<ImageMeasurement>::iterator iter1 ;
    std::list<ImageMeasurement>::reverse_iterator reverse_iterImage ;
    std::list<visensor_node::visensor_imu>::iterator currentIMU_iter;
    ros::Time tImage ;
    ros::Rate r(100) ;

    gravity_b0.setZero() ;
    while ( ros::ok() )
    {
        ros::spinOnce() ;
        image0_queue_mtx.lock();
        image1_queue_mtx.lock();
        image0BufSize = image0Buf.size();
        image1BufSize = image1Buf.size();
        if ( image0BufSize < 8 || image1BufSize < 8 ){
            image0_queue_mtx.unlock();
            image1_queue_mtx.unlock();
            r.sleep() ;
            continue ;
        }
        reverse_iterImage = image0Buf.rbegin() ;
        tImage = reverse_iterImage->t ;
        reverse_iterImage = image1Buf.rbegin() ;
        if ( reverse_iterImage->t < tImage ){
            tImage = reverse_iterImage->t ;
        }
        iter0 = image0Buf.begin();
        iter1 = image1Buf.begin();
        while ( iter0->t < tImage ){
            iter0 = image0Buf.erase( iter0 ) ;
        }
        while ( iter1->t < tImage ){
            iter1 = image1Buf.erase( iter1 ) ;
        }
        image0_queue_mtx.unlock();
        image1_queue_mtx.unlock();

        imu_queue_mtx.lock();
        int imuNum = 0;
        currentIMU_iter = imuBuf.begin() ;
        while( currentIMU_iter->header.stamp < tImage )
        {
            imuNum++;
            gravity_b0(0) += currentIMU_iter->linear_acceleration.x;
            gravity_b0(1) += currentIMU_iter->linear_acceleration.y;
            gravity_b0(2) += currentIMU_iter->linear_acceleration.z;
            currentIMU_iter = imuBuf.erase(currentIMU_iter);
        }
        imu_queue_mtx.unlock();
        gravity_b0 /= imuNum ;
        break ;
    }
    std::cout << "gravity_b0 =\n" ;
    std::cout << gravity_b0 << "\n" ;
}

void LiveSLAMWrapper::Loop()
{
    unsigned int image0BufSize ;
    unsigned int image1BufSize ;
    unsigned int imuBufSize ;
    std::list<ImageMeasurement>::iterator iter0 ;
    std::list<ImageMeasurement>::iterator iter1 ;
    std::list<visensor_node::visensor_imu>::reverse_iterator reverse_iterImu ;
    std::list<visensor_node::visensor_imu>::iterator currentIMU_iter;
    ros::Time tImage ;
    cv::Mat   image0 ;
    cv::Mat   image1 ;
    ros::Rate r(1000.0);
    while ( nh.ok() )
    {
        image0_queue_mtx.lock();
        image1_queue_mtx.lock();
        imu_queue_mtx.lock();
        image0BufSize = image0Buf.size();
        image1BufSize = image1Buf.size();
        imuBufSize = imuBuf.size();
        //printf("%d %d %d\n",image0BufSize, image1BufSize, imuBufSize ) ;
        if ( image0BufSize == 0 || image1BufSize == 0 || imuBufSize == 0 ){
            image0_queue_mtx.unlock();
            image1_queue_mtx.unlock();
            imu_queue_mtx.unlock();
            r.sleep() ;
            continue ;
        }
        iter0 = image0Buf.begin();
        iter1 = image1Buf.begin();
        while ( iter1 != image1Buf.end() && iter0->t > iter1->t ){
            iter1 =  image1Buf.erase( iter1 ) ;
        }
        while ( iter0 != image0Buf.end() && iter0->t < iter1->t ){
            iter0 =  image0Buf.erase( iter0 ) ;
        }
        if ( iter1 == image1Buf.end() || iter0 == image0Buf.end() ){
            image0_queue_mtx.unlock();
            image1_queue_mtx.unlock();
            imu_queue_mtx.unlock();
            r.sleep() ;
            continue ;
        }
        tImage = iter0->t;
        reverse_iterImu = imuBuf.rbegin() ;
        if ( reverse_iterImu->header.stamp < tImage ){
            image0_queue_mtx.unlock();
            image1_queue_mtx.unlock();
            imu_queue_mtx.unlock();
            r.sleep() ;
            continue ;
        }
        imu_queue_mtx.unlock();

        image0 = iter0->image.clone();
        image1 = iter1->image.clone();
        iter1 =  image1Buf.erase( iter1 ) ;
        iter0 =  image0Buf.erase( iter0 ) ;
        image0_queue_mtx.unlock();
        image1_queue_mtx.unlock();

        imu_queue_mtx.lock();
        currentIMU_iter = imuBuf.begin() ;
        Quaternionf q, dq ;
        q.setIdentity() ;
        while (currentIMU_iter->header.stamp < tImage )
        {
            float pre_t = currentIMU_iter->header.stamp.toSec();
            currentIMU_iter = imuBuf.erase(currentIMU_iter);
            float next_t = currentIMU_iter->header.stamp.toSec();
            float dt = next_t - pre_t ;

            //prediction for dense tracking
            dq.x() = currentIMU_iter->angular_velocity.x*dt*0.5 ;
            dq.y() = currentIMU_iter->angular_velocity.y*dt*0.5 ;
            dq.z() = currentIMU_iter->angular_velocity.z*dt*0.5 ;
            dq.w() =  sqrt( 1 - SQ(dq.x()) * SQ(dq.y()) * SQ(dq.z()) ) ;
            q = (q * dq).normalized();
        }
        imu_queue_mtx.unlock();

		// process image
		//Util::displayImage("MyVideo", image.data);
        newImageCallback(image0, image1, tImage);
	}
}


void LiveSLAMWrapper::newImageCallback(const cv::Mat& img0, const cv::Mat& img1, ros::Time imgTime)
{
	++ imageSeqNumber;

	// Assert that we work with 8 bit images
    assert(img1.elemSize() == 1);
    assert(img0.elemSize() == 1);
	assert(fx != 0 || fy != 0);

    // need to initialize
	if(!isInitialized)
    {
        monoOdometry->gtDepthInit(img0, img1, imgTime.toSec(), 1);
		isInitialized = true;
	}
	else if(isInitialized && monoOdometry != nullptr)
	{
        monoOdometry->trackFrame(img0, img1, imageSeqNumber, imgTime.toSec() );
	}
}

void LiveSLAMWrapper::logCameraPose(const SE3& camToWorld, double time)
{
	Sophus::Quaternionf quat = camToWorld.unit_quaternion().cast<float>();
	Eigen::Vector3f trans = camToWorld.translation().cast<float>();

	char buffer[1000];
	int num = snprintf(buffer, 1000, "%f %f %f %f %f %f %f %f\n",
			time,
			trans[0],
			trans[1],
			trans[2],
			quat.x(),
			quat.y(),
			quat.z(),
			quat.w());

	if(outFile == 0)
		outFile = new std::ofstream(outFileName.c_str());
	outFile->write(buffer,num);
	outFile->flush();
}

//void LiveSLAMWrapper::resetAll()
//{
//	if(monoOdometry != nullptr)
//	{
//		delete monoOdometry;
//		printf("Deleted SlamSystem Object!\n");

//		Sophus::Matrix3f K;
//		K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
//        monoOdometry = new SlamSystem(width,height, K, nh );

//	}
//	imageSeqNumber = 0;
//	isInitialized = false;

//	Util::closeAllWindows();

//}

}
