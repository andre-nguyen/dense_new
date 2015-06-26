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

#include "SlamSystem.h"
#include "util/settings.h"
#include "DataStructures/Frame.h"
#include "Tracking/SE3Tracker.h"
#include "Tracking/TrackingReference.h"
#include "LiveSLAMWrapper.h"
#include "util/globalFuncs.h"
#include "IOWrapper/ImageDisplay.h"
#include "DataStructures/FrameMemory.h"
#include <deque>

// for mkdir
#include <sys/types.h>
#include <sys/stat.h>

#ifdef ANDROID
#include <android/log.h>
#endif

#include "opencv2/opencv.hpp"

using namespace lsd_slam;


SlamSystem::SlamSystem(int w, int h, Eigen::Matrix3f K, ros::NodeHandle &n)
{
//    if(w%16 != 0 || h%16!=0)
//    {
//        printf("image dimensions must be multiples of 16! Please crop your images / video accordingly.\n");
//        assert(false);
//    }

	this->width = w;
	this->height = h;
	this->K = K;
    this->nh = n ;
	trackingIsGood = true;
    currentKeyFrame =  nullptr;
	createNewKeyFrame = false;

	tracker = new SE3Tracker(w,h,K);
    trackingReference = new TrackingReference();
    int maxDisparity = 64 ;
    int blockSize = 21 ;
    bm_ = cv::StereoBM( cv::StereoBM::BASIC_PRESET, maxDisparity, blockSize) ;
    initRosPub() ;

	lastTrackingClosenessScore = 0;
	msTrackFrame = msOptimizationIteration = msFindConstraintsItaration = msFindReferences = 0;
	nTrackFrame = nOptimizationIteration = nFindConstraintsItaration = nFindReferences = 0;
	nAvgTrackFrame = nAvgOptimizationIteration = nAvgFindConstraintsItaration = nAvgFindReferences = 0;
	gettimeofday(&lastHzUpdate, NULL);

}

SlamSystem::~SlamSystem()
{
	delete trackingReference;
	delete tracker;

	latestTrackedFrame.reset();
    currentKeyFrame.reset();
	FrameMemory::getInstance().releaseBuffes();
	Util::closeAllWindows();
}

void SlamSystem::debugDisplayDepthMap()
{
//	double scale = 1;
//	if(currentKeyFrame != 0 && currentKeyFrame != 0)
//		scale = currentKeyFrame->getScaledCamToWorld().scale();
//	// debug plot depthmap
//	char buf1[200];

//    snprintf(buf1,200,"dens %2.0f%%; good %2.0f%%; scale %2.2f; res %2.1f/; usg %2.0f%%; Map: %d F, %d KF, %d E, %.1fm Pts",
//			100*currentKeyFrame->numPoints/(float)(width*height),
//			100*tracking_lastGoodPerBad,
//			scale,
//			tracking_lastResidual,
//            100*tracking_lastUsage );


//	if(onSceenInfoDisplay)
//        printMessageOnCVImage(map->debugImageDepth, buf1 );
//	if (displayDepthMap)
//		Util::displayImage( "DebugWindow DEPTH", map->debugImageDepth, false );

//	int pressedKey = Util::waitKey(1);
//	handleKey(pressedKey);
}

void SlamSystem::initRosPub()
{
    pub_path = nh.advertise<visualization_msgs::Marker>("/denseVO/path", 1000);
    pub_cloud = nh.advertise<sensor_msgs::PointCloud>("/denseVO/cloud", 1000);
    pub_odometry = nh.advertise<nav_msgs::Odometry>("/denseVO/odometry", 1000);
    pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/denseVO/pose", 1000);
    pub_resudualMap = nh.advertise<sensor_msgs::Image>("denseVO/residualMap", 100 );
    pub_reprojectMap = nh.advertise<sensor_msgs::Image>("denseVO/reprojectMap", 100 );
    pub_gradientMapForDebug = nh.advertise<sensor_msgs::Image>("denseVO/debugMap", 100 );

    path_line.header.frame_id    = "world";
    path_line.header.stamp       = ros::Time::now();
    path_line.ns                 = "dense_vo";
    path_line.action             = visualization_msgs::Marker::ADD;
    path_line.pose.orientation.w = 1.0;
    path_line.type               = visualization_msgs::Marker::LINE_STRIP;
    path_line.scale.x            = 0.01 ;
    path_line.color.a            = 1.0;
    path_line.color.r            = 1.0;
    path_line.id                 = 1;
    path_line.points.push_back( geometry_msgs::Point());
    pub_path.publish(path_line);
}

void SlamSystem::generateDubugMap(Frame* currentFrame, cv::Mat& gradientMapForDebug )
{
    int n = currentFrame->height() ;
    int m = currentFrame->width() ;
    const float* pIdepth = currentFrame->idepth(0) ;
    for ( int i = 0 ; i < n ; i++ )
    {
        for( int j = 0 ; j < m ; j++ )
        {
            if (  *pIdepth > 0 ){
                gradientMapForDebug.at<cv::Vec3b>(i, j)[0] = 0;
                gradientMapForDebug.at<cv::Vec3b>(i, j)[1] = 255;
                gradientMapForDebug.at<cv::Vec3b>(i, j)[2] = 0;
            }
            pIdepth++ ;
        }
    }
}

void SlamSystem::gtDepthInit(cv::Mat img0, cv::Mat img1, double timeStamp, int id)
{
    cv::Mat disparity, depth ;
    bm_(img1, img0, disparity, CV_32F);
    calculateDepthImage(disparity, depth, 0.11, K(0, 0) );

    currentKeyFrame.reset(new Frame(id, width, height, K, timeStamp, img1.data ));
    Frame* frame = currentKeyFrame.get() ;
    frame->setDepthFromGroundTruth( (float*)depth.data );
    if ( printDebugInfo ){
        cv::cvtColor(img1, gradientMapForDebug, CV_GRAY2BGR ) ;
        generateDubugMap(frame, gradientMapForDebug ) ;
        sensor_msgs::Image msg;
        msg.header.stamp = ros::Time() ;
        sensor_msgs::fillImage(msg, sensor_msgs::image_encodings::BGR8, height, width, width*3,
                               gradientMapForDebug.data );
        pub_gradientMapForDebug.publish(msg) ;
    }
    frame->R_bk_2_b0.setIdentity() ;
    frame->T_bk_2_b0.setZero() ;
    frame->v_bk.setZero() ;
    RefToFrame = SE3();
//    std::cout << RefToFrame.rotationMatrix() << std::endl ;
//    std::cout << RefToFrame.translation() << std::endl ;
}

void SlamSystem::trackFrame(cv::Mat img0, cv::Mat img1, unsigned int frameID, double timestamp)
{
	// Create new frame
    std::shared_ptr<Frame> trackingNewFrame(new Frame(frameID, width, height, K, timestamp, img1.data ));

    if (  trackingReference->keyframe != currentKeyFrame.get() ){
         trackingReference->importFrame( currentKeyFrame.get() );
    }

    //initial guess
    SE3 RefToFrame_initialEstimate = RefToFrame ;

    //track
	struct timeval tv_start, tv_end;
	gettimeofday(&tv_start, NULL);
    RefToFrame = tracker->trackFrame( trackingReference, trackingNewFrame.get(),
                               RefToFrame_initialEstimate );
	gettimeofday(&tv_end, NULL);

    Eigen::Matrix3f R_k_2_c = RefToFrame.rotationMatrix();
    Eigen::Vector3f T_k_2_c = RefToFrame.translation();
    Matrix3f R_bk1_2_b0 = trackingReference->keyframe->R_bk_2_b0 * R_k_2_c.transpose();
    Vector3f T_bk1_2_b0 = trackingReference->keyframe->T_bk_2_b0 + R_bk1_2_b0*T_k_2_c ;
    pubOdometry(-T_bk1_2_b0, R_bk1_2_b0, pub_odometry, pub_pose );
    pubPath(-T_bk1_2_b0, path_line, pub_path );

    //debug information
    //msTrackFrame = 0.9*msTrackFrame + 0.1*((tv_end.tv_sec-tv_start.tv_sec)*1000.0f + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f);
    msTrackFrame = (tv_end.tv_sec-tv_start.tv_sec)*1000.0f + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f ;
    printf("msTrackFrame = %0.f\n", msTrackFrame ) ;
	nTrackFrame++;
	tracking_lastResidual = tracker->lastResidual;
	tracking_lastUsage = tracker->pointUsage;
	tracking_lastGoodPerBad = tracker->lastGoodCount / (tracker->lastGoodCount + tracker->lastBadCount);
	tracking_lastGoodPerTotal = tracker->lastGoodCount / (trackingNewFrame->width(SE3TRACKING_MIN_LEVEL)*trackingNewFrame->height(SE3TRACKING_MIN_LEVEL));

	// Keyframe selection
    if ( trackingReference->keyframe->numFramesTrackedOnThis > MIN_NUM_MAPPED )
	{
        Sophus::Vector3f dist = RefToFrame.translation() * currentKeyFrame->meanIdepth;
        float minVal = 1.0f;

        lastTrackingClosenessScore = getRefFrameScore(dist.dot(dist), tracker->pointUsage, KFDistWeight, KFUsageWeight);
        if (lastTrackingClosenessScore > minVal )
		{
			createNewKeyFrame = true;

           // if(enablePrintDebugInfo && printKeyframeSelectionInfo)
           //     printf("SELECT %d on %d! dist %.3f + usage %.3f = %.3f > 1\n",trackingNewFrame->id(),trackingNewFrame->getTrackingParent()->id(), dist.dot(dist), tracker->pointUsage, lastTrackingClosenessScore );
        }
		else
		{
        //	if(enablePrintDebugInfo && printKeyframeSelectionInfo)
        //       printf("SKIPPD %d on %d! dist %.3f + usage %.3f = %.3f < 1\n",trackingNewFrame->id(),trackingNewFrame->getTrackingParent()->id(), dist.dot(dist), tracker->pointUsage, lastTrackingClosenessScore );
		}
	}
    if ( createNewKeyFrame == true || tracker->trackingWasGood == false )
    {
        cv::Mat disparity, depth ;
        bm_(img1, img0, disparity, CV_32F);
        calculateDepthImage(disparity, depth, 0.11, K(0, 0) );

        currentKeyFrame = trackingNewFrame;
        Frame* frame = currentKeyFrame.get() ;
        frame->setDepthFromGroundTruth( (float*)depth.data);
        if ( printDebugInfo ){
            cv::cvtColor(img1, gradientMapForDebug, CV_GRAY2BGR ) ;
            generateDubugMap(frame, gradientMapForDebug ) ;
            sensor_msgs::Image msg;
            msg.header.stamp = ros::Time() ;
            sensor_msgs::fillImage(msg, sensor_msgs::image_encodings::BGR8, height, width, width*3,
                                   gradientMapForDebug.data );
            pub_gradientMapForDebug.publish(msg) ;
        }
        frame->R_bk_2_b0 = R_bk1_2_b0 ;
        frame->T_bk_2_b0 = T_bk1_2_b0 ;
        frame->v_bk.setZero() ;
        RefToFrame = SE3() ;
        createNewKeyFrame = false;
    }
}

