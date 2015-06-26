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

#pragma once
#include <vector>
#include <boost/thread.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/thread/locks.hpp>
#include "util/settings.h"
#include "util/SophusUtil.h"
#include "util/rosPub.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "util/marginalization.h"

namespace lsd_slam
{

class TrackingReference;
class SE3Tracker;
class Frame;

class SlamSystem
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// settings. Constant from construction onward.
	int width;
	int height;
	Eigen::Matrix3f K;
    SE3 RefToFrame ;
    Eigen::Vector3f gravity_b0;
    bool twoWayMarginalizatonFlag = false;//false, marginalize oldest; true, marginalize newest
    MARGINALIZATION margin;

    bool trackingIsGood;
    float msTrackFrame, msOptimizationIteration, msFindConstraintsItaration, msFindReferences;
    int nTrackFrame, nOptimizationIteration, nFindConstraintsItaration, nFindReferences;
    float nAvgTrackFrame, nAvgOptimizationIteration, nAvgFindConstraintsItaration, nAvgFindReferences;
    struct timeval lastHzUpdate;

    ros::NodeHandle nh ;
    ros::Publisher pub_path ;
    ros::Publisher pub_odometry ;
    ros::Publisher pub_pose ;
    ros::Publisher pub_cloud ;
    ros::Publisher pub_grayImage ;
    ros::Publisher pub_resudualMap ;
    ros::Publisher pub_gradientMapForDebug ;
    ros::Publisher pub_reprojectMap ;
    visualization_msgs::Marker path_line;
    cv::StereoBM bm_ ;
    cv::Mat gradientMapForDebug ;

    SlamSystem(int w, int h, Eigen::Matrix3f K, ros::NodeHandle& n);
	SlamSystem(const SlamSystem&) = delete;
	SlamSystem& operator=(const SlamSystem&) = delete;
	~SlamSystem();

    void initRosPub();
    void generateDubugMap(Frame* currentFrame, cv::Mat& gradientMapForDebug ) ;
    void gtDepthInit(cv::Mat img0, cv::Mat img1, double timeStamp, int id);

	// tracks a frame.
	// first frame will return Identity = camToWord.
	// returns camToWord transformation of the tracked frame.
	// frameID needs to be monotonically increasing.
    void trackFrame(cv::Mat img0, cv::Mat img1, unsigned int frameID, double timestamp);

    /** Returns the current pose estimate. */
    void debugDisplayDepthMap();

	// ============= EXCLUSIVELY TRACKING THREAD (+ init) ===============
	TrackingReference* trackingReference; // tracking reference for current keyframe. only used by tracking.
	SE3Tracker* tracker;
    std::shared_ptr<Frame> currentKeyFrame;	// changed (and, for VO, maybe deleted)  only by Mapping thread within exclusive lock.
    std::shared_ptr<Frame> latestTrackedFrame;
    bool createNewKeyFrame;

	// ============= SHARED ENTITIES =============
	float tracking_lastResidual;
	float tracking_lastUsage;
	float tracking_lastGoodPerBad;
	float tracking_lastGoodPerTotal;
	int lastNumConstraintsAddedOnFullRetrack;
	float lastTrackingClosenessScore;
};

}
