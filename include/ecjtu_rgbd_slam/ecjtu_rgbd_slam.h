/**
 *  This file is part of dvo.
 *
 *  Copyright 2012 Christian Kerl <christian.kerl@in.tum.de> (Technical University of Munich)
 *  For more information see <http://vision.in.tum.de/data/software/dvo>.
 *
 *  dvo is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  dvo is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with dvo.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef ECJTU_RGBD_SLAM_H_
#define ECJTU_RGBD_SLAM_H_


#include <mrpt/base.h>
#include <mrpt/slam.h>
#include <mrpt/gui.h>


#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/console.h>

#include <tf/transform_listener.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <image_transport/image_transport.h>

#include <Eigen/Geometry>
#include <ccny_rgbd/GftDetectorConfig.h>

#include <dynamic_reconfigure/server.h>

#include <dvo_ros/camera_base.h>

#include <dvo/dense_tracking.h>


#include <dvo_ros/CameraDenseTrackerConfig.h>
#include <dvo_ros/visualization/ros_camera_trajectory_visualizer.h>



#include <dvo/core/intrinsic_matrix.h>

#include <dvo/core/rgbd_image.h>



#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/point_cloud.h>
#include <rgbdtools/rgbdtools.h>

#include <ccny_rgbd/util.h>

#include <ccny_rgbd/types.h>


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <ccny_rgbd/FeatureDetectorConfig.h>
#include <ccny_rgbd/GftDetectorConfig.h>
#include <ccny_rgbd/StarDetectorConfig.h>
#include <ccny_rgbd/SurfDetectorConfig.h>
#include <ccny_rgbd/OrbDetectorConfig.h>



// #include <mrpt/gui.h>
// #include <mrpt/obs.h>



#include "ecjtu_rgbd_slam/brand.h"
#include <opencv2/features2d/features2d.hpp>





namespace ecjtu_rgbd_slam
{


using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::opengl;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace std;

using namespace dvo_ros;
using namespace ccny_rgbd;


class EcjtuRgbdSLAM: public CameraBase
{

private:
//   typedef dynamic_reconfigure::Server<ecjtu_rgbd_slam::CameraDenseTrackerConfig> ReconfigureServer;
    typedef dynamic_reconfigure::Server<dvo_ros::CameraDenseTrackerConfig> DVOReconfigureServer;
//  typedef dynamic_reconfigure::Server<ecjtu_rgbd_slam::EcjtuRgbdSLAM> SlamReconfigureServer;

    uint32_t width;
    uint32_t height;

    bool is_pose_3d;

    bool show_opengl;

//     CTicTac kftictac;
    bool  CAMERA_3DSCENE_FOLLOWS_ROBOT;

    mrpt::gui::CDisplayWindow3DPtr win3d;

    dvo::DenseTracker::Config tracker_cfg;

    boost::mutex mutex_; ///< state mutex
    boost::shared_ptr<dvo::DenseTracker> tracker;
    
dvo::visualization::CameraTrajectoryVisualizerInterface* vis_;

    dvo::core::RgbdCameraPyramidPtr camera;
    dvo::core::RgbdImagePyramidPtr current, reference;

//  boost::shared_ptr<dvo::core::RgbdImagePyramid> current, reference;

    Eigen::Affine3d accumulated_transform, from_baselink_to_asus, latest_absolute_transform_,  latest_accumulated_transform;

    size_t frames_since_last_success;

    size_t step;


//     FeatureDetectorConfigServer config_server_; ///< ROS dynamic reconfigure server
//
//     GftDetectorConfigServerPtr gft_config_server_;    ///< ROS dynamic reconfigure server for GFT params
    StarDetectorConfigServerPtr star_config_server_;  ///< ROS dynamic reconfigure server for STAR params
//     SurfDetectorConfigServerPtr surf_config_server_;    ///< ROS dynamic reconfigure server for SURF params
//     OrbDetectorConfigServerPtr orb_config_server_;  ///< ROS dynamic reconfigure server for ORB params


    float testtrans[4][4], testtrans_av[4][4];

    tf::TransformListener tl;

    ros::Publisher pose_pub_;
    ros::Publisher odometry_pub_;

    ros::Publisher feature_cloud_publisher_;

    ros::Publisher covariances_publisher_;  ///< publisher for feature covariances_publisher_

    ros::Subscriber pose_sub_;

//   ReconfigureServer reconfigure_server_;
    DVOReconfigureServer tracker_reconfigure_server_;
//  SlamReconfigureServer slam_reconfigure_server_;

// bool is_run_test;
    

    bool use_dense_tracking_estimate_;


    int smooth, n_features;
    double max_range, max_stdev, min_distance, threshold;


    bool publish_feature_cloud_;
    bool publish_feature_cov_;
    bool publish_cloud_;

    bool publish_covariances_;
    bool show_keypoints_;

    boost::mutex tracker_mutex_;

    FILE * diagnostics_file_;           ///< File for time recording statistics

    bool save_diagnostics_;              ///< indicates whether to save results to file or print to screen
    bool verbose_;                      ///< indicates whether to print diagnostics to screen

    std::string detector_type_;

    boost::shared_ptr<rgbdtools::FeatureDetector> feature_detector_; ///< The feature detector object
    rgbdtools::MotionEstimationICPProbModel motion_estimation_; ///< The motion estimation object
    void resetDetector();

    void publishFeatureCloud(rgbdtools::RGBDFrame& frame);

    void publishFeatureCovariances(rgbdtools::RGBDFrame& frame);


    int  frame_count_; ///< RGBD frame counter

    bool hasChanged(const CameraInfoMsg::ConstPtr& camera_info_msg);
    void reset(const CameraInfoMsg::ConstPtr& camera_info_msg);

    void publishTransform(const std_msgs::Header& header, const Eigen::Affine3d& transform, const std::string frame);
    void publishPose(const std_msgs::Header& header, const Eigen::Affine3d& transform, const std::string frame);

//   bool check_large(Eigen::Affine3d latest_accumulated_transform);

//   void reconfigCallback(FeatureDetectorConfig& config, uint32_t level);
//   void gftReconfigCallback(GftDetectorConfig& config, uint32_t level);
    void starReconfigCallback(StarDetectorConfig& config, uint32_t level);
//   void surfReconfigCallback(SurfDetectorConfig& config, uint32_t level);
//   void orbReconfigCallback(OrbDetectorConfig& config, uint32_t level);




BrandDescriptorExtractor brand_desc;

//做BRAND实验用
         rgbdtools::RGBDFrame ref_frame; 
	 cv::Mat ref_rgb_in;
	 cv::Mat ref_depth_in;
   cv::Mat ref_cloud, ref_normals;

   
//=================   


    void showKeypointImage(rgbdtools::RGBDFrame& frame);

    std::string fixed_frame_; ///< Fixed frame parameter
    std::string base_frame_;  ///< Moving frame parameter

    bool publish_tf_;         ///< Parameter whether to publish a ros tf
    bool publish_path_;       ///< Parameter whether to publish a path message
    bool publish_odom_;       ///< Parameter whether to publish an odom message
    bool publish_pose_;       ///< Parameter whether to publish a pose message


    bool publish_model_cloud_;
    bool publish_model_cov_;

    int queue_size_;  ///< Subscription queue size
//   void publishFeatureCovariances(rgbdtools::RGBDFrame& frame);

    void diagnostics(
        int n_features, int n_valid_features, //int n_model_pts,
        double d_frame, double d_features, double d_reg, double d_total);


    CRangeBearingKFSLAM mapping;

    std::vector<TPose3D>  meanPath; // The estimated path
    CPose3DQuatPDFGaussian   robotPose;
    std::vector<CPoint3D>	 LMs;
    std::map<unsigned int,CLandmark::TLandmarkID>    LM_IDs;
    CMatrixDouble  fullCov;
    CVectorDouble  fullState;

    cv::Mat fulldescriptors;
    
     std::vector<TPose3D>  OdomeanPath; // The Odometry estimated path


    

public:
    EcjtuRgbdSLAM(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    virtual ~EcjtuRgbdSLAM();

    void initParams();

// const ImageMsg::ConstPtr& rgb_msg;

    void compute_normals(const cv::Mat& cloud, cv::Mat& normals);

    virtual void handleImages(
        const ccny_rgbd::ImageMsg::ConstPtr& rgb_image_msg,
        const ccny_rgbd::ImageMsg::ConstPtr& depth_image_msg,
        const ccny_rgbd::CameraInfoMsg::ConstPtr& rgb_camera_info_msg,
        const ccny_rgbd::CameraInfoMsg::ConstPtr& depth_camera_info_msg
    );
    void create_cloud( const cv::Mat &depth,
                       float fx, float fy, float cx, float cy,
                       cv::Mat& cloud );

    void crossCheckMatching( const cv::Mat& descriptors1, const cv::Mat& descriptors2,
                             std::vector<cv::DMatch>& filteredMatches12 );

    void handlePose(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose);

    void handleConfig(dvo_ros::CameraDenseTrackerConfig& config, uint32_t level);
//     void TPose3D(CPose3D OdorobotPoseMean3D);
//     void TPose3D(const CPose3D robotPoseMean3D);
//     void format(const char* arg1, unsigned int arg2, unsigned int arg3);
//     void format(const char* arg1, unsigned int arg2, unsigned int arg3);
//     void star_config_server_();


};

}
#endif
