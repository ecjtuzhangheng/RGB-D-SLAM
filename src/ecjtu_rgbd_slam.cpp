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


#include "ecjtu_rgbd_slam/ecjtu_rgbd_slam.h"
#include <boost/bind.hpp>
#include <cv_bridge/cv_bridge.h>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <dvo/core/surface_pyramid.h>
#include <dvo/util/stopwatch.h>
#include <dvo/visualization/visualizer.h>
//#include <dvo/visualization/pcl_camera_trajectory_visualizer.h>


#include <dvo_ros/util/util.h>
#include <dvo_ros/util/configtools.h>
#include <dvo_ros/visualization/ros_camera_trajectory_visualizer.h>

#include <Eigen/Geometry>

#include <tf_conversions/tf_eigen.h>

#include <ccny_rgbd/types.h>

#include <pcl/features/integral_image_normal.h>


namespace ecjtu_rgbd_slam
{

using namespace dvo;
using namespace dvo::core;
using namespace dvo::util;

EcjtuRgbdSLAM::EcjtuRgbdSLAM(ros::NodeHandle& nh, ros::NodeHandle& nh_private):
    CameraBase(nh, nh_private),
    tracker_cfg(DenseTracker::getDefaultConfig()),
    frames_since_last_success(0),
    tracker_reconfigure_server_(nh_private),
//   config_server_(nh_private),
    brand_desc(10,32),
    vis_(new dvo::visualization::NoopCameraTrajectoryVisualizer()),
    use_dense_tracking_estimate_(false)
{

    ROS_INFO("EcjtuRgbdSLAM::ctor(...)");

    // dynamic reconfigure
//   FeatureDetectorConfigServer::CallbackType f = boost::bind(
//     &EcjtuRgbdSLAM::reconfigCallback, this, _1, _2);
//   config_server_.setCallback(f);
//
    DVOReconfigureServer::CallbackType reconfigure_server_callback = boost::bind(&EcjtuRgbdSLAM::handleConfig, this, _1, _2);
    tracker_reconfigure_server_.setCallback(reconfigure_server_callback);
    // **** initialize ROS parameters


    initParams();






    pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("ecjtuslam/pose", queue_size_);
    feature_cloud_publisher_ = nh_.advertise<PointCloudFeature>("feature/cloud", queue_size_);
    covariances_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("feature/covariances", queue_size_);
    pose_sub_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("pelican/pose", 1, &EcjtuRgbdSLAM::handlePose, this);


//     dvo_ros::util::tryGetTransform(from_baselink_to_asus, tl, "base_link", "asus");
// 
//     ROS_INFO_STREAM("transformation: base_link -> asus" << std::endl << from_baselink_to_asus.matrix());



from_baselink_to_asus.setIdentity();
    latest_absolute_transform_.setIdentity();
    accumulated_transform.setIdentity();
    latest_accumulated_transform.setIdentity();

    dvo::visualization::Visualizer::instance()
    .enabled(false)
    .useExternalWaitKey(false)
    .save(false)
    ;


}


EcjtuRgbdSLAM::~EcjtuRgbdSLAM()
{
    delete vis_;
}


void EcjtuRgbdSLAM::crossCheckMatching( const cv::Mat& descriptors1, const cv::Mat& descriptors2,
                                        std::vector<cv::DMatch>& filteredMatches12 )
{
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    filteredMatches12.clear();
    std::vector<std::vector<cv::DMatch> > matches12, matches21;
    matcher.knnMatch( descriptors1, descriptors2, matches12, 1 );
    matcher.knnMatch( descriptors2, descriptors1, matches21, 1 );
    for( size_t m = 0; m < matches12.size(); m++ )
    {
        bool findCrossCheck = false;
        for( size_t fk = 0; fk < matches12[m].size(); fk++ )
        {
            cv::DMatch forward = matches12[m][fk];

            for( size_t bk = 0; bk < matches21[forward.trainIdx].size(); bk++ )
            {
                cv::DMatch backward = matches21[forward.trainIdx][bk];
                if( backward.trainIdx == forward.queryIdx )
                {
                    filteredMatches12.push_back(forward);
                    findCrossCheck = true;
                    break;
                }
            }
            if( findCrossCheck ) break;
        }
    }
}


void EcjtuRgbdSLAM::compute_normals(const cv::Mat& cloud, cv::Mat& normals)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud( new pcl::PointCloud<pcl::PointXYZ> );

    pcl_cloud->clear();
    pcl_cloud->width     = cloud.cols;
    pcl_cloud->height    = cloud.rows;
    pcl_cloud->points.resize( pcl_cloud->width * pcl_cloud->height);

    for(int y = 0; y < cloud.rows; ++y)
        for(int x = 0; x < cloud.cols; ++x)
        {
            pcl_cloud->at(x,y).x = cloud.at<cv::Point3f>(y,x).x;
            pcl_cloud->at(x,y).y = cloud.at<cv::Point3f>(y,x).y;
            pcl_cloud->at(x,y).z = cloud.at<cv::Point3f>(y,x).z;
        }

    pcl::PointCloud<pcl::Normal>::Ptr pcl_normals (new pcl::PointCloud<pcl::Normal>);
    pcl_normals->clear();
    pcl_normals->width  = pcl_cloud->width;
    pcl_normals->height = pcl_cloud->height;
    pcl_normals->points.resize(pcl_cloud->width * pcl_cloud->height);

    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud( pcl_cloud );

    ne.setNormalSmoothingSize( 5 );
    ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
    ne.compute( *pcl_normals );

    normals.create( cloud.size(), CV_32FC3 );

    for(int y = 0; y < pcl_normals->height; ++y)
        for(int x = 0; x < pcl_normals->width; ++x)
        {
            normals.at<cv::Point3f>(y,x).x = pcl_normals->at(x,y).normal_x;
            normals.at<cv::Point3f>(y,x).y = pcl_normals->at(x,y).normal_y;
            normals.at<cv::Point3f>(y,x).z = pcl_normals->at(x,y).normal_z;
        }
}





void EcjtuRgbdSLAM::create_cloud( const cv::Mat &depth,
                                  float fx, float fy, float cx, float cy,
                                  cv::Mat& cloud )
{
    const float inv_fx = 1.f/fx;
    const float inv_fy = 1.f/fy;

    cloud.create( depth.size(), CV_32FC3 );

    for( int y = 0; y < cloud.rows; y++ )
    {
        cv::Point3f* cloud_ptr = (cv::Point3f*)cloud.ptr(y);
        const uint16_t* depth_prt = (uint16_t*)depth.ptr(y);

        for( int x = 0; x < cloud.cols; x++ )
        {
            float d = (float)depth_prt[x]/1000; // meters
            cloud_ptr[x].x = (x - cx) * d * inv_fx;
            cloud_ptr[x].y = (y - cy) * d * inv_fy;
            cloud_ptr[x].z = d;
        }
    }
}









void EcjtuRgbdSLAM::resetDetector()
{
//   gft_config_server_.reset();
//    star_config_server_.reset();
//   orb_config_server_.reset();
//   surf_config_server_.reset();
//
    if (detector_type_ == "ORB")
    {
        ROS_INFO("Creating ORB detector");
        feature_detector_.reset(new rgbdtools::OrbDetector());

        rgbdtools::OrbDetectorPtr orb_detector = boost::static_pointer_cast<rgbdtools::OrbDetector>(feature_detector_);
        orb_detector->setNFeatures(n_features);
//     orb_detector->setMinDistance(min_distance);
        orb_detector->setSmooth(smooth);
        orb_detector->setMaxRange(max_range);
        orb_detector->setMaxStDev(max_stdev);


//     orb_config_server_.reset(new
//       OrbDetectorConfigServer(ros::NodeHandle(nh_private_, "feature/ORB")));
//
//     // dynamic reconfigure
//     OrbDetectorConfigServer::CallbackType f = boost::bind(
//       &EcjtuRgbdSLAM::orbReconfigCallback, this, _1, _2);
//     orb_config_server_->setCallback(f);
    }
    else if (detector_type_ == "SURF")
    {
        ROS_INFO("Creating SURF detector");
        feature_detector_.reset(new rgbdtools::SurfDetector());

        rgbdtools::SurfDetectorPtr surf_detector = boost::static_pointer_cast<rgbdtools::SurfDetector>(feature_detector_);
//     surf_detector->setNFeatures(n_features);
//     surf_detector->setMinDistance(min_distance);
        surf_detector->setSmooth(smooth);
        surf_detector->setMaxRange(max_range);
        surf_detector->setMaxStDev(max_stdev);
        surf_detector->setThreshold(threshold);



//     surf_config_server_.reset(new
//       SurfDetectorConfigServer(ros::NodeHandle(nh_private_, "feature/SURF")));
//
//     // dynamic reconfigure
//     SurfDetectorConfigServer::CallbackType f = boost::bind(
//       &EcjtuRgbdSLAM::surfReconfigCallback, this, _1, _2);
//     surf_config_server_->setCallback(f);
    }
    else if (detector_type_ == "GFT")
    {
        ROS_INFO("Creating GFT detector");
        feature_detector_.reset(new rgbdtools::GftDetector());

        rgbdtools::GftDetectorPtr gft_detector =
            boost::static_pointer_cast<rgbdtools::GftDetector>(feature_detector_);

        gft_detector->setNFeatures(n_features);
        gft_detector->setMinDistance(min_distance);
        gft_detector->setSmooth(smooth);
        gft_detector->setMaxRange(max_range);
        gft_detector->setMaxStDev(max_stdev);


//     gft_config_server_.reset(new
//       GftDetectorConfigServer(ros::NodeHandle(nh_private_, "feature/GFT")));
//
//     // dynamic reconfigure
//     GftDetectorConfigServer::CallbackType f = boost::bind(
//       &EcjtuRgbdSLAM::gftReconfigCallback, this, _1, _2);
//     gft_config_server_->setCallback(f);
    }
    else if (detector_type_ == "STAR")
    {
        ROS_INFO("Creating STAR detector");
        feature_detector_.reset(new rgbdtools::StarDetector());

        rgbdtools::StarDetectorPtr star_detector = boost::static_pointer_cast<rgbdtools::StarDetector>(feature_detector_);
//     star_detector->setNFeatures(n_features);
        star_detector->setMinDistance(min_distance);
        star_detector->setSmooth(smooth);
        star_detector->setMaxRange(max_range);
        star_detector->setMaxStDev(max_stdev);
        star_detector->setThreshold(0);

//     star_config_server_.reset(new
//       StarDetectorConfigServer(ros::NodeHandle(nh_private_, "feature/STAR")));
//
//     // dynamic reconfigure
//     StarDetectorConfigServer::CallbackType f = boost::bind(
//       &EcjtuRgbdSLAM::starReconfigCallback, this, _1, _2);
//     star_config_server_->setCallback(f);
    }
    else
    {
        ROS_FATAL("%s is not a valid detector type! Using GFT", detector_type_.c_str());
        feature_detector_.reset(new rgbdtools::GftDetector());

        rgbdtools::GftDetectorPtr gft_detector = boost::static_pointer_cast<rgbdtools::GftDetector>(feature_detector_);
        gft_detector->setNFeatures(n_features);
        gft_detector->setMinDistance(min_distance);
        gft_detector->setSmooth(smooth);
        gft_detector->setMaxRange(max_range);
        gft_detector->setMaxStDev(max_stdev);

//     gft_config_server_.reset(new
//       GftDetectorConfigServer(ros::NodeHandle(nh_private_, "feature/GFT")));
//
//     // dynamic reconfigure
//     GftDetectorConfigServer::CallbackType f = boost::bind(
//       &EcjtuRgbdSLAM::gftReconfigCallback, this, _1, _2);
//     gft_config_server_->setCallback(f);
    }
}


void EcjtuRgbdSLAM::initParams()
{
    is_pose_3d = true;
    show_opengl = true;
//     landmark_count = 0;
    step = 0;
    CAMERA_3DSCENE_FOLLOWS_ROBOT = false;
    win3d = mrpt::gui::CDisplayWindow3D::Create("KF-SLAM live view",800,500);
//   if (!nh_private_.getParam ("publish_tf", publish_tf_))
    publish_tf_ = true;
//   if (!nh_private_.getParam ("publish_path", publish_path_))
    publish_path_ = true;
//   if (!nh_private_.getParam ("publish_odom", publish_odom_))
    publish_odom_ = true;
//   if (!nh_private_.getParam ("publish_pose", publish_pose_))
    publish_pose_ = true;
//   if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
//    fixed_frame_ = "/odom";
    fixed_frame_ = "/world";
//   if (!nh_private_.getParam ("base_frame", base_frame_))
    base_frame_ = "/camera_link";
//   if (!nh_private_.getParam ("queue_size", queue_size_))
    queue_size_ = 5;

    // detector params

//   if (!nh_private_.getParam ("feature/publish_feature_cloud", publish_feature_cloud_))
    publish_feature_cloud_ = true;
//   if (!nh_private_.getParam ("feature/publish_feature_covariances", publish_feature_cov_))
    publish_feature_cov_ = true;
//   if (!nh_private_.getParam ("feature/detector_type", detector_type_))
    detector_type_ = "GFT";
//     detector_type_ = "STAR";
//
    smooth = 5;
    n_features = 16;
    max_range = 5.5;
    min_distance = 1.0;
    max_stdev = 0.03;
    threshold = 0.0;



    mutex_.lock();

    resetDetector();


    mutex_.unlock();



mapping.KF_options.method = kfEKFNaive;





//    printf("Smooth:[%d],  MaxRange:[%f], MaxStDev:[%f]", feature_detector_->getSmooth(), feature_detector_->getMaxRange(),feature_detector_->getMaxStDev());


//
//   // registration params
//
//   configureMotionEstimation();
//
//   // diagnostic params
//
//   if (!nh_private_.getParam("verbose", verbose_))
//     verbose_ = true;
//   if (!nh_private_.getParam("save_diagnostics", save_diagnostics_))
//     save_diagnostics_ = false;
//   if (!nh_private_.getParam("diagnostics_file_name", diagnostics_file_name_))
//     diagnostics_file_name_ = "diagnostics.csv";
//
//   if(save_diagnostics_)
//   {
//     diagnostics_file_ = fopen(diagnostics_file_name_.c_str(), "w");
//
//     if (diagnostics_file_ == NULL)
//     {
//       ROS_ERROR("Can't create diagnostic file %s\n", diagnostics_file_name_.c_str());
//       return;
//     }
//
//     // print header
//     fprintf(diagnostics_file_, "%s, %s, %s, %s, %s, %s, %s, %s\n",
//       "Frame id",
//       "Frame dur.",
//       "All features", "Valid features",
//       "Feat extr. dur.",
//       "Model points", "Registration dur.",
//       "Total dur.");
//   }
}



/*

void EcjtuRgbdSLAM::reconfigCallback(FeatureDetectorConfig& config, uint32_t level)
{
  mutex_.lock();
  std::string old_detector_type = detector_type_;
  detector_type_ = config.detector_type;

  if(old_detector_type != detector_type_)
    resetDetector();

  feature_detector_->setSmooth(config.smooth);
  feature_detector_->setMaxRange(config.max_range);
  feature_detector_->setMaxStDev(config.max_stdev);

  publish_cloud_ = config.publish_cloud;
  publish_covariances_ = config.publish_covariances;
  show_keypoints_ = config.show_keypoints;

  mutex_.unlock();
}

void EcjtuRgbdSLAM::gftReconfigCallback(GftDetectorConfig& config, uint32_t level)
{
  rgbdtools::GftDetectorPtr gft_detector =
    boost::static_pointer_cast<rgbdtools::GftDetector>(feature_detector_);

  gft_detector->setNFeatures(config.n_features);
  gft_detector->setMinDistance(config.min_distance);
}
*/
/*

void EcjtuRgbdSLAM::starReconfigCallback(StarDetectorConfig& config, uint32_t level)
{
  rgbdtools::StarDetectorPtr star_detector =
    boost::static_pointer_cast<rgbdtools::StarDetector>(feature_detector_);

  star_detector->setThreshold(config.threshold);
  star_detector->setMinDistance(config.min_distance);
}*/

/*
void EcjtuRgbdSLAM::surfReconfigCallback(SurfDetectorConfig& config, uint32_t level)
{
  rgbdtools::SurfDetectorPtr surf_detector =
    boost::static_pointer_cast<rgbdtools::SurfDetector>(feature_detector_);

  surf_detector->setThreshold(config.threshold);
}

void EcjtuRgbdSLAM::orbReconfigCallback(OrbDetectorConfig& config, uint32_t level)
{
  rgbdtools::OrbDetectorPtr orb_detector =
    boost::static_pointer_cast<rgbdtools::OrbDetector>(feature_detector_);

  orb_detector->setThreshold(config.threshold);
  orb_detector->setNFeatures(config.n_features);
}


*/






bool EcjtuRgbdSLAM::hasChanged(const CameraInfoMsg::ConstPtr& camera_info_msg)
{
    return width != camera_info_msg->width || height != camera_info_msg->height;
}

void EcjtuRgbdSLAM::reset(const CameraInfoMsg::ConstPtr& camera_info_msg)
{
    //IntrinsicMatrix intrinsics = IntrinsicMatrix::create(camera_info_msg->K[0], camera_info_msg->K[4], camera_info_msg->K[2], camera_info_msg->K[5]);
    IntrinsicMatrix intrinsics = IntrinsicMatrix::create(camera_info_msg->P[0], camera_info_msg->P[5], camera_info_msg->P[2], camera_info_msg->P[6]);

    camera.reset(new dvo::core::RgbdCameraPyramid(camera_info_msg->width, camera_info_msg->height, intrinsics));
    camera->build(tracker_cfg.getNumLevels());

    tracker.reset(new DenseTracker(tracker_cfg));

    static RgbdImagePyramid* const __null__ = 0;

    reference.reset(__null__);
    current.reset(__null__);

    width = camera_info_msg->width;
    height = camera_info_msg->height;

    vis_->reset();
}

void EcjtuRgbdSLAM::handleConfig(dvo_ros::CameraDenseTrackerConfig& config, uint32_t level)
{
    if(level == 0) return;

    if(level & dvo_ros::CameraDenseTracker_RunDenseTracking)
    {
        if(config.run_dense_tracking)
        {
            startSynchronizedImageStream();
        }
        else
        {
            stopSynchronizedImageStream();

            // force reset of tracker
            width = 0;
            height = 0;
        }
    }

    if(!config.run_dense_tracking && config.use_dense_tracking_estimate)
    {
        config.use_dense_tracking_estimate = false;
    }

    use_dense_tracking_estimate_ = config.use_dense_tracking_estimate;

    if(level & dvo_ros::CameraDenseTracker_ConfigParam)
    {
        // fix config, so we don't die by accident
        if(config.coarsest_level < config.finest_level)
        {
            config.finest_level = config.coarsest_level;
        }

        dvo_ros::util::updateConfigFromDynamicReconfigure(config, tracker_cfg);

        // we are called in the ctor as well, but at this point we don't have a tracker instance
        if(tracker)
        {
            // lock tracker so we don't reconfigure it while it is running
            boost::mutex::scoped_lock lock(tracker_mutex_);

            tracker->configure(tracker_cfg);
            camera->build(tracker_cfg.getNumLevels());
        }

        ROS_INFO_STREAM("reconfigured tracker, config ( " << tracker_cfg << " )");
    }


    if(level & dvo_ros::CameraDenseTracker_MiscParam)
    {
        vis_->reset();
        delete vis_;

        if(config.reconstruction)
        {
            vis_ = new dvo_ros::visualization::RosCameraTrajectoryVisualizer(nh_);
        }
        else
        {
            vis_ = new dvo::visualization::NoopCameraTrajectoryVisualizer();
        }
    }
}

void EcjtuRgbdSLAM::handlePose(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose)
{
    tf::Transform tmp;

    tf::poseMsgToTF(pose->pose.pose, tmp);
    tf::transformTFToEigen(tmp, latest_absolute_transform_);

    if(!use_dense_tracking_estimate_)
        publishPose(pose->header, latest_absolute_transform_, "baselink_estimate");
}








//#####################################  主循环  #####################################//
void EcjtuRgbdSLAM::handleImages(
    const ImageMsg::ConstPtr& rgb_image_msg,
    const ImageMsg::ConstPtr& depth_image_msg,
    const CameraInfoMsg::ConstPtr& rgb_camera_info_msg,
    const CameraInfoMsg::ConstPtr& depth_camera_info_msg)
{

    static stopwatch sw_callback("callback");
    sw_callback.start();

    // lock tracker so no one can reconfigure it
    boost::mutex::scoped_lock lock(tracker_mutex_);

    // different size of rgb and depth image
    if(depth_camera_info_msg->width != rgb_camera_info_msg->width || depth_camera_info_msg->height != rgb_camera_info_msg->height)
    {
        ROS_WARN("RGB and depth image have different size!");
        return;
    }

    // something has changed
    if(hasChanged(rgb_camera_info_msg))
    {
        ROS_WARN("RGB image size has changed, resetting tracker!");
        reset(rgb_camera_info_msg);
    }


    cv::Mat intensity, depth;
    cv::Mat rgb_in = cv_bridge::toCvShare(rgb_image_msg)->image;

    if(rgb_in.channels() == 3)
    {
        cv::Mat tmp;
        cv::cvtColor(rgb_in, tmp, CV_BGR2GRAY, 1);

        tmp.convertTo(intensity, CV_32F);
    }
    else
    {
        rgb_in.convertTo(intensity, CV_32F);
    }

    cv::Mat depth_in;



    const std::string& enc = depth_image_msg->encoding;
    if (enc.compare("16UC1") == 0)
    {
        depth_in = cv_bridge::toCvShare(depth_image_msg)->image;
        SurfacePyramid::convertRawDepthImageSse(depth_in, depth, 0.001);
    }
    else// if (enc.compare("32FC1") == 0)
    {
        rgbdtools::depthImageFloatTo16bit(cv_bridge::toCvShare(depth_image_msg)->image, depth_in);
        depth = depth_in;
    }



    reference.swap(current);
    current = camera->create(intensity, depth);

    // time delay compensation TODO: use driver settings instead
    std_msgs::Header h = rgb_image_msg->header;
    //h.stamp -= ros::Duration(0.05);

    static Eigen::Affine3d first;
    
//=============================Odometry 噪声设置========================
    float actcov00 = 0.1;            	//square(x * 0.1);
    float actcov11 = 0.1;			//square(y * 0.2);
    float actcov22 = 0.1;  		//square(z * 0.1);
    float actcov33 = 0.02;		//square(0.03);
    float actcov44 = 0.02;		//square(0.03);
    float actcov55 = 0.02;		//square(0.03);
//=============================================================================Observation 噪声设置 =================================
    CMatrixDouble33 obs_cov;
    float obs_z_var = 0.2;
    obs_cov(1,1) = 0.02;
    obs_cov(2,2) = 0.02;
    
    
    double x, y, z, roll, pitch, yaw;

    
    CActionCollectionPtr actions = CActionCollection::Create();
    CSensoryFramePtr  observations = CSensoryFrame::Create();
    
    CObservationBearingRangePtr obs;    
    
    
    
    rgbdtools::RGBDFrame frame;
    
    // prepare opencv intrinsic matrix from incoming camera info
    cv::Mat intr, dist;
    convertCameraInfoToMats(rgb_camera_info_msg, intr, dist);
    /// @todo assert that distortion (dist) is 0
    // prepare rgbdtools header from incoming header
    rgbdtools::Header header;
    header.seq        = rgb_camera_info_msg->header.seq;
    header.frame_id   = "base_link_estimate";
    header.stamp.sec  = rgb_camera_info_msg->header.stamp.sec;
    header.stamp.nsec = rgb_camera_info_msg->header.stamp.nsec;
    
    
    //计算描述符
    cv::Mat cloud, normals;
    double cx = intr.at<double>(0, 2);
    double cy = intr.at<double>(1, 2);
    // focus length
    double fx = intr.at<double>(0, 0);
    double fy = intr.at<double>(1, 1);
    
    CPose3D c2b(0,0,0);
    double out_range, out_yaw, out_pitch;
    mrpt::math::TPoint3D aa;    
    
    Eigen::Affine3d transform;

    dvo::DenseTracker::Result result;
    
    
    
    if(!reference)  //第一次观测
    {
//         accumulated_transform = latest_absolute_transform_ * from_baselink_to_asus;

        accumulated_transform.setIdentity();
	latest_accumulated_transform.setIdentity();

        first = accumulated_transform;

        vis_->camera("first")->
        color(dvo::visualization::Color::blue()).
        update(current->level(0), accumulated_transform).
        show();
	
	obs = CObservationBearingRange::Create();
	obs->validCovariances	= true;
	obs->sensor_std_range	= max_range;
	obs->sensedData.clear();
	obs->timestamp = mrpt::system::getCurrentTime();

	CPose3D incPose3D = CPose3D(0, 0, 0, 0, 0, 0);
	
	CActionRobotMovement3D act;

	act.estimationMethod = CActionRobotMovement3D::emOdometry;
	act.poseChange.mean = incPose3D;
	act.poseChange.cov.eye();
	act.poseChange.cov(0,0) = 0;
	act.poseChange.cov(1,1) = 0;
	act.poseChange.cov(2,2) = 0;
	act.poseChange.cov(3,3) = 0;
	act.poseChange.cov(4,4) = 0;
	act.poseChange.cov(5,5) = 0;
	actions->insert( act );
	
	// initialize the RGBDframe
	frame = rgbdtools::RGBDFrame(rgb_in, depth_in, intr, header);
	
	// visualize
	feature_detector_->findFeatures(frame);
	
	create_cloud( frame.depth_img, fx, fy, cx, cy, cloud );
	compute_normals(cloud, normals);
	
	//用新的算法计算描述符
	brand_desc.compute( frame.rgb_img, cloud, normals, frame.keypoints, frame.descriptors );
	
	for (unsigned int kp_idx = 0; kp_idx < frame.keypoints.size(); ++kp_idx)
	{
	  //不稳定路标特征忽略
	  if (!frame.kp_valid[kp_idx]) continue;
	  const Vector3f& kp_mean = frame.kp_means[kp_idx];
// 	  const Matrix3f& kp_cov  = frame.kp_covariances[kp_idx];
	  
	  //转换成球面坐标
	  aa.x = kp_mean(0, 0);
	  aa.y = kp_mean(1, 0);
	  aa.z = kp_mean(2, 0);
	  c2b.sphericalCoordinates( aa, out_range, out_yaw, out_pitch );
	  
	  mrpt::slam::CObservationBearingRange::TMeasurement newMeas;    
	  
	  newMeas.landmarkID = fulldescriptors.rows;
	  newMeas.range = out_range;
	  newMeas.yaw = out_yaw;
	  newMeas.pitch = out_pitch;
	  obs_cov(0,0) = square(obs_z_var * out_range);
	  newMeas.covariance = obs_cov;
	  
	  // Insert:
	  obs->sensedData.push_back( newMeas );
	  fulldescriptors.push_back(frame.descriptors.row(kp_idx));
	}
	
	observations->push_back(obs);
	mapping.processActionObservation(actions,observations);
	
	mapping.getCurrentState( robotPose,LMs,LM_IDs,fullState,fullCov );
	
	// Get the mean robot pose as 3D:
	const CPose3D robotPoseMean3D = CPose3D(robotPose.mean);
	// Build the path:
	meanPath.push_back( TPose3D(robotPoseMean3D) );
	
	//保存Odometry Path	
	rgbdtools::eigenAffineToXYZRPYd(accumulated_transform, x, y, z, roll, pitch, yaw);  
	const CPose3D OdorobotPoseMean3D = CPose3D(x, y, z, roll, pitch, yaw);
	OdomeanPath.push_back(TPose3D(OdorobotPoseMean3D));

	return;
    }

    

    static stopwatch sw_match("match", 100);
    
    sw_match.start();
    bool success = tracker->match(*reference, *current, result);
    transform = result.Transformation;
    sw_match.stopAndPrint();
    
    
    
        
//     1.000000       -0.000051         0.000034          -0.000075
//     0.000051        1.000000         0.000272          -0.001190
//    -0.000034       -0.000273         1.000000          -0.000071
//     0.000000        0.000000         0.000000           1.000000
   
     transform(0,1) = transform(0,1) + 0.000051;
     transform(0,2) = transform(0,2) - 0.000034;
    transform(0,3) = transform(0,3) + 0.000075;
     transform(1,0) = transform(1,0) - 0.000051;
     transform(1,2) = transform(1,2) - 0.000272;
    transform(1,3) = transform(1,3) + 0.001190;
     transform(2,0) = transform(2,0) + 0.000034;
     transform(2,1) = transform(2,1) + 0.000272;
    transform(2,3) = transform(2,3) + 0.000071;
    
    


    //如果匹配成功，有可能形成一次环境特征观测
    if(!success)
    {
        frames_since_last_success++;
        reference.swap(current);
        ROS_WARN("fail");
    }
    else
    {
        frames_since_last_success = 0;
        accumulated_transform = accumulated_transform * transform;
        latest_accumulated_transform = latest_accumulated_transform * transform;

        vis_->trajectory("estimate")->
        color(dvo::visualization::Color::red())
        .add(accumulated_transform);

        vis_->camera("current")->
        color(dvo::visualization::Color::red()).
        update(current->level(0), accumulated_transform).
        show();

        rgbdtools::eigenAffineToXYZRPYd(latest_accumulated_transform, x, y, z, roll, pitch, yaw);

        if (abs(x) > 0.05 || abs(y) > 0.05 || abs(z) > 0.05 || abs(roll) > 0.05 || abs(pitch) > 0.05 || abs(yaw) > 0.05)
        {
	  step++;
	  
	  
	  
	  
	  
	  obs = CObservationBearingRange::Create();
	  obs->validCovariances	= true;
	  obs->sensor_std_range	= max_range;
	  obs->sensedData.clear();
	  obs->timestamp = mrpt::system::getCurrentTime();
	  
	  CPose3D incPose3D = CPose3D(x, y, z, roll, pitch, yaw);
	  
	  CActionRobotMovement3D    act;
	  act.estimationMethod	= CActionRobotMovement3D::emOdometry;
	  act.poseChange.mean = incPose3D;
	  act.poseChange.cov.eye();
	  act.poseChange.cov(0,0) = square(x * actcov00);
	  act.poseChange.cov(1,1) = square(y * actcov11);
	  act.poseChange.cov(2,2) = square(z * actcov22);
	  act.poseChange.cov(3,3) = square(actcov33);
	  act.poseChange.cov(4,4) = square(actcov44);
	  act.poseChange.cov(5,5) = square(actcov55);
	  
	  actions->insert( act );
	  
	  // initialize the RGBDframe
	  frame = rgbdtools::RGBDFrame(rgb_in, depth_in, intr, header);

            // visualize
	  feature_detector_->findFeatures(frame);
	  
	  //计算描述符
	  create_cloud( frame.depth_img, fx, fy, cx, cy, cloud );
	  compute_normals(cloud, normals);
	  
	  //用新的算法计算描述符
	  brand_desc.compute( frame.rgb_img, cloud, normals, frame.keypoints, frame.descriptors );

	  
	  //利用描述符在已经构建的路标中进行匹配	  
	  std::vector<cv::DMatch> filteredMatches;
	  crossCheckMatching(frame.descriptors, fulldescriptors, filteredMatches);
	  for (unsigned int i = 0; i < filteredMatches.size(); i++)
	  {
	    //不稳定特征忽略
	    if (!frame.kp_valid[filteredMatches[i].queryIdx]) continue;
	    
	    const Vector3f& kp_mean = frame.kp_means[filteredMatches[i].queryIdx];
// 	    const Matrix3f& kp_cov  = frame.kp_covariances[filteredMatches[i].queryIdx];
	    
	    //转换成球面坐标
	    aa.x = kp_mean(0, 0);
	    aa.y = kp_mean(1, 0);
	    aa.z = kp_mean(2, 0);
	    
	    //==========================检测匹配（数据关联）=============================
	    CPoint3D thisfeature(aa);
	    CPose3D predic_robotPoseMean3D = CPose3D(robotPose.mean) + incPose3D;     //当前机器人位姿预测
	    CPoint3D global_thisfeature;
	    global_thisfeature = predic_robotPoseMean3D + thisfeature;
	    
	    CRangeBearingKFSLAM::KFArray_FEAT refMean;
	    mapping.getLandmarkMean(filteredMatches[i].trainIdx, refMean);
	    CPoint3D refMean3D(refMean[0],refMean[1], refMean[2] );
	    
	    if (global_thisfeature.distanceTo(refMean3D) > 0.5) continue;         //被认为是错误匹配
//==========================检测匹配（数据关联）=============================

             frame.kp_valid[filteredMatches[i].queryIdx] = false; //老特征路标，置为false避免后面新增该路标
             c2b.sphericalCoordinates( aa, out_range, out_yaw, out_pitch );

	     mrpt::slam::CObservationBearingRange::TMeasurement newMeas;
	     newMeas.landmarkID = filteredMatches[i].trainIdx; //旧路标++landmark_count;
	     newMeas.range = out_range;
	     newMeas.yaw = out_yaw;
	     newMeas.pitch = out_pitch;
	     obs_cov(0,0) = square(obs_z_var * out_range);
	     newMeas.covariance = obs_cov;
	     // Insert:
	     obs->sensedData.push_back( newMeas );
	    
	  }
	  
	  for (unsigned int kp_idx = 0; kp_idx < frame.keypoints.size(); ++kp_idx)
	  {
	    //不稳定或老路标特征忽略
	    if (!frame.kp_valid[kp_idx]) continue;
	    const Vector3f& kp_mean = frame.kp_means[kp_idx];
// 	    const Matrix3f& kp_cov  = frame.kp_covariances[kp_idx];
	    
	    aa.x = kp_mean(0, 0);
	    aa.y = kp_mean(1, 0);
	    aa.z = kp_mean(2, 0);
	    c2b.sphericalCoordinates( aa, out_range, out_yaw, out_pitch );
	    
	    mrpt::slam::CObservationBearingRange::TMeasurement newMeas;	    
	    newMeas.landmarkID = fulldescriptors.rows;
	    newMeas.range = out_range;
	    newMeas.yaw = out_yaw;
	    newMeas.pitch = out_pitch;
	    
	    obs_cov(0,0) = square(obs_z_var * out_range);
	    newMeas.covariance = obs_cov;
	    
	    // Insert:
	    obs->sensedData.push_back( newMeas );
	    fulldescriptors.push_back(frame.descriptors.row(kp_idx));
	    
	  }
	  //   showKeypointImage(frame);
	  //
	  //   publishFeatureCloud(frame);
	  //
	  //   publishFeatureCovariances(frame);
	  
	  observations->push_back(obs);
	  
	  mapping.processActionObservation(actions,observations);
	  
	  // -------------------------------
	  mapping.getCurrentState( robotPose,LMs,LM_IDs,fullState,fullCov );
	  // Get the mean robot pose as 3D:
	  const CPose3D robotPoseMean3D = CPose3D(robotPose.mean);
	  // Build the path:
	  meanPath.push_back( TPose3D(robotPoseMean3D) );
	  
	  //保存Odometry Path
	  rgbdtools::eigenAffineToXYZRPYd(accumulated_transform, x, y, z, roll, pitch, yaw);
	  const CPose3D OdorobotPoseMean3D = CPose3D(x, y, z, roll, pitch, yaw);
	  OdomeanPath.push_back(TPose3D(OdorobotPoseMean3D));
	  
	  
	    
	    
	    
	    

 
            //==========================opengl 显示=============================
            if (show_opengl)
            {
                COpenGLScenePtr   scene3D = COpenGLScene::Create();
                {
                    opengl::CGridPlaneXYPtr grid = opengl::CGridPlaneXY::Create(-1000,1000,-1000,1000,0,5);
                    grid->setColor(0.4,0.4,0.4);
                    scene3D->insert( grid );
                }
                // Robot path:
                {
                    opengl::CSetOfLinesPtr linesPath = opengl::CSetOfLines::Create();
                    linesPath->setColor(1,0,0);
                    TPose3D init_pose;
                    if (!meanPath.empty())
                        init_pose = TPose3D(CPose3D(meanPath[0]));
//                     int path_decim = 0;
                    for (vector<TPose3D>::iterator it=meanPath.begin(); it!=meanPath.end(); ++it)
                    {
                        linesPath->appendLine(init_pose,*it);
                        init_pose = *it;
//                         if (++path_decim>10)
//                         {
//                             path_decim = 0;
//                             mrpt::opengl::CSetOfObjectsPtr xyz = mrpt::opengl::stock_objects::CornerXYZSimple(0.3f,2.0f);
//                             xyz->setPose(CPose3D(*it));
//                             scene3D->insert(xyz);
// 
//                         }

                    }
                    scene3D->insert( linesPath );
                    // finally a big corner for the latest robot pose:
                    {
                        mrpt::opengl::CSetOfObjectsPtr xyz = mrpt::opengl::stock_objects::CornerXYZSimple(0.5,1.0);
                        xyz->setPose(robotPoseMean3D);
                        scene3D->insert(xyz);

                    }
                    
    //画出Odometry路径
                    opengl::CSetOfLinesPtr linesPath1 = opengl::CSetOfLines::Create();
                    linesPath1->setColor(0,1,0);
                    if (!OdomeanPath.empty())
                        init_pose = TPose3D(CPose3D(OdomeanPath[0]));
//                     path_decim = 0;
                    for (vector<TPose3D>::iterator it=OdomeanPath.begin(); it!=OdomeanPath.end(); ++it)
                    {
                        linesPath1->appendLine(init_pose,*it);
                        init_pose = *it;
//                         if (++path_decim>10)
//                         {
//                             path_decim = 0;
//                             mrpt::opengl::CSetOfObjectsPtr xyz = mrpt::opengl::stock_objects::CornerXYZSimple(0.3f,2.0f);
//                             xyz->setPose(CPose3D(*it));
//                             scene3D->insert(xyz);
// 
//                         }

                    }
                    scene3D->insert( linesPath1 );
                    // finally a big corner for the latest robot pose:
                    {
                        mrpt::opengl::CSetOfObjectsPtr xyz = mrpt::opengl::stock_objects::CornerXYZSimple(0.5,1.0);
                        xyz->setPose(OdorobotPoseMean3D);
                        scene3D->insert(xyz);

                    }
                    
    
    
                    
                    
                    
                    
                    
                    
    //画出Odometry路径           end
    
    
                    
                    // The camera pointing to the current robot pose:
                    if (CAMERA_3DSCENE_FOLLOWS_ROBOT)
                    {
                        win3d->setCameraPointingToPoint(robotPoseMean3D.x(),robotPoseMean3D.y(),robotPoseMean3D.z());
                    }
                }



                // Draw latest data association:
                {
                    const CRangeBearingKFSLAM::TDataAssocInfo & da = mapping.getLastDataAssociation();

                    mrpt::opengl::CSetOfLinesPtr lins = mrpt::opengl::CSetOfLines::Create();
                    lins->setLineWidth(1.2);
                    lins->setColor(1,1,1);
                    for (std::map<observation_index_t,prediction_index_t>::const_iterator it=da.results.associations.begin(); it!=da.results.associations.end(); ++it)
                    {
                        const prediction_index_t idxPred = it->second;
                        // This index must match the internal list of features in the map:
                        CRangeBearingKFSLAM::KFArray_FEAT featMean;
                        mapping.getLandmarkMean(idxPred, featMean);

                        TPoint3D featMean3D;


                        featMean3D.x=featMean[0];
                        featMean3D.y=featMean[1];
                        featMean3D.z=featMean[2];

                        // Line: robot -> landmark:
                        lins->appendLine(
                            robotPoseMean3D.x(),robotPoseMean3D.y(),robotPoseMean3D.z(),
                            featMean3D.x,featMean3D.y,featMean3D.z);
                    }
                    scene3D->insert( lins );
                }



                // The current state of KF-SLAM:
                {
                    opengl::CSetOfObjectsPtr  objs = opengl::CSetOfObjects::Create();
                    mapping.getAs3DObject(objs);
                    scene3D->insert( objs );
                }

                if (win3d.present())
                {
                    mrpt::opengl::COpenGLScenePtr &scn = win3d->get3DSceneAndLock();
                    scn = scene3D;

                    // Update text messages:
                    win3d->addTextMessage(
                        0.02,0.02,
                        format("Step %u - Landmarks in the map: %u",(unsigned int)step, (unsigned int)LMs.size() ),
                        TColorf(1,1,1), 0, MRPT_GLUT_BITMAP_HELVETICA_12 );

                    win3d->addTextMessage(
                        0.02,0.06,
                        format(is_pose_3d ?
                               "Estimated pose: (x y z qr qx qy qz) = %s"
                               :
                               "Estimated pose: (x y yaw) = %s"
                               , robotPose.mean.asString().c_str() ),
                        TColorf(1,1,1), 1, MRPT_GLUT_BITMAP_HELVETICA_12 );

//                     static vector<double> estHz_vals;
//                     const double curHz = 1.0/std::max(1e-9,tim_kf_iter);
//                     estHz_vals.push_back(curHz);
//                     if (estHz_vals.size()>50)
//                         estHz_vals.erase(estHz_vals.begin());
//                     const double meanHz = mrpt::math::mean(estHz_vals);


//                     win3d->addTextMessage(
//                         0.02,0.10,
//                         format("Iteration time: %7ss",
//                                mrpt::utils::unitsFormat(tim_kf_iter).c_str()),
//                         TColorf(1,1,1), 2, MRPT_GLUT_BITMAP_HELVETICA_12 );

//                     win3d->addTextMessage(
//                         0.02,0.14,
//                         format("Execution rate: %7sHz",
//                                mrpt::utils::unitsFormat(meanHz).c_str()),
//                         TColorf(1,1,1), 3, MRPT_GLUT_BITMAP_HELVETICA_12 );

                    win3d->unlockAccess3DScene();
                    win3d->repaint();
                }

// Save to file:
//                  CFileGZOutputStream("/home/zhangheng/slam_results"+format("/kf_state_%05u.3Dscene",(unsigned int)step)) << *scene3D;
		 
		 
                /*
                 *	if ( SAVE_3D_SCENES && !(step % SAVE_LOG_FREQUENCY) )
                 *	{
                 *		// Save to file:
                 *		CFileGZOutputStream(OUT_DIR+format("/kf_state_%05u.3Dscene",(unsigned int)step)) << *scene3D;
                 }

                 */

            }//==========================opengl 显示=============================



 
            latest_accumulated_transform.setIdentity();


            //================================EKF-SLAM================================================

        }

    }
    
    actions.clear_unique();
    observations.clear_unique();

    publishTransform(h, accumulated_transform * from_baselink_to_asus.inverse(), "base_link_estimate");


    if(use_dense_tracking_estimate_)
    {
        publishPose(h, accumulated_transform * from_baselink_to_asus.inverse(), "baselink_estimate");
    }
    

    sw_callback.stopAndPrint();
}





void EcjtuRgbdSLAM::showKeypointImage(rgbdtools::RGBDFrame& frame)
{
    cv::namedWindow("Keypoints", CV_WINDOW_NORMAL);
    cv::Mat kp_img(frame.rgb_img.size(), CV_8UC1);
    cv::drawKeypoints(frame.rgb_img, frame.keypoints, kp_img);
    cv::imshow("Keypoints", kp_img);
    cv::waitKey(1);
}


void EcjtuRgbdSLAM::publishFeatureCovariances(rgbdtools::RGBDFrame& frame)
{
    // create markers

    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;

    marker.header.stamp.sec  = frame.header.stamp.sec;
    marker.header.stamp.nsec = frame.header.stamp.nsec;
    marker.header.seq        = frame.header.seq;
    marker.header.frame_id   = frame.header.frame_id;

//   marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.color.r = 0.6;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;
//   marker.scale.x = 0.0025;

    marker.action = visualization_msgs::Marker::ADD;
    marker.ns = "covariances";

    marker.lifetime = ros::Duration();

    for (unsigned int kp_idx = 0; kp_idx < frame.keypoints.size(); ++kp_idx)
    {
        if (!frame.kp_valid[kp_idx]) continue;

        const Vector3f& kp_mean = frame.kp_means[kp_idx];
        const Matrix3f& kp_cov  = frame.kp_covariances[kp_idx];
        marker.id = kp_idx;


        // transform Eigen to OpenCV matrices
        cv::Mat m(3, 1, CV_64F);
        for (int j = 0; j < 3; ++j)
            m.at<double>(j, 0) = kp_mean(j, 0);


        cv::Mat cov(3, 3, CV_64F);
        for (int j = 0; j < 3; ++j)
            for (int i = 0; i < 3; ++i)
                cov.at<double>(j, i) = kp_cov(j, i);

// compute eigenvectors
        cv::Mat evl(1, 3, CV_64F);
        cv::Mat evt(3, 3, CV_64F);
        cv::eigen(cov, evl, evt);


        double mx = m.at<double>(0,0);
        double my = m.at<double>(1,0);
        double mz = m.at<double>(2,0);

        marker.pose.position.x = mx;		//kp_mean(0, 0);
        marker.pose.position.y = my;		//kp_mean(1, 0);
        marker.pose.position.z = mz;		//kp_mean(2, 0);


        Eigen::Matrix3d fm;
        fm.row(0) = Eigen::Vector3d(evt.at<double>(0,0),evt.at<double>(0,1),evt.at<double>(0,2));
        fm.row(1) = Eigen::Vector3d(evt.at<double>(1,0),evt.at<double>(1,1),evt.at<double>(1,2));
        fm.row(2) = Eigen::Vector3d(evt.at<double>(2,0),evt.at<double>(2,1),evt.at<double>(2,2));


        //    m.col(1) = Vector3d(4,5,6);



        Eigen::Affine3d transform(fm);

//Eigen::
// Transform

        tf::Transform tf_transform;

        tf::transformEigenToTF(transform, tf_transform);


        marker.scale.x = 6* sqrt(evl.at<double>(0,0));
        marker.scale.y = 6* sqrt(evl.at<double>(0,1));
        marker.scale.z = 6* sqrt(evl.at<double>(0,2));


        marker.pose.orientation.x = tf_transform.getRotation().getX();
        marker.pose.orientation.y = tf_transform.getRotation().getY();
        marker.pose.orientation.z = tf_transform.getRotation().getZ();



        marker_array.markers.push_back(marker);




    }

    covariances_publisher_.publish(marker_array);
}



void EcjtuRgbdSLAM::publishFeatureCloud(rgbdtools::RGBDFrame& frame)
{
    PointCloudFeature feature_cloud;
    frame.constructFeaturePointCloud(feature_cloud);
    feature_cloud_publisher_.publish(feature_cloud);
}






void EcjtuRgbdSLAM::diagnostics(
    int n_features, int n_valid_features,
    double d_frame, double d_features, double d_reg, double d_total)
{
//   if(save_diagnostics_ && diagnostics_file_ != NULL)
//   {
//     // print to file
//     fprintf(diagnostics_file_, "%d, %2.1f, %d, %3.1f, %d, %4.1f\n",
//       frame_count_,
//       d_frame,
//       n_valid_features, d_features,
//        d_reg,
//       d_total);
//   }
//   if (verbose_)
//   {
//     // print to screen
//     ROS_INFO("[VO %d] %s[%d]: %.1f Reg[%d]:  TOT: %.1f\n",
//       frame_count_,
//       detector_type_.c_str(), n_valid_features, d_features,
//        d_reg,
//       d_total);
//   }

    return;
}



void EcjtuRgbdSLAM::publishTransform(const std_msgs::Header& header, const Eigen::Affine3d& transform, const std::string frame)
{
    static tf::TransformBroadcaster tb;

    tf::StampedTransform tf_transform;
    tf_transform.frame_id_ = "world";
    tf_transform.child_frame_id_ = frame;
    tf_transform.stamp_ = header.stamp;

    tf::transformEigenToTF(transform, tf_transform);

    tb.sendTransform(tf_transform);
}

void EcjtuRgbdSLAM::publishPose(const std_msgs::Header& header, const Eigen::Affine3d& transform, const std::string frame)
{
    if(pose_pub_.getNumSubscribers() == 0) return;

    geometry_msgs::PoseWithCovarianceStampedPtr msg(new geometry_msgs::PoseWithCovarianceStamped);

    static int seq = 1;

    msg->header.seq = seq++;
    msg->header.frame_id = frame;
    msg->header.stamp = header.stamp;

    tf::Transform tmp;

    tf::transformEigenToTF(transform, tmp);
    tf::poseTFToMsg(tmp, msg->pose.pose);

    msg->pose.covariance.assign(0.0);

    pose_pub_.publish(msg);
}














} /* namespace ecjtu_rgbd_slam */
