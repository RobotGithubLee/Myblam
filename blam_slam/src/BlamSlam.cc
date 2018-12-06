/*
 * Copyright (c) 2016, The Regents of the University of California (Regents).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Please contact the author(s) of this library if you have any questions.
 * Authors: Erik Nelson            ( eanelson@eecs.berkeley.edu )
 */

#include <blam_slam/BlamSlam.h>
#include <geometry_utils/Transform3.h>
#include <parameter_utils/ParameterUtils.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>

namespace pu = parameter_utils;
namespace gu = geometry_utils;

BlamSlam::BlamSlam()
    : estimate_update_rate_(0.0), 
    visualization_update_rate_(0.0),
    mapMsgWithoutNan(new PointCloud()),
    is_Initialize_Location(false),
    is_Initialize_Locating(false),
    is_Regenerating_Map(false),
    is_Regenerated_Map(false),
    integrated_map(new PointCloud)
    {}

BlamSlam::~BlamSlam() {}

bool BlamSlam::Initialize(const ros::NodeHandle &n, bool from_log)
{
  name_ = ros::names::append(n.getNamespace(), "BlamSlam");

  if (!filter_.Initialize(n))
  {
    ROS_ERROR("%s: Failed to initialize point cloud filter.", name_.c_str());
    return false;
  }

  if (!odometry_.Initialize(n))
  {
    ROS_ERROR("%s: Failed to initialize point cloud odometry.", name_.c_str());
    return false;
  }

  if (!loop_closure_.Initialize(n))
  {
    ROS_ERROR("%s: Failed to initialize laser loop closure.", name_.c_str());
    return false;
  }

  if (!localization_.Initialize(n))
  {
    ROS_ERROR("%s: Failed to initialize localization.", name_.c_str());
    return false;
  }

  if (!mapper_.Initialize(n))
  {
    ROS_ERROR("%s: Failed to initialize mapper.", name_.c_str());
    return false;
  }

  if (!LoadParameters(n))
  {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n, from_log))
  {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }
  
  if(!LoadMap())
  {
    ROS_ERROR("%s: Failed to load map.", name_.c_str());
    return false;
  }

  return true;
}

bool BlamSlam::LoadParameters(const ros::NodeHandle &n)
{
  // Load update rates.
  if (!pu::Get("rate/estimate", estimate_update_rate_))
    return false;
  if (!pu::Get("rate/visualization", visualization_update_rate_))
    return false;

  // Load frame ids.
  if (!pu::Get("frame_id/fixed", fixed_frame_id_))
    return false;
  if (!pu::Get("frame_id/base", base_frame_id_))
    return false;
  if (!pu::Get("MapOrLocation/is_Building_Map", is_Building_Map))
   return false;
  if (!pu::Get("mapPath", mapPath))
   return false;
  if (!pu::Get("goalPath", goalPath))
   return false;
  return true;
}

bool BlamSlam::RegisterCallbacks(const ros::NodeHandle &n, bool from_log)
{
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  visualization_update_timer_ = nl.createTimer(
      visualization_update_rate_, &BlamSlam::VisualizationTimerCallback, this);

  if (from_log)
    return RegisterLogCallbacks(n);
  else
    return RegisterOnlineCallbacks(n);
}

bool BlamSlam::RegisterLogCallbacks(const ros::NodeHandle &n)
{
  ROS_INFO("%s: Registering log callbacks.", name_.c_str());
  return CreatePublishers(n);
}

bool BlamSlam::RegisterOnlineCallbacks(const ros::NodeHandle &n)
{
  ROS_INFO("%s: Registering online callbacks.", name_.c_str());

  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  estimate_update_timer_ = nl.createTimer(
      estimate_update_rate_, &BlamSlam::EstimateTimerCallback, this);

  pcld_sub_ = nl.subscribe("/rslidar_points", 1, &BlamSlam::PointCloudCallback, this);
  map_pcld_sub_=nl.subscribe("/velodyne_cloud_registered", 10, &BlamSlam::MapPointCloudCallback, this);
  //Service of close map
  _serviceCloseMap = nl.advertiseService("/closeMapService", &BlamSlam::closeMapCallback, this);
  _serviceStartClosure = nl.advertiseService("/startClosure", &BlamSlam::startClosureCallback, this);
  _serviceSaveEstimatePath = nl.advertiseService("/saveEstimatePath", &BlamSlam::saveEstimatePathCallback, this);
  _serviceSaveMap = nl.advertiseService("/saveMap", &BlamSlam::saveMapCallback, this);
  _serviceExtendedMap = nl.advertiseService("/extendedMap", &BlamSlam::extendedMapCallback, this);
  _serviceInitializeLocation = nl.advertiseService("/initializeLocation", &BlamSlam::InitializeLocationCallback, this);

  return CreatePublishers(n);
}

bool BlamSlam::CreatePublishers(const ros::NodeHandle &n)
{
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  base_frame_pcld_pub_ =
      nl.advertise<PointCloud>("base_frame_point_cloud", 10, false);

  return true;
}

void BlamSlam::PointCloudCallback(const PointCloud::ConstPtr &msg)
{
  PointCloud::Ptr msgWithoutNan(new PointCloud());
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*msg,*msgWithoutNan,indices);
 
  if(msgWithoutNan->points.size()<100)
  {
   return;
  }
  synchronizer_.AddPCLPointCloudMessage(msgWithoutNan);
}

void BlamSlam::MapPointCloudCallback(const PointCloud::ConstPtr &msg)
{
  std::vector<int> indices;
  mapMsgWithoutNan->clear();
  pcl::removeNaNFromPointCloud(*msg,*mapMsgWithoutNan,indices);
 
}

bool BlamSlam::startClosureCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  loop_closure_.setFristLaserLoopkey();
  return true;
}

bool BlamSlam::saveEstimatePathCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  localization_.SaveEstimatePath(goalPath);
  return true;
}

bool BlamSlam::saveMapCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  // We found one - regenerate the 3D map.
  PointCloud::Ptr regenerated_map(new PointCloud);
  loop_closure_.GetMaximumLikelihoodPoints(regenerated_map.get());
  *integrated_map += *regenerated_map;
  pcl::io::savePCDFileASCII (mapPath, *integrated_map);//保存pcd
  return true;
}


bool BlamSlam::extendedMapCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  //Inital IASM for next loop closure 
  loop_closure_.InitialzeIsam();
  is_Building_Map = true;
  is_Regenerated_Map = true;

   // We copy one - regenerate the 3D map.
  PointCloud::Ptr regenerated_map(new PointCloud);
  //pcl::io::loadPCDFile (mapPath, *regenerated_map);
  mapper_.GetMapData(regenerated_map.get());
  *integrated_map += *regenerated_map;
  
  return true;
}

bool BlamSlam::closeMapCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) //
{
  is_Regenerating_Map = true;
  if(!loop_closure_.LoopClosures())
  {
    is_Regenerating_Map = false;
    return false;
  }
  // We found one - regenerate the 3D map.
  PointCloud::Ptr regenerated_map(new PointCloud);
  loop_closure_.GetMaximumLikelihoodPoints(regenerated_map.get());
  *integrated_map += *regenerated_map;
  pcl::io::savePCDFileASCII (mapPath, *integrated_map);//保存pcd
  mapper_.Reset();
  PointCloud::Ptr unused(new PointCloud);
  mapper_.InsertPoints(regenerated_map, unused.get());
 
  // Also reset the robot's estimated position.
  localization_.SetIntegratedEstimate(loop_closure_.GetLastPose());
  loop_closure_.SavePathEstimate(goalPath);
     
  // Visualize the pose graph and current loop closure radius.
  loop_closure_.PublishPoseGraph();
  
  //Inital IASM for next loop closure 
  loop_closure_.InitialzeIsam();
  is_Regenerated_Map = true;
  is_Regenerating_Map = false;

  return true;  
}

bool BlamSlam::InitializeLocationCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  is_Initialize_Location = true;
  return true;
}



bool BlamSlam::LoadMap()
{
  if(!is_Building_Map)
  {
    ROS_INFO("load map... ");
    PointCloud::Ptr regenerated_map(new PointCloud);
    pcl::io::loadPCDFile (mapPath, *regenerated_map);
    mapper_.Reset();
    PointCloud::Ptr unused(new PointCloud);
    mapper_.InsertPoints(regenerated_map, unused.get());
    ROS_INFO("load map over!!!");
  }
  return true;
}

void BlamSlam::EstimateTimerCallback(const ros::TimerEvent &ev)
{
  // Sort all messages accumulated since the last estimate update.
  synchronizer_.SortMessages();

  // Iterate through sensor messages, passing to update functions.
  MeasurementSynchronizer::sensor_type type;
  unsigned int index = 0;
  while (synchronizer_.GetNextMessage(&type, &index))
  {
	//ROS_INFO("GetPCLPointCloudMessage size=%d",synchronizer_.GetMessageSize());
    switch (type)
    {

    // Point cloud messages.
    case MeasurementSynchronizer::PCL_POINTCLOUD:
    {
      const MeasurementSynchronizer::Message<PointCloud>::ConstPtr &m =
          synchronizer_.GetPCLPointCloudMessage(index);

      if(is_Building_Map)
      {
        if(!is_Regenerating_Map)
          ProcessPointCloudMessage(m->msg);
      }
      else if(is_Initialize_Location)
      {
        if(!is_Initialize_Locating)
        {
          is_Initialize_Locating = true;
          InitializeLocation(m->msg);   
        }        
      }
      else
      {
        LocalizationBaseMap(m->msg);
      } 
      break;
    }

    // Unhandled sensor messages.
    default:
    {
      ROS_WARN("%s: Unhandled measurement type (%s).", name_.c_str(),
               MeasurementSynchronizer::GetTypeString(type).c_str());
      break;
    }
    }
  }

  // Remove processed messages from the synchronizer.
  synchronizer_.ClearMessages();
}

void BlamSlam::VisualizationTimerCallback(const ros::TimerEvent &ev)
{
  mapper_.PublishMap();
}




void BlamSlam::LocalizationBaseMap(const PointCloud::ConstPtr &msg)
{
// Filter the incoming point cloud message.
  PointCloud::Ptr msg_filtered(new PointCloud);
  filter_.Filter(msg, msg_filtered);

  // Update odometry by performing ICP.
  if (!odometry_.UpdateEstimate(*msg_filtered))
  {
    return;
  }

  // Containers.
  PointCloud::Ptr msg_transformed(new PointCloud);
  PointCloud::Ptr msg_neighbors(new PointCloud);
  PointCloud::Ptr msg_base(new PointCloud);
  PointCloud::Ptr msg_fixed(new PointCloud);

  // Transform the incoming point cloud to the best estimate of the base frame.
  localization_.MotionUpdate(odometry_.GetIncrementalEstimate());
  localization_.TransformPointsToFixedFrame(*msg_filtered,
                                            msg_transformed.get());

  // Get approximate nearest neighbors from the map.
  mapper_.ApproxNearestNeighbors(*msg_transformed, msg_neighbors.get());

  // Transform those nearest neighbors back into sensor frame to perform ICP.
  localization_.TransformPointsToSensorFrame(*msg_neighbors, msg_neighbors.get());

  // Localize to the map. Localization will output a pointcloud aligned in the
  // sensor frame.
  localization_.MeasurementUpdate(msg_filtered, msg_neighbors, msg_base.get());

  localization_.PublishPath();

  // Publish the incoming point cloud message from the base frame.
  if (base_frame_pcld_pub_.getNumSubscribers() != 0)
  {
    PointCloud base_frame_pcld = *msg;
    base_frame_pcld.header.frame_id = base_frame_id_;
    base_frame_pcld_pub_.publish(base_frame_pcld);
  }

}

void BlamSlam::ProcessPointCloudMessage(const PointCloud::ConstPtr &msg)
{

  // Filter the incoming point cloud message.
  PointCloud::Ptr msg_filtered(new PointCloud);
  filter_.Filter(msg, msg_filtered);

  // Update odometry by performing ICP.
  if (!odometry_.UpdateEstimate(*msg_filtered))
  {
    // First update ever.
    PointCloud::Ptr unused(new PointCloud);
    mapper_.InsertPoints(msg_filtered, unused.get());
    loop_closure_.AddKeyScanPair(0, msg);
    is_Regenerated_Map = false;
    return;
  }

  if(is_Regenerated_Map)
  {
    loop_closure_.AddKeyScanPair(0, msg);
    is_Regenerated_Map = false;
    return;
  }

  // Containers.
  PointCloud::Ptr msg_transformed(new PointCloud);
  PointCloud::Ptr msg_neighbors(new PointCloud);
  PointCloud::Ptr msg_base(new PointCloud);
  PointCloud::Ptr msg_fixed(new PointCloud);

  // Transform the incoming point cloud to the best estimate of the base frame.
  localization_.MotionUpdate(odometry_.GetIncrementalEstimate());
  localization_.TransformPointsToFixedFrame(*msg_filtered,
                                            msg_transformed.get());

  // Get approximate nearest neighbors from the map.
  mapper_.ApproxNearestNeighbors(*msg_transformed, msg_neighbors.get());

  // Transform those nearest neighbors back into sensor frame to perform ICP.
  localization_.TransformPointsToSensorFrame(*msg_neighbors, msg_neighbors.get());

  // Localize to the map. Localization will output a pointcloud aligned in the
  // sensor frame.
  localization_.MeasurementUpdate(msg_filtered, msg_neighbors, msg_base.get());

  // Check for new loop closures.
  bool new_keyframe;
  if (HandleLoopClosures(msg, &new_keyframe))
  {
    // We found one - regenerate the 3D map.
    PointCloud::Ptr regenerated_map(new PointCloud);
    loop_closure_.GetMaximumLikelihoodPoints(regenerated_map.get());

    mapper_.Reset();
    PointCloud::Ptr unused(new PointCloud);
    mapper_.InsertPoints(regenerated_map, unused.get());

    // Also reset the robot's estimated position.
    localization_.SetIntegratedEstimate(loop_closure_.GetLastPose());
  }
  else
  {
    // No new loop closures - but was there a new key frame? If so, add new
    // points to the map.
    if (new_keyframe)
    {
      localization_.MotionUpdate(gu::Transform3::Identity());
      localization_.TransformPointsToFixedFrame(*msg, msg_fixed.get());
      PointCloud::Ptr unused(new PointCloud);
      mapper_.InsertPoints(msg_fixed, unused.get());
    }
  }

  // Visualize the pose graph and current loop closure radius.
  loop_closure_.PublishPoseGraph();

  // Publish the incoming point cloud message from the base frame.
  if (base_frame_pcld_pub_.getNumSubscribers() != 0)
  {
    PointCloud base_frame_pcld = *msg;
    base_frame_pcld.header.frame_id = base_frame_id_;
    base_frame_pcld_pub_.publish(base_frame_pcld);
  }
}

bool BlamSlam::HandleLoopClosures(const PointCloud::ConstPtr &scan,
                                  bool *new_keyframe)
{
  if (new_keyframe == NULL)
  {
    ROS_ERROR("%s: Output boolean for new keyframe is null.", name_.c_str());
    return false;
  }

  // Add the new pose to the pose graph.
  unsigned int pose_key;
  gu::MatrixNxNBase<double, 6> covariance;
  covariance.Zeros();
  for (int i = 0; i < 3; ++i)
    covariance(i, i) = 0.01;
  for (int i = 3; i < 6; ++i)
    covariance(i, i) = 0.004;

  const ros::Time stamp = pcl_conversions::fromPCL(scan->header.stamp);
  if (!loop_closure_.AddBetweenFactor(localization_.GetIncrementalEstimate(),
                                      covariance, stamp, &pose_key))
  {
    return false;
  }
  *new_keyframe = true;

  if (!loop_closure_.AddKeyScanPair(pose_key, scan))
  {
    return false;
  }

  std::vector<unsigned int> closure_keys;
  if (!loop_closure_.FindLoopClosures(pose_key, &closure_keys))
  {
    return false;
  }

  for (const auto &closure_key : closure_keys)
  {
    ROS_INFO("%s: Closed loop between poses %u and %u.", name_.c_str(),
             pose_key, closure_key);
  }
  return true;
}

bool BlamSlam::InitializeLocation(const PointCloud::ConstPtr &msg)
{
  std::fstream inStream;
  int num_skip = 10;
  int i = 0;
  double estimate_Score = 100.0;
  gu::Transform3 integrated_estimate_;
  
  //Filter the incoming point cloud message.
  PointCloud::Ptr msg_filtered(new PointCloud);
  filter_.Filter(msg, msg_filtered);

  inStream.open(goalPath,std::ios::in);
  if(!inStream.is_open())
  {
    ROS_INFO("open goal.txt false!!!");  
    is_Initialize_Locating = false;
    return false;     
  }

  while(!inStream.eof())
  {
   geometry_msgs::Point P,rot;
   inStream>>P.x>>P.y>>P.z>>rot.x>>rot.y>>rot.z;
   i++;
   if(i >= num_skip)
   {    
    gu::Transform3 pose;
    pose.translation = gu::Vec3(P.x, P.y, P.z);
    pose.rotation =  gu::Vec3(rot.x, rot.y, rot.z);
     // Containers.
    PointCloud::Ptr msg_transformed(new PointCloud);
    PointCloud::Ptr msg_neighbors(new PointCloud);

    // Transform the incoming point cloud to the best estimate of the base frame.
    localization_.SetIntegratedEstimate(pose);
    localization_.TransformPointsToFixedFrame(*msg_filtered,
                                            msg_transformed.get());
    // Get approximate nearest neighbors from the map.
    mapper_.ApproxNearestNeighbors(*msg_transformed, msg_neighbors.get());

    // Transform those nearest neighbors back into sensor frame to perform ICP.
    localization_.TransformPointsToSensorFrame(*msg_neighbors, msg_neighbors.get());

    // Localize to the map. Localization will output a pointcloud aligned in the
    // sensor frame.
    gu::Transform3 integrated_estimate;
    double Score = localization_.InitilaizeIntegratedEstimate(msg_filtered, msg_neighbors, &integrated_estimate);
     ROS_INFO("Score = %f",Score); 
    if(estimate_Score > Score)
    {
      integrated_estimate_ = integrated_estimate;
      estimate_Score = Score;
    }
    i=0;
   }
  }
  localization_.SetIntegratedEstimate(integrated_estimate_);
  ROS_INFO("1111integrated_estimate_ x=%.3f,y=%.3f,z=%.3f",integrated_estimate_.translation.X(),integrated_estimate_.translation.Y(),integrated_estimate_.translation.Z());
  ROS_INFO("1111integrated_estimate_ Roll=%.3f,Pitch=%.3f,Yaw=%.3f",integrated_estimate_.rotation.Roll(),integrated_estimate_.rotation.Pitch(),integrated_estimate_.rotation.Yaw());
  
  is_Initialize_Locating = false;
  is_Initialize_Location = false;

  return true;
}
