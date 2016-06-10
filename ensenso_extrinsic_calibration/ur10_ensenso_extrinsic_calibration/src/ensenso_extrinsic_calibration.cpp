// Standard headers
#include <string>
#include <fstream>

// ROS headers
#include <ros/ros.h>
#include <ros/service.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>

// PCL headers
#include <pcl/common/colors.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ensenso_grabber.h>

// Service descriptions
#include <ur10_ensenso_extrinsic_calibration/PerformEnsensoCalibration.h>
#include <ur10_ensenso_extrinsic_calibration/TestEnsensoCalibration.h>

// TODO: Things you MUST tweak if you change the robot description
/** Name of the move_group used to move the robot during calibration */
const std::string move_group_name("ensenso_n10");
/** Name of the TCP that should be used to compute the trajectories */
const std::string tcp_name("/ensenso_tcp");
// TODO end

/** Pair of PCL images */
typedef std::pair<pcl::PCLImage, pcl::PCLImage> PairOfImages;

/** PCL point object. */
typedef pcl::PointXYZ PointXYZ;

/** PCL Point cloud object */
typedef pcl::PointCloud<PointXYZ> PointCloudXYZ;

/** Pointer to the node */
boost::shared_ptr<ros::NodeHandle> node;

/** Pointer to the move group */
boost::shared_ptr<move_group_interface::MoveGroup> group;

/** Calibration status publisher */
boost::shared_ptr<ros::Publisher> status_pub;

/** PCL Ensenso object pointer */
pcl::EnsensoGrabber::Ptr ensenso;
/** Left Ensenso image publisher */
boost::shared_ptr<ros::Publisher> l_image_pub;
/** Right Ensenso image publisher */
boost::shared_ptr<ros::Publisher> r_image_pub;

/** Calibration test cloud publisher */
boost::shared_ptr<ros::Publisher> calib_test_cloud_pub;

/** Publisher for the sphere marker */
boost::shared_ptr<ros::Publisher> sphere_pub;
/** Publisher for the calibration plate marker */
boost::shared_ptr<ros::Publisher> plate_pub;
/** Sphere marker representing the movement of the robot */
visualization_msgs::Marker sphere_marker;
/** Marker representing the calibration plate */
visualization_msgs::Marker plate_marker;

/**
 * Process & display grabber images by publishing into 2 ROS topics
 * @param[in] images Ensenso left and right images */
void ensensoGrabberCallback(const boost::shared_ptr<PairOfImages>& images)
{
  unsigned char *l_image_array = reinterpret_cast<unsigned char *>(&images->first.data[0]);
  unsigned char *r_image_array = reinterpret_cast<unsigned char *>(&images->second.data[0]);

  int type(CV_8UC1);
  std::string encoding("mono8");
  if (images->first.encoding == "CV_8UC3")
  {
    type = CV_8UC3;
    encoding = "bgr8";
  }
  cv::Mat l_image(images->first.height, images->first.width, type, l_image_array);
  cv::Mat r_image(images->first.height, images->first.width, type, r_image_array);

  l_image_pub->publish(cv_bridge::CvImage(std_msgs::Header(), encoding, l_image).toImageMsg());
  r_image_pub->publish(cv_bridge::CvImage(std_msgs::Header(), encoding, r_image).toImageMsg());
}

/**
 * Generate a random pose in the upper hemisphere calculated thanks to @c obj_origin and @c tool_origin
 * @param[in] obj_origin the object origin (we turn around this object)
 * @param[in] tool_origin the robot origin at the time we were looking at the object
 * @return A random 3D pose for the robot looking into the direction of @c tool_origin and at the same distance as
 * in the original pose */
Eigen::Affine3d generateRandomHemispherePose(const Eigen::Vector3d &obj_origin, const Eigen::Vector3d &tool_origin)
{
  // Generate random point on upper hemisphere :
  Eigen::Vector3d point;
  point[2] = obj_origin[2];
  double radius = (obj_origin - tool_origin).norm();

  while (point[2] < obj_origin[2] + 0.9 * radius)
  {
    double phy = rand() % 161 + 10;
    double teta = rand() % 360;

    point[0] = radius * cos(phy) * cos(teta) + obj_origin[0];
    point[1] = radius * cos(phy) * sin(teta) + obj_origin[1];
    point[2] = radius * sin(phy) + obj_origin[2];
  }

  // Z axis = obj_origin -> point
  Eigen::Vector3d z_axis;
  z_axis = obj_origin - point;
  z_axis.normalize();

  // Y axis = Generate random point on plane represented by Z vector
  Eigen::Vector3d y_axis;
  y_axis << z_axis[1], z_axis[0], z_axis[2];
  y_axis = y_axis.cross(z_axis); // Temporary y axis

  y_axis = y_axis + point;
  y_axis.normalize();

  // X axis = Cross product
  Eigen::Vector3d x_axis(y_axis.cross(z_axis));
  x_axis.normalize();

  // Recompute Y axis
  y_axis = (z_axis.cross(x_axis));
  y_axis.normalize();

  // Assign rotations and translation
  Eigen::Affine3d pose(Eigen::Affine3d::Identity());
  pose.matrix().col(0) << x_axis, 0;
  pose.matrix().col(1) << y_axis, 0;
  pose.matrix().col(2) << z_axis, 0;
  pose.translation() = point;
  return pose;
}

/**
 * Extrinsic calibration service, this is triggered when the button "Start calibration" is hit in the RViz panel
 * @param req[in]
 * @param res[out]
 * @return Always true
 */
bool performCalibration(ur10_ensenso_extrinsic_calibration::PerformEnsensoCalibration::Request &req,
                        ur10_ensenso_extrinsic_calibration::PerformEnsensoCalibration::Response &res)
{
  // Get parameters from the message
  const unsigned int number_of_poses(req.number_of_poses);
  const float grid_spacing(req.grid_spacing);
  const float calibration_plate_distance(req.calibration_plate_distance);
  const bool store_to_eeprom(req.store_to_EEPROM);

  // Setup Ensenso
  ensenso->stop();
  ensenso->clearCalibrationPatternBuffer(); // In case a previous calibration was launched!
  ensenso->initExtrinsicCalibration(grid_spacing);
  ensenso->start();

  // Get initial (current) pose where the pattern is visible
  tf::TransformListener listener;
  listener.waitForTransform("/base", tcp_name, ros::Time::now(), ros::Duration(1.0));
  tf::StampedTransform transform_stamp;
  Eigen::Affine3d initial_pose;

  std_msgs::String status;
  try
  {
    listener.lookupTransform("/base", tcp_name, ros::Time(0), transform_stamp);
    transformTFToEigen(transform_stamp, initial_pose);
  }
  catch (tf::TransformException &ex)
  {
    status.data = ex.what();
    status_pub->publish(status);
    res.return_message = status.data;
    res.return_status = false;
    return true;
  }

  // The robot should be vertical to the calibration plate at this moment
  const Eigen::Vector3d tool_origin(initial_pose.translation()[0],
                                    initial_pose.translation()[1],
                                    initial_pose.translation()[2]),
                        obj_origin(tool_origin[0],
                                   tool_origin[1],
                                   tool_origin[2] - (calibration_plate_distance / 1000.0)); // Focal distance of the Ensenso

                                   // Marker showing the calibration plate location and sphere of motion
  sphere_marker.header.stamp = ros::Time::now();
  // Marker pose
  sphere_marker.pose.position.x = obj_origin[0];
  sphere_marker.pose.position.y = obj_origin[1];
  sphere_marker.pose.position.z = obj_origin[2];
  // Marker scale
  sphere_marker.scale.x = 2 * (obj_origin - tool_origin).norm();
  sphere_marker.scale.y = 2 * (obj_origin - tool_origin).norm();
  sphere_marker.scale.z = 2 * (obj_origin - tool_origin).norm();

  plate_marker.header.stamp = ros::Time::now();
  // Marker pose
  plate_marker.pose.position.x = obj_origin[0];
  plate_marker.pose.position.y = obj_origin[1];
  plate_marker.pose.position.z = obj_origin[2];

  // Publish markers
  while (sphere_pub->getNumSubscribers() < 1 && plate_pub->getNumSubscribers() < 1)
  {
    ROS_WARN_STREAM_ONCE("No subscriber to the markers: Please create the 2 subscribers in RViz");
    sleep(1);
  }
  sphere_pub->publish(sphere_marker);
  plate_pub->publish(plate_marker);

  // Initialize trajectory
  moveit_msgs::ExecuteKnownTrajectory srv;
  srv.request.wait_for_execution = true;
  ros::ServiceClient executeKnownTrajectoryServiceClient = node->serviceClient<moveit_msgs::ExecuteKnownTrajectory>(
      "/execute_kinematic_path");
  std::vector<geometry_msgs::Pose> way_points_msg(1);

  // Capture calibration data from the sensor, move the robot and repeat until enough data is acquired
  std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > robot_poses;
  unsigned int failed_count = 0; // Counts the number of consecutive failures, abort if too much
  while (node->ok() && robot_poses.size() < number_of_poses)
  {
    if (failed_count > 3)
    {
      status.data = "Failed more than 3 times, aborting!";
      status_pub->publish(status);
      res.return_message = status.data;
      res.return_status = false;

      // Go back to initial pose (if possible)
      tf::poseEigenToMsg(initial_pose, way_points_msg[0]);
      listener.waitForTransform("/base", tcp_name, ros::Time::now(), ros::Duration(1.5));

      if (group->computeCartesianPath(way_points_msg, 0.05, 0, srv.request.trajectory) > 0.5)
      {
        for (unsigned i = 0; i < srv.request.trajectory.joint_trajectory.points.size(); ++i)
        {
          for (unsigned j = 0; j < 6; ++j)
            srv.request.trajectory.joint_trajectory.points[i].velocities.push_back(0.5);
        }
        executeKnownTrajectoryServiceClient.call(srv);
      }
      return true;
    }

    ensenso->start();
    tf::poseEigenToMsg(generateRandomHemispherePose(obj_origin, tool_origin), way_points_msg[0]);
    listener.waitForTransform("/base", tcp_name, ros::Time::now(), ros::Duration(1.5));
    if (group->computeCartesianPath(way_points_msg, 0.05, 0, srv.request.trajectory) < 0.95)
    {
      ROS_WARN_STREAM("Cannot reach pose: skipping to next pose");
      failed_count++;
      continue;
    }

    std::stringstream ss;
    ss << robot_poses.size() << " of " << number_of_poses << " data acquired";
    status.data = ss.str();
    status_pub->publish(status);

    for (unsigned i = 0; i < srv.request.trajectory.joint_trajectory.points.size(); ++i)
    {
      for (unsigned j = 0; j < 6; ++j)
        srv.request.trajectory.joint_trajectory.points[i].velocities.push_back(0.5);
    }
    executeKnownTrajectoryServiceClient.call(srv);
    ensenso->stop();
    sleep(1); // Sleep time: the robot might oscillate little bit after moving
    if (ensenso->captureCalibrationPattern() == -1)
    {
      ROS_WARN_STREAM("Failed to capture calibration pattern: skipping to next pose");
      failed_count++;
      continue;
    }

    try // Collect robot pose in tool tool0 frame
    {
      ros::Time now(ros::Time::now());
      listener.waitForTransform("/base", "/tool0", now, ros::Duration(1.5));
      listener.lookupTransform("/base", "/tool0", now, transform_stamp);
      Eigen::Affine3d robot_pose;
      tf::transformTFToEigen(transform_stamp, robot_pose);
      robot_poses.push_back(robot_pose);
    }
    catch (tf::TransformException &ex)
    {
      status.data = ex.what();
      status_pub->publish(status);
      res.return_message = status.data;
      res.return_status = false;
      return true;
    }
    failed_count = 0;
  }

  sleep(1);

  // Go back to initial pose (if possible)
  status.data = "Moving robot to initial pose";
  status_pub->publish(status);
  tf::poseEigenToMsg(initial_pose, way_points_msg[0]);
  listener.waitForTransform("/base", tcp_name, ros::Time::now(), ros::Duration(1.5));
  if (group->computeCartesianPath(way_points_msg, 0.05, 0, srv.request.trajectory) > 0.95)
  {
    for (unsigned i = 0; i < srv.request.trajectory.joint_trajectory.points.size(); ++i)
    {
      for (unsigned j = 0; j < 6; ++j)
        srv.request.trajectory.joint_trajectory.points[i].velocities.push_back(0.5);
    }
    executeKnownTrajectoryServiceClient.call(srv);
  }

  // Compute calibration matrix
  // TODO: Add guess calibration support
  status.data = "Computing calibration matrix...";
  status_pub->publish(status);
  std::string result;

  if (!ensenso->computeCalibrationMatrix(robot_poses, result, "Moving", "Hand"))
  {
    status.data = "Failed to compute calibration";
    status_pub->publish(status);
    res.return_status = false;
    res.return_message = result; // JSON string
    return true;
  }

  // Store calibration matrix into EEPROM if required
  if (store_to_eeprom)
  {
    ensenso->storeEEPROMExtrinsicCalibration();
    status.data = "Calibration computation successful and stored into the EEPROM";
  }
  else
    status.data = "Calibration computation successful";

  ensenso->start();
  status_pub->publish(status);
  res.return_status = true;
  res.return_message = result;
  return true;
}

/**
 * Reset calibration, this is triggered when the button "Reset calibration" is hit in the RViz panel
 * @param msg[in] The message content is not used here
 */
void resetCalibration(const std_msgs::String::ConstPtr& msg)
{
  ensenso->stop();
  std_msgs::String status;
  if (!ensenso->clearEEPROMExtrinsicCalibration())
  {
    status.data = "Could not reset extrinsic calibration!";
    status_pub->publish(status);
    return;
  }
  ensenso->start();
  status.data = "Extrinsic calibration reset into EEPROM";
  status_pub->publish(status);
}

/**
 * Extrinsic calibration test service, this is triggered when the button "Test calibration" is hit in the RViz panel
 * @param req[in]
 * @param res[out]
 * @return Always true
 */
bool testCalibration(ur10_ensenso_extrinsic_calibration::TestEnsensoCalibration::Request &req,
                     ur10_ensenso_extrinsic_calibration::TestEnsensoCalibration::Response &res)
{
  // Get parameters from the message
  const unsigned int number_of_poses(req.number_of_poses);
  const float calibration_plate_distance(req.calibration_plate_distance);
  ensenso->stop();

  // Delete calibration plate marker
  plate_marker.header.stamp = ros::Time::now();
  plate_marker.action = visualization_msgs::Marker::DELETE;
  plate_pub->publish(plate_marker);

  // Get initial (current) pose where the pattern is visible
  tf::TransformListener listener;
  listener.waitForTransform("/base", tcp_name, ros::Time::now(), ros::Duration(1.0));
  tf::StampedTransform transform_stamp;
  Eigen::Affine3d initial_pose;

  std_msgs::String status;
  try
  {
    listener.lookupTransform("/base", tcp_name, ros::Time(0), transform_stamp);
    transformTFToEigen(transform_stamp, initial_pose);
  }
  catch (tf::TransformException &ex)
  {
    status.data = ex.what();
    status_pub->publish(status);
    res.return_message = status.data;
    res.return_status = false;
    return true;
  }

  // The robot should be vertical to the calibration plate at this moment
  const Eigen::Vector3d tool_origin(initial_pose.translation()[0],
                                    initial_pose.translation()[1],
                                    initial_pose.translation()[2]),
                        obj_origin(tool_origin[0],
                                   tool_origin[1],
                                   tool_origin[2] - calibration_plate_distance / 1000.0); // Focal distance of the Ensenso

                                   // Marker showing the sphere of motion
  sphere_marker.header.stamp = ros::Time::now();
  // Marker pose
  sphere_marker.pose.position.x = obj_origin[0];
  sphere_marker.pose.position.y = obj_origin[1];
  sphere_marker.pose.position.z = obj_origin[2];
  sphere_marker.pose.orientation.x = 0.0;
  sphere_marker.pose.orientation.y = 0.0;
  sphere_marker.pose.orientation.z = 0.0;
  sphere_marker.pose.orientation.w = 1.0;
  // Marker scale
  sphere_marker.scale.x = 2 * (obj_origin - tool_origin).norm();
  sphere_marker.scale.y = 2 * (obj_origin - tool_origin).norm();
  sphere_marker.scale.z = 2 * (obj_origin - tool_origin).norm();
  // Marker color
  sphere_marker.color.r = 0.6f;
  sphere_marker.color.g = 0.6f;
  sphere_marker.color.b = 0.6f;
  sphere_marker.color.a = 0.5;

  // Publish markers
  while (sphere_pub->getNumSubscribers() < 1)
  {
    ROS_WARN_STREAM_ONCE("No subscriber to the marker: Please create the subscriber in RViz");
    sleep(1);
  }
  sphere_pub->publish(sphere_marker);

  // Initialize trajectory
  moveit_msgs::ExecuteKnownTrajectory srv;
  srv.request.wait_for_execution = true;
  ros::ServiceClient executeKnownTrajectoryServiceClient = node->serviceClient<moveit_msgs::ExecuteKnownTrajectory>(
      "/execute_kinematic_path");
  std::vector<geometry_msgs::Pose> way_points_msg(1);

  // Move the robot and capture point clouds, then send it
  std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > robot_poses;
  std::vector<PointCloudXYZ::Ptr> clouds;
  unsigned int failed_count = 0; // Counts the number of consecutive failures, abort if too much
  while (node->ok() && robot_poses.size() < number_of_poses)
  {
    if (failed_count > 3)
    {
      status.data = "Failed more than 3 times, aborting!";
      status_pub->publish(status);
      res.return_message = status.data;
      res.return_status = false;
      return true;
    }

    tf::poseEigenToMsg(generateRandomHemispherePose(obj_origin, tool_origin), way_points_msg[0]);
    listener.waitForTransform("/base", tcp_name, ros::Time::now(), ros::Duration(1.5));
    if (group->computeCartesianPath(way_points_msg, 0.05, 0, srv.request.trajectory) < 0.95)
    {
      ROS_WARN_STREAM("Cannot reach pose: skipping to next pose");
      failed_count++;
      continue;
    }

    for (unsigned i = 0; i < srv.request.trajectory.joint_trajectory.points.size(); ++i)
    {
      for (unsigned j = 0; j < 6; ++j)
        srv.request.trajectory.joint_trajectory.points[i].velocities.push_back(0.5);
    }
    executeKnownTrajectoryServiceClient.call(srv);

    try // Collect robot pose in tool0 frame
    {
      ros::Time now(ros::Time::now());
      listener.waitForTransform("/base", "/tool0", now, ros::Duration(1.5));
      listener.lookupTransform("/base", "/tool0", now, transform_stamp);
      Eigen::Affine3d robot_pose;
      tf::transformTFToEigen(transform_stamp, robot_pose);
      robot_poses.push_back(robot_pose);
    }
    catch (tf::TransformException &ex)
    {
      status.data = ex.what();
      status_pub->publish(status);
      res.return_message = status.data;
      res.return_status = false;
      return true;
    }

    sleep(1); // Sleep time: the robot might oscillate little bit after moving
    PointCloudXYZ::Ptr cloud(new PointCloudXYZ);
    clouds.push_back(cloud);
    if (!ensenso->grabSingleCloud(*clouds[robot_poses.size() - 1]))
    {
      status.data = "Could not capture point cloud";
      status_pub->publish(status);
      res.return_message = status.data;
      res.return_status = false;
      return true;
    }

    std::stringstream ss;
    ss << robot_poses.size() << " of " << number_of_poses << " point clouds acquired";
    status.data = ss.str();
    status_pub->publish(status);
    failed_count = 0;
  }

  if (robot_poses.size() != clouds.size())
  {
    status.data = "Number of clouds != number of poses. Aborting!";
    status_pub->publish(status);
    std::stringstream ss;
    ss << "robot_poses.size() = " << robot_poses.size() << " | clouds.size() = " << clouds.size();
    res.return_message = ss.str();
    res.return_status = false;
    return true;
  }

  // Go back to initial pose (if possible)
  tf::poseEigenToMsg(initial_pose, way_points_msg[0]);
  listener.waitForTransform("/base", tcp_name, ros::Time::now(), ros::Duration(1.5));
  if (group->computeCartesianPath(way_points_msg, 0.05, 0, srv.request.trajectory) > 0.95)
  {
    for (unsigned i = 0; i < srv.request.trajectory.joint_trajectory.points.size(); ++i)
    {
      for (unsigned j = 0; j < 6; ++j)
        srv.request.trajectory.joint_trajectory.points[i].velocities.push_back(0.5);
    }
    status.data = "Moving robot to initial pose";
    status_pub->publish(status);
    executeKnownTrajectoryServiceClient.call(srv);
  }

  // Transform / color point clouds and stack them into one point cloud
  // FIXME: This code will EASILY crash on a machine with available low memory (or many data are asked from the user)
  // The vector must be contiguous in the memory, so the system might be stuck because no "big" memory segment is available!
  pcl::GlasbeyLUT colors;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr clouds_stacked(new pcl::PointCloud<pcl::PointXYZRGB>);

  for (unsigned int i = 0; i < clouds.size(); ++i)
  {
    pcl::PointCloud<pcl::PointXYZRGB> cloud_xyzrgb;
    copyPointCloud(*clouds[i], cloud_xyzrgb);
    pcl::transformPointCloud(cloud_xyzrgb, cloud_xyzrgb, robot_poses[i]);

    for (pcl::PointCloud<pcl::PointXYZRGB>::iterator cloud_it(cloud_xyzrgb.begin()); cloud_it != cloud_xyzrgb.end(); ++cloud_it)
      cloud_it->rgb = colors.at(i).rgb;

    *clouds_stacked += cloud_xyzrgb;
  }

  // Publish point clouds
  clouds_stacked->header.frame_id = "base";
  calib_test_cloud_pub->publish(clouds_stacked);

  res.return_status = true;
  res.return_message = ""; // No error message
  return true;
}

/**
 * The main function
 * @param[in] argc
 * @param[in] argv
 * @return Exit status */
int main(int argc, char **argv)
{
  srand(time(0)); // Random number generator seed

  ros::init(argc, argv, "ensenso_extrinsic_calibration");
  node.reset(new ros::NodeHandle);
  // Subscribe to service
  ros::Subscriber reset_calib_sub = node->subscribe("reset_ensenso_calibration", 1, resetCalibration);
  // Advertise services
  ros::ServiceServer perform_calib_service = node->advertiseService("perform_ensenso_calibration", performCalibration);
  ros::ServiceServer test_calib_service = node->advertiseService("test_ensenso_calibration", testCalibration);
  // Create publisher for the calibration status
  status_pub.reset(new ros::Publisher);
  *status_pub = node->advertise<std_msgs::String>("ensenso_calibration_status", 1);

  // Create publishers for the calibration plate and sphere motion visualization messages
  sphere_pub.reset(new ros::Publisher);
  plate_pub.reset(new ros::Publisher);
  *sphere_pub = node->advertise<visualization_msgs::Marker>("sphere_motion", 1, true);
  *plate_pub = node->advertise<visualization_msgs::Marker>("calibration_plate", 1, true);

  // Parameter the markers
  sphere_marker.header.frame_id = "/base";
  sphere_marker.ns = "basic_shapes";
  sphere_marker.id = 0;
  sphere_marker.type = visualization_msgs::Marker::SPHERE;
  sphere_marker.action = visualization_msgs::Marker::ADD;
  // Marker pose
  sphere_marker.pose.position.x = 0;
  sphere_marker.pose.position.y = 0;
  sphere_marker.pose.position.z = 0;
  sphere_marker.pose.orientation.x = 0.0;
  sphere_marker.pose.orientation.y = 0.0;
  sphere_marker.pose.orientation.z = 0.0;
  sphere_marker.pose.orientation.w = 1.0;
  // Marker scale
  sphere_marker.scale.x = 1;
  sphere_marker.scale.y = 1;
  sphere_marker.scale.z = 1;
  // Marker color
  sphere_marker.color.r = 0.6f;
  sphere_marker.color.g = 0.6f;
  sphere_marker.color.b = 0.6f;
  sphere_marker.color.a = 0.5;
  sphere_marker.lifetime = ros::Duration();

  plate_marker.header.frame_id = "/base";
  plate_marker.ns = "basic_shapes";
  plate_marker.id = 0;
  plate_marker.type = visualization_msgs::Marker::CUBE;
  plate_marker.action = visualization_msgs::Marker::ADD;
  // Marker pose
  plate_marker.pose.position.x = 0;
  plate_marker.pose.position.y = 0;
  plate_marker.pose.position.z = 0;
  plate_marker.pose.orientation.x = 0.0;
  plate_marker.pose.orientation.y = 0.0;
  plate_marker.pose.orientation.z = 0.0;
  plate_marker.pose.orientation.w = 1.0;
  // Marker scale
  plate_marker.scale.x = 0.05;
  plate_marker.scale.y = 0.05;
  plate_marker.scale.z = 0.001;
  // Marker color
  plate_marker.color.r = 0.8f;
  plate_marker.color.g = 0.3f;
  plate_marker.color.b = 0.3f;
  plate_marker.color.a = 1.0;
  plate_marker.lifetime = ros::Duration();

  // Initialize move group
  group.reset(new move_group_interface::MoveGroup(move_group_name));
  group->setPoseReferenceFrame("/base");
  group->setPlanningTime(2);

  // Initialize Ensenso
  ensenso.reset(new pcl::EnsensoGrabber);
  ensenso->openDevice(0);
  ensenso->openTcpPort();
  ensenso->configureCapture(true, true, 1, 0.32, true, 1, false, false, false, 10, false); // Disable front light projector, enable IR led light

  l_image_pub.reset(new ros::Publisher);
  r_image_pub.reset(new ros::Publisher);
  *l_image_pub = node->advertise<sensor_msgs::Image>("ensenso_l_image", 2);
  *r_image_pub = node->advertise<sensor_msgs::Image>("ensenso_r_image", 2);

  calib_test_cloud_pub.reset(new ros::Publisher);
  *calib_test_cloud_pub = node->advertise<sensor_msgs::PointCloud2>("calib_test_cloud", 2, true); // Latched

  // Register Ensenso callback and start it
  boost::function<void(const boost::shared_ptr<PairOfImages>&)> f = boost::bind(&ensensoGrabberCallback, _1);
  ensenso->registerCallback(f);
  ensenso->start();

  // Spin
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while (node->ok())
  {
  }

  ensenso->closeTcpPort();
  ensenso->closeDevice();
  return 0;
}
