#include "husky_velocity_control_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <algorithm>

namespace husky_gazebo_plugins
{

HuskyVelocityControlPlugin::HuskyVelocityControlPlugin()
  : wheel_diameter_(0.3302)
  , max_accel_(5.0)
  , max_speed_(1.0)
  , prev_speed_left_(0.0)
  , prev_speed_right_(0.0)
{
}

HuskyVelocityControlPlugin::~HuskyVelocityControlPlugin()
{
}

bool HuskyVelocityControlPlugin::initSim(
  const std::string& robot_namespace,
  ros::NodeHandle model_nh,
  gazebo::physics::ModelPtr parent_model,
  const urdf::Model *const urdf_model,
  std::vector<transmission_interface::TransmissionInfo> transmissions)
{
  parent_model_ = parent_model;

  // Load parameters
  model_nh.param<double>("wheel_diameter", wheel_diameter_, 0.3302);
  model_nh.param<double>("max_accel", max_accel_, 5.0);
  model_nh.param<double>("max_speed", max_speed_, 1.0);

  // Joint names matching Husky hardware
  std::vector<std::string> joint_names;
  joint_names.push_back("front_left_wheel");
  joint_names.push_back("front_right_wheel");
  joint_names.push_back("rear_left_wheel");
  joint_names.push_back("rear_right_wheel");

  // Initialize joints
  joints_.resize(joint_names.size());
  for (size_t i = 0; i < joint_names.size(); ++i)
  {
    joints_[i].name = joint_names[i];
    joints_[i].gazebo_joint = parent_model_->GetJoint(joint_names[i]);

    if (!joints_[i].gazebo_joint)
    {
      ROS_ERROR_STREAM("Joint " << joint_names[i] << " not found in Gazebo model");
      return false;
    }

    // Register joint state interface
    hardware_interface::JointStateHandle state_handle(
      joint_names[i],
      &joints_[i].position,
      &joints_[i].velocity,
      &joints_[i].effort);
    joint_state_interface_.registerHandle(state_handle);

    // Register velocity command interface
    hardware_interface::JointHandle vel_handle(
      state_handle,
      &joints_[i].velocity_command);
    velocity_joint_interface_.registerHandle(vel_handle);

    ROS_INFO_STREAM("Registered joint: " << joint_names[i]);
  }

  // Register interfaces
  registerInterface(&joint_state_interface_);
  registerInterface(&velocity_joint_interface_);

  // Initialize odometry publisher
  ros::NodeHandle nh;
  odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 50);

  ROS_INFO("Husky Velocity Control Plugin initialized successfully");
  return true;
}

void HuskyVelocityControlPlugin::readSim(ros::Time time, ros::Duration period)
{
  // Read joint states from Gazebo
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    joints_[i].position = joints_[i].gazebo_joint->Position(0);
    joints_[i].velocity = joints_[i].gazebo_joint->GetVelocity(0);
    joints_[i].effort = joints_[i].gazebo_joint->GetForce(0);
  }

  // Get ground truth pose from Gazebo (world frame)
  ignition::math::Pose3d pose = parent_model_->WorldPose();
  ignition::math::Vector3d linear_vel = parent_model_->WorldLinearVel();
  ignition::math::Vector3d angular_vel = parent_model_->WorldAngularVel();

  // Publish TF: odom -> base_link (using ground truth)
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = pose.Pos().X();
  odom_trans.transform.translation.y = pose.Pos().Y();
  odom_trans.transform.translation.z = pose.Pos().Z();

  odom_trans.transform.rotation.x = pose.Rot().X();
  odom_trans.transform.rotation.y = pose.Rot().Y();
  odom_trans.transform.rotation.z = pose.Rot().Z();
  odom_trans.transform.rotation.w = pose.Rot().W();

  tf_broadcaster_.sendTransform(odom_trans);

  // Publish Odometry message
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = time;
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";

  odom_msg.pose.pose.position.x = pose.Pos().X();
  odom_msg.pose.pose.position.y = pose.Pos().Y();
  odom_msg.pose.pose.position.z = pose.Pos().Z();

  odom_msg.pose.pose.orientation.x = pose.Rot().X();
  odom_msg.pose.pose.orientation.y = pose.Rot().Y();
  odom_msg.pose.pose.orientation.z = pose.Rot().Z();
  odom_msg.pose.pose.orientation.w = pose.Rot().W();

  odom_msg.twist.twist.linear.x = linear_vel.X();
  odom_msg.twist.twist.linear.y = linear_vel.Y();
  odom_msg.twist.twist.linear.z = linear_vel.Z();

  odom_msg.twist.twist.angular.x = angular_vel.X();
  odom_msg.twist.twist.angular.y = angular_vel.Y();
  odom_msg.twist.twist.angular.z = angular_vel.Z();

  odom_pub_.publish(odom_msg);
}

void HuskyVelocityControlPlugin::writeSim(ros::Time time, ros::Duration period)
{
  // Get velocity commands for left and right wheels
  // Indices: 0=front_left, 1=front_right, 2=rear_left, 3=rear_right
  double speed_left = joints_[0].velocity_command;   // front_left
  double speed_right = joints_[1].velocity_command;  // front_right

  // Apply velocity and acceleration limits
  limitDifferentialSpeed(speed_left, speed_right);

  // Apply acceleration limits
  double dt = period.toSec();
  if (dt > 0)
  {
    double max_delta_v = max_accel_ * dt;

    // Limit left wheel acceleration
    double delta_left = speed_left - prev_speed_left_;
    if (std::abs(delta_left) > max_delta_v)
    {
      speed_left = prev_speed_left_ + (delta_left > 0 ? max_delta_v : -max_delta_v);
    }

    // Limit right wheel acceleration
    double delta_right = speed_right - prev_speed_right_;
    if (std::abs(delta_right) > max_delta_v)
    {
      speed_right = prev_speed_right_ + (delta_right > 0 ? max_delta_v : -max_delta_v);
    }
  }

  // Store for next iteration
  prev_speed_left_ = speed_left;
  prev_speed_right_ = speed_right;

  // Apply velocities to wheels
  // Left wheels (front and rear)
  joints_[0].gazebo_joint->SetVelocity(0, speed_left);  // front_left
  joints_[2].gazebo_joint->SetVelocity(0, speed_left);  // rear_left

  // Right wheels (front and rear)
  joints_[1].gazebo_joint->SetVelocity(0, speed_right); // front_right
  joints_[3].gazebo_joint->SetVelocity(0, speed_right); // rear_right
}

void HuskyVelocityControlPlugin::limitDifferentialSpeed(double &speed_left, double &speed_right)
{
  // Convert angular velocity to linear velocity
  double linear_left = speed_left * wheel_diameter_ / 2.0;
  double linear_right = speed_right * wheel_diameter_ / 2.0;

  // Find max speed
  double large_speed = std::max(std::abs(linear_left), std::abs(linear_right));

  // Scale if over limit
  if (large_speed > max_speed_)
  {
    double scale = max_speed_ / large_speed;
    speed_left *= scale;
    speed_right *= scale;
  }
}

}  // namespace husky_gazebo_plugins

// Register plugin with pluginlib
PLUGINLIB_EXPORT_CLASS(husky_gazebo_plugins::HuskyVelocityControlPlugin, gazebo_ros_control::RobotHWSim)
