#ifndef HUSKY_VELOCITY_CONTROL_PLUGIN_H
#define HUSKY_VELOCITY_CONTROL_PLUGIN_H

#include <ros/ros.h>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros_control/robot_hw_sim.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

namespace husky_gazebo_plugins
{

class HuskyVelocityControlPlugin : public gazebo_ros_control::RobotHWSim
{
public:
  HuskyVelocityControlPlugin();
  virtual ~HuskyVelocityControlPlugin();

  virtual bool initSim(
    const std::string& robot_namespace,
    ros::NodeHandle model_nh,
    gazebo::physics::ModelPtr parent_model,
    const urdf::Model *const urdf_model,
    std::vector<transmission_interface::TransmissionInfo> transmissions);

  virtual void readSim(ros::Time time, ros::Duration period);

  virtual void writeSim(ros::Time time, ros::Duration period);

private:
  // Joint structure
  struct Joint
  {
    std::string name;
    gazebo::physics::JointPtr gazebo_joint;
    double position;
    double velocity;
    double effort;
    double velocity_command;

    Joint() : position(0), velocity(0), effort(0), velocity_command(0) {}
  };

  std::vector<Joint> joints_;

  // Hardware interfaces
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;

  // Gazebo pointers
  gazebo::physics::ModelPtr parent_model_;

  // ROS parameters
  double wheel_diameter_;
  double max_accel_;
  double max_speed_;

  // Velocity limiting
  void limitDifferentialSpeed(double &speed_left, double &speed_right);

  // Previous velocities for acceleration limiting
  double prev_speed_left_;
  double prev_speed_right_;

  // TF and odometry
  tf::TransformBroadcaster tf_broadcaster_;
  ros::Publisher odom_pub_;
};

}  // namespace husky_gazebo_plugins

#endif  // HUSKY_VELOCITY_CONTROL_PLUGIN_H
