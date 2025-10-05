/*
 * Husky Gazebo Differential Drive Plugin
 *
 * 실제 허스키 하드웨어와 동일한 방식으로 동작:
 * - cmd_vel을 직접 바퀴 속도로 변환
 * - PID 제어 없이 SetVelocity() 직접 호출
 * - ros_control과 동일한 로직
 */

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo
{
  class HuskyDiffDrivePlugin : public ModelPlugin
  {
  public:
    HuskyDiffDrivePlugin() : ModelPlugin()
    {
      // 초기화
      this->wheel_separation_ = 0.5708;  // 기본값 (URDF에서 override 가능)
      this->wheel_diameter_ = 0.3302;
      this->update_rate_ = 50.0;
      this->publish_odom_ = true;
      this->publish_odom_tf_ = true;  // Gazebo ground truth 사용
      this->publish_map_odom_tf_ = false;  // static transform으로 처리
    }

    virtual ~HuskyDiffDrivePlugin()
    {
      this->update_connection_.reset();
      this->queue_.clear();
      this->queue_.disable();
      this->callback_queue_thread_.join();
      this->rosnode_->shutdown();
    }

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      this->model_ = _model;
      this->world_ = _model->GetWorld();

      // ROS 노드 초기화
      if (!ros::isInitialized())
      {
        ROS_FATAL_STREAM("ROS node not initialized");
        return;
      }
      this->rosnode_.reset(new ros::NodeHandle(""));

      // SDF 파라미터 로드
      if (_sdf->HasElement("wheelSeparation"))
        this->wheel_separation_ = _sdf->Get<double>("wheelSeparation");

      if (_sdf->HasElement("wheelDiameter"))
        this->wheel_diameter_ = _sdf->Get<double>("wheelDiameter");

      if (_sdf->HasElement("updateRate"))
        this->update_rate_ = _sdf->Get<double>("updateRate");

      if (_sdf->HasElement("publishOdomTF"))
        this->publish_odom_tf_ = _sdf->Get<bool>("publishOdomTF");

      if (_sdf->HasElement("publishMapOdomTF"))
        this->publish_map_odom_tf_ = _sdf->Get<bool>("publishMapOdomTF");

      // 조인트 가져오기
      std::string left_front_joint_name = "front_left_wheel_joint";
      std::string left_rear_joint_name = "rear_left_wheel_joint";
      std::string right_front_joint_name = "front_right_wheel_joint";
      std::string right_rear_joint_name = "rear_right_wheel_joint";

      if (_sdf->HasElement("leftFrontJoint"))
        left_front_joint_name = _sdf->Get<std::string>("leftFrontJoint");
      if (_sdf->HasElement("leftRearJoint"))
        left_rear_joint_name = _sdf->Get<std::string>("leftRearJoint");
      if (_sdf->HasElement("rightFrontJoint"))
        right_front_joint_name = _sdf->Get<std::string>("rightFrontJoint");
      if (_sdf->HasElement("rightRearJoint"))
        right_rear_joint_name = _sdf->Get<std::string>("rightRearJoint");

      ROS_DEBUG("Loading joints: LF=%s, LR=%s, RF=%s, RR=%s",
                left_front_joint_name.c_str(), left_rear_joint_name.c_str(),
                right_front_joint_name.c_str(), right_rear_joint_name.c_str());

      this->joints_[0] = this->model_->GetJoint(left_front_joint_name);
      this->joints_[1] = this->model_->GetJoint(left_rear_joint_name);
      this->joints_[2] = this->model_->GetJoint(right_front_joint_name);
      this->joints_[3] = this->model_->GetJoint(right_rear_joint_name);

      for (int i = 0; i < 4; i++)
      {
        if (!this->joints_[i])
        {
          ROS_FATAL_STREAM("Joint " << i << " not found!");

          // Print all available joints for debugging
          ROS_ERROR("Available joints in model:");
          auto joints = this->model_->GetJoints();
          for (auto& j : joints)
            ROS_ERROR("  - %s", j->GetName().c_str());

          return;
        }
      }

      // ROS 통신 설정
      std::string command_topic = "husky_velocity_controller/cmd_vel";
      if (_sdf->HasElement("commandTopic"))
        command_topic = _sdf->Get<std::string>("commandTopic");

      std::string odom_topic = "husky_velocity_controller/odom";
      if (_sdf->HasElement("odometryTopic"))
        odom_topic = _sdf->Get<std::string>("odometryTopic");

      this->odom_frame_ = "odom";
      this->base_frame_ = "base_link";
      this->map_frame_ = "map";

      if (_sdf->HasElement("odometryFrame"))
        this->odom_frame_ = _sdf->Get<std::string>("odometryFrame");
      if (_sdf->HasElement("robotBaseFrame"))
        this->base_frame_ = _sdf->Get<std::string>("robotBaseFrame");
      if (_sdf->HasElement("mapFrame"))
        this->map_frame_ = _sdf->Get<std::string>("mapFrame");

      // Subscriber & Publisher
      ros::SubscribeOptions so =
        ros::SubscribeOptions::create<geometry_msgs::Twist>(
          command_topic, 1,
          boost::bind(&HuskyDiffDrivePlugin::OnCmdVel, this, _1),
          ros::VoidPtr(), &this->queue_);
      this->cmd_vel_subscriber_ = this->rosnode_->subscribe(so);

      if (this->publish_odom_)
      {
        this->odom_publisher_ = this->rosnode_->advertise<nav_msgs::Odometry>(odom_topic, 10);
      }

      // 콜백 큐 스레드 시작
      this->callback_queue_thread_ = boost::thread(
        boost::bind(&HuskyDiffDrivePlugin::QueueThread, this));

      // 업데이트 이벤트 연결
      this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&HuskyDiffDrivePlugin::OnUpdate, this));

      this->last_update_time_ = this->world_->SimTime();

      // Odometry 초기화
      this->odom_pose_[0] = 0.0;
      this->odom_pose_[1] = 0.0;
      this->odom_pose_[2] = 0.0;

      ROS_DEBUG("Husky Gazebo Differential Drive Plugin loaded successfully!");
    }

  private:
    void OnCmdVel(const geometry_msgs::Twist::ConstPtr& msg)
    {
      boost::mutex::scoped_lock scoped_lock(this->lock_);
      this->cmd_vel_ = *msg;
    }

    void OnUpdate()
    {
      common::Time current_time = this->world_->SimTime();
      double dt = (current_time - this->last_update_time_).Double();

      if (dt < (1.0 / this->update_rate_))
        return;

      boost::mutex::scoped_lock scoped_lock(this->lock_);

      // cmd_vel을 바퀴 속도로 변환 (실제 허스키와 동일한 방식)
      double linear_vel = this->cmd_vel_.linear.x;
      double angular_vel = this->cmd_vel_.angular.z;

      double wheel_radius = this->wheel_diameter_ / 2.0;

      // Differential drive kinematics
      double left_wheel_vel = (linear_vel - angular_vel * this->wheel_separation_ / 2.0) / wheel_radius;
      double right_wheel_vel = (linear_vel + angular_vel * this->wheel_separation_ / 2.0) / wheel_radius;

      // 바퀴 속도 직접 설정 (PID 없음!)
      this->joints_[0]->SetVelocity(0, left_wheel_vel);   // front left
      this->joints_[1]->SetVelocity(0, left_wheel_vel);   // rear left
      this->joints_[2]->SetVelocity(0, right_wheel_vel);  // front right
      this->joints_[3]->SetVelocity(0, right_wheel_vel);  // rear right

      // Odometry 계산 (encoder 피드백)
      double left_actual_vel = (this->joints_[0]->GetVelocity(0) + this->joints_[1]->GetVelocity(0)) / 2.0;
      double right_actual_vel = (this->joints_[2]->GetVelocity(0) + this->joints_[3]->GetVelocity(0)) / 2.0;

      double actual_linear_vel = (left_actual_vel + right_actual_vel) * wheel_radius / 2.0;
      double actual_angular_vel = (right_actual_vel - left_actual_vel) * wheel_radius / this->wheel_separation_;

      // Gazebo ground truth 위치 사용 (시뮬레이션이므로 오차 없음)
      ignition::math::Pose3d gazebo_pose = this->model_->WorldPose();

      this->odom_pose_[0] = gazebo_pose.Pos().X();
      this->odom_pose_[1] = gazebo_pose.Pos().Y();
      this->odom_pose_[2] = gazebo_pose.Rot().Yaw();

      // Odometry 발행
      if (this->publish_odom_)
      {
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = this->odom_frame_;
        odom.child_frame_id = this->base_frame_;

        odom.pose.pose.position.x = this->odom_pose_[0];
        odom.pose.pose.position.y = this->odom_pose_[1];
        odom.pose.pose.position.z = 0.0;

        tf::Quaternion quat;
        quat.setRPY(0, 0, this->odom_pose_[2]);
        odom.pose.pose.orientation.x = quat.x();
        odom.pose.pose.orientation.y = quat.y();
        odom.pose.pose.orientation.z = quat.z();
        odom.pose.pose.orientation.w = quat.w();

        odom.twist.twist.linear.x = actual_linear_vel;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = actual_angular_vel;

        // Covariance (실제 허스키 control.yaml과 동일)
        odom.pose.covariance[0] = 0.001;
        odom.pose.covariance[7] = 0.001;
        odom.pose.covariance[14] = 0.001;
        odom.pose.covariance[21] = 0.001;
        odom.pose.covariance[28] = 0.001;
        odom.pose.covariance[35] = 0.03;

        odom.twist.covariance[0] = 0.001;
        odom.twist.covariance[7] = 0.001;
        odom.twist.covariance[14] = 0.001;
        odom.twist.covariance[21] = 0.001;
        odom.twist.covariance[28] = 0.001;
        odom.twist.covariance[35] = 0.03;

        this->odom_publisher_.publish(odom);

        // TF 발행 (ground truth 사용)
        if (this->publish_odom_tf_)
        {
          tf::Transform transform;
          transform.setOrigin(tf::Vector3(this->odom_pose_[0], this->odom_pose_[1], 0.0));
          transform.setRotation(quat);
          this->tf_broadcaster_.sendTransform(
            tf::StampedTransform(transform, odom.header.stamp, this->odom_frame_, this->base_frame_));
        }
      }

      this->last_update_time_ = current_time;
    }

    void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosnode_->ok())
      {
        this->queue_.callAvailable(ros::WallDuration(timeout));
      }
    }

  private:
    // Gazebo
    physics::ModelPtr model_;
    physics::WorldPtr world_;
    physics::JointPtr joints_[4];
    event::ConnectionPtr update_connection_;

    // ROS
    boost::shared_ptr<ros::NodeHandle> rosnode_;
    ros::Subscriber cmd_vel_subscriber_;
    ros::Publisher odom_publisher_;
    ros::CallbackQueue queue_;
    boost::thread callback_queue_thread_;
    tf::TransformBroadcaster tf_broadcaster_;

    // 파라미터
    double wheel_separation_;
    double wheel_diameter_;
    double update_rate_;
    bool publish_odom_;
    bool publish_odom_tf_;
    bool publish_map_odom_tf_;
    std::string odom_frame_;
    std::string base_frame_;
    std::string map_frame_;

    // 상태
    geometry_msgs::Twist cmd_vel_;
    double odom_pose_[3];  // x, y, theta
    common::Time last_update_time_;
    boost::mutex lock_;
  };

  GZ_REGISTER_MODEL_PLUGIN(HuskyDiffDrivePlugin)
}
