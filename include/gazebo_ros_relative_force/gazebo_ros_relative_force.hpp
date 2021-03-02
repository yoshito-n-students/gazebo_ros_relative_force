#ifndef GAZEBO_ROS_RELATIVE_FORCE
#define GAZEBO_ROS_RELATIVE_FORCE

#include <functional> // for std::bind()
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <geometry_msgs/Wrench.h>
#include <ros/callback_queue.h>
#include <ros/console.h>
#include <ros/node_handle.h>
#include <ros/spinner.h>
#include <ros/subscriber.h>

namespace gazebo {

class GazeboRosRelativeForce : public ModelPlugin {
public:
  GazeboRosRelativeForce() : spinner_(/* thread_count = */ 1, /* callback_queue = */ &queue_) {
    msg_.force.x = msg_.force.y = msg_.force.z = 0.;
    msg_.torque.x = msg_.torque.y = msg_.torque.z = 0.;
  }

  virtual ~GazeboRosRelativeForce() {
    // stop stuffs in reverse order of Load() (would not be required, but just in case)
    update_connection_.reset();
    spinner_.stop();
    sub_.shutdown();
  }

  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override {
    // find the target link
    const std::string link_name = param(_sdf, "bodyName", "");
    if (link_name.empty()) {
      ROS_FATAL("GazeboRosRelativeForce::Load(): Missing or empty <bodyName>");
      return;
    }
    link_ = _model->GetLink(link_name);
    if (!link) {
      ROS_FATAL_STREAM("GazeboRosRelativeForce::Load(): Cannot find the link '" << link_name
                                                                                << "'");
      return;
    }

    // setup ROS node
    if (!ros::isInitialized()) {
      ROS_FATAL("GazeboRosRelativeForce::Load(): A ROS node for Gazebo has not been initialized");
      return;
    }
    ros::NodeHandle nh(param(_sdf, "robotNamespace", ""));
    nh.setCallbackQueue(&queue_);

    // setup ROS subscriber
    const std::string topic_name = param(_sdf, "topicName", "");
    if (topic_name.empty()) {
      ROS_FATAL("GazeboRosRelativeForce::Load(): Missing or empty <topicName>");
      return;
    }
    sub_ = nh.subscribe(topic_name, 1, &GazeboRosRelativeForce::OnRosMsgReceived, this);

    // start subscribing ROS messages
    spinner_.start();

    // start updating force acting on the link
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&GazeboRosRelativeForce::OnWorldUpdateBegin, this));
  }

protected:
  void OnWorldUpdateBegin() {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    link_->AddRelativeForce(ignition::math::Vector3d(msg_.force.x, msg_.force.y, msg_.force.z));
    link_->AddRelativeTorque(ignition::math::Vector3d(msg_.torque.x, msg_.torque.y, msg_.torque.z));
  }

  void OnRosMsgReceived(const geometry_msgs::Wrench::ConstPtr &_msg) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    msg_ = *_msg;
  }

  static std::string param(sdf::ElementPtr _sdf, const std::string &key,
                           const std::string &default_val) {
    return _sdf->HasElement(key) ? _sdf->GetElement(key)->Get<std::string>() : default_val;
  }

protected:
  // Gazebo stuffs
  physics::LinkPtr link_;
  event::ConnectionPtr update_connection_;

  // Ros stuffs
  ros::Subscriber sub_;
  ros::CallbackQueue queue_;
  ros::AsyncSpinner spinner_;
  geometry_msgs::Wrench msg_;
  std::mutex msg_mutex_;
};
} // namespace gazebo

#endif