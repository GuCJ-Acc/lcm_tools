#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Geometry>
#include <tinyxml.h>
#include <array>
#include <deque>
#include <map>
#include <sstream>
#include <stdexcept>
#include <algorithm>

using V = Eigen::Vector3d;
using Q = Eigen::Quaterniond;
struct Joint { V offset, axis; Eigen::Matrix3d rotation; };
struct ImuSample { ros::Time stamp; V omega; Q attitude; bool valid; };

class WheelLegOdometry {
  ros::NodeHandle nh_, pnh_{"~"};
  ros::Subscriber joints_sub_, imu_sub_;
  ros::Publisher pub_;
  tf::TransformBroadcaster broadcaster_;
  std::array<std::array<Joint, 4>, 4> legs_;
  std::deque<ImuSample> imu_;
  std::vector<double> signs_;
  std::string odom_frame_, base_frame_;
  double radius_, timeout_, max_dt_, residual_, pose_var_, twist_var_;
  int min_contacts_;
  bool publish_tf_, use_orientation_, initialized_ = false;
  bool yaw_initialized_ = false;
  ros::Time last_stamp_;
  Q attitude_ = Q::Identity();
  double yaw_origin_ = 0;
  V position_ = V::Zero(), last_velocity_ = V::Zero();
  static V vector(const char* text) {
    V v; std::istringstream stream(text ? text : "");
    if (!(stream >> v.x() >> v.y() >> v.z()) || !v.allFinite())
      throw std::runtime_error("Invalid URDF vector");
    return v;
  }
  void loadModel(const std::string& path) {
    TiXmlDocument doc(path);
    if (!doc.LoadFile()) throw std::runtime_error("Cannot read URDF: " + path);
    std::map<std::string, Joint> model;
    auto root = doc.RootElement();
    if (!root) throw std::runtime_error("Empty URDF");
    for (auto e = root->FirstChildElement("joint"); e; e = e->NextSiblingElement("joint")) {
      Joint j{V::Zero(), V::UnitY(), Eigen::Matrix3d::Identity()};
      if (auto o = e->FirstChildElement("origin")) {
        if (o->Attribute("xyz")) j.offset = vector(o->Attribute("xyz"));
        if (o->Attribute("rpy")) {
          V r = vector(o->Attribute("rpy"));
          j.rotation = (Eigen::AngleAxisd(r.z(), V::UnitZ()) *
                        Eigen::AngleAxisd(r.y(), V::UnitY()) *
                        Eigen::AngleAxisd(r.x(), V::UnitX())).toRotationMatrix();
        }
      }
      if (auto a = e->FirstChildElement("axis")) j.axis = vector(a->Attribute("xyz"));
      if (j.axis.norm() < 1e-9) throw std::runtime_error("Zero joint axis");
      j.axis.normalize();
      if (e->Attribute("name")) model[e->Attribute("name")] = j;
    }
    const std::array<std::string, 4> names{{"FR", "FL", "RR", "RL"}};
    const std::array<std::string, 4> suffix{{"_hip_joint", "_thigh_joint", "_calf_joint", "_wheel_joint"}};
    for (int i = 0; i < 4; ++i)
      for (int k = 0; k < 4; ++k) legs_[i][k] = model.at(names[i] + suffix[k]);
  }
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    ImuSample s;
    s.stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    s.omega = V(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    s.attitude = Q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    s.valid = msg->orientation_covariance[0] >= 0 && s.attitude.coeffs().allFinite() && s.attitude.norm() > 1e-6;
    if (!s.omega.allFinite() || msg->angular_velocity_covariance[0] < 0) return;
    if (s.valid) s.attitude.normalize();
    if (!imu_.empty() && s.stamp < imu_.back().stamp) imu_.clear();
    imu_.push_back(s);
    while (imu_.size() > 1000) imu_.pop_front();
  }
  void jointCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    // Use the newest IMU sample at or before the joint stamp (no future data).
    auto it = std::find_if(imu_.rbegin(), imu_.rend(), [&](const ImuSample& s) { return s.stamp <= stamp; });
    if (it == imu_.rend() || (stamp - it->stamp).toSec() > timeout_) {
      ROS_WARN_THROTTLE(2, "No time-aligned imu_data; odometry paused");
      initialized_ = false; return;
    }
    const ImuSample& imu = *it;
    double dt = initialized_ ? (stamp - last_stamp_).toSec() : 0;
    if (initialized_ && dt <= 0) return;
    if (dt > max_dt_) { initialized_ = false; dt = 0; }
    Q next = attitude_;
    if (use_orientation_ && imu.valid) {
      if (!yaw_initialized_) {
        Eigen::Matrix3d r = imu.attitude.toRotationMatrix();
        yaw_origin_ = std::atan2(r(1,0), r(0,0));
        yaw_initialized_ = true;
      }
      next = Q(Eigen::AngleAxisd(-yaw_origin_, V::UnitZ())) * imu.attitude;
    } else {
      double angle = imu.omega.norm() * dt;
      if (angle > 1e-12) next = attitude_ * Q(Eigen::AngleAxisd(angle, imu.omega.normalized()));
    }
    next.normalize();
    std::map<std::string, size_t> indices;
    for (size_t i = 0; i < msg->name.size(); ++i) indices[msg->name[i]] = i;
    std::vector<V> candidates;
    const std::array<std::string, 4> prefixes{{"FR", "FL", "RR", "RL"}};
    for (int leg = 0; leg < 4; ++leg) {
      double q[4], dq[4]; bool valid = true;
      const std::array<std::string, 4> suffix{{"_hip_joint", "_thigh_joint", "_calf_joint", "_wheel_joint"}};
      for (int k = 0; k < 4; ++k) {
        auto index = indices.find(prefixes[leg] + suffix[k]);
        if (index == indices.end() || index->second >= msg->position.size() || index->second >= msg->velocity.size()) { valid = false; break; }
        q[k] = msg->position[index->second]; dq[k] = msg->velocity[index->second];
        if (!std::isfinite(q[k]) || !std::isfinite(dq[k])) valid = false;
      }
      if (!valid) continue;
      V center = V::Zero(); Eigen::Matrix3d r = Eigen::Matrix3d::Identity();
      std::array<V, 3> origins, axes;
      for (int k = 0; k < 4; ++k) {
        const Joint& j = legs_[leg][k];
        center += r * j.offset; r = r * j.rotation;
        if (k < 3) {
          origins[k] = center; axes[k] = r * j.axis;
          r = r * Eigen::AngleAxisd(q[k], j.axis).toRotationMatrix();
        }
      }
      V axle = r * legs_[leg][3].axis;
      V down = next.conjugate() * V(0, 0, -1);
      // Contact radius lies in the wheel plane, towards gravity.
      V radial = down - axle * axle.dot(down);
      if (radial.norm() < 1e-6) continue;
      radial = radius_ * radial.normalized();
      V contact = center + radial;
      V relative = V::Zero();
      // Include leg-induced wheel rotation as well as encoder wheel rotation.
      for (int k = 0; k < 3; ++k) relative += axes[k].cross(contact - origins[k]) * dq[k];
      relative += axle.cross(radial) * dq[3] * signs_[leg];
      // No slip: v_body + omega_body x contact + v_relative = 0.
      candidates.push_back(-relative - imu.omega.cross(contact));
    }
    if (static_cast<int>(candidates.size()) < min_contacts_) { initialized_ = false; return; }
    V median;
    for (int k = 0; k < 3; ++k) {
      std::vector<double> values;
      for (const auto& v : candidates) values.push_back(v[k]);
      std::sort(values.begin(), values.end());
      size_t n = values.size(); median[k] = (values[n/2] + values[(n-1)/2]) * 0.5;
    }
    V velocity = V::Zero(); int count = 0;
    for (const auto& v : candidates) if ((v - median).norm() <= residual_) { velocity += v; ++count; }
    if (count < min_contacts_) {
      ROS_WARN_THROTTLE(2, "Wheel velocity constraints disagree; odometry paused");
      initialized_ = false; return;
    }
    velocity /= count;
    V world_velocity = next * velocity;
    if (initialized_) position_ += 0.5 * (last_velocity_ + world_velocity) * dt;
    last_velocity_ = world_velocity; last_stamp_ = stamp; attitude_ = next; initialized_ = true;
    nav_msgs::Odometry out;
    out.header.stamp = stamp; out.header.frame_id = odom_frame_; out.child_frame_id = base_frame_;
    out.pose.pose.position.x = position_.x(); out.pose.pose.position.y = position_.y(); out.pose.pose.position.z = position_.z();
    out.pose.pose.orientation.w = next.w(); out.pose.pose.orientation.x = next.x();
    out.pose.pose.orientation.y = next.y(); out.pose.pose.orientation.z = next.z();
    out.twist.twist.linear.x = velocity.x(); out.twist.twist.linear.y = velocity.y(); out.twist.twist.linear.z = velocity.z();
    out.twist.twist.angular.x = imu.omega.x(); out.twist.twist.angular.y = imu.omega.y(); out.twist.twist.angular.z = imu.omega.z();
    for (int k = 0; k < 6; ++k) { out.pose.covariance[k*7] = pose_var_; out.twist.covariance[k*7] = twist_var_; }
    pub_.publish(out);
    if (publish_tf_) broadcaster_.sendTransform(tf::StampedTransform(
      tf::Transform(tf::Quaternion(next.x(), next.y(), next.z(), next.w()), tf::Vector3(position_.x(), position_.y(), position_.z())), stamp, odom_frame_, base_frame_));
  }
public:
  WheelLegOdometry() {
    std::string model, joint_topic, imu_topic, odom_topic;
    if (!pnh_.getParam("urdf_file", model)) throw std::runtime_error("urdf_file is required; use the launch file");
    loadModel(model);
    std::string leg_ns; pnh_.param<std::string>("leg_config_namespace", leg_ns, "/njust_v2");
    if (!nh_.getParam(leg_ns + "/Wheel_r", radius_) || !std::isfinite(radius_) || radius_ <= 0)
      throw std::runtime_error("Load LegConfig.yaml: valid Wheel_r required");
    pnh_.param<std::string>("joint_topic", joint_topic, "joint_states_full");
    pnh_.param<std::string>("imu_topic", imu_topic, "imu_data");
    pnh_.param<std::string>("odom_topic", odom_topic, "kinematic_odom");
    pnh_.param<std::string>("odom_frame", odom_frame_, "odom_kinematic");
    pnh_.param<std::string>("base_frame", base_frame_, "trunk");
    pnh_.param("imu_timeout", timeout_, 0.05); pnh_.param("max_dt", max_dt_, 0.1);
    pnh_.param("velocity_residual_threshold", residual_, 0.5);
    pnh_.param("min_contact_wheels", min_contacts_, 2);
    pnh_.param("publish_tf", publish_tf_, false); pnh_.param("use_imu_orientation", use_orientation_, false);
    pnh_.param("pose_variance", pose_var_, 0.05); pnh_.param("twist_variance", twist_var_, 0.02);
    if (!pnh_.getParam("wheel_velocity_signs", signs_)) signs_ = {1, 1, 1, 1};
    if (signs_.size() != 4 || timeout_ <= 0 || max_dt_ <= 0 || residual_ <= 0 || min_contacts_ < 1 || min_contacts_ > 4 || pose_var_ < 0 || twist_var_ < 0)
      throw std::runtime_error("Invalid odometry configuration");
    for (double sign : signs_) if (sign != 1 && sign != -1) throw std::runtime_error("Wheel signs must be +1 or -1");
    pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic, 10);
    imu_sub_ = nh_.subscribe(imu_topic, 200, &WheelLegOdometry::imuCallback, this);
    joints_sub_ = nh_.subscribe(joint_topic, 20, &WheelLegOdometry::jointCallback, this);
    ROS_INFO("Wheel-leg odometry ready: %s", odom_topic.c_str());
  }
};
int main(int argc, char** argv) {
  ros::init(argc, argv, "wheel_leg_odometry");
  try { WheelLegOdometry node; ros::spin(); }
  catch (const std::exception& e) { ROS_FATAL("%s", e.what()); return 1; }
  return 0;
}
