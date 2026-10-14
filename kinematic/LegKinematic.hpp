#pragma once

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <yaml-cpp/yaml.h>

// body: X 向前、Y 向左、Z 向上。所有矩阵列顺序为 [LF, RF, LB, RB]。
// joint_states 已在发布端完成电机符号和零偏修正，此处不再修正。
class LegKinematic
{
public:
    enum Leg { LF = 0, RF = 1, LB = 2, RB = 3 };
    using LegMatrix = Eigen::Matrix<double, 3, 4>;
    using LegRow = Eigen::Matrix<double, 1, 4>;

    std::string root_dir;
    LegMatrix foot_angle = LegMatrix::Zero();
    LegMatrix foot_angle_vel = LegMatrix::Zero();
    LegMatrix foot_torque = LegMatrix::Zero();
    LegMatrix foot_force = LegMatrix::Zero();
    LegRow foot_force_z = LegRow::Zero();
    LegRow foot_contact = LegRow::Zero();
    LegMatrix wheel_pos_body = LegMatrix::Zero();
    LegMatrix foot_pos_body = LegMatrix::Zero();
    LegMatrix foot_vel_body = LegMatrix::Zero();
    LegMatrix foot_pos_imu = LegMatrix::Zero();
    LegMatrix foot_vel_imu = LegMatrix::Zero();
    // world 系数据由调用者结合机体姿态更新。
    LegMatrix foot_pos_rel = LegMatrix::Zero();
    LegMatrix foot_vel_rel = LegMatrix::Zero();

    // 默认从本头文件所在包读取配置，不使用其他包定义的 ROOT_DIR。
    // 安装或移动头文件后，可显式传入 LegConfig.yaml 的路径。
    explicit LegKinematic(const std::string& config_path = defaultConfigPath())
    {
        const std::size_t separator = config_path.find_last_of("/\\");
        root_dir = separator == std::string::npos ? "" : config_path.substr(0, separator + 1);
        const YAML::Node cfg = YAML::LoadFile(config_path)["njust_v2"];
        L1 = cfg["L1"].as<double>();
        L2 = cfg["L2"].as<double>();
        L3 = cfg["L3"].as<double>();
        wheel_radius = cfg["Wheel_r"].as<double>();
        ox = cfg["ox"].as<double>();
        oy = cfg["oy"].as<double>();
        oz = cfg["oz"].as<double>();
        body_to_imu << cfg["body2imu_x"].as<double>(),
                       cfg["body2imu_y"].as<double>(),
                       cfg["body2imu_z"].as<double>();
        force_threshold = cfg["force_threshold"].as<double>();
        if (!(std::isfinite(L1) && L1 > 0 && std::isfinite(L2) && L2 > 0 &&
              std::isfinite(L3) && L3 > 0 && std::isfinite(wheel_radius) && wheel_radius >= 0 &&
              std::isfinite(ox) && ox >= 0 && std::isfinite(oy) && oy >= 0 &&
              std::isfinite(oz) && body_to_imu.allFinite() &&
              std::isfinite(force_threshold) && force_threshold >= 0))
            throw std::invalid_argument("Invalid njust_v2 leg configuration: " + config_path);
    }

    // 髋坐标系与 body 系轴向一致；返回轮心相对髋关节的位置。
    // 与 X20 calculateFootState_Body 的模型一致，左右腿使用不同 L1 符号。
    Eigen::Vector3d calcEe2H(const Eigen::Vector3d& angle, Leg leg = LF) const
    {
        const double side = sideSign(leg);
        const double s1 = std::sin(angle(0)), c1 = std::cos(angle(0));
        const double reach = L2 * std::cos(angle(1)) + L3 * std::cos(angle(1) + angle(2));
        return Eigen::Vector3d(-L2 * std::sin(angle(1)) - L3 * std::sin(angle(1) + angle(2)),
                               side * L1 * c1 + reach * s1,
                               side * L1 * s1 - reach * c1);
    }

    Eigen::Matrix3d calcJco(double q1, double q2, double q3, Leg leg = LF) const
    {
        const double side = sideSign(leg);
        const double s1 = std::sin(q1), c1 = std::cos(q1);
        const double reach = L2 * std::cos(q2) + L3 * std::cos(q2 + q3);
        const double bend = L2 * std::sin(q2) + L3 * std::sin(q2 + q3);
        Eigen::Matrix3d J;
        J << 0, -reach, -L3 * std::cos(q2 + q3),
             reach * c1 - side * L1 * s1, -bend * s1, -L3 * s1 * std::sin(q2 + q3),
             reach * s1 + side * L1 * c1, bend * c1, L3 * c1 * std::sin(q2 + q3);
        return J;
    }

    Eigen::Vector3d calcVEe2H(const Eigen::Vector3d& angle,
                             const Eigen::Vector3d& omega, Leg leg = LF) const
    {
        return calcJco(angle(0), angle(1), angle(2), leg) * omega;
    }

    // 与参考文件一致：返回 J^(-T) tau；foot_force 保存其相反数（地面对腿的力）。
    // 奇异构型不能可靠估计接触力，返回零并由全身计算取消接触标记。
    Eigen::Vector3d calcFoc(const Eigen::Vector3d& angle,
                           const Eigen::Vector3d& torque, Leg leg = LF) const
    {
        Eigen::FullPivLU<Eigen::Matrix3d> solver(
            calcJco(angle(0), angle(1), angle(2), leg).transpose());
        solver.setThreshold(1e-8);
        if (!solver.isInvertible()) return Eigen::Vector3d::Zero();
        return solver.solve(torque);
    }

    void calculateFootState_Body()
    {
        for (int i = 0; i < 4; ++i)
        {
            const Leg leg = static_cast<Leg>(i);
            const Eigen::Vector3d angle = foot_angle.col(i);
            const double front = (leg == LF || leg == RF) ? 1.0 : -1.0;
            wheel_pos_body.col(i) = calcEe2H(angle, leg) +
                Eigen::Vector3d(front * ox, sideSign(leg) * oy, oz);
            // 水平地面、轮底沿 body -Z 的近似；不包含轮速或机体角速度。
            foot_pos_body.col(i) = wheel_pos_body.col(i) - Eigen::Vector3d(0, 0, wheel_radius);
            foot_pos_imu.col(i) = foot_pos_body.col(i) - body_to_imu;
            foot_vel_body.col(i) = calcVEe2H(angle, foot_angle_vel.col(i), leg);
            foot_vel_imu.col(i) = foot_vel_body.col(i); // body 与 IMU 仅平移，轴向一致
            foot_force.col(i) = -calcFoc(angle, foot_torque.col(i), leg);
            foot_force_z(i) = foot_force(2, i);
            foot_contact(i) = foot_force_z(i) > force_threshold ? 1.0 : 0.0;
        }
    }

    // 接到 joint_states 的回调。优先按名称匹配，允许消息关节重排。
    // 无 name 时仅接受发布端的 12 个关节顺序 [FR, FL, RR, RL]。
    void getNjustInfoFromMsg(const sensor_msgs::JointState::ConstPtr& msg)
    {
        if (!msg) return;
        const std::array<std::string, 4> prefixes = {{"FL", "FR", "RL", "RR"}};
        const std::array<std::string, 3> suffixes = {{"_hip_joint", "_thigh_joint", "_calf_joint"}};
        const std::array<std::size_t, 4> starts = {{3, 0, 9, 6}};
        LegMatrix angles, velocities, torques;
        if (msg->name.empty() && (msg->position.size() != 12 ||
            msg->velocity.size() != 12 || msg->effort.size() != 12))
        {
            ROS_WARN_THROTTLE(1.0, "LegKinematic: unnamed joint_states requires 12 positions, velocities and efforts");
            return;
        }
        for (int leg = 0; leg < 4; ++leg)
        {
            for (int joint = 0; joint < 3; ++joint)
            {
                std::size_t index = starts[leg] + joint;
                if (!msg->name.empty())
                {
                    const std::string name = prefixes[leg] + suffixes[joint];
                    index = msg->name.size();
                    for (std::size_t k = 0; k < msg->name.size(); ++k)
                    {
                        if (msg->name[k] != name) continue;
                        if (index != msg->name.size())
                        {
                            ROS_WARN_THROTTLE(1.0, "LegKinematic: duplicate joint name in joint_states");
                            return;
                        }
                        index = k;
                    }
                }
                if ((!msg->name.empty() && index == msg->name.size()) ||
                    index >= msg->position.size() || index >= msg->velocity.size() || index >= msg->effort.size())
                {
                    ROS_WARN_THROTTLE(1.0, "LegKinematic: joint_states is missing a required leg joint or its data");
                    return;
                }
                angles(joint, leg) = msg->position[index];
                velocities(joint, leg) = msg->velocity[index];
                torques(joint, leg) = msg->effort[index];
            }
        }
        if (!angles.allFinite() || !velocities.allFinite() || !torques.allFinite())
        {
            ROS_WARN_THROTTLE(1.0, "LegKinematic: non-finite joint_states data");
            return;
        }
        foot_angle = angles;
        foot_angle_vel = velocities;
        foot_torque = torques;
        calculateFootState_Body();
    }

    // 保留参考文件回调名称，便于已有调用代码迁移。
    void getX20InfoFromMsg(const sensor_msgs::JointState::ConstPtr& msg)
    {
        getNjustInfoFromMsg(msg);
    }

    static Eigen::Matrix3d skew(const Eigen::Vector3d& a)
    {
        Eigen::Matrix3d result;
        result << 0, -a.z(), a.y(), a.z(), 0, -a.x(), -a.y(), a.x(), 0;
        return result;
    }

    static Eigen::Matrix3d Exp(const Eigen::Vector3d& r)
    {
        const double theta = r.norm();
        if (theta < 1e-12) return Eigen::Matrix3d::Identity();
        return Eigen::AngleAxisd(theta, r / theta).toRotationMatrix();
    }

    static Eigen::Vector3d Log(const Eigen::Matrix3d& rotation)
    {
        const Eigen::AngleAxisd angle_axis(rotation);
        return angle_axis.angle() * angle_axis.axis();
    }

private:
    double L1, L2, L3, wheel_radius;
    double ox, oy, oz, force_threshold;
    Eigen::Vector3d body_to_imu;

    static double sideSign(Leg leg)
    {
        if (leg < LF || leg > RB) throw std::out_of_range("Invalid leg index");
        return (leg == LF || leg == LB) ? 1.0 : -1.0;
    }

    static std::string defaultConfigPath()
    {
        const std::string header = __FILE__;
        return header.substr(0, header.find_last_of("/\\") + 1) + "../config/LegConfig.yaml";
    }
};
