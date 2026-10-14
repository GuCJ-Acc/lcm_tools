#!/usr/bin/env python3
"""打印轮关节反馈；旋转方向按 URDF 轴的右手定则解释。"""
import math

import rospy
from sensor_msgs.msg import JointState


class WheelStatePrinter:
    WHEELS = (
        ('FR', '右前', 'FR_wheel_joint', '+Y', 1),
        ('FL', '左前', 'FL_wheel_joint', '-Y', -1),
        ('RR', '右后', 'RR_wheel_joint', '+Y', 1),
        ('RL', '左后', 'RL_wheel_joint', '-Y', -1),
    )

    def __init__(self):
        self.topic = rospy.get_param('~joint_topic', 'joint_states_full')
        self.print_rate = float(rospy.get_param('~print_rate', 5.0))
        self.threshold = float(rospy.get_param('~velocity_threshold', 0.01))
        if not math.isfinite(self.print_rate) or self.print_rate <= 0:
            raise ValueError('print_rate 必须为有限正数')
        if not math.isfinite(self.threshold) or self.threshold < 0:
            raise ValueError('velocity_threshold 必须为有限非负数')
        self.last_print = None
        self.subscriber = rospy.Subscriber(
            self.topic, JointState, self.callback, queue_size=1)
        rospy.loginfo('订阅 %s；打印频率 %.1f Hz；角度 rad，速度 rad/s',
                      self.topic, self.print_rate)
        rospy.loginfo('正/负转遵循 URDF 关节轴右手定则；前进/后退推断假设轮子接地、机身水平且腿部静止。')

    def callback(self, msg):
        # 用接收时刻限频；ROS 时间回退后立即恢复打印。
        now = rospy.Time.now().to_sec()
        if (self.last_print is not None and now >= self.last_print
                and now - self.last_print < 1.0 / self.print_rate):
            return
        self.last_print = now
        indices = {name: index for index, name in enumerate(msg.name)}
        lines = ['\n轮端反馈 stamp={:.6f} | topic={}'.format(
            msg.header.stamp.to_sec(), self.topic),
            '轮子       URDF轴  角度(rad)     角度(deg)     速度(rad/s)   方向']
        for label, chinese, name, axis, axis_sign in self.WHEELS:
            index = indices.get(name)
            if index is None:
                lines.append('{}({})  缺少关节 {}'.format(label, chinese, name))
                continue
            angle = msg.position[index] if index < len(msg.position) else None
            velocity = msg.velocity[index] if index < len(msg.velocity) else None
            angle_valid = angle is not None and math.isfinite(angle)
            velocity_valid = velocity is not None and math.isfinite(velocity)
            angle_text = '{:+.6f}'.format(angle) if angle_valid else '无效/缺失'
            degrees_text = '{:+.3f}'.format(math.degrees(angle)) if angle_valid else '无效/缺失'
            velocity_text = '{:+.6f}'.format(velocity) if velocity_valid else '无效/缺失'
            if not velocity_valid:
                direction = '无法判断'
            elif abs(velocity) <= self.threshold:
                direction = '停止/低于阈值'
            else:
                rotation = '正转' if velocity > 0 else '负转'
                rolling = '前进' if velocity * axis_sign > 0 else '后退'
                direction = '{}；推断{}'.format(rotation, rolling)
            lines.append('{}({})  {:>3}  {:>12}  {:>12}  {:>12}  {}'.format(
                label, chinese, axis, angle_text, degrees_text, velocity_text, direction))
        print('\n'.join(lines), flush=True)


def main():
    rospy.init_node('print_wheel_states')
    try:
        printer = WheelStatePrinter()
        rospy.spin()
    except (ValueError, TypeError) as error:
        rospy.logfatal('参数错误：%s', error)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
