#! /usr/bin/python
import serial
import struct
import rospy
from sensor_msgs.msg import JointState


## rad from -6.2 to 6.2
def dm_data_to_rad(data):
    x_min = -12.5
    x_max = 12.5
    bits = 16
    span = x_max - x_min
    offset = x_min
    return data * span / ((1 << bits) - 1) + offset
def dji_data_to_rad(data):
    return data / 8192.0 * 6.5


def publish_joint_state(pub, joint_values):
    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    joint_state.position = joint_values
    pub.publish(joint_state)

if __name__ == '__main__':
    ## init ros
    rospy.init_node('custom_contoller_node', anonymous=True)
    rate = rospy.Rate(10)
    pub = rospy.Publisher('/joint_set_states', JointState, queue_size=10)

    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    print(f"serial opened: {ser.portstr}")

    while not rospy.is_shutdown():
        data = ser.read(12)
        joint_values = list(range(6))
        if len(data) == 12:
            data = struct.unpack('6H', data)
            for i in range(6):
                if i <= 3:
                    joint_values[i] = dm_data_to_rad(data[i])
                else:
                    joint_values[i] = dji_data_to_rad(data[i])
            publish_joint_state(pub, joint_values)
            print(f"joint state: {joint_values}")

        rate.sleep()