#! /usr/bin/python
import serial
import struct
import time
import rospy
from sensor_msgs.msg import JointState
from hjarm_engineer_driver.msg import ControlInfo
import threading

car_joint_state = JointState()
car_joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']  # 设置关节名称
car_joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]       # 初始化关节位置
car_joint_state.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]       # 初始化关节速度
car_joint_state.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]         # 初始化关节力矩

target_positions = list(range(6))

# rc r switch is up, car_mode = 2
# rc r switch is mid, car_mode = 1
# else, car_mode = 0
car_mode = 0

joint4_offset = 0


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

def move_joint_with_velocity_limit(pub, max_vel, rate):
    """
    分段执行，从 current_positions 到 target_positions，速度不超过 max_vel
    """

    step_time = 1.0 / rate
    car_current_joint_state = car_joint_state.position[:]
    while not rospy.is_shutdown():
        # keep update pose before auto mode
        if car_mode != 1:
            car_current_joint_state = car_joint_state.position[:]
            rospy.sleep(step_time)
            continue

        global target_positions
        for i in range(len(car_current_joint_state)):
            diff = target_positions[i] - car_current_joint_state[i]
            step = max_vel * step_time
            if abs(diff) > step:
                # wait for each joint to reach the target position
                car_current_joint_state[i] += step if diff > 0 else -step
        # print("joint published: ", car_current_joint_state)
        publish_joint_state(pub, car_current_joint_state)
        rospy.sleep(step_time)
    
def motor_position_callback(data):
    """
    订阅电机位置话题后的回调函数
    """
    global car_joint_state, car_mode
    car_joint_state.position[0] = data.joint1_angle
    car_joint_state.position[1] = data.joint2_angle
    car_joint_state.position[2] = data.joint3_angle
    car_joint_state.position[3] = data.joint4_angle
    car_joint_state.position[4] = data.joint5_angle
    car_joint_state.position[5] = data.joint6_angle
    car_mode = data.mode
    # rospy.loginfo(f"当前电机位置: {joint_state.position[2]} \n")
    
def convert_cc_to_car(joint_values):
    """
    Convert the joint values from the custom controller to the car's joint values
    """
    global joint4_offset

    
    car_joint_values = joint_values[:]
    if joint4_offset == 0:
        joint4_offset = car_joint_values[5]
    
    car_joint_values[0] = joint_values[0]
    if car_joint_values[0] > 3.1:
        car_joint_values[0] -= 3.1
    elif car_joint_values[0] < -3.1:
        car_joint_values[0] += 3.1
    car_joint_values[0] += 0.7
    
    car_joint_values[1] = -joint_values[1]
    car_joint_values[1] /= 4

    car_joint_values[2] = -joint_values[2]
    car_joint_values[2] -= 2.14
    car_joint_values[2] /= 2
    
    if car_joint_values[3] > 3.1:
        car_joint_values[3] -= 3.1
    elif car_joint_values[3] < -3.1:
        car_joint_values[3] += 3.1
    
    # car_joint_values[4] -= 3.1
    car_joint_values[4] -= joint4_offset
    # car_joint_values[4] = -car_joint_values[4]
    
    car_joint_values[5] -= 3.1
    
    return car_joint_values



if __name__ == '__main__':
    ## init ros
    rospy.init_node('custom_contoller_node', anonymous=True)
    rate = rospy.Rate(10)
    pub = rospy.Publisher('/joint_set_states', JointState, queue_size=10)
    rospy.Subscriber('/controlInfo', ControlInfo, motor_position_callback)


    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    print(f"serial opened: {ser.portstr}")
    threading.Thread(target=move_joint_with_velocity_limit, args=(pub, 0.4, 1000)).start()
    while not rospy.is_shutdown():
        data = ser.read(12)
        if len(data) == 12:
            data = struct.unpack('6H', data)
            for i in range(6):
                if i <= 3:
                    target_positions[i] = dm_data_to_rad(data[i])
                else:
                    target_positions[i] = dji_data_to_rad(data[i])
            target_positions = convert_cc_to_car(target_positions)