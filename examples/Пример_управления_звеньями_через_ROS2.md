```python
'''
позволяет одному звену h1 задать какое-то значние
'''
import time
import json
import socket
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from unitree_go.msg import LowState
from unitree_go.msg import IMUState
from unitree_go.msg import LowCmd
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
init_time_var = 0
period_var = 0

LIMITS_OF_JOINTS_UNITREE_H1 = {
    0: [-0.43, 0.43],  # right_hip_roll_joint M
    1: [-3.14, 2.53],  # right_hip_pitch_joint M
    2: [-0.26, 2.05],  # right_knee_joint L
    3: [-0.43, 0.43],  # left_hip_roll_joint M
    4: [-3.14, 2.53],  # left_hip_pitch_joint M
    5: [0.26, 2.05],  # left_knee_joint L
    6: [-2.35, 2.35],  # torso_joint M
    7: [-0.43, 0.43],  # left_hip_yaw_joint M
    8: [-0.43, 0.43],  # right_hip_yaw_joint M
    9: [None, None],  # NOT USED
    10: [-0.87, 0.52],  # left_ankle_joint S
    11: [-0.87, 0.52],  # right_ankle_joint S
    12: [-2.87, 2.87],  # right_shoulder_pitch_joint M
    13: [-3.11, 0.34],  # right_shoulder_roll_joint M
    14: [-4.45, 1.3],  # right_shoulder_yaw_joint M
    15: [-1.25, 2.61],  # right_elbow_joint M
    16: [-2.87, 2.87],  # left_shoulder_pitch_joint M
    17: [-0.34, 3.11],  # left_shoulder_roll_joint M
    18: [-1.3, 4.45],  # left_shoulder_yaw_joint M
    19: [-1.25, 2.61]  # left_elbow_joint M
}

JOINT_INDEX = {
    'right_hip_roll_joint': 0,
    'right_hip_pitch_joint': 1,
    'right_knee_joint': 2,
    'left_hip_roll_joint': 3,
    'left_hip_pitch_joint': 4,
    'left_knee_joint': 5,
    'torso_joint': 6,
    'left_hip_yaw_joint': 7,
    'right_hip_yaw_joint': 8,
    'NOT USED': 9,
    'left_ankle_joint': 10,
    'right_ankle_joint': 11,
    'right_shoulder_roll_joint': 12,
    'right_shoulder_pitch_joint': 13,
    'right_shoulder_yaw_joint': 14,
    'right_elbow_joint': 15,
    'left_shoulder_roll_joint': 16,
    'left_shoulder_pitch_joint': 17,
    'left_shoulder_yaw_joint': 18,
    'left_elbow_joint': 19
}


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        self.active_joints = [JOINT_INDEX['right_shoulder_roll_joint'],
                              JOINT_INDEX['right_shoulder_pitch_joint'],
                              JOINT_INDEX['right_shoulder_yaw_joint'],
                              JOINT_INDEX['right_elbow_joint'],
                              JOINT_INDEX['left_shoulder_pitch_joint'],
                              JOINT_INDEX['left_shoulder_roll_joint'],
                              JOINT_INDEX['left_shoulder_yaw_joint'],
                              JOINT_INDEX['left_elbow_joint'],
                              JOINT_INDEX['left_ankle_joint'],
                              JOINT_INDEX['left_ankle_joint'],
                              JOINT_INDEX['left_hip_yaw_joint'],
                              JOINT_INDEX['right_hip_yaw_joint'],
                              JOINT_INDEX['left_knee_joint'],
                              JOINT_INDEX['left_knee_joint'],
                              JOINT_INDEX['right_hip_pitch_joint'],
                              JOINT_INDEX['right_hip_roll_joint'],
                              JOINT_INDEX['left_hip_roll_joint'],
                              JOINT_INDEX['left_hip_pitch_joint'],
                              JOINT_INDEX['torso_joint']
                              ]

        # текущая позиция
        self.current_jpos = {
            0: 0.0,  # right_hip_roll_joint M
            1: 0.0,  # right_hip_pitch_joint M
            2: 0.0,  # right_knee_joint L
            3: 0.0,  # left_hip_roll_joint M
            4: 0.0,  # left_hip_pitch_joint M
            5: 0.0,  # left_knee_joint L
            6: 0.0,  # torso_joint M
            7: 0.0,  # left_hip_yaw_joint M
            8: 0.0,  # right_hip_yaw_joint M
            9: 0.0,  # NOT USED
            10: 0.0,  # left_ankle_joint S
            11: 0.0,  # right_ankle_joint S
            12: 0.0,  # right_shoulder_pitch_joint M
            13: 0.0,  # right_shoulder_roll_joint M
            14: 0.0,  # right_shoulder_yaw_joint M
            15: 0.0,  # right_elbow_joint M
            16: 0.0,  # left_shoulder_pitch_joint M
            17: 0.0,  # left_shoulder_roll_joint M
            18: 0.0,  # left_shoulder_yaw_joint M
            19: 0.0  # left_elbow_joint M
        }

        # нулевая позиция
        self.init_pos = {
            0: 0.0,  # right_hip_roll_joint M
            1: 0.0,  # right_hip_pitch_joint M
            2: 0.0,  # right_knee_joint L
            3: 0.0,  # left_hip_roll_joint M
            4: 0.0,  # left_hip_pitch_joint M
            5: 0.0,  # left_knee_joint L
            6: 0.0,  # torso_joint M
            7: 0.0,  # left_hip_yaw_joint M
            8: 0.0,  # right_hip_yaw_joint M
            9: 0.0,  # NOT USED
            10: 0.0,  # left_ankle_joint S
            11: 0.0,  # right_ankle_joint S
            12: 0.0,  # right_shoulder_pitch_joint M
            13: 0.0,  # right_shoulder_roll_joint M
            14: 0.0,  # right_shoulder_yaw_joint M
            15: 0.0,  # right_elbow_joint M
            16: 0.0,  # left_shoulder_pitch_joint M
            17: 0.0,  # left_shoulder_roll_joint M
            18: 0.0,  # left_shoulder_yaw_joint M
            19: 0.0  # left_elbow_joint M
        }

        self.kPi_2 = 1.57079632

        self.target_pos = {
            0: 0.0,  # right_hip_roll_joint M
            1: 0.0,  # right_hip_pitch_joint M
            2: 0.0,  # right_knee_joint L
            3: 0.0,  # left_hip_roll_joint M
            4: 0.0,  # left_hip_pitch_joint M
            5: 0.0,  # left_knee_joint L
            6: 0.0,  # torso_joint M
            7: 0.0,  # left_hip_yaw_joint M
            8: 0.0,  # right_hip_yaw_joint M
            9: 0.0,  # NOT USED
            10: 0.0,  # left_ankle_joint S
            11: 0.0,  # right_ankle_joint S
            12: 0.0,  # right_shoulder_pitch_joint M
            13: 0.0,  # right_shoulder_roll_joint M
            14: 0.0,  # right_shoulder_yaw_joint M
            15: 0.0,  # right_elbow_joint M
            16: 0.0,  # left_shoulder_pitch_joint M
            17: 0.0,  # left_shoulder_roll_joint M
            18: 0.0,  # left_shoulder_yaw_joint M
            19: 0.0  # left_elbow_joint M
        }

        # целевая позиция
        # self.target_pos = {
        #     0: 0.0,  # right_hip_roll_joint M
        #     1: 0.0,  # right_hip_pitch_joint M
        #     2: 0.0,  # right_knee_joint L
        #     3: 0.0,  # left_hip_roll_joint M
        #     4: 0.0,  # left_hip_pitch_joint M
        #     5: 0.0,  # left_knee_joint L
        #     6: 0.0,  # torso_joint M
        #     7: 0.0,  # left_hip_yaw_joint M
        #     8: 0.0,  # right_hip_yaw_joint M
        #     9: 0.0,  # NOT USED
        #     10: 0.0,  # left_ankle_joint S
        #     11: 0.0,  # right_ankle_joint S
        #     12: right_shoulder_roll_joint,  # right_shoulder_pitch_joint M
        #     13: right_shoulder_pitch_joint,  # right_shoulder_roll_joint M
        #     14: right_shoulder_yaw_joint,  # right_shoulder_yaw_joint M
        #     15: right_elbow_joint,  # right_elbow_joint M
        #     16: left_shoulder_roll_joint,  # left_shoulder_pitch_joint M
        #     17: left_shoulder_pitch_joint,  # left_shoulder_roll_joint M
        #     18: left_shoulder_yaw_joint,  # left_shoulder_yaw_joint M
        #     19: left_elbow_joint  # left_elbow_joint M
        # }

    #   arm_joints = [JointIndex['kLeftShoulderPitch'],  JointIndex['kLeftShoulderRoll'],
    #       JointIndex['kLeftShoulderYaw'],    JointIndex['kLeftElbow'],
    #       JointIndex['kRightShoulderPitch'], JointIndex['kRightShoulderRoll'],
    #       JointIndex['kRightShoulderYaw'],   JointIndex['kRightElbow'], JointIndex['kWaistYaw']]

        # self.target_pos2 = [0.0, -0.1,  0.0, self.kPi_2, 0.0, 0.1, 0.0, self.kPi_2]
        # self.target_pos3 = [-0.78, 0.5, 0.0, self.kPi_2,-self.kPi_2, -0.5, 0.0, self.kPi_2, 0.0]
        # self.target_pos4 = [2.87, 0.1, 0.0, self.kPi_2, 2.87, -0.1, 0.0, self.kPi_2, 0.0]
        # self.target_pos5 = [0.0, 2.2,  0.0, self.kPi_2, 0.0, -2.2, 0.0, self.kPi_2, -0.5]

        # текущая позиция, которая обновляется в фазе движения к целевой позиции
        self.current_jpos_des = {
            0: 0.0,  # right_hip_roll_joint M
            1: 0.0,  # right_hip_pitch_joint M
            2: 0.0,  # right_knee_joint L
            3: 0.0,  # left_hip_roll_joint M
            4: 0.0,  # left_hip_pitch_joint M
            5: 0.0,  # left_knee_joint L
            6: 0.0,  # torso_joint M
            7: 0.0,  # left_hip_yaw_joint M
            8: 0.0,  # right_hip_yaw_joint M
            9: 0.0,  # NOT USED
            10: 0.0,  # left_ankle_joint S
            11: 0.0,  # right_ankle_joint S
            12: 0.0,  # right_shoulder_pitch_joint M
            13: 0.0,  # right_shoulder_roll_joint M
            14: 0.0,  # right_shoulder_yaw_joint M
            15: 0.0,  # right_elbow_joint M
            16: 0.0,  # left_shoulder_pitch_joint M
            17: 0.0,  # left_shoulder_roll_joint M
            18: 0.0,  # left_shoulder_yaw_joint M
            19: 0.0  # left_elbow_joint M
        }

        # флаг показывает будет ли производиться в начале движения перевод звена в нулевое положение
        self.init_flag = True
        self.flag_task_complite = 0

        self.weight = 0.0
        self.weight_rate = 0.2

        self.kp = 60.0
        self.kd = 1.5
        self.dq = 0.0
        self.tau_ff = 0.0

        # частота обновления
        self.control_dt = 0.02
        self.max_joint_velocity = 0.5

        self.delta_weight = self.weight_rate * self.control_dt
        self.max_joint_delta = self.max_joint_velocity * self.control_dt

        # время основного цикла движения
        self.period = period_var + init_time_var
        # считает количество итераций цикла timer_callback
        self.timer_call_count = 0
        self.num_time_steps = (self.period)/(self.control_dt)

        # время перевода звена в нулевое положение
        self.init_time = init_time_var
        self.init_time_steps = int(self.init_time/self.control_dt)
        self.phase = 0.0

        self.subscription = self.create_subscription(
            LowState,
            'lowstate',
            self.listener_callback,
            10)

        self.cmd_msg = LowCmd
        # коррекция ошибки - отсутсвие запрашиваемого поля в полях сообщения LowCmd
        setattr(self.cmd_msg, '__idl_typename__',
                'unitree_go.msg.dds_.LowCmd_')

        self.publisher = self.create_publisher(LowCmd, 'lowcmd', 10)
        self.timer = self.create_timer(self.control_dt, self.timer_callback)

        self.subscription  # prevent unused variable warning

        ChannelFactoryInitialize(0)

        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(5.0)
        self.msc.Init()

        status, result = self.msc.CheckMode()
        while result['name']:
            self.msc.ReleaseMode()
            status, result = self.msc.CheckMode()
            time.sleep(1)

    def listener_callback(self, msg):
        i = 0

        # для фазы инициализации узнаем текущее положение
        if self.init_flag:
            for i in self.active_joints:
                self.current_jpos[i] = msg.motor_state[i].q

    def timer_callback(self):
        self.cmd_msg = LowCmd()

        # константы сообщений
        self.cmd_msg.head[0] = 254
        self.cmd_msg.head[1] = 239
        self.cmd_msg.level_flag = 255
        self.cmd_msg.gpio = 0

        #     self.cmd_msg.frame_reserve = 0
        #     self.cmd_msg.sn[0] = 0
        #     self.cmd_msg.sn[1] = 0
        #     self.cmd_msg.version[0] = 0
        #     self.cmd_msg.version[1] = 0
        #     self.cmd_msg.bandwidth = 0

        #     msg.BmsCmd.bms_cmd = 0
        #     self.cmd_msg.wireless_remote = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        #     self.cmd_msg.led = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        #     self.cmd_msg.fan = [0, 0]
        #     self.cmd_msg.reserve = 0

        # Первая фаза: Установка используемых звеньев робота в нулевое положение
        if self.timer_call_count == 0:
            print('Initializing arms...')

        if self.timer_call_count < self.init_time_steps:
            self.weight = 1.0

            # ???
            self.cmd_msg.motor_cmd[JOINT_INDEX['NOT USED']].q = self.weight

            self.phase = 1.0 * self.timer_call_count / self.init_time_steps

            for i in self.active_joints:
                coeff_and_mode = determine_coeff_and_mode(i)  # (Kp, Kd, mode)
                self.cmd_msg.motor_cmd[i].q = self.init_pos[i] * \
                    self.phase + self.current_jpos[i] * (1 - self.phase)
                self.cmd_msg.motor_cmd[i].dq = self.dq
                self.cmd_msg.motor_cmd[i].tau = self.tau_ff
                self.cmd_msg.motor_cmd[i].kp = coeff_and_mode[0]
                self.cmd_msg.motor_cmd[i].kd = coeff_and_mode[1]
                self.cmd_msg.motor_cmd[i].mode = coeff_and_mode[2]

        # Вторая фаза: Установка звеньев робота в целевое пложение
        elif self.timer_call_count == self.init_time_steps:
            print("Arms initialized!")
            self.init_flag = False

        elif self.timer_call_count > self.init_time_steps and self.timer_call_count < self.num_time_steps:
            for i in self.active_joints:
                self.current_jpos_des[i] += clamp(-self.max_joint_delta,
                                                  self.target_pos[i]-self.current_jpos_des[i], self.max_joint_delta)

            for j in self.active_joints:
                coeff_and_mode = determine_coeff_and_mode(j)  # (Kp, Kd, mode)
                self.cmd_msg.motor_cmd[j].q = self.current_jpos_des[j]
                self.cmd_msg.motor_cmd[j].dq = 0.0
                self.cmd_msg.motor_cmd[j].tau = 0.5
                self.cmd_msg.motor_cmd[j].kp = coeff_and_mode[0]
                self.cmd_msg.motor_cmd[j].kd = coeff_and_mode[1]
                self.cmd_msg.motor_cmd[j].mode = coeff_and_mode[2]

        else:
            if self.flag_task_complite == 0:
                print("Moving is done!")
                self.flag_task_complite = 1
            return 0

        # Подсчет контрольной суммы, использует CRC для проверки целостности команд
        self.crc = CRC()
        self.cmd_msg.crc = self.crc.Crc(self.cmd_msg)

        self.publisher.publish(self.cmd_msg)
        # print(self.cmd_msg)

        self.timer_call_count += 1

# функция ограничения перемещения, что бы звено не дергалось слишком резко


def clamp(minimum, x, maximum):
    return max(minimum, min(x, maximum))

# функция определения коэффицмэнтов Kp, Kd и мода для моторов


def determine_coeff_and_mode(index_of_joint_of_unitree_h1: int) -> tuple:
    size_S = [10, 11]
    size_L = [2, 5]

    # determine Kp and Kd
    if index_of_joint_of_unitree_h1 in size_S:
        Kp = 80.0
        Kd = 2.0

    elif index_of_joint_of_unitree_h1 in size_L:
        Kp = 200.0
        Kd = 5.0

    else:
        Kp = 100.0
        Kd = 3.0

    # determine mode for enable
    if index_of_joint_of_unitree_h1 < 9:
        mode = 0x0A
    elif index_of_joint_of_unitree_h1 > 9:
        mode = 0x01
    else:
        mode = 0x00

    return (Kp, Kd, mode)


def main(args=None):
    global init_time_var
    global period_var

    init_time_var = float(input('init_time: '))
    period_var = float(input('period: '))

    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()

    joint = int(input('joint: '))

    print('limits of joint {}:'.format(joint))
    print(LIMITS_OF_JOINTS_UNITREE_H1[joint])

    value = float(input('value: '))
    minimal_subscriber.target_pos[joint] = value

    try:
        rclpy.spin(minimal_subscriber)
    finally:
        minimal_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

```