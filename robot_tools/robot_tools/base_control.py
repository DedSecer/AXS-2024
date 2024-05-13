from . import transformations, CoordinateTools

import numpy as np
import time
from typing import Union
from threading import Thread


class BaseControlTools(object):
    """机器人底盘运动控制工具类：
    pose(tuple)：
        0:position：（x,y,z）
        1:rotation：（r,p,y）（欧拉角，按xyz顺序，r:[-pi,pi],p:[-pi/2,pi/2],y:[-pi,pi]）(函数参数设置也可使用四元数，按xyzw顺序)
    velocity(tuple)：
        0:linear：（x,y,z）
        1:angular：（r,p,y）（与rotation对应）
    """

    coor = CoordinateTools()
    _TEST_ = False

    @staticmethod
    def target_pose_to_velocity(
        posi_or_rota: Union[float, np.ndarray], kp, limits: tuple, dead_zone: float
    ) -> Union[float, np.ndarray]:
        """根据目标在机器人坐标系下的位置或姿态和给定参数计算机器人的线速度或角速度"""
        if limits[0] > limits[1]:
            limits = (limits[1], limits[0])
        if dead_zone > limits[0]:
            dead_zone = limits[0]
        if isinstance(posi_or_rota, np.ndarray):
            raw_velocity_norm = np.linalg.norm(posi_or_rota)
        else:
            raw_velocity_norm = abs(posi_or_rota)
        target_linear_velocity_norm = kp * raw_velocity_norm
        if target_linear_velocity_norm == 0:
            target_velocity = posi_or_rota
        elif target_linear_velocity_norm > limits[1]:
            target_velocity = limits[1] / raw_velocity_norm * posi_or_rota
        elif target_linear_velocity_norm < dead_zone:
            target_velocity *= 0.0
        elif target_linear_velocity_norm < limits[0]:
            target_velocity = limits[0] / raw_velocity_norm * posi_or_rota
        else:
            target_velocity = (
                target_linear_velocity_norm / raw_velocity_norm * posi_or_rota
            )
        return target_velocity

    @staticmethod
    def min_rotation_move(distance: float, direcition: float) -> tuple:
        """将机器人从当前方向轴按最小方向转向目标方向轴，使得机器人的方向轴（如x轴）与目标方向共线（同向或反向）。
        输入：
            distance: 机器人与目标点的距离 >= 0
            direction: 机器人与目标点的方向 [-pi, pi]
        输出：
            distance: 机器人与目标点的距离，若转向后机器人方向轴与目标方向同向则为正，反向则为负
            direction: 机器人与目标点的方向 [-pi/2, pi/2]
        """
        if direcition < -np.pi / 2:
            direcition += np.pi
            distance *= -1
        elif direcition > np.pi / 2:
            direcition -= np.pi
            distance *= -1
        return distance, direcition

    @classmethod
    def composit_velocity(
        cls, target_pose, current_pose, kp, limits, dead_zone
    ) -> tuple:
        """根据目标位姿和当前位姿及给定参数计算机器人的线速度和角速度"""
        raw_linear_velocity, raw_angular_velocity = cls.coor.to_robot_coordinate(
            target_pose, current_pose
        )
        target_linear_velocity = cls.target_pose_to_velocity(
            raw_linear_velocity, kp[0], limits[0], dead_zone[0]
        )
        target_angular_velocity = cls.target_pose_to_velocity(
            raw_angular_velocity, kp[1], limits[1], dead_zone[1]
        )

        return target_linear_velocity, target_angular_velocity

    @classmethod
    def three_stage_control(
        self, target_pose, current_pose, tolerance, kp, limits, dead_zone
    ) -> tuple:
        """三阶段底盘位置-速度控制法(先旋转调整方向，再平移调整位置，然后再选择调整姿态；根据误差反馈动态调整)"""
        target_linear_velocity = np.zeros(3, dtype=np.float64)
        target_angular_velocity = np.zeros(3, dtype=np.float64)
        stage, bias = self.get_stage_and_bias(
            target_pose, current_pose, tolerance[:2], tolerance[2], improve=False
        )
        if stage in [1, 3]:
            target_angular_velocity[2] = self.target_pose_to_velocity(
                bias, kp[1], limits[1], dead_zone[1]
            )
        elif stage == 2:
            target_linear_velocity[0] = self.target_pose_to_velocity(
                bias, kp[0], limits[0], dead_zone[0]
            )
        return target_linear_velocity, target_angular_velocity

    @classmethod
    def get_stage_and_bias(
        cls,
        target_pose: tuple,
        current_pose: tuple,
        pose_tolerance: tuple,
        direction_tolerance: Union[float, tuple, None] = None,
        improve=False,
        last_stage=None,
        new_target=1,
        avoid_321=False,
        same_ignore=False,
        avoid_swing=False,
    ) -> tuple:
        """获得三阶段控制法的当前阶段及其相关偏差量"""
        position_error, rotation_error = cls.coor.get_pose_error_in_axis(
            target_pose, current_pose
        )  # rotation_error范围为[-pi, pi]
        position_distance, rotation_distance = cls.coor.norm(
            position_error
        ), cls.coor.norm(
            rotation_error
        )  # 计算位置以及姿态（优弧范围内）的距离
        same_target = same_ignore and new_target == 0
        if cls._TEST_:
            print("position_error:", position_error)
            print("rotation_error:", rotation_error)
            print("position_distance:", position_distance)
            print("rotation_distance:", rotation_distance)
            print("new_target:", new_target)
            print("same_target:", same_target)
        if (
            position_distance <= pose_tolerance[0]
            or (avoid_321 and last_stage == 3 and new_target == 0)
            or (last_stage == -1 and same_target)
        ):
            if rotation_distance <= pose_tolerance[1]:
                return -1, 0  # end and stop
            else:
                return 3, rotation_error[2]
        else:
            pose_in_robot = cls.coor.to_robot_coordinate(target_pose, current_pose)
            position_in_robot = pose_in_robot[0]
            direction_error = cls.coor.get_spherical(position_in_robot)[2]
            # 劣弧处理成优弧（position_distance这时带有正负了）
            position_distance, direction_error = cls.min_rotation_move(
                position_distance, direction_error
            )
            if cls._TEST_:
                print("direction_error:", direction_error)
            direction_error_abs = abs(direction_error)
            if improve:  # 使用改进三阶段控制法
                if (
                    ((last_stage in [None, 1, -1]) or not same_target)
                    and direction_error_abs > direction_tolerance[0]
                ) or direction_error_abs > direction_tolerance[1]:
                    return 1, direction_error
                elif direction_error_abs <= 0:  # 完全对齐后才开始绝对直线
                    return 2, position_distance
                elif direction_error_abs <= direction_tolerance[1]:
                    if avoid_swing:
                        direction_error = 0  # 不再修正方向，避免来回摆动
                    if position_distance < 0:
                        direction_error *= -1  # 倒车修正
                    return 1.5, (position_distance, direction_error)  # 1.5的bias是个tuple
            else:
                if (
                    direction_tolerance is None
                    and direction_error_abs <= pose_tolerance[2]
                ) or (direction_error_abs <= direction_tolerance):
                    return 2, position_distance
                else:
                    return 1, direction_error

    @classmethod
    def three_stage_control_improved(
        self,
        target_pose,
        current_pose,
        pose_tolerance,
        direction_tolerance,
        kp,
        limits,
        dead_zone,
        enhance=1,
        last_stage=None,
        new_target=1,
        avoid_321=False,
        same_ignore=False,
        avoid_swing=False,
    ) -> tuple:
        """改进三阶段底盘位置-速度控制法(增加单轴移动+旋转叠加阶段)"""
        target_linear_velocity = np.zeros(3, dtype=np.float64)
        target_angular_velocity = np.zeros(3, dtype=np.float64)
        stage, bias = self.get_stage_and_bias(
            target_pose,
            current_pose,
            pose_tolerance,
            direction_tolerance,
            improve=True,
            last_stage=last_stage,
            new_target=new_target,
            avoid_321=avoid_321,
            same_ignore=same_ignore,
            avoid_swing=avoid_swing,
        )
        if self._TEST_:
            print("stage:", stage)
        if stage in [1, 1.5, 3]:
            if stage == 1.5:
                bias_r = bias[1]
            else:
                bias_r = bias
            target_angular_velocity[2] = (
                self.target_pose_to_velocity(bias_r, kp[1], limits[1], dead_zone[1])
                / enhance
            )
        if stage in [1.5, 2]:
            if stage == 1.5:
                bias_t = bias[0]
            else:
                bias_t = bias
            target_linear_velocity[0] = (
                self.target_pose_to_velocity(bias_t, kp[0], limits[0], dead_zone[0])
                * enhance
            )
        return (target_linear_velocity, target_angular_velocity), stage

    @staticmethod
    def four_steering_wheel_ik(
        linear_vx, linear_vy, angular_vz, width, lenth, radius=1, reduction=1
    ) -> tuple:
        """四舵轮底盘的逆运动学（给定目标速度和相关参数，计算四轮转向和轮速）"""

        r = np.hypot(lenth, width)

        # 各轮的合成分速度
        A = linear_vy - angular_vz * lenth / r  # 右轮的y方向分速度
        B = linear_vy + angular_vz * lenth / r  # 左轮的y方向分速度
        C = linear_vx - angular_vz * width / r  # 前轮的x方向分速度
        D = linear_vx + angular_vz * width / r  # 后轮的x方向分速度

        # 各轮合成速度大小
        ws1 = np.sqrt(B**2 + C**2)
        ws2 = np.sqrt(B**2 + D**2)
        ws3 = np.sqrt(A**2 + D**2)
        ws4 = np.sqrt(A**2 + C**2)

        # 各轮合成速度方向
        wa1 = np.arctan2(B, C)
        wa2 = np.arctan2(B, D)
        wa3 = np.arctan2(A, D)
        wa4 = np.arctan2(A, C)

        # 车轮自身转速缩放因子（减速比/轮半径）
        rotating_factor = reduction / radius

        # 各轮转速大小（rotating_factor为1时为线速度）
        speeds = np.array([ws1, ws2, ws3, ws4], dtype=np.float64) * rotating_factor
        # 各轮旋转角度
        angles = np.array([wa1, wa2, wa3, wa4], dtype=np.float64)

        return speeds, angles


class BaseControl(object):
    """机器人底盘运动控制顶层类"""

    _TEST_ = False

    def __init__(self) -> None:
        self._move_kp = (100, 100)
        self._velocity_dead_zone = (0.0, 0.0)
        self._linear_limits = (0.2, 0.3)
        self._angular_limits = (0.17, 0.18)
        self._direction_tolerance = 0.2
        self._direction_tolerance_improved = (0.1, 0.3)
        self._wait_tolerance = (0.1, 0.1)
        self._improve_enhance = 1
        self._last_stage = None
        self._wait_timeout = 5
        self._wait_period = 0.01
        self._move_stop = True
        self._move_method = "three_stage_improved"
        self._current_position = self._current_rotation = None
        BaseControlTools._TEST_ = self._TEST_
        self._tools = BaseControlTools()
        self._muilti_avoid = {"set": False, "get": False}
        self._new_target = 1
        self._avoid_321 = False
        self._same_ignore = False
        self._position_target = np.zeros(3, dtype=np.float64)
        self._rotation_target = np.zeros(3, dtype=np.float64)
        self._last_orientation_cmd = np.zeros(4, dtype=np.float64)
        self._last_pose_ref = "world"

    def move_to(self, position: np.ndarray, rotation: np.ndarray) -> bool:
        """设置并移动机器人到指定的目标位姿"""
        self.set_target_pose(position, rotation)
        result = self.move()
        return result

    def shift_pose(self, axis: int, target: float) -> bool:
        """在机器人当前位姿的基础上，沿指定轴移动指定距离"""
        position, rotation = self.get_current_world_pose()
        if axis < 3:
            position[axis] += target
        else:
            rotation[axis-3] += target
        result = self.move_to(position, rotation)
        return result

    def set_target_pose(
        self, position: np.ndarray, rotation: np.ndarray, ref: str = "world"
    ) -> None:
        """
        设置机器人在世界（ref=world）/自身（ref=robot）当前坐标系下的目标位姿；
        默认为世界坐标系下的目标位姿；
        """
        len_rotation = len(rotation)
        # 目标相同是否忽略
        if self._avoid_321:
            if self._last_pose_ref == ref and (position == self._position_target).all():
                if len_rotation == 4:
                    if (rotation == self._last_orientation_cmd).all():
                        return
                elif (rotation == self._rotation_target).all():
                    return
            else:
                self._new_target += 1
        # 重复设置目标位姿时的警告
        if self._muilti_avoid["set"]:
            print("Warning: set_target_pose is called before the last is finished!")
            return
        else:
            self._muilti_avoid["set"] = True
        # 设置目标位姿
        if ref == "robot":
            position, rotation = self._tools.coor.to_world_coordinate(
                (position, rotation), self.get_current_world_pose()
            )
        else:
            if ref != "world":
                print("ref is not 'world' or 'robot', by default 'world' is used!")
                ref = "world"
            if len_rotation == 4:
                self._last_orientation_cmd = rotation
                rotation = transformations.euler_from_quaternion(rotation)
                rotation = np.array(rotation, dtype=np.float64)
        self._last_pose_ref = ref
        self._position_target, self._rotation_target = position, rotation
        self._muilti_avoid["set"] = False

    def get_target_pose(self) -> tuple:
        """获取机器人在世界坐标系下的目标位姿"""
        return self._position_target, self._rotation_target

    def get_velocity_cmd(self, ignore_stop=False) -> tuple:
        """获取机器人的速度指令"""
        if self._muilti_avoid["get"]:
            print("Warning: get_velocity_cmd is called before the last is finished!")
            return self._vel_cmd
        else:
            self._muilti_avoid["get"] = True
        if self._move_stop and not ignore_stop:
            self._vel_cmd = (np.array((0, 0, 0)), np.array((0, 0, 0)))
        elif self._move_method == "three_stage":
            self._three_stage_control()
        elif self._move_method == "three_stage_improved":
            self._three_stage_control_improved()
        else:
            self._composit_velocity()
        self._muilti_avoid["get"] = False
        if self._new_target > 0:
            self._new_target -= 1
        return self._vel_cmd

    def move(self, time_out=None) -> bool:
        """移动机器人到最新设置的目标位姿"""
        self._move_stop = False
        start_time = time.time()
        self.get_velocity_cmd()  # 防止第一次速度指令为0
        wait_time = time_out if time_out is not None else self._wait_timeout
        while (self._vel_cmd[0] != 0).any() and (self._vel_cmd[1] != 0).any():
            if (time.time() - start_time) > wait_time:
                print(
                    "Move timeout, the robot may be stuck! The motion will be stopped and the program will continue!"
                )
                self._move_stop = True
                return False
            time.sleep(self._wait_period)
        self._move_stop = True
        return True

    def set_wait_tolerance(
        self, position: float, orientation: float, timeout: float, frequency: float
    ):
        """设置等待的误差容限和超时时间，以及等待的刷新频率"""
        self._wait_tolerance = np.array([position, orientation], dtype=np.float64)
        self._wait_timeout = timeout
        self._wait_period = 1 / frequency

    def avoid_321(self, avoid: bool = True, same_ignore: bool = True):
        """避免3阶段旋转误差造成的3阶段中、3阶段结束后相同命令引起的退化"""
        self._avoid_321 = avoid
        self._same_ignore = same_ignore

    def avoid_swing(self, avoid: bool = True):
        self._avoid_swing = avoid

    def set_move_kp(self, tarns: float, rotat: float):
        """设置机器人运动控制的比例增益"""
        self._move_kp = (tarns, rotat)

    def set_velocity_limits(self, linear: tuple, angular: tuple):
        """设置机器人运动的线速度和角速度的上下限"""
        self._linear_limits = linear
        self._angular_limits = angular

    def set_velocity_dead_zone(self, linear_dead_zone: float, angular_dead_zone: float):
        """设置机器人运动的线速度和角速度死区"""
        self._velocity_dead_zone = (linear_dead_zone, angular_dead_zone)

    def set_direction_tolerance(self, tolerance: Union[float, tuple, list]):
        """设置机器人运动的方向容限"""
        if isinstance(tolerance, (tuple, list)):
            self._direction_tolerance_improved = tolerance
        else:
            self._direction_tolerance = tolerance

    def set_improve_enhance(self, enhance: float):
        """设置机器人运动的方向容限"""
        self._improve_enhance = enhance

    def set_move_method(self, method: str):
        """设置机器人运动的控制方法"""
        self._move_method = method

    def _composit_velocity(self):
        velocity = self._tools.composit_velocity(
            self.get_target_pose(),
            self.get_current_world_pose(),
            self._move_kp,
            (self._linear_limits, self._angular_limits),
            self._velocity_dead_zone,
        )
        self._vel_cmd = velocity
        return velocity

    def _three_stage_control(self):
        velocity = self._tools.three_stage_control(
            self.get_target_pose(),
            self.get_current_world_pose(),
            list(self._wait_tolerance) + [self._direction_tolerance],
            self._move_kp,
            (self._linear_limits, self._angular_limits),
            self._velocity_dead_zone,
        )
        self._vel_cmd = velocity
        return velocity

    def _three_stage_control_improved(self) -> tuple:
        velocity, self._last_stage = self._tools.three_stage_control_improved(
            self.get_target_pose(),
            self.get_current_world_pose(),
            self._wait_tolerance,
            self._direction_tolerance_improved,
            self._move_kp,
            (self._linear_limits, self._angular_limits),
            self._velocity_dead_zone,
            enhance=self._improve_enhance,
            last_stage=self._last_stage,
            new_target=self._new_target,
            avoid_321=self._avoid_321,
            same_ignore=self._same_ignore,
            avoid_swing=self._avoid_swing,
        )
        if self._TEST_:
            print("target_rotation:", self.get_target_pose()[1])
            print("current_rotation:", self.get_current_world_pose()[1])
            print("target_position:", self.get_target_pose()[0])
            print("current_position:", self.get_current_world_pose()[0])
            print("velocity cmd:", velocity)
            print(" ")
        self._vel_cmd = velocity
        return velocity

    def _get_wait_error(self) -> np.ndarray:
        """得到机器人当前与目标位姿的误差（欧式距离）"""
        wait_error = self._tools.coor.get_pose_distance(
            self.get_target_pose(), self.get_current_world_pose()
        )
        return wait_error

    def set_control_handel(self, hanle, frequency=None):
        """设置外部控制函数，将启动一个线程来循环执行该函数"""

        def thread_handle():
            while True:
                hanle(self.get_velocity_cmd())
                if frequency is not None:
                    time.sleep(1 / frequency)

        Thread(target=thread_handle, daemon=True).start()

    def set_current_pose_handle(self, handle):
        """设置获得当前位姿的外部句柄函数调用(重写get_current_world_pose)"""
        self.get_current_world_pose = handle

    def set_current_world_pose(self, position, rotation):
        """设置机器人当前位姿"""
        if len(rotation) == 4:
            rotation = transformations.euler_from_quaternion(rotation)
            rotation = np.array(rotation, dtype=np.float64)
        self._current_position, self._current_rotation = position, rotation

    def get_current_world_pose(self) -> tuple:
        """
        获取机器人在世界坐标系下的位置和姿态；
        若未设置过当前位姿，则返回目标位姿（不论目标位姿是否设置过）；
        """
        if self._current_position is None or self._current_rotation is None:
            return self._position_target, self._rotation_target
        else:
            return self._current_position, self._current_rotation

    def set_model_four_steering_wheel(self, width, lenth, radius=1, reduction=1):
        """设置四舵轮机器人底盘模型"""
        self.chasis_width = width
        self.chasis_lenth = lenth
        self.wheel_radius = radius
        self.wheel_reduction = reduction

    def ik_four_steering_wheel(self, linear_vx, linear_vy, angular_vz):
        """四舵轮底盘的逆运动学（给定目标速度和相关参数，计算四轮转向和轮速）"""
        speeds, angles = self._tools.four_steering_wheel_ik(
            linear_vx,
            linear_vy,
            angular_vz,
            self.chasis_width,
            self.chasis_lenth,
            self.wheel_radius,
            self.wheel_reduction,
        )
        return speeds, angles

    def pose_ik_four_steering_wheel(self):
        """根据当前位姿和目标位姿，直接解算得到四轮转角和转速"""
        vel = self._composit_velocity()
        speeds, angles = self.ik_four_steering_wheel(vel[0][0], vel[0][1], vel[1][2])
        return speeds, angles


if __name__ == "__main__":
    import rospy
    from geometry_msgs.msg import Pose, Twist
    from . import to_ros_msgs

    rospy.init_node("test_base_control")

    base_control = BaseControl()
    base_control.set_move_method("three_stage_improved")
    base_control.set_move_kp(1.5, 0.2)
    base_control.set_velocity_limits((0.0, 0.3), (0.0, 0.18))
    base_control.set_velocity_dead_zone(0.0, 0.0)
    base_control.set_direction_tolerance(0.2)
    base_control.set_wait_tolerance(0.1, 0.1, 60, 200)
    base_control.set_improve_enhance(2)

    # 注册机器人底盘外部控制接口
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def vel_cmd_pub(raw_cmd):
        pub.publish(to_ros_msgs.to_Twist(raw_cmd))

    base_control.set_control_handel(vel_cmd_pub, frequency=200)

    # 实现机器人当前位姿更新接口
    def pose_cb(msg: Pose):
        euler = transformations.euler_from_quaternion(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        )
        base_control.set_current_world_pose(
            np.array(
                [msg.position.x, msg.position.y, msg.position.z], dtype=np.float64
            ),
            np.array(euler, dtype=np.float64),
        )

    rospy.Subscriber("/airbot/pose", Pose, pose_cb)

    rospy.sleep(1)
    target_pose0 = base_control.get_current_world_pose()
    target_pose1 = (target_pose0[0], np.array([0.0, 0.0, 1.5], dtype=np.float64))
    target_pose2 = (np.array([-2.5, -4, 0.5], dtype=np.float64), target_pose1[1])

    base_control.set_target_pose(*target_pose2)
    print("*************************************")
    print("Start moving!")
    base_control.move()
    print("Move finished!")
    print("*************************************")
    rospy.sleep(0.5)
