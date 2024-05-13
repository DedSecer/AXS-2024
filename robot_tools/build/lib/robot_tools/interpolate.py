from scipy.interpolate import CubicSpline, make_interp_spline, interp1d
import matplotlib.pyplot as plt
import numpy as np
import functools


class Interpolate(object):
    """轨迹插值工具类"""

    @staticmethod
    def time_clip(start, end, interval, unit="s", end_control=False):
        """ms级的时间细化
        unit表示所给三个时间的单位，s或ms两种。
        end_control决定是否根据插分后的最后一个值与差分前的最后值相等进行末尾时间控制
        """
        if unit == "s":
            precision = 0.001
        elif unit == "ms":
            precision = 1
        time_line = (np.array([start, end, interval]) / precision).astype("int32")
        time_clipped = np.arange(time_line[0], time_line[1], step=time_line[2])
        if end_control:
            if time_clipped[-1] != time_line[1]:
                time_clipped = np.append(time_clipped, time_line[1])
        else:
            time_clipped = np.append(time_clipped, time_line[1])
        time_clipped = time_clipped.astype("float64")
        time_clipped *= precision
        return time_clipped

    @classmethod
    def no_interpolate(cls, y, t, t_i, unit="s", sort=False, plot=False):
        """无插值"""
        # 对时间序列进行排序，以保证时间单调递增
        if sort:
            idx = np.argsort(t)
            t, y = t[idx], y[idx]
        # 对插值后的时间序列进行扩展
        if isinstance(t_i, (float, int)):
            flag = True
            t_interp = cls.time_clip(t[0], t[-1], t_i, unit)
        else:
            flag = False
            t_interp = t_i
        # 无插值
        stage_num = len(t)

        def stage(t_):
            for i in range(stage_num - 1):
                if t[i] <= t_ <= t[i + 1]:
                    return y[i + 1]

        y_interp = []
        for t_ in t_interp:
            y_interp.append(stage(t_))
        # 是否绘制曲线
        if plot:
            cls.plot(t, y, t_interp, y_interp)
        # 返回插值后的时间序列和函数值
        if flag:
            return t_interp, y_interp
        else:
            return y_interp

    @classmethod
    def linear_interpolate(cls, y, t, t_i, unit="s", sort=False, plot=False):
        # 对时间序列进行排序，以保证时间单调递增
        if sort:
            idx = np.argsort(t)
            t, y = t[idx], y[idx]
        # 对插值后的时间序列进行扩展
        if isinstance(t_i, (float, int)):
            flag = True
            t_interp = cls.time_clip(t[0], t[-1], t_i, unit)
        else:
            flag = False
            t_interp = t_i
        # 使用LinearNDInterpolator进行线性插值
        y_interp = interp1d(t, y)(t_interp)
        if plot:
            cls.plot(t, y, t_interp, y_interp)
        # 返回插值后的时间序列和函数值
        if flag:
            return t_interp, y_interp
        else:
            return y_interp

    @classmethod
    def cubic_spline(
        cls, y, t, t_i, unit="s", bc_type="clamped", sort=False, plot=False
    ):
        """
        使用 UnivariateSpline 实现 1-5次样条插值，常用3次和5次
        参数:
            t: 时间序列，一个一维数组
            y: 时间序列对应的函数值，一个一维数组
            ti:
            unit: 时间单位是s还是ms
            bc_type: 'not-a-knot','natural'，'clamped'
        返回值:
            返回一个二元组，包含插值后的时间序列和对应的函数值
        """
        # 对时间序列进行排序，以保证时间单调递增
        if sort:
            idx = np.argsort(t)
            t, y = t[idx], y[idx]
        # 对插值后的时间序列进行扩展
        if isinstance(t_i, (float, int)):
            flag = True
            t_interp = cls.time_clip(t[0], t[-1], t_i, unit)
        else:
            flag = False
            t_interp = t_i
        # 使用CubicSpline进行3次样条插值
        cnt = 1
        while True:
            try:
                y_interp = CubicSpline(t, y, bc_type=bc_type)(t_interp)
            except:
                t[-cnt] += (
                    0.0003 / cnt
                )  #  Expect x to not have duplicates or x must be strictly increasing sequence.
            else:
                break
            cnt += 1
        if plot:
            cls.plot(t, y, t_interp, y_interp)
        # 返回插值后的时间序列和函数值
        if flag:
            return t_interp, y_interp
        else:
            return y_interp

    @classmethod
    def spline(cls, y, t: np.ndarray, t_i, k=5, unit="s", sort=False, plot=False):
        """
        使用 make_interp_spline 实现 1-5次样条插值，常用3次和5次
        参数:
            t: 时间序列，一个一维数组
            y: 时间序列对应的函数值，一个一维数组
        返回值:
            当t_i为间隔时，返回一个二元组，包含插值后的时间序列和对应的函数值
            当t_i为时间序列时，仅返回插值后的函数值
        """
        # 对时间序列进行排序，以保证时间单调递增
        if sort:
            idx = np.argsort(t)
            t, y = t[idx], y[idx]
        # 对插值后的时间序列进行扩展
        if isinstance(t_i, (float, int)):  # 指定频率
            flag = True
            t_interp = cls.time_clip(t[0], t[-1], t_i, unit)
        else:  # 指定已经细化的时间点
            flag = False
            t_interp = t_i
        # 使用make_interp_spline进行样条插值
        cnt = 1
        while True:
            try:
                y_interp = make_interp_spline(
                    t, y, k=k, bc_type=(((1, 0), (2, 0)), ((1, 0), (2, 0)))
                )(t_interp)
            except:
                t[-cnt] += (
                    0.0003 / cnt
                )  #  Expect x to not have duplicates or x must be strictly increasing sequence.
            else:
                break
            cnt += 1
        if plot:
            cls.plot(t, y, t_interp, y_interp)
        # 返回插值后的时间序列和函数值
        if flag:
            return t_interp, y_interp
        else:
            return y_interp

    @staticmethod
    def linear_speed_calculate(y: np.ndarray, t):
        """计算曲线两点间的直线变化率"""
        y_cp = y.copy()
        y1 = np.delete(y, 0)
        y2 = np.delete(y_cp, -1)
        if isinstance(t, (np.ndarray, list, tuple)):
            return (y1 - y2) / (t[1:] - t[:-1])
        else:
            return (y1 - y2) / t

    @staticmethod
    def interval_limit(y_i: np.ndarray, min_delta=0.001, max_delta=3):
        """控制插值后y方向的间隔"""
        end_index = y_i.shape[0] - 1
        i = 1
        # 中间元素限幅
        while i < end_index:
            if np.fabs(y_i[i] - y_i[i - 1]) < min_delta:
                y_i = np.delete(y_i, i)
                end_index -= 1  # 删除中间元素后有效长度-1（即有效长度始终记录的是0-原末尾元素的长度）
                y_i = np.append(y_i, y_i[end_index])
            else:
                i += 1  # 未删除元素计数+1
        # 末尾（原）两元素限幅
        if np.fabs(y_i[end_index] - y_i[end_index - 1]) < min_delta:
            y_i[end_index - 1] = y_i[end_index]
        return y_i

    @classmethod
    def way_points_interpolate(
        cls, way_points: np.ndarray, time_points: np.ndarray, freq: float, k=5
    ) -> list:
        """路点插值"""
        execute_time_array = cls.time_clip(0, time_points[-1], 1 / freq)
        interpolate = functools.partial(
            Interpolate.spline, t=time_points, t_i=execute_time_array, k=k
        )
        joints_matrix_new: np.ndarray = np.apply_along_axis(
            interpolate, axis=0, arr=way_points
        )
        return joints_matrix_new.tolist()

    @staticmethod
    def plot(t, y, t_interp, y_interp, pause=0, clear=True, ion=False, block=False):
        """
        绘制样条插值的结果:
            主要参数:
                t: 时间序列，一个一维数组
                y: 时间序列对应的函数值，一个一维数组
                t_interp: 插值后的时间序列，一个一维数组
                y_interp: 插值后的时间序列对应的函数值，一个一维数组
            辅助参数:
                block: 图片显示后是否阻塞
                pause: 图片显示后的延迟时间,block为False时生效
                clear: 本次图片是否覆盖上次图片
                ion: 是否开启交互模式
            默认是“不阻塞+无等待+覆盖+无交互”，用于实时显示最新的插值结果
        """
        if ion:
            plt.ion()  # 开启交互模式
        if clear:
            plt.clf()
        # 绘制原始数据点
        plt.plot(t, y, "ro", label="original", markersize=3)
        # 绘制样条插值的结果
        plt.plot(t_interp, y_interp, "g-", label="interpolated", markersize=3)
        # 添加图例和标签
        plt.legend(loc="best")
        plt.xlabel("t")
        plt.ylabel("y")
        plt.title("Interpolation Result")
        # 显示图形
        plt.show(block=block)  # 非block并不会清除原来显示的图片，而是等下次图片来覆盖（前提是使用了plt.clf()）
        if pause > 0 and not block:
            plt.pause(pause)
