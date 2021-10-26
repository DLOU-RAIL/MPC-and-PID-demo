#coding:utf-8
import numpy as np
import math
import matplotlib.pyplot as plt
from math import cos, sin, pi

# 线速度pid参数
Kp = 0.3
Ki = 0.01
Kd = 0.5
# 角速度pid参数
Kp1 = 1.7
Ki1 = 0.01
Kd1 = 0.1

dt = 0.1  # 时间间隔，单位：s
k = 1   # 速度系数
global inded
inded = 0


class VehicleState:
    # 车辆初始化
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, w=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.w = w


def update(state, vi, wi):
    # 更新函数
    state.v = vi
    state.w = wi
    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + state.w * dt

    return state

# 线速度pid控制器
def pidcontrol(state, cx_target, cy_target):
    err = math.sqrt((cx_target - state.x) ** 2 + (cy_target - state.y) ** 2)
    sum_err = 0
    sum_err += err
    last_err = err
    vi = Kp * err + Ki * sum_err + Kd * (err - last_err)
    return vi

# 角速度pid控制器
def pidcontrol1(state, cx_target, cy_target):
    err = math.atan2(cy_target - state.y,  cx_target - state.x) - state.yaw
    sum_err = 0
    sum_err += err
    last_err = err
    wi = Kp1 * err + Ki1 * sum_err + Kd1 * (err - last_err)
    return wi


def pure_pursuit_control(state, cx, cy, pind):
    #  cx,cy 最近路点坐标赋值给ind
    ind = calc_target_index(state, cx, cy)
    #  pind值大，将pind赋值给ind

    if pind >= ind:
        ind = pind
    # ind 在列表里，tx 是ind x坐标， ty 是ind y坐标
    if ind < len(cx):
        tx = cx[ind]
        ty = cy[ind]
    # 如果 ind下标值超出范围， tx为当前cx下标 -1， ty为当前cy下标 -1；
    # ind 为 cx长度 - 1
    else:
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1
    return ind
    alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw


    if state.v < 0:  # back
        alpha = math.pi - alpha
    L1 = state.v * dt
    wi = math.atan2(2.0 * math.sin(alpha) / L1, 1.0)


def calc_target_index(state, cx, cy):
    # 搜索最临近的路点
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]
    d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
    ind = d.index(min(d))

    L = 0.0
    L1 = k * state.v

    while L1 > L and (ind+1) < len(cx):
        dx = cx[ind + 1] - cx[ind]
        dy = cy[ind + 1] - cy[ind]
        L += math.sqrt(dx ** 2 + dy ** 2)
        ind += 1

    return ind


def calc_target_index1(state, cx, cy):
    dx = cx[1] - cx[0]  # dx 第一个点 - 第二个点的x坐标
    dy = cy[1] - cy[0]  # dy 第一个点 - 第二个点的y坐标
    dist = math.sqrt(dx ** 2 + dy ** 2) #求 第一个点和第二个点的距离
    min_dist = 0.01 * dist  # 两个点的距离
    global inded # 计数器
    if inded == 0:
        ind = 1  # 下标为1

#     dx = [state.x - icx for icx in cx]
#     dy = [state.y - icy for icy in cy]
#     d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
#     min_d = min(d)
#     ind = d.index(min(d)) #最小距离的下标
#     if ind > inded:  # 如果下标值 大于 计数器的值
#         if min_d > min_dist:  # 两点最小距离 >两个点的距离
#             ind = inded + 1
#         else:
#             inded += 1
#             ind = inded + 1
#     else:
#         ind = ind + 1
#
#     return ind
# #
def circles():
    x0, y0 = (0, 0)
    r = 30.0  # 半径
    angle = - 180  # x轴的夹角
    i = 1
    x11 = []
    y11 = []
    if i < 10:
        while i < 10:
            angle += 18
            x1 = x0 + r * cos(angle * pi / 180)
            y1 = math.sqrt(r ** 2 - (x1 - x0) ** 2) + y0
            x11.append(x1)
            y11.append(y1)
            i += 1
    if i >= 10:
        while i < 21:
            angle += 18
            x1 = x0 + r * cos(angle * pi / 180)
            y1 = - math.sqrt(r ** 2 - (x1 - x0) ** 2) + y0
            #y1 = - math.sqrt(r ** 2 - x1 ** 2)
            x11.append(x1)
            y11.append(y1)
            i += 1

    cx = x11
    cy = y11
    print(x11)
    print(y11)
    return cx, cy


def main():

    cx, cy = circles()
    #cx = np.arange(0, 100, 2)
    #cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in cx]

    T = 200.0  # 最大模拟时间

    # 设置车辆的初始状态
    state = VehicleState(x=-28.531695488854606, y=9.270509831248427, yaw=0.0, v=0.1, w=0.0)
    lastIndex = len(cx) - 1

    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    w = [state.w]
    t = [0.0]

    target_ind = calc_target_index(state, cx, cy)

    while T >= time and lastIndex > target_ind:
        vi = pidcontrol(state, cx[target_ind], cy[target_ind])
        wi = pidcontrol1(state, cx[target_ind], cy[target_ind])
        target_ind = calc_target_index(state, cx, cy)
        if state.v < 0:
            state.yaw = math.pi - state.yaw
        state = update(state, vi, wi)

        time = time + dt

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        w.append(state.w)
        t.append(time)
        print(vi)
        print(wi)
        plt.cla()
        plt.plot(cx, cy, ".r", label="course")
        plt.plot(x, y, "-b", label="trajectory")
        plt.plot(cx[target_ind], cy[target_ind], "go", label="target")
        plt.axis("equal")
        plt.grid(True)
        plt.title("Speed[m/s]:" + str(state.v))
        plt.pause(0.001)

if __name__ == '__main__':
    main()
