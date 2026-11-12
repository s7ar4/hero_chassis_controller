# hero_chassis_controller

麦克纳姆轮底盘控制器（ROS 1 / Noetic）。订阅 `/cmd_vel`，用逆运动学算出四个轮子的目标转速，每个轮子一个 `control_toolbox::Pid` 闭环跟踪；再用轮子实际转速做正运动学，积分出 `/odom` 和 `odom -> base_link` 的 tf。基于 [rm_template](https://github.com/gdut-dynamic-x/rm_template) 创建，在 Gazebo 的 hero 底盘上验证。

**关键词：** ROS, RoboMaster, 麦克纳姆轮, chassis controller, ros_control, pluginlib, DynamicX

## 已实现的功能

| 需求项 | 状态 | 说明 |
|---|---|---|
| 用 rm_template 新建包 | 完成 | 包名 hero_chassis_controller，CMakeLists.txt / package.xml / |
| 版本管理 | 完成 | 按功能分阶段提交，见 `git log` |
| 代码规范 | 完成 | ROS C++ Style：2 空格缩进、`namespace` 嵌套、成员变量后缀 `_`；CMakeLists 中启用 `roslint_cpp()` |
| PID 控制轮速（选做 4） | 完成 | 四个轮子四套 `control_toolbox::Pid`，参数在 yaml 中逐轮配置，支持 `rqt_reconfigure` 在线调参、`rqt_plot` 观察 |
| 逆运动学（选做 5） | 完成 | `/cmd_vel` 视为底盘坐标系（base_link）速度，按麦轮 45° 辊子几何解四轮转速 |
| 正运动学里程计（选做 6） | 完成 | 用轮子实际转速反解底盘速度，发布 `/odom`（nav_msgs/Odometry）并广播 `odom -> base_link` |
| 世界坐标系速度控制（选做 7） | 完成 | `velocity_mode` 参数切换 chassis / world 两种模式，world 模式用 tf 把指令旋到底盘系 |
| 特色功能（选做 8） | 完成 | 轮速加速度限幅、`cmd_vel` 超时自动停车、无指令时的测试转速、兼容 `teleop_twist_keyboard` |

## 依赖

- ROS Noetic
- [catkin_tools](https://catkin-tools.readthedocs.io/)（作业要求用 `catkin build`，不用 `catkin_make`）
- `ros_control`、`controller_manager`、`control_toolbox`、`tf`、`realtime_tools`、`pluginlib`
- 仿真部分：`rm_description_for_task`（模型）、`rm_gazebo`（提供 `RmRobotHWSim` 硬件接口）
- 可选：`rqt_plot`、`rqt_reconfigure`、`teleop_twist_keyboard`

## 编译

在工作空间根目录：

```bash
source /opt/ros/noetic/setup.bash
cd ~/ros1_ws
rosdep install --from-paths src --ignore-src -r -y
catkin build hero_chassis_controller rm_description_for_task rm_gazebo
source devel/setup.bash
```

产物：

```
devel/lib/libhero_chassis_controller_controller.so   # 控制器插件
```

## 运行

### 一条命令起整套仿真

```bash
roslaunch hero_chassis_controller run_hero_sim.launch
```

这个 launch 做三件事：加载 `rm_description_for_task/hero.launch`（`load_chassis:=true`、`roller_type:=realistic`，用 16 个真实辊子，横移和自转才符合物理）；把 `config/controllers.yaml` 读进参数服务器；用 `controller_manager/spawner` 加载并启动 `hero_chassis_controller` 和 `joint_state_controller`。

起来之后检查：

```bash
rosservice call /controller_manager/list_controllers
rostopic list | grep -E 'cmd_vel|odom|target_wheel_speeds'
```

### 只加载控制器（调试用）

用于先起一个 `paused:=true` 的仿真，attach 到 `gzserver` 之后再加载控制器：

```bash
roslaunch hero_chassis_controller load_hero_controller.launch
```

### 发速度指令

```bash
# 前进 0.3 m/s
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.3}}'
# 左平移 0.3 m/s
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {y: 0.3}}'
# 原地左转 0.8 rad/s
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{angular: {z: 0.8}}'
```

也可以用键盘：

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

### 两种速度坐标系模式

改 `config/controllers.yaml` 里的 `velocity_mode`，重启 launch：

- `chassis`：`/cmd_vel` 是底盘坐标系下的速度（默认）
- `world`：`/cmd_vel` 是世界坐标系（`world_frame`，默认 `odom`）下的速度，控制器用 tf 查询 `base_link <- odom` 的变换，把线速度旋到底盘系再执行

切换后用 `rosparam get /controller/hero_chassis_controller/velocity_mode` 确认。写其它值会在 `init()` 里报错并返回 `false`，控制器不会带着错配置运行。

## 话题

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | 订阅 | 底盘速度指令，坐标系由 `velocity_mode` 决定 |
| `/odom` | `nav_msgs/Odometry` | 发布 | 里程计：位姿 + 底盘速度（底盘系），`frame_id: odom`，`child_frame_id: base_link` |
| `/target_wheel_speeds` | `std_msgs/Float64MultiArray` | 发布 | 逆运动学算出的四轮目标转速，顺序见下；约 50 Hz，用于和 `/joint_states` 逐项对表 |
| `/joint_states` | `sensor_msgs/JointState` | 发布（joint_state_controller） | 四个车轮的实际位置/速度/力矩 |
| `/tf` | `tf2_msgs/TFMessage` | 发布 | `odom -> base_link` |

轮子顺序固定为：**左前、右前、左后、右后**（`left_front`、`right_front`、`left_back`、`right_back`），与 `controllers.yaml` 的 `joints` 顺序、`/target_wheel_speeds` 的 data 下标一致。

## 参数

参数位于 `/controller/hero_chassis_controller/` 下（yaml 顶层是 `controller:` 所以多了这一层）。

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `joints` | string[] | 四个车轮关节 | 顺序决定逆/正运动学的下标对应关系 |
| `wheel_radius` | double | `0.07625` | 轮半径，m，取自 `hero.urdf.xacro` |
| `half_wheelbase` | double | `0.2` | 前/后轮到 base_link 的 x 距离，m |
| `half_track` | double | `0.2` | 左/右轮到 base_link 的 y 距离，m |
| `max_effort` | double | `8.0` | 单轮力矩上限，N·m |
| `max_wheel_speed` | double | `20.0` | 单轮角速度上限，rad/s |
| `max_wheel_acceleration` | double | `5.0` | 单轮角加速度上限，rad/s²；`<= 0` 关闭限幅 |
| `cmd_vel_timeout` | double | `0.5` | 超过该时间没有 `/cmd_vel` 就停车，s；`<= 0` 关闭 |
| `velocity_mode` | string | `chassis` | `chassis` 或 `world` |
| `world_frame` | string | `odom` | world 模式下 `/cmd_vel` 所在的坐标系 |
| `odom_frame` | string | `odom` | 里程计父坐标系 |
| `base_frame` | string | `base_link` | 底盘坐标系 |
| `target_wheel_speeds` | double[4] | `[0.2, 0.2, 0.2, 0.2]` | 没收到 `/cmd_vel` 时保持的测试转速，rad/s；演示 PID 跟踪用，正常运行时建议置 0 |
| `gains/<joint>/{p,i,d}` | double | `8.0 / 2.0 / 0.0` | 每个轮子一套 PID，可用 `rqt_reconfigure` 在线改 |
| `gains/<joint>/{i_clamp_max,i_clamp_min,antiwindup}` | double/bool | `0.5 / -0.5 / true` | 积分限幅与抗饱和 |

## 算法

### 逆运动学

底盘坐标系 x 向前、y 向左、z 向上，`wz > 0` 为逆时针。轮子 i 的接地速度为 `Vx_i = vx - wz*y_i`、`Vy_i = vy + wz*x_i`；45° 辊子把这两个分量按符号叠加成轮子滚动速度，代入 `x_i = ±lx`、`y_i = ±ly`：

```
k  = half_wheelbase + half_track
w0 = (vx - vy - k*wz) / R      # 左前
w1 = (vx + vy + k*wz) / R      # 右前
w2 = (vx + vy - k*wz) / R      # 左后
w3 = (vx - vy + k*wz) / R      # 右后
```

### 正运动学

逆运动学矩阵是 4x3 超定方程，列互相正交，最小二乘解即列方向的加权和：

```
vx = R/4 * ( w0 + w1 + w2 + w3)
vy = R/4 * (-w0 + w1 + w2 - w3)
wz = R/4 * (-w0 + w1 - w2 + w3) / k
```

用的是关节**实际**速度 `getVelocity()`，不是期望速度，否则里程计只反映指令而不反映实际运动。

### 里程计积分

每个控制周期用首末平均航向角投影位移，避免转弯时按末端航向角投影带来的系统性外偏：

```
theta_prev = theta
theta      = normalize(theta + wz*dt)
mean_theta = 0.5 * (theta_prev + theta)
x += (vx*cos(mean_theta) - vy*sin(mean_theta)) * dt
y += (vx*sin(mean_theta) + vy*cos(mean_theta)) * dt
```

位姿同时发给 `/odom` 和 `odom -> base_link` 的 tf，两者同源。

### 世界坐标系模式

`/cmd_vel` 到达时，如果 `velocity_mode == world`，用 `tf_listener_.lookupTransform(base_frame, world_frame, ...)` 取旋转，把线速度旋到底盘系（`transform.getBasis() * v`，等价于 `tf::Transform::transformVector`）；角速度 `wz` 在平面内旋转不变，不做变换。tf 尚未发布时退化为用自己积分的 `odom_theta_` 旋转，并限流打印警告。

### 轮速控制

`update()` 每个周期：确定目标轮速（`/cmd_vel` 有效 → 逆运动学；超时 → 0；从未收到 → `target_wheel_speeds`）→ 角速度限幅 → 加速度限幅 → 逐轮计算 `error = target - measured`，`control_toolbox::Pid::computeCommand(error, period)` 出力矩 → `max_effort` 限幅 → `setCommand()`。

## 实测（Gazebo 仿真）

| 指令 | `/odom` 实测 | 误差 |
|---|---|---|
| vx = 0.30 m/s | 0.3003 | +0.1% |
| vy = 0.30 m/s | 0.2991 | -0.3% |
| wz = 0.80 rad/s | 0.7980 | -0.25% |
| vx = -0.30 m/s | -0.3013 | +0.4% |
| 世界系 vx = 0.30（车头 91.4°） | 位移 1.195 m / 4 s ≈ 0.299 m/s，横向位移 ≈ 0 | -0.3% |

直行 0.3 m/s 时 `/target_wheel_speeds` 为 `[3.934, 3.934, 3.934, 3.934]`，与 `0.3 / 0.07625` 一致；左平移为 `[-3.934, +3.934, +3.934, -3.934]`，自转 0.8 为 `[-4.197, +4.197, -4.197, +4.197]`，符号与上面的逆运动学式子逐项对应。

误差来源主要是轮子打滑和 Gazebo 的接触模型；换向、原地转的误差略大于直行。

## 目录结构

```
hero_chassis_controller/
├── CMakeLists.txt
├── package.xml
├── hero_chassis_controller_plugins.xml      # pluginlib 插件描述
├── config/
│   ├── controllers.yaml                     # 控制器参数（几何、限幅、模式、PID）
│   └── default.yaml                         # 模板遗留的普通节点参数
├── include/hero_chassis_controller/
│   └── HeroChassisController.hpp
├── src/
│   └── HeroChassisController.cpp
├── launch/
│   ├── run_hero_sim.launch                  # Gazebo + 模型 + 控制器，一条命令
│   ├── load_hero_controller.launch          # 只加载控制器（配合 attach 调试）
│   ├── hero_chassis_controller.launch       # 模板遗留：普通节点
│   └── hero_chassis_controller_overlying_params.launch
└── doc/
```

## 调试

控制器是运行在 `gzserver` 进程里的插件，不是独立节点，所以要在 CLion 里 `Run -> Attach to Process` 附加到 `gzserver`，而不是 Debug 自己的可执行文件。顺序是先 attach、再用 `load_hero_controller.launch` 加载控制器，`init()` 只在加载时调用一次。

## License

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: s7ar4<br />
Maintainer: s7ar4, linxhs7ar444@gmail.com**

[ROS]: http://www.ros.org
