# 学长代码复用记录

来源：`ytz7-xzg/wisecar`，`main` 提交 `b85cb7ba3aad14d23da719adb515cf215d45ca6f`。

| 原文件 / 逻辑 | 本分支文件 | 迁移处理 |
|---|---|---|
| `init.cpp::servo_motor_pwmInit()`、`utils.cpp` | `hardware/chassis.*`、`hardware/pigpio_driver.*` | 保留 PWM 频率、范围、引脚来源；逐步检查失败；控制引脚与返回值集中管理 |
| `speed_control.cpp::set_speed()` | `hardware/chassis.cpp::set_motor_pwm()` | 保留 PWM 输出，移除修改巡线全局参数的副作用；越界请求停车并返回失败 |
| `speed_control.cpp` 的停车中位 | `hardware/chassis.cpp::stop()` | 输出配置的停车 PWM；不包含固定时长制动 |
| `vector_control.cpp::servo_control()` | `control/steering_pd.*` | 保留 PD 公式、整数截断、限幅；去除 GPIO；增加状态重置、非有限输入处理 |
| `vector_fix.cpp::get_error_angle()` | `control/lane_geometry.*` | 保留平行道路边界的几何计算；明确厘米单位、右正前正、边界有效性 |
| `speed_control.cpp::threshold_l` | `control/lane_geometry.cpp::lookahead_from_pwm()` | 保留 3200 PWM 区间对应 122cm 的旧启发式，取消初始化和运行时两套不同公式 |
| `get_binary_frame.cpp` | `vision/preprocess.*` | 保留灰度、高斯、Canny、膨胀；中间图像改为局部变量 |
| `get_lines.cpp::get_lines()` | `vision/lane_detector.*` | 保留 Hough、斜率筛选；左右均按线段长度加权；每帧重新返回有效性 |
| `blue_card_find.cpp` | `vision/blue_barrier.*` | 保留 HSV 和占比；参数显式传入；模糊后恢复二值图再计数；无效帧单独标记 |
| `undistort.cpp` | `vision/undistorter.*` | 标定参数由调用方提供；映射表只生成一次；独立输出缓冲；检查标定分辨率 |
| `init.cpp::camera_init()` | `hardware/camera.*` | 摄像头编号可指定；逐帧检查失败；附帧号和时间；不混入 GPIO 初始化 |
| `get_config()`、`config.txt` | `common/config.*`、`config/vehicle.conf` | 保留 key/value 简单格式；拒绝未知/重复字段、非法值和不存在的文件 |

上表路径均相对于 `src/`（配置目录除外）。控制配置允许只写部分字段，其余使用同一份默认值；整个文件校验成功后才更新参数。

## 接口约定

- `set_motor_pwm()` 返回失败时，上层应停止任务；I/O 失败会关闭底盘，必须显式重新打开。
- `set_steering_pwm()` 把请求限制到配置范围；舵机正负方向以实车标定为准。
- `Chassis` 和相机各由单一线程持有；后端必须比 `Chassis` 活得更久。
- `ImageLine` 为像素坐标 `y=m*x+b`；`GroundLine` 为厘米坐标 `x=m*y+b`。
- `heading_error()` 要求两侧地面边界平行、右侧截距大于左侧；目标比例 0 到 1、前视距离大于 0。
- `lookahead_from_pwm()` 只继承旧启发式；输出不能当作实测速度或停止距离。
- `BarrierResult.valid=false` 表示无法判断，不能用于触发起步。
- `LaneObservation.frame_valid=true` 且两侧 `valid=false` 表示本帧丢线；不保留旧边界。
- 同一帧的检测器输入可以使用共享只读 Mat；调用方不得同时修改该缓冲区。

## 暂未迁移

- `transform.cpp`、`map_lines()`：旧 60x320 映射与当前裁剪不一致、索引缺少保护，需结合实际安装标定。
- `kick_start()`、`kick_stop()`、旧 `brake()`、`lane_change_A/B()`：固定冲量/反向输出/时间动作待实车核对；应放在可中断任务层。
- `tasks.cpp`、旧 `main.cpp`：含旧赛题顺序、共享全局变量和定时动作，不作为新项目入口。
- 锥桶、斑马线、停车区任务：作为后续赛题模块迁入，需要位置/有效性结果和今年任务逻辑。
- `QR.py`：原脚本只有识别显示和保存，并会清空输出目录；后续提取纯函数时再接远端操作。
- 旧相机内参、安装高度、赛道宽度：不默认为本车标定值。

本分支先提供基础库，不提供完整自动驾驶程序。下一步接入任务前先完成底盘参数和相机到地面坐标标定。
