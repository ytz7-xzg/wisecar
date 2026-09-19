# Our Car

本分支用于存放我们自己的智能车代码。

当前版本是从学长代码提取的基础库：底盘 PWM、转向 PD、前视几何、道路边界检测、蓝色挡板检测、摄像头采集和去畸变。没有接入今年比赛流程、5G 控制权切换或自动起步。

## 文件职责

| 目录 | 内容 |
|---|---|
| `src/hardware/` | 底盘接口、可选 pigpio 后端、摄像头采集 |
| `src/control/` | 舵机 PD、地面平行道路边界的方向误差计算 |
| `src/vision/` | 道路边缘、Hough 左右边界、蓝色挡板、去畸变 |
| `src/common/` | 严格读取运动控制参数 |
| `apps/control_demo.cpp` | 只打印控制计算结果，不初始化 GPIO |
| `apps/vision_debug.cpp` | 查看摄像头/图片、蓝色占比和道路边缘，不链接底盘输出 |
| `config/vehicle.conf` | 学长车辆的运动参数，需实车核对 |
| `tests/` | 无硬件的控制行为测试、合成图像测试 |
| `docs/reuse.md` | 原文件对应关系、接口约定和暂未迁移的内容 |

## 构建

在仓库根目录执行。要求 C++11、CMake >= 3.10；完整构建需要 OpenCV 4 的 C++ 开发文件。

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
cd build
ctest --output-on-failure
cd ..
./build/control_demo config/vehicle.conf
```

默认不编译 pigpio 后端，测试和示例均不操作电机。仅编译运动计算与无硬件底盘测试：

```bash
cmake -S . -B build-core -DOUR_CAR_WITH_VISION=OFF
cmake --build build-core -j2
cd build-core
ctest --output-on-failure
cd ..
```

树莓派已安装 pigpio 头文件与库后，可以额外编译实车后端：

```bash
cmake -S . -B build-pi -DOUR_CAR_WITH_PIGPIO=ON
cmake --build build-pi -j2
```

启用选项只构建 `our_car_pigpio` 库，不会运行车辆。未来车端入口在代码中显式创建 `PigpioDriver` 和 `Chassis`，检查每次操作返回值。一个进程只持有一个 pigpio 后端，所有模式共用同一底盘输出。

## 查看画面

需要图形桌面或可用的远程图形显示。将编号替换为实际相机；同一路相机由一个采集程序占用。

```bash
./build/vision_debug --camera 0
./build/vision_debug --camera 2
./build/vision_debug --image /path/to/frame.jpg
```

相机模式按 `q` 退出；图片模式按任意键退出。窗口显示蓝色占比、挡板判断、左右直线和边缘图。
目前使用整幅图像验证检测能力，实际任务需要按现场视角选择 ROI。没有接入挡板撤除起步动作。

## 控制接口示例

```cpp
#include "hardware/chassis.hpp"
#include "hardware/pigpio_driver.hpp"
#include "common/config.hpp"

// 在你们自己的车端入口中调用；先核对 config/vehicle.conf。
auto cfg = our_car::load_control_config("config/vehicle.conf");
our_car::PigpioDriver driver;
our_car::Chassis chassis(driver, cfg.chassis);
if (!chassis.open()) return 1;  // 只写停车值和转向中位
if (!chassis.stop()) return 1;
// 后续任务通过 set_motor_pwm()/set_steering_pwm() 提交指令。
// close()/析构会尽力回到停车值与中位，再释放 GPIO。
```

`motor_pwm` 是占空比计数，代码没有编码器车速反馈；`stop()` 是输出停车值，不是定时反向制动。
旧车停车值为 10000、舵机中位为 847，均未在你们的实体车上重新标定。GPIO 释放后的电调行为也需要实车确认。

`SteeringPD::compute()` 只计算舵机 PWM。误差以度为单位，保留原离散差分；固定调用周期，重新接管巡线前调用 `reset()`。
`heading_error()` 接收厘米单位的地面边界；`LaneDetector` 输出的是像素坐标边界。二者之间的相机到地面标定尚未实现，不能直接连起来驱动车辆。

`Camera::read()` 返回图像、相机编号、递增帧号和单调时钟时间。视觉/图传应共享该帧，并由上层检查新鲜度；本版本没有线程调度和指令超时看门狗。

## 验证范围

测试使用模拟 PWM 后端和合成图像，验证软件分支与接口；不代表实际电调制动、舵机方向、摄像头标定或赛道识别效果已经通过。
具体构建环境与结果记录在 `docs/validation.md`。
