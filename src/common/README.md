# common

供任意 ROS package 使用的通用配置包，不依赖电机型号、CAN、主从或其它业务模块。
Python 使用 `common.config.ConfigManager`，C++ 使用 `common/config.hpp`。
模块仅负责单文件读取、类型检查和 package/node/launch 查询；业务参数校验由使用它的 package 完成。

组织思路参考 [robot_dc 的 common](https://github.com/yizhongzhang1989/robot_dc/tree/main/colcon_ws/src/common)，
但采用显式 package/launch 配置，不搜索任意工作目录、不静默回退或生成默认文件。

## 单一文件与路径

本工作空间的配置源为 [config/dm_motors.yaml](../../config/dm_motors.yaml)，使用 schema **version 2**。
包安装时将它链接/复制为 `share/common/config/runtime.yaml`；
`--symlink-install` 下两者是同一文件。文件名属于工作空间部署，加载器没有 dm 专用字段。

优先级：

1. 显式 launch `config_file:=/path/to/site.yaml` 或节点 `-p config_file:=...`。
2. 环境变量 `COMMON_CONFIG`。
3. `common` 安装目录中的 `config/runtime.yaml`。

旧 `dm_motor_common` 包、`DM_MOTORS_CONFIG` 环境变量及 version 1 配置结构已移除。
配置按进程启动加载，不在控制周期读取磁盘，也不通过修改 `config_file` 参数热切换。
缺失文件、重复 key、格式/类型错误或缺少所查询的参数均明确报错。

## 按 package 和完整 launch 文件名组织

```yaml
version: 2
packages:
  camera_driver:
    nodes:
      camera:
        fps: 30
        device: /dev/video0
    launches:
      preview.launch.py:
        parameters:
          dashboard_host: 0.0.0.0
          dashboard_port: 9010
        nodes:
          camera_driver:
            camera:
              fps: 10
      record.launch.py:
        parameters:
          dashboard_host: 127.0.0.1
          dashboard_port: 9190
        nodes:
          camera_driver:
            camera:
              fps: 60
```

- `packages.<package>.nodes.<node>`：直接运行节点时的默认参数。
- `packages.<package>.launches.<完整文件名>.parameters`：该入口的 launch 参数值。
- 同一入口下的 `nodes.<组件package>.<node>`：该入口对组成节点的参数覆盖。
- 启动参数未在 `parameters` 定义时，`ConfigValue` 可从指定节点默认参数和该入口的节点覆盖中取值。
- 显式命令行参数优先于配置。不同入口只读取自身 profile，不自动借用同包其它 launch 的参数。
- 组合 launch 传递 `config_package/config_launch`，使所有被包含的组件使用所选顶层入口的覆盖，
  而非加载中途切换到子 launch 的 profile。单独运行组件 launch 时使用其自身 profile。

完整 launch 名称包含点号，因此要通过 `launch(package, filename)` 或元组键查询，
不能把 `preview.launch.py` 拆成三层路径。
`system` 等业务数据可放在对应 launch 下，但 `common` 不解释它们。
本工作空间由 `dm_motor_control.launch_utils` 解析 `system` 拓扑，而不是由公共包解析。

## 每个 Dashboard 明确指定端口

组合入口的示例配置：

```yaml
packages:
  dm_motor_bringup:
    launches:
      lead_follow.launch.py:
        parameters:
          can_dashboard_ports:
            main: 8080
          motor_dashboard_ports:
            motor_1: 8081
            motor_2: 8082
          dashboard_port: 8083
```

不存在 `base_port`、ID 偏移或按顺序递增。端口可以不连续，调整电机列表顺序不会改变映射；
添加面板时必须显式增加对应实例端口。映射可只选择部分实例，未列出的实例不启动面板。

组件/单独面板入口的 `dashboard_port` 是该入口自己的单个端口，改变 `motor_id` 不自动改端口；
启动第二个单机面板时需另传端口，或使用已明确列出各实例的组合入口。

原有可选启动规则不变：

- 省略端口参数：不开面板，即使配置中定义了端口。
- `dashboard_port:=config`：使用当前入口配置中的单个端口。
- `dashboard_port:=9183`：显式数字覆盖。
- `can_dashboard_ports:=config motor_dashboard_ports:=config`：使用当前组合入口的命名映射。
- `motor_dashboard_ports:='{motor_1: 9181, motor_2: 9388}'`：逐实例覆盖，不做自动分配。

```bash
ros2 launch dm_motor_bringup lead_follow.launch.py \
  can_dashboard_ports:=config motor_dashboard_ports:=config dashboard_port:=config
```

重复、超范围、未知实例、非整数端口会在创建设备节点前报错。HTTP 主机默认由所选入口的
`dashboard_host` 决定，当前工作空间为 `0.0.0.0`，仅用于可信内网。

## API

```python
from common import ConfigManager

config = ConfigManager()
profile = config.launch("camera_driver", "preview.launch.py")
camera = config.node_parameters(
    "camera_driver", "camera", profile=("camera_driver", "preview.launch.py"))
port = config.get(
    ("packages", "camera_driver", "launches", "preview.launch.py", "parameters", "dashboard_port"), int)
```

C++ 的 `common::Config::get<T>()` 支持点分节点路径，也支持 `std::vector<std::string>` 的逐层键，
用于访问包含点号的 launch 文件名。节点的显式 ROS 参数覆盖文件默认值。

## 验证

```bash
colcon test --packages-select common can_bus dm_motor_control dm_motor_lead_follow dm_motor_bringup
colcon test-result --verbose
```

测试覆盖完全不含电机字段的相机配置、Python/C++ 查询、路径优先级、launch 名字中的点号、
两个入口的拓扑/节点参数/端口隔离、非连续且乱序的命名端口、无端口不开面板以及缺失配置拒绝。
