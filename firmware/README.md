# STM32 固件修改指南

> 基于 [zero-robotic-arm](https://gitee.com/dearxie/zero-robotic-arm) (STM32F407VET6)，
> 新增 LeRobot 适配所需的 4 条串口命令。

---

## 一、修改概览

**只修改 1 个文件**：`Core/Src/robot_cmd.c`

不需要修改 `robot_cmd.h`（命令处理器均为 `static` 函数，通过命令表注册）。

### 修改清单

| 修改位置 | 内容 |
|----------|------|
| 文件头 | 新增 `#include "can.h"` (访问 `can.rxData`, `can.rxFrameFlag`) |
| 文件头 | 新增 4 个 forward declaration |
| `robot_zero_handle` 之后 | 新增 4 个命令处理器实现 (~180 行) |
| `robot_uart1_cmd_table[]` | 新增 4 个命令表项 |

---

## 二、新增串口命令

| 命令 | 格式 | 响应 | 功能 |
|------|------|------|------|
| `get_state` | `get_state\n` | `STATE:j1,...,j6,v1,...,v6,l1,...,l6\n` | 读取关节角度(°)、速度(°/s)、负载(mA) |
| `set_joints` | `set_joints 90 45 -20 0 90 0\n` | `OK\n` | 设置全部关节目标角度 |
| `set_torque` | `set_torque 1\n` | `OK\n` 或 `OK:FREE\n` | 电机使能/禁用 |
| `e_stop` | `e_stop\n` | `ESTOP\n` | 紧急停止 (保持力矩) |

---

## 三、与已有命令的兼容性

已有 8 条命令**不受影响**：
`remote_enable`, `remote_disable`, `remote_event`, `rel_rotate`,
`auto`, `hard_reset`, `soft_reset`, `zero`

`sscanf` 解析器 (`"%19s %f %f %f %f %f %f"`) 对无参数命令返回 1，
对有参数命令返回 2~7，自动兼容。

---

## 四、编译与烧录

1. STM32CubeIDE 打开 `zero-robotic-arm/2. Software/robot/`
2. `Project → Build All` (Ctrl+B)
3. ST-LINK 烧录
4. **验证**：串口工具 115200bps 连接 USART1
   ```
   发送: get_state
   应收到: STATE:90.00,45.00,-20.00,0.00,90.00,0.00,0.00,...,0.00,0.00,...
   ```

---

## 五、实际修改内容

完整的修改已直接应用于：
```
d:\robo arm\software\zero-robotic-arm\2. Software\robot\Core\Src\robot_cmd.c
```

该文件中搜索 `LeRobot` 即可定位所有新增代码。

### 关键实现细节

**`get_state`**：
- 角度直接取 `g_robot.joints[i].current_angle` (避免 CAN 读取耗时)
- 速度通过 `S_VEL` (0x35) 读取，按 360/65536/reduction_ratio 转换
- 负载通过 `S_CPHA` (0x27) 读取相电流
- 单次 CAN 读取超时 `ROBOT_CAN_TIMEOUT` (10ms)，6 关节总耗时 ~10-20ms
- `vTaskSuspendAll()` 保证 CAN 读写原子性（与现有代码一致）

**`set_joints`**：
- 调用现有的 `robot_send_abs_rotate_event()` 逐关节发送事件
- 事件进入 FreeRTOS 队列，由控制任务按序执行

**`set_torque`**：
- 调用 `Emm_V5_En_Control(addr, enable, true)`
- `enable=false` 时响应 `OK:FREE`（手动示教模式确认）

**`e_stop`**：
- 调用 `Emm_V5_Stop_Now(addr, true)` 立即停止
- 清除 `ROBOT_STATUS_RMODE_ENABLE` 标志

---

## 六、30fps 可行性

每个 `get_state` 调用读取 6 关节的 2 个寄存器 (S_VEL + S_CPHA)，
共 12 次 CAN 事务。每次 <2ms，总计 <24ms，满足 30fps (33ms 周期) 要求。

如果实际测试发现超时，可优化为只读角度（跳过 S_VEL/S_CPHA），
速度/负载信息在数据采集中非必需。
