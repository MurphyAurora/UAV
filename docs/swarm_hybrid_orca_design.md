# 无人机集群高密度动态避障设计文档（分层混合 + ORCA）

## 1. 目标
- 场景：高密度、多动态障碍、多机协同。
- 目标：在保持编队任务可控的前提下，实现实时避碰与稳定飞行。
- 约束：优先复用现有工程，先从3机验证，再扩展到5机、10机。

## 2. 总体架构（你已确认）
- 上层：集中式主控（任务与队形管理）
- 下层：分布式局部避障（每机独立ORCA求解）
- 通信：轻量状态交换（不带意图字段，先保证稳定）

逻辑分层：
- 层A 任务层：主控发布队形模式和目标参考。
- 层B 安全层：每机根据邻机状态和障碍状态运行ORCA，输出安全速度。
- 层C 执行层：通信节点做指令仲裁，将安全速度映射到PX4 Offboard。

群体状态交换节点已上线
见 scripts/swarm_state_exchange.py
通信节点已接入避障仲裁输入（avoid_cmd_vel_ned）
见 src/xtd2_communication/xtd2_communication/multirotor_communication.py
hybrid 启动分支会拉起状态交换和 ORCA
见 scripts/start1.sh

## 3. 节点职责划分
### 3.1 swarm_master（集中主控）
- 输入：任务脚本/操作员模式切换。
- 输出：队形模式、队形参数、全局参考速度。
- 频率：2Hz到5Hz。
- 说明：仅负责任务目标，不直接做碰撞几何计算。

### 3.2 swarm_state_exchange（状态交换）
- 输入：每架机本地状态（位置、速度、航向、时间戳）。
- 输出：全局状态包 + 每机邻居状态包。
- 频率：10Hz到20Hz。
- 说明：轻量字段，减轻总线压力。

### 3.3 local_avoid_orca（每机局部避障）
- 输入：
  - 自身状态
  - 邻机状态
  - 动态障碍状态
  - 主控参考速度
- 输出：本机安全速度修正。
- 频率：20Hz（建议与通信节点控制周期匹配）。
- 说明：ORCA只输出速度，不直接切模式。

### 3.4 multirotor_communication（执行与仲裁）
- 输入：
  - 任务速度/位置指令
  - ORCA安全速度修正
- 输出：PX4 Offboard 控制量。
- 说明：增加指令仲裁器，避免任务指令与避障指令相互覆盖。

## 4. 话题设计（建议）
### 4.1 主控下发
- /xtdrone2/swarm/formation_mode
  - 字段：mode, kp, spacing, ref_x, ref_y, ref_z, timestamp

### 4.2 状态交换
- /xtdrone2/swarm/state_exchange
  - 字段：drone_id, x, y, z, vx, vy, vz, heading, stamp
- /xtdrone2/x500_i/swarm_neighbors
  - 字段：neighbors数组（同上）

### 4.3 避障输出
- /xtdrone2/x500_i/avoid_cmd_vel_ned
  - 字段：vx, vy, vz, quality, risk_level, stamp

### 4.4 动态障碍输入
- /xtdrone2/swarm/dynamic_obstacles
  - 字段：obs_id, x, y, z, vx, vy, vz, radius, stamp

## 5. 指令仲裁策略（关键）
在通信节点内实现仲裁，不让多个节点直接抢写同一控制话题。

优先级建议：
- 最高：紧急避障速度（risk_level高）
- 中：普通避障速度
- 低：任务层队形参考

融合方式：
- v_out = w_task * v_task + w_avoid * v_avoid
- 风险升高时，自动提升w_avoid，降低w_task。
- 限幅：水平速度、垂向速度、加速度都做饱和。

## 6. 状态机设计
每机状态机：
- INIT
  - 条件：本地位置与状态可用
- WARMUP
  - 条件：Offboard流预热计数达到门槛
- ARMED
  - 条件：ARM确认
- OFFBOARD_ACTIVE
  - 条件：进入Offboard
- MISSION_TRACK
  - 条件：任务可执行，风险低
- AVOID_ACTIVE
  - 条件：风险高于阈值
- RECOVERY
  - 条件：风险回落，平滑返回任务轨迹

切换原则：
- 只允许单通道输出到PX4，避免多源覆盖。
- AVOID_ACTIVE进入时记住任务参考，退出后平滑回归。

## 7. ORCA参数建议（首版）
- 通信周期：0.05秒（20Hz）
- 邻居半径：3.0米
- 安全半径：机体半径 + 0.3米冗余
- 时间地平线：2.0秒
- 最大水平速度：1.0米每秒（首版可降到0.6）
- 最大垂向速度：0.3米每秒
- 丢包超时：0.3秒（超时邻居从约束中剔除）

## 8. start1.sh建议改造点
文件位置：[scripts/start1.sh](scripts/start1.sh)

建议：
- 增加模式开关：SWARM_MODE=baseline或hybrid
- baseline：保留当前流程
- hybrid：
  - 关闭固定点任务直推
  - 启动状态交换
  - 启动局部ORCA节点
  - 控制命令改为内联可靠路径

重点：避免在同一阶段同时运行固定点发布和避障发布。

## 9. multirotor_communication.py建议改造点
文件位置：[src/xtd2_communication/xtd2_communication/multirotor_communication.py](src/xtd2_communication/xtd2_communication/multirotor_communication.py)

建议新增：
- 订阅avoid_cmd_vel_ned
- 新增仲裁函数 resolve_command()
- 新增risk_level门控
- 在OFFBOARD未激活时拒绝避障速度直接下发
- 增加日志：任务速度、避障速度、输出速度

## 10. 三阶段实施计划
### 阶段1（最小可用，3机）
- 状态交换节点上线
- 通信节点仲裁上线
- ORCA基础避障上线
- 验证不碰撞和可持续飞行

### 阶段2（稳定增强，5机）
- 增加动态障碍输入
- 增加恢复态平滑回归
- 增加参数热更新

### 阶段3（规模扩展，10机）
- 邻居筛选与通信降载
- 分组通信与局部广播
- 统计评估与自动调参

## 11. 测试计划
### 11.1 功能测试
- 3机无障碍：编队稳定
- 3机动态障碍：最小距离大于安全阈值
- 5机交叉穿越：无碰撞

### 11.2 鲁棒性测试
- 随机丢包
- 部分节点延迟
- 单机异常退出

### 11.3 指标
- 最小机间距离
- 任务偏差均值
- 避障触发频次
- 指令仲裁延迟

## 12. 结论
- 在你的场景下，分层混合通信是最合适路径。
- 先做轻量状态交换和ORCA局部避障，能在不大改架构的前提下尽快落地。
- 下一步可直接进入阶段1代码实施。