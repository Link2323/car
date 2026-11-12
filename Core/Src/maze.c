#include "maze.h"
#include "tof.h"
#include "heading.h"
#include "turn_control.h"
#include "speed_control.h"
#include "encoder.h"

/* 判墙阈值：距离小于该值视为该侧有墙。实测迷宫格宽约 200mm：
   有墙时该侧距离约 100mm，开口时测到对面墙不小于 200mm，
   取 180mm 分界，实跑后可调。 */
#define MAZE_WALL_MM 180U
/* 前方停车距离：直行中前墙近于该值即停车决策。实测通道尽头的角落
   常有左开口，停得太远（150mm）时左侧传感器还对着墙、误判死角
   掉头；60mm 贴近尽头墙再决策，可检测到紧邻墙角的开口。 */
#define MAZE_FRONT_STOP_MM 60U
/* 直行基础轮速 counts/s（约 40 mm/s），迷宫内低速保证感知与制动距离。 */
#define MAZE_DRIVE_COUNTS_PER_S 800.0f
/* 前向光束偏高，远处矮墙可能无回波；此时每轮最多约 20 mm/s 接近。 */
#define MAZE_NO_ECHO_COUNTS_PER_S 400.0f
/* 直行航向纠偏增益：counts/s 每度偏差；与差速修正限幅配套。 */
#define MAZE_HEADING_KP 15.0f
#define MAZE_HEADING_MAX_COUNTS 150.0f
/* 事件确认次数：须连续 N 次读数成立才动作，滤瞬时误读。
   前墙关系安全保持 8 次（约数十毫秒）快速响应；左口实测存在行驶
   晃动引起的短瞬态假开口（左侧实际 90mm 有墙仍触发），加严到
   25 次（约 250ms），真开口持续存在不受影响。 */
#define MAZE_FRONT_CONFIRM 8U
#define MAZE_LEFT_CONFIRM 25U
/* 单段直行超时：墙距随机时兜底最长直行时间，触发即停车重新决策。 */
#define MAZE_DRIVE_TIMEOUT_MS 8000U
/* 停车到感知的最短间隔：等待车体制动残余晃动衰减再读距离。 */
#define MAZE_SETTLE_MS 250U
/* 左口确认后的前滑距离与轮程折算：先滑过开口再停车左转，转弯落点
   落在开口深处更居中的位置，车体旋转扫过的圆远离两侧墙角。
   实测 60mm 停点（前 76/右 71mm）仍会蹭墙角，加大到 90mm。
   双轮平均轮程，约 20.05 counts/mm（2770 counts/圈 × 138.23mm
   周长），90mm 约 1804 counts。滑行中前方 <60mm 自动急停保护。 */
#define MAZE_CREEP_COUNTS 1800
/* 滑行段超时兜底：正常 60mm 约 1.5s，含起停余量。 */
#define MAZE_CREEP_TIMEOUT_MS 2500U

static Maze_Data maze;
static uint8_t initialized;
static uint32_t drive_started_ms;
static uint32_t settle_started_ms;
static uint8_t front_close_count;
static uint8_t left_open_count;
static int64_t creep_base_counts;
static uint32_t creep_started_ms;
static uint8_t front_no_echo;
static uint8_t front_near_stop;

/* 读单路测距并返回有效性；无效时输出距离填 0 并清确认计数。 */
static uint8_t Maze_ReadSensor(TOF_Sensor sensor, uint16_t *distance_mm)
{
  TOF_Data data;
  if (TOF_GetSensorData(sensor, &data) != HAL_OK || data.valid == 0U)
  {
    *distance_mm = 0U;
    return 0U;
  }
  *distance_mm = data.distance_mm;
  return 1U;
}

/* 前路只对现场确认的无回波码放行，距离仍记为未知；左右必须有效。
   初始化失败、I2C 错误、超时、旧帧和其他测距错误不属于无回波模式。 */
static uint8_t Maze_UpdateSensors(void)
{
  uint8_t valid = 0U;
  TOF_Data front = {0};
  front_no_echo = 0U;
  maze.front_mm = 0U;
  if (TOF_GetSensorData(TOF_FRONT, &front) == HAL_OK &&
      (uint32_t)(HAL_GetTick() - front.timestamp_ms) <= TOF_SAMPLE_TIMEOUT_MS)
  {
    if (front.valid != 0U)
    {
      maze.front_mm = front.distance_mm;
      valid |= 1U;
      /* 已看见停车线内的近墙就记住它，不能被后续无回波抹掉。
         转向时视场正在变化，不把扫过的墙面锁存为新方向的近墙。 */
      if (front.distance_mm < MAZE_FRONT_STOP_MM && maze.state != MAZE_TURNING)
      {
        front_near_stop = 1U;
      }
    }
    else if (front.state == TOF_STATE_RANGE_INVALID &&
             front.raw_distance_mm == 8190U &&
             (front.raw_status == 6U || front.raw_status == 9U))
    {
      front_no_echo = 1U;
    }
  }
  if (Maze_ReadSensor(TOF_LEFT, &maze.left_mm) != 0U) { valid |= 2U; }
  if (Maze_ReadSensor(TOF_RIGHT, &maze.right_mm) != 0U) { valid |= 4U; }
  maze.sensor_valid = valid;
  return (uint8_t)((valid & 6U) == 6U &&
                   ((valid & 1U) != 0U || front_no_echo != 0U));
}

/* 统一直行与前滑的目标计算；无回波时纠偏也不能让任一轮超出低速上限。 */
static void Maze_SetDriveTarget(const Heading_Data *heading)
{
  float base = front_no_echo != 0U ? MAZE_NO_ECHO_COUNTS_PER_S : MAZE_DRIVE_COUNTS_PER_S;
  float correction = MAZE_HEADING_KP * (maze.heading_target_deg - heading->z_deg);
  float left;
  float right;
  if (correction > MAZE_HEADING_MAX_COUNTS) { correction = MAZE_HEADING_MAX_COUNTS; }
  if (correction < -MAZE_HEADING_MAX_COUNTS) { correction = -MAZE_HEADING_MAX_COUNTS; }
  left = base - correction;
  right = base + correction;
  if (front_no_echo != 0U)
  {
    if (left > base) { left = base; }
    if (right > base) { right = base; }
  }
  (void)SpeedControl_SetTarget(left, right);
}

/* 复位直行事件确认计数。 */
static void Maze_ResetEvents(void)
{
  front_close_count = 0U;
  left_open_count = 0U;
}

HAL_StatusTypeDef Maze_Init(void)
{
  if (initialized != 0U) { return HAL_BUSY; }
  maze.state = MAZE_IDLE;
  maze.decision = MAZE_DECISION_NONE;
  maze.left_mm = 0U;
  maze.front_mm = 0U;
  maze.right_mm = 0U;
  maze.sensor_valid = 0U;
  maze.heading_target_deg = 0.0f;
  maze.drive_elapsed_ms = 0U;
  maze.steps = 0U;
  front_no_echo = 0U;
  front_near_stop = 0U;
  Maze_ResetEvents();
  initialized = 1U;
  return HAL_OK;
}

HAL_StatusTypeDef Maze_Start(void)
{
  const Heading_Data *heading;
  if (initialized == 0U) { return HAL_ERROR; }
  if (maze.state != MAZE_IDLE) { return HAL_BUSY; }
  if (TurnControl_IsActive() != 0U) { return HAL_BUSY; }
  heading = Heading_GetData();
  if (heading == NULL || heading->valid == 0U) { return HAL_ERROR; }
  /* 首轮决策立即开始，进入 DECIDE 前先撤销任何速度环残留。 */
  SpeedControl_Stop();
  maze.decision = MAZE_DECISION_NONE;
  maze.steps = 0U;
  front_no_echo = 0U;
  front_near_stop = 0U;
  maze.drive_elapsed_ms = 0U;
  maze.state = MAZE_DECIDE;
  settle_started_ms = HAL_GetTick();
  return HAL_OK;
}

void Maze_Stop(void)
{
  SpeedControl_Stop();
  maze.state = MAZE_IDLE;
  maze.fault_reason = 0U;
  front_no_echo = 0U;
  front_near_stop = 0U;
  Maze_ResetEvents();
}

/* 停车感知与左手法则决策：左能走走左，否则直行，再否则右转，全堵掉头。
   前方新鲜无回波时允许低速接近；通信异常和侧向无效仍保护停机。 */
static void Maze_Decide(uint32_t now)
{
  uint16_t front;
  uint16_t left;
  uint16_t right;
  const Heading_Data *heading;
  float delta_deg;

  /* 停车后等晃动衰减再感知；间隔未到不动作。 */
  if ((uint32_t)(now - settle_started_ms) < MAZE_SETTLE_MS) { return; }

  if (Maze_UpdateSensors() == 0U)
  {
    SpeedControl_Stop();
    maze.fault_reason = 1U; maze.state = MAZE_FAULT;
    return;
  }
  front = maze.front_mm;
  left = maze.left_mm;
  right = maze.right_mm;

  if (left >= MAZE_WALL_MM)
  {
    maze.decision = MAZE_DECISION_LEFT;
    delta_deg = 90.0f;
  }
  else if (front >= MAZE_WALL_MM ||
           (front_no_echo != 0U && front_near_stop == 0U))
  {
    /* 直行不需要转向：直接进入驱动段并锁定当前航向。 */
    heading = Heading_GetData();
    if (heading == NULL || heading->valid == 0U)
    {
      maze.fault_reason = 4U; maze.state = MAZE_FAULT;
      return;
    }
    maze.decision = MAZE_DECISION_STRAIGHT;
    maze.heading_target_deg = heading->z_deg;
    /* 使能失败不能进入直行态，保留故障等待人工清除。 */
    if (SpeedControl_Enable() != HAL_OK)
    {
      maze.fault_reason = 3U; maze.state = MAZE_FAULT;
      return;
    }
    drive_started_ms = now;
    maze.drive_elapsed_ms = 0U;
    Maze_ResetEvents();
    maze.state = MAZE_DRIVING;
    maze.steps++;
    return;
  }
  else if (right >= MAZE_WALL_MM)
  {
    maze.decision = MAZE_DECISION_RIGHT;
    delta_deg = -90.0f;
  }
  else
  {
    maze.decision = MAZE_DECISION_UTURN;
    delta_deg = 180.0f;
  }
  maze.steps++;
  if (TurnControl_Start(delta_deg) != HAL_OK)
  {
    maze.fault_reason = 3U; maze.state = MAZE_FAULT;
    return;
  }
  /* 即将改变观察方向，旧方向的近墙记录不带入下一条通道。 */
  front_near_stop = 0U;
  maze.state = MAZE_TURNING;
}

/* 转向段：等待 turn_control 完成，成功后立即进入直行段。 */
static void Maze_TurnProcess(uint32_t now)
{
  TurnControl_Data turn;
  const Heading_Data *heading;
  (void)TurnControl_GetData(&turn);
  if (turn.state == TURN_FAULT)
  {
    maze.fault_reason = 2U; maze.state = MAZE_FAULT;
    return;
  }
  if (turn.state == TURN_DONE)
  {
    heading = Heading_GetData();
    if (heading == NULL || heading->valid == 0U)
    {
      maze.fault_reason = 4U; maze.state = MAZE_FAULT;
      return;
    }
    maze.heading_target_deg = heading->z_deg;
    if (SpeedControl_Enable() != HAL_OK)
    {
      maze.fault_reason = 3U; maze.state = MAZE_FAULT;
      return;
    }
    drive_started_ms = now;
    maze.drive_elapsed_ms = 0U;
    Maze_ResetEvents();
    maze.state = MAZE_DRIVING;
  }
}

/* 直行段：航向比例差速纠偏 + 三类事件监测（前墙/左口/超时）。 */
static void Maze_DriveProcess(uint32_t now)
{
  const Heading_Data *heading;
  SpeedControl_Data speed;
  uint16_t front;
  uint16_t left;

  maze.drive_elapsed_ms = now - drive_started_ms;

  /* 直行同样依赖陀螺仪，数据失效立即保护停机。 */
  heading = Heading_GetData();
  if (heading == NULL || heading->valid == 0U)
  {
    maze.fault_reason = 4U; maze.state = MAZE_FAULT;
    SpeedControl_Stop();
    return;
  }
  if (SpeedControl_GetData(&speed) != HAL_OK ||
      speed.state == SPEED_CONTROL_FAULT)
  {
    maze.fault_reason = 2U; maze.state = MAZE_FAULT;
    return;
  }
  /* 速度环瞬时回待机（主循环偶发阻塞触发反馈保护）：重新使能继续。 */
  if (speed.state == SPEED_CONTROL_IDLE)
  {
    if (SpeedControl_Enable() != HAL_OK)
    {
      maze.fault_reason = 3U; maze.state = MAZE_FAULT;
      return;
    }
  }

  /* 前墙接近：连续确认后停车决策，防止撞墙。 */
  front = maze.front_mm;
  if ((front != 0U && front < MAZE_FRONT_STOP_MM) ||
      (front_no_echo != 0U && front_near_stop != 0U))
  {
    if (front_close_count < 255U) { front_close_count++; }
  }
  else
  {
    front_close_count = 0U;
  }

  /* 左侧开口：左手法则的核心触发，连续确认防误读。 */
  left = maze.left_mm;
  if (left >= MAZE_WALL_MM)
  {
    if (left_open_count < 255U) { left_open_count++; }
  }
  else
  {
    left_open_count = 0U;
  }

  /* 事件分派：前墙与超时立即停车；左口先滑行到开口中部再停车，
     使随后的左转落在开口中央而不是贴边。 */
  if (front_close_count >= MAZE_FRONT_CONFIRM ||
      maze.drive_elapsed_ms > MAZE_DRIVE_TIMEOUT_MS)
  {
    SpeedControl_Stop();
    maze.state = MAZE_DECIDE;
    settle_started_ms = now;
    Maze_ResetEvents();
    return;
  }
  if (left_open_count >= MAZE_LEFT_CONFIRM)
  {
    Encoder_Data encoder;
    if (Encoder_GetData(&encoder) == HAL_OK && encoder.total_valid != 0U)
    {
      creep_base_counts = (encoder.left_total + encoder.right_total) / 2;
      creep_started_ms = now;
      Maze_ResetEvents();
      maze.state = MAZE_CREEPING;
    }
    else
    {
      /* 轮程不可用时退回立即停车决策。 */
      SpeedControl_Stop();
      maze.state = MAZE_DECIDE;
      settle_started_ms = now;
      Maze_ResetEvents();
    }
    return;
  }

  /* 航向保持：差速方向与转向定义一致（右轮快=左转）。右偏时 z 低于
     目标、误差为正，需向左修正，故右轮加左轮减。 */
  Maze_SetDriveTarget(heading);
}

/* 滑行段：保持航向继续直行，轮程走满 60mm 或超时后停车决策；
   期间出现前墙立即停车（安全优先于滑行到位）。 */
static void Maze_CreepProcess(uint32_t now)
{
  const Heading_Data *heading;
  SpeedControl_Data speed;
  Encoder_Data encoder;
  uint16_t front;
  int64_t travelled;

  heading = Heading_GetData();
  if (SpeedControl_GetData(&speed) != HAL_OK ||
      speed.state == SPEED_CONTROL_FAULT ||
      heading == NULL || heading->valid == 0U)
  {
    SpeedControl_Stop();
    maze.fault_reason = 2U; maze.state = MAZE_FAULT;
    Maze_ResetEvents();
    return;
  }
  /* 速度环瞬时回待机（主循环偶发阻塞触发反馈保护）：重新使能继续，
     只有锁存故障或陀螺仪失效才真正停机。 */
  if (speed.state == SPEED_CONTROL_IDLE)
  {
    if (SpeedControl_Enable() != HAL_OK)
    {
      maze.fault_reason = 3U; maze.state = MAZE_FAULT;
      Maze_ResetEvents();
      return;
    }
  }
  /* 前墙出现：连续确认后放弃滑行立即停车。 */
  front = maze.front_mm;
  if ((front != 0U && front < MAZE_FRONT_STOP_MM) ||
      (front_no_echo != 0U && front_near_stop != 0U))
  {
    if (front_close_count < 255U) { front_close_count++; }
  }
  else
  {
    front_close_count = 0U;
  }
  travelled = 0;
  if (Encoder_GetData(&encoder) == HAL_OK && encoder.total_valid != 0U)
  {
    travelled = (encoder.left_total + encoder.right_total) / 2 -
                creep_base_counts;
  }
  if (front_close_count >= MAZE_FRONT_CONFIRM ||
      travelled >= MAZE_CREEP_COUNTS ||
      (uint32_t)(now - creep_started_ms) > MAZE_CREEP_TIMEOUT_MS)
  {
    SpeedControl_Stop();
    maze.state = MAZE_DECIDE;
    settle_started_ms = now;
    Maze_ResetEvents();
    return;
  }
  /* 与直行段相同的航向保持纠偏。 */
  Maze_SetDriveTarget(heading);
}

void Maze_Process(void)
{
  uint32_t now;
  if (initialized == 0U || maze.state == MAZE_IDLE) { return; }
  /* 直行、滑行和转向都检查感知状态；前路仅对白名单新鲜无回波放行。
     读取失败得到的零值仍属故障，不等同于前方开阔。
     先停止转向控制，防止其在下一轮重新向速度环下发运动目标。 */
  if (maze.state != MAZE_DECIDE && maze.state != MAZE_FAULT &&
      Maze_UpdateSensors() == 0U)
  {
    TurnControl_Stop();
    SpeedControl_Stop();
    maze.fault_reason = 1U;
    maze.state = MAZE_FAULT;
    Maze_ResetEvents();
    return;
  }
  now = HAL_GetTick();
  if (maze.state == MAZE_DECIDE)
  {
    Maze_Decide(now);
  }
  else if (maze.state == MAZE_TURNING)
  {
    Maze_TurnProcess(now);
  }
  else if (maze.state == MAZE_DRIVING)
  {
    Maze_DriveProcess(now);
  }
  else if (maze.state == MAZE_CREEPING)
  {
    Maze_CreepProcess(now);
  }
}

uint8_t Maze_IsActive(void)
{
  return (uint8_t)(maze.state != MAZE_IDLE && maze.state != MAZE_FAULT);
}

HAL_StatusTypeDef Maze_GetData(Maze_Data *data)
{
  if (data == NULL) { return HAL_ERROR; }
  *data = maze;
  return HAL_OK;
}
