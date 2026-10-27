#ifndef CAR_TOF_H
#define CAR_TOF_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 三路共享 I2C1，地址均为七位地址，通过 XSHUT 逐路释放后改址。 */
#define TOF_FRONT_ADDRESS       0x31U
#define TOF_LEFT_ADDRESS        0x30U
#define TOF_RIGHT_ADDRESS       0x32U
#define TOF_POLL_PERIOD_MS      5U
#define TOF_SAMPLE_TIMEOUT_MS   200U
/* 前方模块临时偏移补偿：输出距离 = 原始距离 - 本值，单位 mm。
   根据 100→130、200→232、225→260 的实测数据，取平均偏差约 32 mm。
   此值仅适用于当前前方模块及测量基准；改为 0 可关闭补偿。 */
#ifndef TOF_FRONT_OFFSET_MM
#define TOF_FRONT_OFFSET_MM     32
#endif

/* 左右模块尚未标定，不沿用前方模块的实测补偿。 */
#ifndef TOF_LEFT_OFFSET_MM
#define TOF_LEFT_OFFSET_MM      0
#endif
#ifndef TOF_RIGHT_OFFSET_MM
#define TOF_RIGHT_OFFSET_MM     0
#endif

typedef enum
{
  TOF_FRONT = 0,
  TOF_LEFT,
  TOF_RIGHT,
  TOF_SENSOR_COUNT
} TOF_Sensor;

typedef enum
{
  TOF_STATE_OK = 0,          /* 新鲜且通过器件状态检查的距离。 */
  TOF_STATE_INIT_ERROR = 1,  /* 未初始化、身份不符或初始化失败。 */
  TOF_STATE_I2C_ERROR = 2,   /* 寄存器读写失败。 */
  TOF_STATE_TIMEOUT = 3,     /* 连续测距超过 200 ms 没有新结果。 */
  TOF_STATE_RANGE_INVALID = 4, /* 回波无效或超出本项目 1~2000 mm 接收范围。 */
  TOF_STATE_WAITING = 5      /* 已启动，尚未收到第一帧。 */
} TOF_State;

typedef struct
{
  uint16_t distance_mm;    /* 补偿后的距离；仅 valid=1 时可用于控制。 */
  uint8_t valid;           /* 0 时禁止把旧距离用于避障。 */
  uint8_t raw_status;      /* 器件原始状态码，尚无结果时为 255。 */
  TOF_State state;         /* 蓝牙输出的诊断状态，见上方枚举。 */
  uint32_t timestamp_ms;   /* 最近一帧读取成功的时刻，无效回波也会更新。 */
  uint16_t raw_distance_mm; /* 器件原始距离，不做偏移补偿，便于调试和重新标定。 */
} TOF_Data;

/* 初始化全部三路并逐路改址；仅在 GPIO/I2C 初始化后的主循环调用。
   上电初始化包含有界延时；单路失败会关闭该路并继续初始化其他路。
   全部成功返回 HAL_OK，否则返回首个错误；再次调用会复位全部三路。 */
HAL_StatusTypeDef TOF_Init(void);
/* 每轮主循环调用：独立轮询三路，不等待测距完成。
   任一路故障仍处理其余路；返回首个错误、或有新帧时 HAL_OK、否则 HAL_BUSY。
   I2C 访问仍为有界阻塞，故障路关闭后需 TOF_Init 重新初始化。 */
HAL_StatusTypeDef TOF_Update(void);
/* 复制前方快照，保留原有接口；同时校验数据新鲜度。 */
HAL_StatusTypeDef TOF_GetData(TOF_Data *data);
/* 按方向复制快照并检查新鲜度；方向越界或 NULL 返回 HAL_ERROR。 */
HAL_StatusTypeDef TOF_GetSensorData(TOF_Sensor sensor, TOF_Data *data);
/* 返回指定路的最近初始化/读取结果；未就绪不覆盖上一结果。 */
HAL_StatusTypeDef TOF_GetSensorStatus(TOF_Sensor sensor);

#ifdef __cplusplus
}
#endif
#endif
