#include "heading.h"

static Heading_Data heading;

/* 清零全部状态；上电初始化和串口清零命令共用，避免残留旧积分。 */
void Heading_Reset(void)
{
  heading.x_dps = 0.0f;
  heading.y_dps = 0.0f;
  heading.z_dps = 0.0f;
  heading.x_deg = 0.0f;
  heading.y_deg = 0.0f;
  heading.z_deg = 0.0f;
  heading.sample_count = 0U;
  heading.last_sample_ms = 0U;
  heading.valid = 0U;
}

/* 按相邻样本的真实毫秒差积分三轴角速度；无符号减法对节拍回绕安全。 */
void Heading_Update(const MPU6050_Gyro *gyro, uint32_t timestamp_ms)
{
  if (gyro == NULL)
  {
    return;
  }
  heading.x_dps = gyro->x_dps;
  heading.y_dps = gyro->y_dps;
  heading.z_dps = gyro->z_dps;
  if (heading.valid != 0U)
  {
    uint32_t interval_ms = timestamp_ms - heading.last_sample_ms;
    /* 间隔异常大时丢弃这份样本的积分，只刷新基准，不累积错误角度。 */
    if (interval_ms <= HEADING_MAX_INTERVAL_MS)
    {
      float interval_s = (float)interval_ms / 1000.0f;
      heading.x_deg += gyro->x_dps * interval_s;
      heading.y_deg += gyro->y_dps * interval_s;
      heading.z_deg += gyro->z_dps * interval_s;
    }
  }
  heading.last_sample_ms = timestamp_ms;
  heading.valid = 1U;
  heading.sample_count++;
}

const Heading_Data *Heading_GetData(void)
{
  return &heading;
}
