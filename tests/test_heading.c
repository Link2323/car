/* 主机回归测试：直接构造角速度样本和时间戳，不需要传感器和 HAL 桩。 */
#include "heading.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

/* 检查积分误差，容许单精度浮点的正常舍入。 */
static void CheckNear(float actual, float expected)
{
  assert(fabsf(actual - expected) < 0.001f);
}

int main(void)
{
  MPU6050_Gyro gyro = {0.0f, 0.0f, 0.0f};
  const Heading_Data *data = Heading_GetData();
  unsigned int index;

  /* 未收到任何样本前积分基准无效，角度字段无意义。 */
  assert(data->valid == 0U && data->sample_count == 0U);
  /* NULL 指针安全忽略，不得建立基准。 */
  Heading_Update(NULL, 0U);
  assert(data->valid == 0U && data->sample_count == 0U);

  /* 首份样本只建立时间基准，不产生角度。 */
  gyro.z_dps = 100.0f;
  Heading_Update(&gyro, 1000U);
  assert(data->valid == 1U && data->sample_count == 1U);
  CheckNear(data->z_deg, 0.0f);

  /* 恒定 100 °/s、每份 5 ms，再积分 200 份恰好得到 100 度。 */
  for (index = 0U; index < 200U; ++index)
  {
    Heading_Update(&gyro, 1000U + (index + 1U) * 5U);
  }
  CheckNear(data->z_deg, 100.0f);
  CheckNear(data->x_deg, 0.0f);
  CheckNear(data->y_deg, 0.0f);
  assert(data->sample_count == 201U);
  CheckNear(data->z_dps, 100.0f);

  /* Reset 清零全部角度和统计，回到无基准状态。 */
  Heading_Reset();
  assert(data->valid == 0U && data->sample_count == 0U && data->z_deg == 0.0f);

  /* 时间戳跨 32 位回绕时仍按真实经过时间积分：0xFFFFFFF0 之后 32 ms 到 0x10。 */
  gyro.x_dps = -50.0f;
  gyro.z_dps = 0.0f;
  Heading_Update(&gyro, 0xFFFFFFF0U);
  assert(data->valid == 1U);
  Heading_Update(&gyro, 0x10U);
  CheckNear(data->x_deg, -50.0f * 32.0f / 1000.0f);

  /* 间隔超过 HEADING_MAX_INTERVAL_MS 的样本丢弃积分，只刷新基准。 */
  Heading_Update(&gyro, 0x10U + HEADING_MAX_INTERVAL_MS + 5000U);
  CheckNear(data->x_deg, -50.0f * 32.0f / 1000.0f);
  assert(data->sample_count == 3U);
  /* 基准刷新后恢复正常积分：再过 10 ms 应累计 -0.5 度。 */
  Heading_Update(&gyro, 0x10U + HEADING_MAX_INTERVAL_MS + 5010U);
  CheckNear(data->x_deg, -50.0f * 32.0f / 1000.0f - 0.5f);

  puts("Heading integration tests passed");
  return 0;
}
