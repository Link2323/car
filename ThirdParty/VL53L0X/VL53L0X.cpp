/* 基于 Pololu VL53L0X 1.3.1 移植；保留原算法和说明，许可见 LICENSE.txt。
   本地修改：移除 Arduino/Wire，改用 STM32 HAL，并锁存通信错误。 */
// Most of the functionality of this library is based on the VL53L0X API
// provided by ST (STSW-IMG005), and some of the explanatory comments are quoted
// or paraphrased from the API source code, API user manual (UM2039), and the
// VL53L0X datasheet.

#include "VL53L0X.h"
#include <string.h>

// Defines /////////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define ADDRESS_DEFAULT 0x29

// Record the current time to check an upcoming timeout against
#define startTimeout() (timeout_start_ms = HAL_GetTick())

// Check if timeout is enabled (set to nonzero value) and has expired
#define checkTimeoutExpired() (bus_error != HAL_OK || (io_timeout > 0 && ((uint16_t)(HAL_GetTick() - timeout_start_ms) > io_timeout)))

// Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
// from register value
// based on VL53L0X_decode_vcsel_period()
#define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)

// Encode VCSEL pulse period register value from period in PCLKs
// based on VL53L0X_encode_vcsel_period()
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

// Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
// based on VL53L0X_calc_macro_period_ps()
// PLL_period_ps = 1655; macro_period_vclks = 2304
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

// Constructors ////////////////////////////////////////////////////////////////

/* 构造器件对象并建立默认地址和通信状态。 */
VL53L0X::VL53L0X()
  : bus_error(HAL_OK)
  , last_status(0)
  , address(ADDRESS_DEFAULT)
  , io_timeout(0) // no timeout
  , did_timeout(false)
  , timeout_start_ms(0)
  , stop_variable(0)
  , measurement_timing_budget_us(0)
{
}

// Public Methods //////////////////////////////////////////////////////////////

/* 修改器件七位地址，必须在其他同地址器件关闭时调用。 */
void VL53L0X::setAddress(uint8_t new_addr)
{
  writeReg(I2C_SLAVE_DEVICE_ADDRESS, new_addr & 0x7F);
  address = new_addr;
}

// Initialize sensor using sequence based on VL53L0X_DataInit(),
// VL53L0X_StaticInit(), and VL53L0X_PerformRefCalibration().
// This function does not perform reference SPAD calibration
// (VL53L0X_PerformRefSpadManagement()), since the API user manual says that it
// is performed by ST on the bare modules; it seems like that should work well
// enough unless a cover glass is added.
// If io_2v8 (optional) is true or not given, the sensor is configured for 2V8
// mode.
/* 执行器件身份检查、参考 SPAD 配置、调优和参考校准。 */
bool VL53L0X::init(bool io_2v8)
{
  /* 核对型号身份，避免将其他 I2C 器件误配置为测距传感器。 */
  // check model ID register (value specified in datasheet)
  if (readReg(IDENTIFICATION_MODEL_ID) != 0xEE) { return false; }

  // VL53L0X_DataInit() begin

  // sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
  /* 采用模块常见的 2.8 V I/O 模式，沿用上游默认配置。 */
  if (io_2v8)
  {
    writeReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
      readReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01); // set bit 0
  }

  // "Set I2C standard mode"
  writeReg(0x88, 0x00);

  writeReg(0x80, 0x01);
  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x00);
  stop_variable = readReg(0x91);
  writeReg(0x00, 0x01);
  writeReg(0xFF, 0x00);
  writeReg(0x80, 0x00);

  // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
  writeReg(MSRC_CONFIG_CONTROL, readReg(MSRC_CONFIG_CONTROL) | 0x12);

  // set final range signal rate limit to 0.25 MCPS (million counts per second)
  setSignalRateLimit(0.25);

  writeReg(SYSTEM_SEQUENCE_CONFIG, 0xFF);

  // VL53L0X_DataInit() end

  // VL53L0X_StaticInit() begin

  uint8_t spad_count;
  bool spad_type_is_aperture;
  if (!getSpadInfo(&spad_count, &spad_type_is_aperture)) { return false; }

  // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
  // the API, but the same data seems to be more easily readable from
  // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
  uint8_t ref_spad_map[6];
  readMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

  // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

  writeReg(0xFF, 0x01);
  writeReg(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
  writeReg(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
  writeReg(0xFF, 0x00);
  writeReg(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

  uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
  uint8_t spads_enabled = 0;

  /* 遍历配置项或等待器件状态；器件等待在循环内检查超时。 */
  for (uint8_t i = 0; i < 48; i++)
  {
    /* 跳过不适用的感光单元，达到出厂参考数量后关闭其余单元。 */
    if (i < first_spad_to_enable || spads_enabled == spad_count)
    {
      // This bit is lower than the first one that should be enabled, or
      // (reference_spad_count) bits have already been enabled, so zero this bit
      ref_spad_map[i / 8] &= ~(1 << (i % 8));
    }
    else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
    {
      spads_enabled++;
    }
  }

  writeMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

  // -- VL53L0X_set_reference_spads() end

  // -- VL53L0X_load_tuning_settings() begin
  // DefaultTuningSettings from vl53l0x_tuning.h

  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x00);

  writeReg(0xFF, 0x00);
  writeReg(0x09, 0x00);
  writeReg(0x10, 0x00);
  writeReg(0x11, 0x00);

  writeReg(0x24, 0x01);
  writeReg(0x25, 0xFF);
  writeReg(0x75, 0x00);

  writeReg(0xFF, 0x01);
  writeReg(0x4E, 0x2C);
  writeReg(0x48, 0x00);
  writeReg(0x30, 0x20);

  writeReg(0xFF, 0x00);
  writeReg(0x30, 0x09);
  writeReg(0x54, 0x00);
  writeReg(0x31, 0x04);
  writeReg(0x32, 0x03);
  writeReg(0x40, 0x83);
  writeReg(0x46, 0x25);
  writeReg(0x60, 0x00);
  writeReg(0x27, 0x00);
  writeReg(0x50, 0x06);
  writeReg(0x51, 0x00);
  writeReg(0x52, 0x96);
  writeReg(0x56, 0x08);
  writeReg(0x57, 0x30);
  writeReg(0x61, 0x00);
  writeReg(0x62, 0x00);
  writeReg(0x64, 0x00);
  writeReg(0x65, 0x00);
  writeReg(0x66, 0xA0);

  writeReg(0xFF, 0x01);
  writeReg(0x22, 0x32);
  writeReg(0x47, 0x14);
  writeReg(0x49, 0xFF);
  writeReg(0x4A, 0x00);

  writeReg(0xFF, 0x00);
  writeReg(0x7A, 0x0A);
  writeReg(0x7B, 0x00);
  writeReg(0x78, 0x21);

  writeReg(0xFF, 0x01);
  writeReg(0x23, 0x34);
  writeReg(0x42, 0x00);
  writeReg(0x44, 0xFF);
  writeReg(0x45, 0x26);
  writeReg(0x46, 0x05);
  writeReg(0x40, 0x40);
  writeReg(0x0E, 0x06);
  writeReg(0x20, 0x1A);
  writeReg(0x43, 0x40);

  writeReg(0xFF, 0x00);
  writeReg(0x34, 0x03);
  writeReg(0x35, 0x44);

  writeReg(0xFF, 0x01);
  writeReg(0x31, 0x04);
  writeReg(0x4B, 0x09);
  writeReg(0x4C, 0x05);
  writeReg(0x4D, 0x04);

  writeReg(0xFF, 0x00);
  writeReg(0x44, 0x00);
  writeReg(0x45, 0x20);
  writeReg(0x47, 0x08);
  writeReg(0x48, 0x28);
  writeReg(0x67, 0x00);
  writeReg(0x70, 0x04);
  writeReg(0x71, 0x01);
  writeReg(0x72, 0xFE);
  writeReg(0x76, 0x00);
  writeReg(0x77, 0x00);

  writeReg(0xFF, 0x01);
  writeReg(0x0D, 0x01);

  writeReg(0xFF, 0x00);
  writeReg(0x80, 0x01);
  writeReg(0x01, 0xF8);

  writeReg(0xFF, 0x01);
  writeReg(0x8E, 0x01);
  writeReg(0x00, 0x01);
  writeReg(0xFF, 0x00);
  writeReg(0x80, 0x00);

  // -- VL53L0X_load_tuning_settings() end

  // "Set interrupt config to new sample ready"
  // -- VL53L0X_SetGpioConfig() begin

  writeReg(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
  writeReg(GPIO_HV_MUX_ACTIVE_HIGH, readReg(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
  writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

  // -- VL53L0X_SetGpioConfig() end

  measurement_timing_budget_us = getMeasurementTimingBudget();

  // "Disable MSRC and TCC by default"
  // MSRC = Minimum Signal Rate Check
  // TCC = Target CentreCheck
  // -- VL53L0X_SetSequenceStepEnable() begin

  writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

  // -- VL53L0X_SetSequenceStepEnable() end

  // "Recalculate timing budget"
  setMeasurementTimingBudget(measurement_timing_budget_us);

  // VL53L0X_StaticInit() end

  // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

  // -- VL53L0X_perform_vhv_calibration() begin

  writeReg(SYSTEM_SEQUENCE_CONFIG, 0x01);
  /* VHV 参考校准失败时不能继续启动测量。 */
  if (!performSingleRefCalibration(0x40)) { return false; }

  // -- VL53L0X_perform_vhv_calibration() end

  // -- VL53L0X_perform_phase_calibration() begin

  writeReg(SYSTEM_SEQUENCE_CONFIG, 0x02);
  /* 相位参考校准失败时将初始化结果标记为失败。 */
  if (!performSingleRefCalibration(0x00)) { return false; }

  // -- VL53L0X_perform_phase_calibration() end

  // "restore the previous Sequence Config"
  writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

  // VL53L0X_PerformRefCalibration() end

  return true;
}

// Write an 8-bit register
/* 写单字节寄存器，底层会锁存首次通信错误。 */
void VL53L0X::writeReg(uint8_t reg, uint8_t value)
{
  writeMulti(reg, &value, 1);
}
/* 将十六位值按高字节在前写入。 */
void VL53L0X::writeReg16Bit(uint8_t reg, uint16_t value)
{
  uint8_t bytes[2] = {(uint8_t)(value >> 8), (uint8_t)value};
  writeMulti(reg, bytes, 2);
}
/* 将三十二位值按高字节在前写入。 */
void VL53L0X::writeReg32Bit(uint8_t reg, uint32_t value)
{
  uint8_t bytes[4] = {(uint8_t)(value >> 24), (uint8_t)(value >> 16),
                      (uint8_t)(value >> 8), (uint8_t)value};
  writeMulti(reg, bytes, 4);
}
/* 读取单字节寄存器；失败返回零，错误原因保留在 bus_error。 */
uint8_t VL53L0X::readReg(uint8_t reg)
{
  uint8_t value = 0;
  readMulti(reg, &value, 1);
  return value;
}
/* 读取并合并十六位寄存器。 */
uint16_t VL53L0X::readReg16Bit(uint8_t reg)
{
  uint8_t bytes[2];
  readMulti(reg, bytes, 2);
  return ((uint16_t)bytes[0] << 8) | bytes[1];
}
/* 读取并合并三十二位寄存器。 */
uint32_t VL53L0X::readReg32Bit(uint8_t reg)
{
  uint8_t bytes[4];
  readMulti(reg, bytes, 4);
  return ((uint32_t)bytes[0] << 24) | ((uint32_t)bytes[1] << 16) |
         ((uint32_t)bytes[2] << 8) | bytes[3];
}
/* 通过共享 I2C1 写寄存器；HAL 地址只在此处左移，单次最多等待 10 ms。 */
void VL53L0X::writeMulti(uint8_t reg, uint8_t const *src, uint8_t count)
{
  /* 一次配置中任何访问失败，后续写操作均停止，避免部分配置被误认为成功。 */
  if (bus_error != HAL_OK) { return; }
  bus_error = HAL_I2C_Mem_Write(&hi2c1, (uint16_t)address << 1, reg,
               I2C_MEMADD_SIZE_8BIT, (uint8_t *)src, count, 10U);
  last_status = (uint8_t)bus_error;
}
/* 通过共享 I2C1 读取连续寄存器；失败清零输出，状态由上层统一检查。 */
void VL53L0X::readMulti(uint8_t reg, uint8_t *dst, uint8_t count)
{
  memset(dst, 0, count);
  if (bus_error != HAL_OK) { return; }
  bus_error = HAL_I2C_Mem_Read(&hi2c1, (uint16_t)address << 1, reg,
               I2C_MEMADD_SIZE_8BIT, dst, count, 10U);
  last_status = (uint8_t)bus_error;
}

/* 设置最低返回信号强度阈值。 */
bool VL53L0X::setSignalRateLimit(float limit_Mcps)
{
  if (limit_Mcps < 0 || limit_Mcps > 511.99) { return false; }

  // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
  writeReg16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit_Mcps * (1 << 7));
  return true;
}

// Get the return signal rate limit check value in MCPS
/* 读取最低返回信号强度阈值。 */
float VL53L0X::getSignalRateLimit()
{
  return (float)readReg16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7);
}

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement; the ST API and this library take care of splitting the
// timing budget among the sub-steps in the ranging sequence. A longer timing
// budget allows for more accurate measurements. Increasing the budget by a
// factor of N decreases the range measurement standard deviation by a factor of
// sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
// based on VL53L0X_set_measurement_timing_budget_micro_seconds()
/* 分配测距时序预算，拒绝短于规定下限的预算。 */
bool VL53L0X::setMeasurementTimingBudget(uint32_t budget_us)
{
  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead     = 1910;
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  uint32_t used_budget_us = StartOverhead + EndOverhead;

  getSequenceStepEnables(&enables);
  getSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss)
  {
    used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables.msrc)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range)
  {
    used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range)
  {
    used_budget_us += FinalRangeOverhead;

    // "Note that the final range timeout is determined by the timing
    // budget and the sum of all other timeouts within the sequence.
    // If there is no room for the final range timeout, then an error
    // will be set. Otherwise the remaining time will be applied to
    // the final range."

    /* 剩余预算不足时拒绝配置，防止无符号减法下溢。 */
    if (used_budget_us > budget_us)
    {
      // "Requested timeout too big."
      return false;
    }

    uint32_t final_range_timeout_us = budget_us - used_budget_us;

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    uint32_t final_range_timeout_mclks =
      timeoutMicrosecondsToMclks(final_range_timeout_us,
                                 timeouts.final_range_vcsel_period_pclks);

    if (enables.pre_range)
    {
      final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    writeReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      encodeTimeout(final_range_timeout_mclks));

    // set_sequence_step_timeout() end

    measurement_timing_budget_us = budget_us; // store for internal reuse
  }
  return true;
}

// Get the measurement timing budget in microseconds
// based on VL53L0X_get_measurement_timing_budget_micro_seconds()
// in us
/* 根据序列各阶段计算当前测距时序预算。 */
uint32_t VL53L0X::getMeasurementTimingBudget()
{
  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead     = 1910;
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  // "Start and end overhead times always present"
  uint32_t budget_us = StartOverhead + EndOverhead;

  getSequenceStepEnables(&enables);
  getSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss)
  {
    budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables.msrc)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range)
  {
    budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range)
  {
    budget_us += (timeouts.final_range_us + FinalRangeOverhead);
  }

  measurement_timing_budget_us = budget_us; // store for internal reuse
  return budget_us;
}

// Set the VCSEL (vertical cavity surface emitting laser) pulse period for the
// given period type (pre-range or final range) to the given value in PCLKs.
// Longer periods seem to increase the potential range of the sensor.
// Valid values are (even numbers only):
//  pre:  12 to 18 (initialized default: 14)
//  final: 8 to 14 (initialized default: 10)
// based on VL53L0X_set_vcsel_pulse_period()
/* 调整激光脉冲周期并重新校准相位。 */
bool VL53L0X::setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks)
{
  uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);

  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  getSequenceStepEnables(&enables);
  getSequenceStepTimeouts(&enables, &timeouts);

  // "Apply specific settings for the requested clock period"
  // "Re-calculate and apply timeouts, in macro periods"

  // "When the VCSEL period for the pre or final range is changed,
  // the corresponding timeout must be read from the device using
  // the current VCSEL period, then the new VCSEL period can be
  // applied. The timeout then must be written back to the device
  // using the new VCSEL period.
  //
  // For the MSRC timeout, the same applies - this timeout being
  // dependant on the pre-range vcsel period."


  if (type == VcselPeriodPreRange)
  {
    // "Set phase check limits"
    switch (period_pclks)
    {
      case 12:
        writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
        break;

      case 14:
        writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
        break;

      case 16:
        writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
        break;

      case 18:
        writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
        break;

      default:
        // invalid period
        return false;
    }
    writeReg(PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

    // apply new VCSEL period
    writeReg(PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

    uint16_t new_pre_range_timeout_mclks =
      timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);

    writeReg16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      encodeTimeout(new_pre_range_timeout_mclks));

    // set_sequence_step_timeout() end

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

    uint16_t new_msrc_timeout_mclks =
      timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);

    writeReg(MSRC_CONFIG_TIMEOUT_MACROP,
      (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));

    // set_sequence_step_timeout() end
  }
  else if (type == VcselPeriodFinalRange)
  {
    switch (period_pclks)
    {
      case 8:
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
        writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
        writeReg(0xFF, 0x01);
        writeReg(ALGO_PHASECAL_LIM, 0x30);
        writeReg(0xFF, 0x00);
        break;

      case 10:
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
        writeReg(0xFF, 0x01);
        writeReg(ALGO_PHASECAL_LIM, 0x20);
        writeReg(0xFF, 0x00);
        break;

      case 12:
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
        writeReg(0xFF, 0x01);
        writeReg(ALGO_PHASECAL_LIM, 0x20);
        writeReg(0xFF, 0x00);
        break;

      case 14:
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
        writeReg(0xFF, 0x01);
        writeReg(ALGO_PHASECAL_LIM, 0x20);
        writeReg(0xFF, 0x00);
        break;

      default:
        // invalid period
        return false;
    }

    // apply new VCSEL period
    writeReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    uint16_t new_final_range_timeout_mclks =
      timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);

    if (enables.pre_range)
    {
      new_final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    writeReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      encodeTimeout(new_final_range_timeout_mclks));

    // set_sequence_step_timeout end
  }
  else
  {
    // invalid type
    return false;
  }

  // "Finally, the timing budget must be re-applied"

  setMeasurementTimingBudget(measurement_timing_budget_us);

  // "Perform the phase calibration. This is needed after changing on vcsel period."
  // VL53L0X_perform_phase_calibration() begin

  uint8_t sequence_config = readReg(SYSTEM_SEQUENCE_CONFIG);
  writeReg(SYSTEM_SEQUENCE_CONFIG, 0x02);
  performSingleRefCalibration(0x0);
  writeReg(SYSTEM_SEQUENCE_CONFIG, sequence_config);

  // VL53L0X_perform_phase_calibration() end

  return true;
}

// Get the VCSEL pulse period in PCLKs for the given period type.
// based on VL53L0X_get_vcsel_pulse_period()
/* 读取指定阶段的激光脉冲周期。 */
uint8_t VL53L0X::getVcselPulsePeriod(vcselPeriodType type)
{
  if (type == VcselPeriodPreRange)
  {
    return decodeVcselPeriod(readReg(PRE_RANGE_CONFIG_VCSEL_PERIOD));
  }
  else if (type == VcselPeriodFinalRange)
  {
    return decodeVcselPeriod(readReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD));
  }
  else { return 255; }
}

// Start continuous ranging measurements. If period_ms (optional) is 0 or not
// given, continuous back-to-back mode is used (the sensor takes measurements as
// often as possible); otherwise, continuous timed mode is used, with the given
// inter-measurement period in milliseconds determining how often the sensor
// takes a measurement.
// based on VL53L0X_StartMeasurement()
/* 恢复停止变量并启动连续测距，可指定测量间隔。 */
void VL53L0X::startContinuous(uint32_t period_ms)
{
  writeReg(0x80, 0x01);
  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x00);
  writeReg(0x91, stop_variable);
  writeReg(0x00, 0x01);
  writeReg(0xFF, 0x00);
  writeReg(0x80, 0x00);

  if (period_ms != 0)
  {
    // continuous timed mode

    // VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin

    uint16_t osc_calibrate_val = readReg16Bit(OSC_CALIBRATE_VAL);

    if (osc_calibrate_val != 0)
    {
      period_ms *= osc_calibrate_val;
    }

    writeReg32Bit(SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);

    // VL53L0X_SetInterMeasurementPeriodMilliSeconds() end

    writeReg(SYSRANGE_START, 0x04); // VL53L0X_REG_SYSRANGE_MODE_TIMED
  }
  else
  {
    // continuous back-to-back mode
    writeReg(SYSRANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
  }
}

// Stop continuous measurements
// based on VL53L0X_StopMeasurement()
/* 停止连续测距并恢复内部状态。 */
void VL53L0X::stopContinuous()
{
  writeReg(SYSRANGE_START, 0x01); // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT

  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x00);
  writeReg(0x91, 0x00);
  writeReg(0x00, 0x01);
  writeReg(0xFF, 0x00);
}

// Returns a range reading in millimeters when continuous mode is active
// (readRangeSingleMillimeters() also calls this function after starting a
// single-shot range measurement)
/* 等待并读取连续测距结果，上层主循环不使用此阻塞接口。 */
uint16_t VL53L0X::readRangeContinuousMillimeters()
{
  startTimeout();
  /* 等待器件状态；循环内检查通信错误与超时，避免永久等待。 */
  while ((readReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0)
  {
    if (checkTimeoutExpired())
    {
      did_timeout = true;
      return 65535;
    }
  }

  // assumptions: Linearity Corrective Gain is 1000 (default);
  // fractional ranging is not enabled
  uint16_t range = readReg16Bit(RESULT_RANGE_STATUS + 10);

  writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

  return range;
}

// Performs a single-shot range measurement and returns the reading in
// millimeters
// based on VL53L0X_PerformSingleRangingMeasurement()
/* 触发并等待一次测距，上层主循环不使用此阻塞接口。 */
uint16_t VL53L0X::readRangeSingleMillimeters()
{
  writeReg(0x80, 0x01);
  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x00);
  writeReg(0x91, stop_variable);
  writeReg(0x00, 0x01);
  writeReg(0xFF, 0x00);
  writeReg(0x80, 0x00);

  writeReg(SYSRANGE_START, 0x01);

  // "Wait until start bit has been cleared"
  startTimeout();
  /* 等待器件状态；循环内检查通信错误与超时，避免永久等待。 */
  while (readReg(SYSRANGE_START) & 0x01)
  {
    if (checkTimeoutExpired())
    {
      did_timeout = true;
      return 65535;
    }
  }

  return readRangeContinuousMillimeters();
}

// Did a timeout occur in one of the read functions since the last call to
// timeoutOccurred()?
/* 读取并清除历史超时标志。 */
bool VL53L0X::timeoutOccurred()
{
  bool tmp = did_timeout;
  did_timeout = false;
  return tmp;
}

// Private Methods /////////////////////////////////////////////////////////////

// Get reference SPAD (single photon avalanche diode) count and type
// based on VL53L0X_get_info_from_device(),
// but only gets reference SPAD count and type
/* 读取参考 SPAD 数量和类型，等待过程受超时限制。 */
bool VL53L0X::getSpadInfo(uint8_t * count, bool * type_is_aperture)
{
  uint8_t tmp;

  writeReg(0x80, 0x01);
  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x00);

  writeReg(0xFF, 0x06);
  writeReg(0x83, readReg(0x83) | 0x04);
  writeReg(0xFF, 0x07);
  writeReg(0x81, 0x01);

  writeReg(0x80, 0x01);

  writeReg(0x94, 0x6b);
  writeReg(0x83, 0x00);
  startTimeout();
  /* 等待器件状态；循环内检查通信错误与超时，避免永久等待。 */
  while (readReg(0x83) == 0x00)
  {
    if (checkTimeoutExpired()) { return false; }
  }
  writeReg(0x83, 0x01);
  tmp = readReg(0x92);

  *count = tmp & 0x7f;
  *type_is_aperture = (tmp >> 7) & 0x01;

  writeReg(0x81, 0x00);
  writeReg(0xFF, 0x06);
  writeReg(0x83, readReg(0x83)  & ~0x04);
  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x01);

  writeReg(0xFF, 0x00);
  writeReg(0x80, 0x00);

  return true;
}

// Get sequence step enables
// based on VL53L0X_GetSequenceStepEnables()
/* 读取当前测量序列各阶段使能状态。 */
void VL53L0X::getSequenceStepEnables(SequenceStepEnables * enables)
{
  uint8_t sequence_config = readReg(SYSTEM_SEQUENCE_CONFIG);

  enables->tcc          = (sequence_config >> 4) & 0x1;
  enables->dss          = (sequence_config >> 3) & 0x1;
  enables->msrc         = (sequence_config >> 2) & 0x1;
  enables->pre_range    = (sequence_config >> 6) & 0x1;
  enables->final_range  = (sequence_config >> 7) & 0x1;
}

// Get sequence step timeouts
// based on get_sequence_step_timeout(),
// but gets all timeouts instead of just the requested one, and also stores
// intermediate values
/* 读取各阶段超时并换算时间单位。 */
void VL53L0X::getSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts)
{
  timeouts->pre_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodPreRange);

  timeouts->msrc_dss_tcc_mclks = readReg(MSRC_CONFIG_TIMEOUT_MACROP) + 1;
  timeouts->msrc_dss_tcc_us =
    timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->pre_range_mclks =
    decodeTimeout(readReg16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
  timeouts->pre_range_us =
    timeoutMclksToMicroseconds(timeouts->pre_range_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->final_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodFinalRange);

  timeouts->final_range_mclks =
    decodeTimeout(readReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

  if (enables->pre_range)
  {
    timeouts->final_range_mclks -= timeouts->pre_range_mclks;
  }

  timeouts->final_range_us =
    timeoutMclksToMicroseconds(timeouts->final_range_mclks,
                               timeouts->final_range_vcsel_period_pclks);
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
/* 将器件指数格式超时解码为时钟周期数。 */
uint16_t VL53L0X::decodeTimeout(uint16_t reg_val)
{
  // format: "(LSByte * 2^MSByte) + 1"
  return (uint16_t)((reg_val & 0x00FF) <<
         (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
/* 将时钟周期数编码为器件指数格式。 */
uint16_t VL53L0X::encodeTimeout(uint32_t timeout_mclks)
{
  // format: "(LSByte * 2^MSByte) + 1"

  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;

  if (timeout_mclks > 0)
  {
    ls_byte = timeout_mclks - 1;

    /* 遍历配置项或等待器件状态；器件等待在循环内检查超时。 */
    while ((ls_byte & 0xFFFFFF00) > 0)
    {
      ls_byte >>= 1;
      ms_byte++;
    }

    return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  else { return 0; }
}

// Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_us()
/* 将测量时钟周期数换算为微秒。 */
uint32_t VL53L0X::timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return ((timeout_period_mclks * macro_period_ns) + 500) / 1000;
}

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()
/* 将微秒换算为测量时钟周期数。 */
uint32_t VL53L0X::timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}


// based on VL53L0X_perform_single_ref_calibration()
/* 执行一次参考校准，超时或通信失败则退出。 */
bool VL53L0X::performSingleRefCalibration(uint8_t vhv_init_byte)
{
  writeReg(SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

  startTimeout();
  /* 等待器件状态；循环内检查通信错误与超时，避免永久等待。 */
  while ((readReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0)
  {
    if (checkTimeoutExpired()) { return false; }
  }

  writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

  writeReg(SYSRANGE_START, 0x00);

  return true;
}
