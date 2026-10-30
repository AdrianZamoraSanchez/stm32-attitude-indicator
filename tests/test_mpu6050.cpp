#include "driver/MPU6050.h"
#include "fakes/FakeI2C.h"

#include <algorithm>
#include <gtest/gtest.h>

namespace mpu = drivers::MPU6050;

namespace {

using Driver = mpu::MPU6050<HAL::I2C<FakeI2C>, HAL::Time<FakeTime>>;

class MPU6050Test : public ::testing::Test {
protected:
  FakeI2C i2c_fake;
  FakeTime time_fake;

  HAL::I2C<FakeI2C> &i2c = i2c_fake;
  HAL::Time<FakeTime> &time = time_fake;

  Driver imu{i2c, time};

  void SetUp() override { i2c_fake.regs[mpu::WHO_AM_I] = mpu::WHO_AM_I_MPU6050; }

  bool was_written(u8 reg, u8 value) const {
    return std::any_of(i2c_fake.writes.begin(), i2c_fake.writes.end(),
                       [&](const FakeI2C::RegWrite &w) {
                         return w.reg == reg && w.value == value;
                       });
  }
};

} // namespace

// ---- init ------------------------------------------------------------------

TEST_F(MPU6050Test, InitResetsFirstAndConfiguresRanges) {
  ASSERT_TRUE(imu.init());

  ASSERT_FALSE(i2c_fake.writes.empty());
  EXPECT_EQ(i2c_fake.writes.front().reg, mpu::PWR_MGMT_1);
  EXPECT_EQ(i2c_fake.writes.front().value, mpu::PWR_RESET);

  EXPECT_TRUE(was_written(mpu::PWR_MGMT_1, mpu::PWR_WAKE_PLL_XGYRO));
  EXPECT_TRUE(was_written(mpu::MPU_CONFIG, mpu::DLPF_20HZ));
  EXPECT_TRUE(was_written(mpu::SMPLRT_DIV, mpu::SAMPLE_RATE_100HZ));
  EXPECT_TRUE(was_written(mpu::GYRO_CONFIG, mpu::GYRO_FS_250DPS));
  EXPECT_TRUE(was_written(mpu::ACCEL_CONFIG, mpu::ACCEL_FS_2G));
}

TEST_F(MPU6050Test, InitWaitsAfterReset) {
  ASSERT_TRUE(imu.init());
  EXPECT_GE(time_fake.elapsed_ms, 100U);
}

TEST_F(MPU6050Test, InitAcceptsMPU6500) {
  i2c_fake.regs[mpu::WHO_AM_I] = mpu::WHO_AM_I_MPU6500;
  EXPECT_TRUE(imu.init());
}

TEST_F(MPU6050Test, InitRejectsUnknownDevice) {
  i2c_fake.regs[mpu::WHO_AM_I] = 0x12;
  EXPECT_FALSE(imu.init());
  EXPECT_TRUE(i2c_fake.writes.empty()); // never touches an unknown chip
}

TEST_F(MPU6050Test, InitFailsWhenNoDeviceAcks) {
  i2c_fake.device_addr = 0x69; // AD0 high: nobody at 0x68
  EXPECT_FALSE(imu.init());
}

// ---- Reading data ----------------------------------------------------------

TEST_F(MPU6050Test, ReadAccelDecodesBigEndianSignedValues) {
  i2c_fake.set_i16(mpu::ACCEL_X_HI, -1234);
  i2c_fake.set_i16(mpu::ACCEL_Y_HI, 16384);
  i2c_fake.set_i16(mpu::ACCEL_Z_HI, -32768);

  core::AxesMeasurement m;
  ASSERT_TRUE(imu.read_accel(m));

  EXPECT_EQ(m.getAxes().data_x, -1234);
  EXPECT_EQ(m.getAxes().data_y, 16384);
  EXPECT_EQ(m.getAxes().data_z, -32768);
}

TEST_F(MPU6050Test, ReadFailsWhenDeviceDisappears) {
  i2c_fake.device_addr = 0x00;
  core::AxesMeasurement m;
  EXPECT_FALSE(imu.read_accel(m));
  EXPECT_FALSE(imu.read_gyro(m));
}

TEST_F(MPU6050Test, AccelOrientationWhenFlat) {
  i2c_fake.set_i16(mpu::ACCEL_Z_HI, 16384); // 1 g on Z

  core::AxesMeasurement m;
  ASSERT_TRUE(imu.read_accel_orientation(m));

  EXPECT_NEAR(m.getOrientation().roll, 0.0f, 0.01f);
  EXPECT_NEAR(m.getOrientation().pitch, 0.0f, 0.01f);
}

// ---- Gyro calibration ------------------------------------------------------

TEST_F(MPU6050Test, CalibrationRemovesBias) {
  i2c_fake.set_i16(mpu::GYRO_X_HI, 100);
  i2c_fake.set_i16(mpu::GYRO_Y_HI, -50);
  i2c_fake.set_i16(mpu::GYRO_Z_HI, 7);

  ASSERT_TRUE(imu.calibrate_gyro(10, 2));
  EXPECT_EQ(time_fake.elapsed_ms, 20U);

  core::AxesMeasurement m;
  ASSERT_TRUE(imu.read_gyro(m));
  EXPECT_EQ(m.getAxes().data_x, 0);
  EXPECT_EQ(m.getAxes().data_y, 0);
  EXPECT_EQ(m.getAxes().data_z, 0);

  // A real rotation on top of the bias shows up unbiased
  i2c_fake.set_i16(mpu::GYRO_X_HI, 100 + 131); // +1 deg/s
  ASSERT_TRUE(imu.read_gyro(m));
  EXPECT_EQ(m.getAxes().data_x, 131);
  EXPECT_FLOAT_EQ(Driver::gyro_raw_to_dps(m.getAxes().data_x), 1.0f);
}

TEST_F(MPU6050Test, BiasCorrectionSaturatesInsteadOfWrapping) {
  i2c_fake.set_i16(mpu::GYRO_X_HI, -30000);
  ASSERT_TRUE(imu.calibrate_gyro(1, 0));

  i2c_fake.set_i16(mpu::GYRO_X_HI, 30000); // 30000 - (-30000) = 60000

  core::AxesMeasurement m;
  ASSERT_TRUE(imu.read_gyro(m));
  EXPECT_EQ(m.getAxes().data_x, 32767);
}

TEST_F(MPU6050Test, CalibrationFailsWithoutSamples) {
  EXPECT_FALSE(imu.calibrate_gyro(0));
}

TEST_F(MPU6050Test, CalibrationFailsWhenEveryReadFails) {
  i2c_fake.device_addr = 0x00;
  EXPECT_FALSE(imu.calibrate_gyro(5, 1));
}

// ---- Unit conversion -------------------------------------------------------

TEST(MPU6050Scale, RawToPhysicalUnits) {
  EXPECT_FLOAT_EQ(Driver::accel_raw_to_g(16384), 1.0f);
  EXPECT_FLOAT_EQ(Driver::accel_raw_to_g(-8192), -0.5f);
  EXPECT_FLOAT_EQ(Driver::gyro_raw_to_dps(131), 1.0f);
  EXPECT_FLOAT_EQ(Driver::gyro_raw_to_dps(-262), -2.0f);
}
