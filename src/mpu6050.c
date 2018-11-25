/* Basic driver for the MPU6050 IMU
 *
 * Derived from the register map at https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
 */

#include <zephyr.h>
#include <device.h>
#include <i2c.h>
#include <misc/byteorder.h>

#include "mpu6050.h"

#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_DATA_BASE 0x3B
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_WHOAMI 0x75

int mpu6050_init(struct device *i2c, u8_t addr) {
  int err;
  u8_t res;

  // Check that the MPU6050 is working
  err = i2c_reg_read_byte(i2c, addr, MPU6050_WHOAMI, &res);
  if (err) {
    printk("Failed to read mpu6050 identity\n");
    return err;
  }
  if ((res & 0x7E) != 0x68) {
    printk("mpu6050 wmoami check failed\n");
    return 1;
  }

  // 0b00011000 -> FS_SEL=3 -> max range (±2000°/s)
  err = i2c_reg_write_byte(i2c, addr, MPU6050_GYRO_CONFIG, 0x18);
  if (err) {
    printk("Failed to configure gyrpscope\n");
    return err;
  }

  // 0b00011000 -> AFS_SEL=3 -> max range (±16g)
  err = i2c_reg_write_byte(i2c, addr, MPU6050_ACCEL_CONFIG, 0x18);
  if (err) {
    printk("Failed to configure gyrpscope\n");
    return err;
  }

  // 0x00 -> SLEEP=0 -> wake up
  err = i2c_reg_write_byte(i2c, addr, MPU6050_PWR_MGMT_1, 0x00);
  if (err) {
    printk("Failed to wake mpu6050\n");
    return err;
  }

  return 0;
}

int mpu6050_read(struct device *i2c, u8_t addr, struct mpu6050_data *buf) {
  int err;

  err = i2c_burst_read(i2c, addr, MPU6050_DATA_BASE, (u8_t *)buf, sizeof(struct mpu6050_data));

  if (err) {
    printk("Failed to read from mpu6050\n");
    return err;
  }

  buf->x_accel = sys_be16_to_cpu(buf->x_accel);
  buf->y_accel = sys_be16_to_cpu(buf->y_accel);
  buf->z_accel = sys_be16_to_cpu(buf->z_accel);
  buf->temp    = sys_be16_to_cpu(buf->temp);
  buf->x_gyro  = sys_be16_to_cpu(buf->x_gyro);
  buf->y_gyro  = sys_be16_to_cpu(buf->y_gyro);
  buf->z_gyro  = sys_be16_to_cpu(buf->z_gyro);

  return 0;
}
