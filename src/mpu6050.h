#include <stdint.h>

struct mpu6050_data {
  int16_t x_accel;
  int16_t y_accel;
  int16_t z_accel;
  int16_t temp;
  int16_t x_gyro;
  int16_t y_gyro;
  int16_t z_gyro;
} __attribute__ ((packed));



int mpu6050_init(struct device *i2c, u8_t addr);
int mpu6050_read(struct device *i2c, u8_t addr, struct mpu6050_data *buf);
