#include "mpu6050.h"
#include <string.h>
#include <math.h>

#define I2C_TIMEOUT	100

// ---------FUNCTIONS---------------

// low level read/writes

status_t read_regs(mpu6050_t *dev, uint8_t reg, uint8_t *buf, uint16_t len) {
	if (HAL_I2C_Mem_Read(dev->hi2c, dev->addr, reg, I2C_MEMADD_SIZE_8BIT, buf, len, I2C_TIMEOUT) != HAL_OK) {
		return ERR_I2C;
	}
	return OK;
}

status_t write_reg(mpu6050_t *dev, uint8_t reg, uint8_t buf) {
	if (HAL_I2C_Mem_Write(dev->hi2c, dev->addr, reg, I2C_MEMADD_SIZE_8BIT, &buf, 1, I2C_TIMEOUT) != HAL_OK) {
		return ERR_I2C;
	}
	return OK;
}

status_t write_bits(mpu6050_t *dev, uint8_t reg, uint8_t mask, uint8_t shift, uint8_t val) {
	uint8_t cur;
	if (read_regs(dev, reg, &cur, 1) != OK) {
		return ERR_I2C;
	}

	cur = (uint8_t)((cur & ~mask) | ((val << shift) & mask));
	return write_reg(dev, reg, cur);
}

// verify id

bool mpu6050_whoami(mpu6050_t *dev) {
	uint8_t addr;
	if (read_regs(dev, MPU6050_WHO_AM_I, &addr, 1) != OK) {
		return false;
	}

	if (addr != 0x68) {
		return false;
	}
	return true;
}

// reset/sleep/init

status_t mpu6050_reset(mpu6050_t *dev) {
	if (write_reg(dev, MPU6050_PWR_MGMT_1, 0x80) != OK) {
		return ERR_I2C;
	}
	return OK;
}

status_t mpu6050_sleep(mpu6050_t *dev, bool sleep) {
	if (write_bits(dev, MPU6050_PWR_MGMT_1, 0x40, 6, sleep) != OK) {
		return ERR_I2C;
	}
	return OK;
}

status_t mpu6050_init(mpu6050_t *dev, I2C_HandleTypeDef *hi2c, uint8_t ad0, config_t *cfg) {
	if (dev == NULL || hi2c == NULL || cfg == NULL) {
		return ERR_PARAM;
	}

	memset(dev, 0, sizeof(*dev));
	dev->hi2c = hi2c;
	dev->addr = (uint8_t)((ad0 ? MPU6050_ADDR_AD0_HIGH : MPU6050_ADDR_AD0_LOW) << 1);
	dev->cfg = *cfg;
	dev->state = IDLE;

	if (mpu6050_whoami(dev) != true) {
		return ERR_WHO_AM_I;
	}

	// wakeup
	if (mpu6050_sleep(dev, false) != OK) {
		return ERR_I2C;
	}

	// clock select
	if (write_bits(dev, MPU6050_PWR_MGMT_1, 0x7, 0, cfg->clksel) != OK) {
		return ERR_I2C;
	}

	// accel
	if (set_accel(dev, cfg->accel_fs) != OK) {
		return ERR_I2C;
	}

	// gyro
	if (set_gyro(dev, cfg->gyro_fs) != OK) {
		return ERR_I2C;
	}

	// dlpf
	if (set_dlpf(dev, cfg->dlpf) != OK) {
		return ERR_I2C;
	}
	// smplrtdiv
	if (set_smplrt(dev, cfg->smplrt_div) != OK) {
		return ERR_I2C;
	}
	// enable interrupt
	if (cfg->interrupt) {
		if (enable_data_ready_int(dev) != OK) {
				return ERR_I2C;
			}
	}

	return OK;
}

// configurations (set accel/gyro/etc)

static float accel_fs_to_sens(accel_fs_t fs) {
	switch (fs) {
		case ACCEL_2g: return 16384.0f;
		case ACCEL_4g: return 8192.0f;
		case ACCEL_8g: return 4096.0f;
		case ACCEL_16g: return 2048.0f;
		default: return 16384.0f;
	}
}

status_t set_accel(mpu6050_t *dev, accel_fs_t fs) {
	if (write_bits(dev, MPU6050_ACCEL_CONFIG, 0x18, 3, fs) != OK) {
		return ERR_I2C;
	}
	dev->cfg.accel_fs = fs;
	dev->accel_sens = accel_fs_to_sens(fs);

	return OK;
}

static float gyro_fs_to_sens(gyro_fs_t fs) {
	switch (fs) {
		case GYRO_250: return 131.0f;
		case GYRO_500: return 65.5f;
		case GYRO_1000: return 32.8f;
		case GYRO_2000: return 16.4f;
		default: return 131.0f;
	}
}

status_t set_gyro(mpu6050_t *dev, gyro_fs_t fs) {
	if (write_bits(dev, MPU6050_GYRO_CONFIG, 0x18, 3, fs) != OK) {
		return ERR_I2C;
	}
	dev->cfg.gyro_fs = fs;
	dev->gyro_sens = gyro_fs_to_sens(fs);

	return OK;
}

status_t set_dlpf(mpu6050_t *dev, dlpf_t dlpf) {
	if (write_bits(dev, MPU6050_CONFIG, 0x7, 0, dlpf) != OK) {
		return ERR_I2C;
	}

	dev->cfg.dlpf = dlpf;

	return OK;
}

status_t set_smplrt(mpu6050_t *dev, uint16_t hz) {
	// if dlpf is off (260 Hz, then gyro rate is 8000, otherwise 1000)
	uint16_t gyro_rate = ( (dev->cfg.dlpf == DLPF_260) ? 8000 : 1000);

	if (hz == 0 || hz > gyro_rate) {
		return ERR_PARAM;
	}

	uint32_t raw_div = ((gyro_rate/hz)-1);
	if (raw_div > 255) {
		return ERR_PARAM;
	}

	uint8_t div = (uint8_t)raw_div;
	if (write_reg(dev, MPU6050_SMPLRT_DIV, div) != OK) {
		return ERR_I2C;
	}
	dev->cfg.smplrt_div = div;

	return OK;
}

// blocking reads

static void convert_raw_accel(mpu6050_t *dev) {
	int16_t x_raw = (dev->raw_buf[0] << 8) | (dev->raw_buf[1]);
	int16_t y_raw = (dev->raw_buf[2] << 8) | (dev->raw_buf[3]);
	int16_t z_raw = (dev->raw_buf[4] << 8) | (dev->raw_buf[5]);

	float sens = dev->accel_sens;

	vec accel_calc = {
			x_raw / sens,// - dev->calib.accel_bias.x,
			y_raw / sens,// - dev->calib.accel_bias.y,
			z_raw / sens// - dev->calib.accel_bias.z
	};

	dev->accel_data = accel_calc;
}

status_t read_accel(mpu6050_t *dev) {
	// read from registers, return if ok
	if (read_regs(dev, MPU6050_ACCEL_XOUT_H, dev->raw_buf, 6) != OK) {
		return ERR_I2C;
	}
	// call static function
	convert_raw_accel(dev);
	return OK;
}

static void convert_raw_gyro(mpu6050_t *dev) {
	int16_t x_raw = (dev->raw_buf[8] << 8) | (dev->raw_buf[9]);
	int16_t y_raw = (dev->raw_buf[10] << 8) | (dev->raw_buf[11]);
	int16_t z_raw = (dev->raw_buf[12] << 8) | (dev->raw_buf[13]);

	float sens = dev->gyro_sens;

	vec gyro_calc = {
			x_raw / sens - dev->calib.gyro_bias.x,
			y_raw / sens - dev->calib.gyro_bias.y,
			z_raw / sens - dev->calib.gyro_bias.z
	};

	dev->gyro_data = gyro_calc;
}
status_t read_gyro(mpu6050_t *dev) {
	// read from registers, return if ok
	if (read_regs(dev, MPU6050_GYRO_XOUT_H, dev->raw_buf+8, 6) != OK) {
		return ERR_I2C;
	}
	// call static function
	convert_raw_gyro(dev);
	return OK;
}

static void convert_raw_temp(mpu6050_t *dev) {
	int16_t raw = (dev->raw_buf[6] << 8) | (dev->raw_buf[7]);

	dev->temp_data = raw/340.0 + 36.53;
}
status_t read_temp(mpu6050_t *dev) {
	// read from registers, return if ok
	if (read_regs(dev, MPU6050_TEMP_OUT_H, dev->raw_buf+6, 2) != OK) {
		return ERR_I2C;
	}
	// call static function
	convert_raw_temp(dev);
	return OK;
}

status_t read_all(mpu6050_t *dev) {
	if (read_regs(dev, MPU6050_ACCEL_XOUT_H, dev->raw_buf, 14) != OK) {
		return ERR_I2C;
	}
	convert_raw_accel(dev);
	convert_raw_gyro(dev);
	convert_raw_temp(dev);

	return OK;
}

// nonblocking reads + callback

status_t read_all_it(mpu6050_t *dev) {
	if (dev->state != READY && dev->state != IDLE) {
		return ERR_NOT_READY;
	} else {
		dev->state = BUSY;
		if (HAL_I2C_Mem_Read_IT(dev->hi2c, dev->addr, MPU6050_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, dev->raw_buf, 14) != HAL_OK) {
			dev->state = ERROR_STATE;
			return ERR_I2C;
		}
		return OK;
	}
}

status_t read_all_dma(mpu6050_t *dev) {
	if (dev->state != READY && dev->state != IDLE) {
		return ERR_NOT_READY;
	} else {
		dev->state = BUSY;
		if (HAL_I2C_Mem_Read_DMA(dev->hi2c, dev->addr, MPU6050_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, dev->raw_buf, 14) != HAL_OK) {
			dev->state = ERROR_STATE;
			return ERR_I2C;
		}
	}
	return OK;
}

void mpu6050_i2c_rx_cmplt_callback(mpu6050_t *dev) {
	convert_raw_accel(dev);
	convert_raw_gyro(dev);
	convert_raw_temp(dev);

	dev->state = READY;
}

// calibration

status_t mpu6050_calibrate(mpu6050_t *dev, uint8_t num_samples) {
	if (num_samples == 0) {
		return ERR_PARAM;
	}

	float sum_ax = 0, sum_ay = 0, sum_az = 0, sum_gx = 0, sum_gy = 0, sum_gz = 0;

	// save old calibration if something goes wrong
	calib_t prev_calib = dev->calib;
	dev->calib.accel_bias = (vec){0};
	dev->calib.gyro_bias = (vec){0};

	for (uint16_t i = 0; i < num_samples; i++) {
		// if read doesnt work, cancel and use previously saved values
		if (read_all(dev) != OK) {
			dev->calib = prev_calib;
			return ERR_I2C;
		}

		sum_ax += (dev->accel_data.x);
		sum_ay += (dev->accel_data.y);
		sum_az += (dev->accel_data.z);
		sum_gx += (dev->gyro_data.x);
		sum_gy += (dev->gyro_data.y);
		sum_gz += (dev->gyro_data.z);
		// delay between reads so data values dont get reread
		HAL_Delay(2);
	}

	dev->calib.accel_bias.x = sum_ax / num_samples;
	dev->calib.accel_bias.y = sum_ay / num_samples;
	dev->calib.accel_bias.z = sum_az / num_samples;
	dev->calib.gyro_bias.x = sum_gx / num_samples;
	dev->calib.gyro_bias.y = sum_gy / num_samples;
	dev->calib.gyro_bias.z = sum_gz / num_samples;

	dev->calib.is_calib = true;

	return OK;
}

// fifo (config + reads)

status_t mpu6050_fifo_enable(mpu6050_t *dev) {
	if (write_bits(dev, MPU6050_USER_CTRL, 0x40, 6, 1) != OK) {
		return ERR_I2C;
	}

	if (write_reg(dev, MPU6050_FIFO_EN, 0x78) != OK) {
		return ERR_I2C;
	}

	return OK;
}

status_t mpu6050_fifo_reset(mpu6050_t *dev) {
	if (write_bits(dev, MPU6050_USER_CTRL, 0x04, 2, 1) != OK) {
		return ERR_I2C;
	}
	// device needs time to reboot
	HAL_Delay(100);
	return OK;
}

status_t mpu6050_fifo_count(mpu6050_t *dev, uint16_t *count) {
	uint8_t b[2];
	if (read_regs(dev, MPU6050_FIFO_COUNTH, b, 2) != OK) {
		return ERR_I2C;
	} else {
		*count = (b[0] << 8) | (b[1]);
	}

	return OK;
}

status_t mpu6050_fifo_read_packet(mpu6050_t *dev) {
	uint8_t b[12];

	if (read_regs(dev, MPU6050_FIFO_R_W, b, 12) != OK) {
		return ERR_I2C;
	}

	memcpy(dev->raw_buf, b, 6);
	memcpy(dev->raw_buf + 8, b + 6, 6);

	convert_raw_accel(dev);
	convert_raw_gyro(dev);

	return OK;
}

// interrupt configuration

status_t enable_data_ready_int(mpu6050_t *dev) {
	// configure pin (doesnt enable yet)
	if (write_reg(dev, MPU6050_INT_PIN_CFG, 0x00) != OK) {
		return ERR_I2C;
	}

	// enable data ready interrupt
	if (write_bits(dev, MPU6050_INT_ENABLE, 0x01, 0, 1) != OK) {
		return ERR_I2C;
	}

	return OK;
}

status_t clear_int_status(mpu6050_t *dev, uint8_t *status) {
	if (read_regs(dev, MPU6050_INT_STATUS, status, 1) != OK) {
		return ERR_I2C;
	}
	return OK;
}

// pitch/roll calculations
void calculate_pitch_roll(mpu6050_t *dev) {
	const float alpha = 0.98f;
	uint32_t now = HAL_GetTick();

	float dt = 0.1f;
	if (dev->last_update_tick != 0) {
		dt = (now - dev->last_update_tick) / 1000.0f;
		if (dt <= 0.0f || dt > 1.0f) dt = 0.01f;
	}
	dev->last_update_tick = now;

    /* Accel-derived angles (valid only when net acceleration ~= gravity) */
    float accel_pitch = atan2f(-dev->accel_data.x,
                                sqrtf(dev->accel_data.y * dev->accel_data.y +
                                      dev->accel_data.z * dev->accel_data.z)) * 57.2957795f;
    float accel_roll  = atan2f(dev->accel_data.y, dev->accel_data.z) * 57.2957795f;

    /* Gyro-integrated angles */
    float gyro_pitch = dev->pitch + dev->gyro_data.y * dt;
    float gyro_roll  = dev->roll  + dev->gyro_data.x * dt;

    dev->pitch = alpha * gyro_pitch + (1.0f - alpha) * accel_pitch;
    dev->roll  = alpha * gyro_roll  + (1.0f - alpha) * accel_roll;
}



