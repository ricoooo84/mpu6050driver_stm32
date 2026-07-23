#ifndef MPU6050_H
#define MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g0xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// register map

#define MPU6050_SELF_TEST_X		0x0D
#define MPU6050_SELF_TEST_Y		0x0E
#define MPU6050_SELF_TEST_Z		0x0F
#define MPU6050_SELF_TEST_A		0x10
#define MPU6050_SMPLRT_DIV		0x19
#define MPU6050_CONFIG			0x1A
#define MPU6050_GYRO_CONFIG		0x1B
#define MPU6050_ACCEL_CONFIG	0x1C

#define MPU6050_FIFO_EN			0x23

#define MPU6050_INT_PIN_CFG		0x37
#define MPU6050_INT_ENABLE		0x38
#define MPU6050_INT_STATUS		0x3A
#define MPU6050_ACCEL_XOUT_H	0x3B
#define MPU6050_ACCEL_XOUT_L	0x3C
#define MPU6050_ACCEL_YOUT_H	0x3D
#define MPU6050_ACCEL_YOUT_L	0x3E
#define MPU6050_ACCEL_ZOUT_H	0x3F
#define MPU6050_ACCEL_ZOUT_L	0x40
#define MPU6050_TEMP_OUT_H		0x41
#define MPU6050_TEMP_OUT_L		0x42
#define MPU6050_GYRO_XOUT_H		0x43
#define MPU6050_GYRO_XOUT_L		0x44
#define MPU6050_GYRO_YOUT_H		0x45
#define MPU6050_GYRO_YOUT_L		0x46
#define MPU6050_GYRO_ZOUT_H		0x47
#define MPU6050_GYRO_ZOUT_L		0x48

#define MPU6050_SIGNAL_PATH_RESET	0x68
#define MPU6050_USER_CTRL		0x6A
#define MPU6050_PWR_MGMT_1		0x6B
#define MPU6050_PWR_MGMT_2		0x6C
#define MPU6050_FIFO_COUNTH		0x72
#define MPU6050_FIFO_COUNTL		0x73
#define MPU6050_FIFO_R_W		0x74
#define MPU6050_WHO_AM_I		0x75

#define MPU6050_ADDR_AD0_LOW	0x68
#define MPU6050_ADDR_AD0_HIGH	0x69

// enums (options)

typedef enum {
	ACCEL_2g = 0,
	ACCEL_4g,
	ACCEL_8g,
	ACCEL_16g
} accel_fs_t;

typedef enum {
	GYRO_250 = 0,
	GYRO_500,
	GYRO_1000,
	GYRO_2000
} gyro_fs_t;

typedef enum {
	DLPF_260 = 0,
	DLPF_184,
	DLPF_94,
	DLPF_44,
	DLPF_21,
	DLPF_10,
	DLPF_5
} dlpf_t;

typedef enum {
	CLKSEL_8MHz = 0,
	CLKSEL_PLLx,
	CLKSEL_PLLy,
	CLKSEL_PLLz,
	CLKSEL_32,
	CLKSEL_19,
	CLKSEL_RESET = 7
} clksel_t;

// enums (states)

typedef enum {
	OK = 0,
	ERR_WHO_AM_I,
	ERR_I2C,
	ERR_PARAM,
	ERR_NOT_READY,
	ERR_TIMING
} status_t;

typedef enum {
	IDLE = 0,
	READY,
	BUSY,
	ERROR_STATE
} state_t;

// structs (handles)

typedef struct {
	float x, y, z;
} vec;

typedef struct {
	vec accel_bias;
	vec gyro_bias;
	bool is_calib;
} calib_t;

typedef struct {
	accel_fs_t accel_fs;
	gyro_fs_t gyro_fs;
	dlpf_t dlpf;
	clksel_t clksel;
	uint8_t smplrt_div;
	bool interrupt;
} config_t;

#define RAW_BUF_LEN	14 // 6 accel, 2 temp, 6 gyro

typedef struct {
	I2C_HandleTypeDef *hi2c;
	uint8_t addr;

	config_t cfg;
	calib_t calib;

	float accel_sens;
	float gyro_sens;

	vec accel_data;
	vec gyro_data;
	float temp_data;

	float pitch;
	float roll;
	uint32_t last_update_tick;

	volatile state_t state;
	uint8_t raw_buf[RAW_BUF_LEN];
} mpu6050_t;

// functions (low level read/write)
status_t read_regs(mpu6050_t *dev, uint8_t reg, uint8_t *buf, uint16_t len);

status_t write_reg(mpu6050_t *dev, uint8_t reg, uint8_t buf);

status_t write_bits(mpu6050_t *dev, uint8_t reg, uint8_t mask, uint8_t shift, uint8_t val);

// public api (inits/other)
bool mpu6050_whoami(mpu6050_t *dev);

status_t mpu6050_reset(mpu6050_t *dev);
status_t mpu6050_sleep(mpu6050_t *dev, bool sleep);
status_t mpu6050_init(mpu6050_t *dev, I2C_HandleTypeDef *hi2c, uint8_t ad0, config_t *cfg);

// public api (configuration)
status_t set_accel(mpu6050_t *dev, accel_fs_t fs);
status_t set_gyro(mpu6050_t *dev, gyro_fs_t fs);
status_t set_dlpf(mpu6050_t *dev, dlpf_t dlpf);
status_t set_smplrt(mpu6050_t *dev, uint16_t hz);

// public api (blocking reads)
status_t read_accel(mpu6050_t *dev);
status_t read_gyro(mpu6050_t *dev);
status_t read_temp(mpu6050_t *dev);
status_t read_all(mpu6050_t *dev);

// public api (nonblocking reads)

status_t read_all_it(mpu6050_t *dev);

status_t read_all_dma(mpu6050_t *dev);

void mpu6050_i2c_rx_cmplt_callback(mpu6050_t *dev);

// public api (calibration)

status_t mpu6050_calibrate(mpu6050_t *dev, uint8_t num_samples);

// public api (fifo)

status_t mpu6050_fifo_enable(mpu6050_t *dev);
status_t mpu6050_fifo_reset(mpu6050_t *dev);
status_t mpu6050_fifo_count(mpu6050_t *dev, uint16_t *count);
status_t mpu6050_fifo_read_packet(mpu6050_t *dev);

// public api (interrupts)

status_t enable_data_ready_int(mpu6050_t *dev);
status_t clear_int_status(mpu6050_t *dev, uint8_t *status);

// public api (pitch/roll)

void calculate_pitch_roll(mpu6050_t *dev);


#if __cplusplus
}
#endif

#endif
