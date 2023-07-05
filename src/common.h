#pragma once
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>

struct avb_sensor_data {
	struct k_mutex lock;
	k_timeout_t timeout;

	/* Accel, magnetometer & temp from once device */
	struct sensor_value accel[3];
	struct sensor_value magn[3];
	struct sensor_value temp;
	uint64_t accel_ts;

	/* GYRO is read from another device */
	struct sensor_value gyro[3];
	uint64_t gyro_ts;
};

int data_init(struct avb_sensor_data *d, int timeout_us);
int data_get(struct avb_sensor_data *d);
int data_put(struct avb_sensor_data *d);

uint64_t gptp_ts(void);
void gptp_init(void);

int gyro_init(struct avb_sensor_data *sensor_data);
void gyro_collector(void);


int accel_init(struct avb_sensor_data *sensor_data);
void accel_collector(void);
