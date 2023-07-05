#include <zephyr/kernel.h>
#include <stdio.h>
#include <zephyr/drivers/sensor.h>
#include "common.h"

int main(void)
{
	/* Setup Time first */
	gptp_init();

	struct avb_sensor_data data;
	if (data_init(&data, 250)) {
		printf("Failed initializing data container\n");
		return 0;
	}

	/* ------------------------------------------------------
	 * FXOS8700 (accel, magn, temp)
	 */
	accel_init(&data);

	/* ------------------------------------------------------
	 * FXAS21002 - Gyro
	 * Collector runs in own thread waiting for gyro_init() to be called
	 */
	gyro_init(&data);

	/* ------------------------------------------------------
	 * Print loop, 10 Hz
	 */
	while (1) {
		k_sleep(K_MSEC(100));
		if (data_get(&data) == 0) {
			double diff_ms = (int64_t)(data.accel_ts - data.gyro_ts) / 1e6;
			printf("[%"PRIu64"] (%6.3f ms) ", data.gyro_ts, diff_ms);

			printf("GX=%10.3f GY=%10.3f GZ=%10.3f ",
				sensor_value_to_double(&data.gyro[0]),
				sensor_value_to_double(&data.gyro[1]),
				sensor_value_to_double(&data.gyro[2]));

			printf("AX=%10.6f AY=%10.6f AZ=%10.6f ",
				sensor_value_to_double(&data.accel[0]),
				sensor_value_to_double(&data.accel[1]),
				sensor_value_to_double(&data.accel[2]));

			/* Print mag x,y,z data */
			printf("MX=%10.6f MY=%10.6f MZ=%10.6f ",
				sensor_value_to_double(&data.magn[0]),
				sensor_value_to_double(&data.magn[1]),
				sensor_value_to_double(&data.magn[2]));

			/* Print accel x,y,z and mag x,y,z data */
			printf("T=%10.6f", sensor_value_to_double(&data.temp));
			printf("\n");
			data_put(&data);
		}
	}
	return 0;
}
K_THREAD_DEFINE(GYRO_COLLECTOR, 1024, gyro_collector  , NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(ACCEL_COLLECTOR, 1024, accel_collector  , NULL, NULL, NULL, 6, 0, 0);
