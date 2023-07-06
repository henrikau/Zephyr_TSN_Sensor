#include <zephyr/kernel.h>
#include <stdio.h>
#include <zephyr/drivers/sensor.h>
#include "common.h"

int main(void)
{
	bool startup_err = false;

	/* Setup Time first */
	gptp_init();

	struct avb_sensor_data *data = alloca(sizeof(*data));
	if (data_init(data, 5000)) {
		printf("Failed initializing data container\n");
		startup_err = true;
	}

	/* ------------------------------------------------------
	 * FXAS21002 - Gyro
	 * Collector runs in own thread waiting for gyro_init() to be called
	 */
	if (gyro_init(data) != 0) {
		printf("Failed starting gyro!\n");
		startup_err = true;
	}

	/* ------------------------------------------------------
	 * FXOS8700 (accel, magn, temp)
	 */
	if (accel_init(data) != 0) {
		printf("Failed starting accel/magn/temp!\n");
		startup_err = true;
	}


	/* ------------------------------------------------------
	 * network setup, making addresses, buffers, CBS etc ready
	 *
	 * accel reads data at 100Hz (10ms)
	 */
	if (network_init(data, 10*NSEC_PER_MSEC) != 0) {
		printf("Failed starting network\n");
		startup_err = true;
	}


	if (startup_err) {
		printf("Startup errors exists, aborting..\n");
		return -1;
	}

	/* ------------------------------------------------------
	 * Print loop, 10 Hz
	 */
	while (1) {
		k_sleep(K_MSEC(250));

/* To enable this, call west with: -DEXTRA_CFLAGS="-DDEBUG=1" */
#ifdef DEBUG
		if (data_get(data) == 0) {
			double diff_ms = (int64_t)(data->accel_ts - data->gyro_ts) / 1e6;
			printf("[%"PRIu64"] (%8.3f ms) ", data->gyro_ts, diff_ms);

			printf("GX=%10.3f GY=%10.3f GZ=%10.3f ",
				sensor_value_to_double(&data->gyro[0]),
				sensor_value_to_double(&data->gyro[1]),
				sensor_value_to_double(&data->gyro[2]));

			printf("AX=%10.6f AY=%10.6f AZ=%10.6f ",
				sensor_value_to_double(&data->accel[0]),
				sensor_value_to_double(&data->accel[1]),
				sensor_value_to_double(&data->accel[2]));

			/* Print mag x,y,z data */
			printf("MX=%10.6f MY=%10.6f MZ=%10.6f ",
				sensor_value_to_double(&data->magn[0]),
				sensor_value_to_double(&data->magn[1]),
				sensor_value_to_double(&data->magn[2]));

			/* Print accel x,y,z and mag x,y,z data */
			printf("T=%10.6f", sensor_value_to_double(&data->temp));
			printf("\n");
			data_put(data);
		}
#endif /* DEBUG */
	}
	return 0;
}

K_THREAD_DEFINE(GYRO_COLLECTOR,  1024, gyro_collector    , NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(ACCEL_COLLECTOR, 1024, accel_collector   , NULL, NULL, NULL, 6, 0, 0);
K_THREAD_DEFINE(NETWORK_SENDER,  1024, network_sender    , NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(CBS_REFILLER,    2048, network_cbs_refill, NULL, NULL, NULL, 4, 0, 0);
