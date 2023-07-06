#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include "common.h"

static struct avb_sensor_data *_data = NULL;
static const struct device * dev_a = NULL;
static bool valid = false;

K_SEM_DEFINE(sem_a, 0, 1);	/* starts off "not available" */

static void th_accel(const struct device *dev,
		const struct sensor_trigger *trigger)
{
	if (sensor_sample_fetch(dev)) {
		printf("[ACCEL] sensor_sample_fetch() FAILED\n");
		return;
	}
	k_sem_give(&sem_a);
}


int accel_init(struct avb_sensor_data *sensor_data)
{
	if (!sensor_data)
		return -1;

	_data = sensor_data;

	dev_a = DEVICE_DT_GET_ONE(nxp_fxos8700);
	if (!device_is_ready(dev_a)) {
		printf("Device %s is not ready\n", dev_a->name);
		return -1;
	}

	/* Run sensor at 100Hz (commented out: 6 Hz  */
	struct sensor_value attr_a = {
		.val1 = 100, // 6
		.val2 = 0,   // 250000
	};

	if (sensor_attr_set(dev_a, SENSOR_CHAN_ALL,
				SENSOR_ATTR_SAMPLING_FREQUENCY, &attr_a)) {
		printf("Could not set sampling frequency for %s\n", dev_a->name);
		return -1;
	}

	struct sensor_trigger trig_a = {
		.type = SENSOR_TRIG_DATA_READY,
		.chan = SENSOR_CHAN_ACCEL_XYZ,
	};
	if (sensor_trigger_set(dev_a, &trig_a, th_accel)) {
		printf("Could not set trigger for %s\n", dev_a->name);
		return -1;
	}

	printf("Device %s is ready and triggers configured.\n", dev_a->name);
	valid = true;
	return 0;
}


void accel_collector(void)
{
	/* Wait for accel_init() to be called */
	do {
		k_sleep(K_SECONDS(1));
	} while (!valid);

	while (valid && _data != NULL) {
		k_sem_take(&sem_a, K_FOREVER);
		uint64_t ts = gptp_ts();
		if (data_get(_data) == 0) {
			sensor_channel_get(dev_a, SENSOR_CHAN_ACCEL_XYZ, &_data->accel[0]);
			sensor_channel_get(dev_a, SENSOR_CHAN_MAGN_XYZ, &_data->magn[0]);
			sensor_channel_get(dev_a, SENSOR_CHAN_DIE_TEMP, &_data->temp);
			_data->accel_ts = ts;
			data_put(_data);
		}
	}
}
