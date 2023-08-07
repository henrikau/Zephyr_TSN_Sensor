#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include "common.h"

static struct avb_sensor_data *_data = NULL;
static const struct device * dev_g = NULL;
static bool valid = false;

K_SEM_DEFINE(sem_g, 0, 1);	/* starts off "not available" */
static void th_gyro(const struct device *dev,
		const struct sensor_trigger *trigger)
{
	if (sensor_sample_fetch(dev)) {
		printf("[GYRO] sensor_sample_fetch() FAILED\n");
		return;
	}
	k_sem_give(&sem_g);
}


int gyro_init(struct avb_sensor_data *sensor_data)
{
	if (!sensor_data)
		return -1;

	_data = sensor_data;

	dev_g = DEVICE_DT_GET_ANY(nxp_fxas21002);
	if (dev_g == NULL || !device_is_ready(dev_g)) {
		printf("Device %s is not ready.\n", dev_g->name);
		return -1;
	}

	struct sensor_trigger trig_g = {
		.type = SENSOR_TRIG_DATA_READY,
		.chan = SENSOR_CHAN_GYRO_XYZ,
	};
	if (sensor_trigger_set(dev_g, &trig_g, th_gyro)) {
		printf("Could not set trigger for %s.\n", dev_g->name);
		return -1;
	}

	printf("Device %s is ready and triggers configured.\n", dev_g->name);
	valid = true;
	return 0;
}


void gyro_collector(void)
{
	do {
		k_sleep(K_SECONDS(1));
	} while (!valid);

	while (valid) {
		k_sem_take(&sem_g, K_FOREVER);
		uint64_t ts = gptp_ts();

		data_get(_data);
		sensor_channel_get(dev_g, SENSOR_CHAN_GYRO_XYZ, &_data->gyro[0]);
		_data->gyro_ts = ts;
		_data->gyro_ctr++;
		data_put(_data);
	}
}
