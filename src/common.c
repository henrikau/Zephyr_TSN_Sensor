#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(avb_sensor_node, LOG_LEVEL_DBG);

#include <zephyr/kernel.h>

#include <string.h> 		/* memset */
#include <errno.h>
#include <stdio.h>

#include "common.h"

int data_init(struct avb_sensor_data *data, int timeout_us)
{
	if (!data)
		return -ENOMEM;

	if (k_mutex_init(&data->lock)) {
		printf("Failed initializing data lock.\n");
		return -EINVAL;
	}

	data->timeout = K_USEC(timeout_us);
	data_get(data);

	memset(data->accel, 0, sizeof(struct sensor_value)*3);
	memset(data->magn, 0, sizeof(struct sensor_value)*3);
	memset(&data->temp, 0, sizeof(struct sensor_value));
	data->accel_ts = 0;

	memset(data->gyro, 0, sizeof(struct sensor_value)*3);
	data->gyro_ts = 0;

	data_put(data);
	return 0;
}

int data_get(struct avb_sensor_data *data)
{
	if (!data)
		return -EINVAL;
	int retries = 3;

retry:
	int res = k_mutex_lock(&data->lock, data->timeout);
	if (res == -EAGAIN && retries-- > 0)
		goto retry;

	return res;
}
int data_put(struct avb_sensor_data *data)
{
	if (!data)
		return -EINVAL;
	return k_mutex_unlock(&data->lock);
}

int data_wait_ready(struct avb_sensor_data *data, int timeout_ms)
{
	int wait_ms = 0;
	bool ready = false;

	if (!data)
		return -EINVAL;

	do {
		if (data_get(data) == 0) {
			ready = data->ready;
			data_put(data);
		}
		k_sleep(K_MSEC(50));
		wait_ms += 50;
		if (wait_ms > timeout_ms)
			return -ETIMEDOUT;
	} while (!ready);

	return 0;
}

bool data_valid(struct avb_sensor_data *data)
{
	if (!data)
		return false;

	bool valid = false;
	if (data_get(data) == 0) {
		valid = data->running;
		data_put(data);
	}
	return valid;
}

/* Copied from zephyr/samples/net/gptp/src/main.c */
#include <zephyr/net/gptp.h>
#include <zephyr/net/net_core.h>
#include <zephyr/net/net_l2.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/ethernet.h>

uint64_t gptp_ts(void)
{
	struct net_ptp_time ptpts;
	bool gm_present;
	gptp_event_capture(&ptpts, &gm_present);
	return ptpts.second * NSEC_PER_SEC + ptpts.nanosecond;
}

static struct gptp_phase_dis_cb phase_dis;
static void gptp_phase_dis_cb(uint8_t *gm_identity,
			      uint16_t *time_base,
			      struct gptp_scaled_ns *last_gm_ph_change,
			      double *last_gm_freq_change)
{
	char output[sizeof("xx:xx:xx:xx:xx:xx:xx:xx")];
	static uint8_t id[8];

	if (memcmp(id, gm_identity, sizeof(id))) {
		memcpy(id, gm_identity, sizeof(id));

		LOG_DBG("GM %s last phase %d.%" PRId64 "",
			gptp_sprint_clock_id(gm_identity, output, sizeof(output)),
			last_gm_ph_change->high,
			last_gm_ph_change->low);
	}
}

void gptp_init(void)
{
	gptp_register_phase_dis_cb(&phase_dis, gptp_phase_dis_cb);
}
