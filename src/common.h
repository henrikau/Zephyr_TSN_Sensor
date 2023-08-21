#pragma once
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>

struct avb_sensor_data {
	struct k_mutex lock;
	k_timeout_t timeout;

	/* System flag to indicate if we are running and ready to start */
	bool ready;
	bool running;

	/* Accel, magnetometer & temp from once device */
	struct sensor_value accel[3];
	struct sensor_value magn[3];
	struct sensor_value temp;
	uint64_t accel_ts;
	uint64_t accel_ctr;

	/* GYRO is read from another device */
	struct sensor_value gyro[3];
	uint64_t gyro_ts;
	uint64_t gyro_ctr;
};

/*
 * Sensor value payload
 */
struct sensor_set {
	int64_t magn[3];
	int64_t gyro[3];
	int64_t accel[3];
	int64_t temp;
	uint64_t gyro_ts_ns;
	uint64_t accel_ts_ns;
	uint64_t sent_ts_ns;
} __attribute__((packed));

int data_init(struct avb_sensor_data *d, int timeout_us);
int data_get(struct avb_sensor_data *d);
int data_put(struct avb_sensor_data *d);

/* Wait for data to become available.
 * Needed at:
 *    - startup, when sensor setup is still running
 *    - error, if one or more of the sensors are unavailable.
 */
int data_wait_ready(struct avb_sensor_data *data, int timeout_ms);

/* If system becomes invalid, indicates to threads that they should close down.
 *
 * If data is invalid, cannot continue.
 */
bool data_valid(struct avb_sensor_data *data);

uint64_t gptp_ts(void);
void gptp_init(void);

int gyro_init(struct avb_sensor_data *sensor_data);
void gyro_collector(void);

int accel_init(struct avb_sensor_data *sensor_data);
void accel_collector(void);

/* Initialize the network, set addresses, ready CBS credit calculation
 * etc.
 *
 * sensor_data: containing struct for data. We expect the data to be
 * updated asynhcronously and will send whatever's in the buffer when
 * CBS allows us to
 *
 * tx_interval: Target interval. This is used as input to the CBS
 * machinery to compute the correct idleSlope which in turn will send
 * data at the desired rate.
 */
int network_init(struct avb_sensor_data *sensor_data, int tx_interval_ns);

/* Worker sending data from as quickly as CBS will allow it to.
 *
 * The design hinges on a single sender (this thread) sending sensor
 * data of a known size. It does not currently support multiple outgoing
 * streams.
 */
void network_sender(void);

/* Low-pri worker that drains the Rx buffer of a socket.
 *
 * This is a know bug found by Einar, that when gPTP runs, it places the
 * iface in promiscous mode, and the gPTP driver does not properly drain
 * the incming packages, resulting in overflowing buffers.
 *
 * [00:12:17.839,000] <err> net_pkt: Data buffer (68) allocation failed.
 *
 */
void network_rx_drain(void);

/* Worker that periodically refills the credits. Once credits reaches >=
 * 0, potential waiters are signalled.
 */
void network_cbs_refill(void);

