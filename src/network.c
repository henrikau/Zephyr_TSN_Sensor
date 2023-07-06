#include "common.h"
#include "avtp.h"
#include <zephyr/net/socket.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_core.h>
#include <zephyr/net/net_l2.h>
#include <zephyr/net/net_if.h>
#include "avtp.h"
#include "avtp_stream.h"

#include <stdio.h>		/* printf() */

#define DATA_LEN		sizeof(struct sensor_set)
#define PDU_SIZE		(sizeof(struct avtp_stream_pdu) + DATA_LEN)
#define STREAM_ID		42


/*
 * Reference to sensor data. The collector-threads will feed data into
 * this whenever their sensor is ready. When we can transmit, we reserve
 * this, copy out data and send.
 */
static struct avb_sensor_data *_data;

/* cbs_can_tx
 * We use this to signal that credits are available.
 *
 * Upper limit is 1, i.e. we can give this semaphore as many times as we
 * like, but we will never have a queue of 'unused locks' which could
 * lead to a massive overspending of credits.
 */
K_SEM_DEFINE(cbs_can_tx, 0, 1);

/* change this to mutex? */
K_SEM_DEFINE(cbs_credit_lock, 1, 1);

/* Worker that periodically refills the credits. Once credits reaches >=
 * 0, potential waiters are signalled.
 */

/* CBS refill signal */
K_SEM_DEFINE(cbs_timer_sem,0,1);
static struct k_timer cbs_timer;
void cbs_timeout(struct k_timer *timer_id) {
	k_sem_give(&cbs_timer_sem);
}
void network_cbs_refill(void)
{
	/* FIXME Use these values (alongsize tx_interval) to find the
	 * correct idleSlope
	 */
#if 0
	int portTransmitRate = 100000000;	//bits per second on port
	int idleSlope = 1920;			//bits per second used by avtp
	int sendSlope = idleSlope - portTransmitRate;
	bool transmit = false;
#endif
	int credit = 0;

	/*
	 * Run periodically at X Hz (find this in .1BA/.1Q#L
	 * FIXME: add a timer (prematurely) stopped callback
	 *
	 * Currently this is hardcoded to run at 100Hz, we replenish
	 * with 1 framesize every 10ms.
	 *
	 * This is *not* the correct way.
	 */
	k_timer_init(&cbs_timer, cbs_timeout, NULL);
	k_timer_start(&cbs_timer, K_USEC(250), K_MSEC(10));

	while (1) {
		/* 1. Replenish credit, adjust for timer being late */
		k_sem_take(&cbs_credit_lock, K_FOREVER);
		if (credit < 0) {
			// credit += 4800; //12500 bits per interrupt x 1920 / 100000000 (x10000)
			credit += DATA_LEN;
		}

		if(credit >= 0) {
			//credit -= 599988; //60 bits to send x sendSlope / 100000000 (x10000)

			/* From Einar's code, why? */
			// credit -= 600000; //make up for not pausing timer

			/* 2. notify waiters */
			k_sem_give(&cbs_can_tx);
		}
		k_sem_give(&cbs_credit_lock);

		/* 3. Wait for timer and goto #1*/
		k_sem_take(&cbs_timer_sem, K_FOREVER);
	}
}


/* Grab hold of the credit value before transmitting.
 *
 * If the credit is 0, this function will block and the caller must wait
 * for the credits to be replensished.
 *
 * Once it returs (provided 0), the caller now has the 'credit lock' and
 * is the sole thread capable of transmitting.
 */
int cbs_credit_get(void)
{
	return k_sem_take(&cbs_can_tx, K_FOREVER);
}

/* Decrement the credit with the consumed size of the transmitted
 * data. The functino will also release the 'credit-lock'
 */
int cbs_credit_put(int sz)
{
	if (k_sem_take(&cbs_credit_lock, K_FOREVER) == 0) {
		/* reduce credit with transmitted data */
		k_sem_give(&cbs_credit_lock);
		return 0;
	}
	return -EBUSY;
}


int pdu_add_data(struct avb_sensor_data *data, struct avtp_stream_pdu *pdu)
{
	if (!data || !pdu)
		return -EINVAL;;

	if (data_get(data) == 0) {
		struct sensor_set *set = (struct sensor_set *)pdu->avtp_payload;
		/* all values are in micro-units */
		for (int i = 0; i < 3; i++) {
			set->magn[i]  = sensor_value_to_micro(&data->magn[i]);
			set->gyro[i]  = sensor_value_to_micro(&data->gyro[i]);
			set->accel[i] = sensor_value_to_micro(&data->accel[i]);
		}
		set->temp = sensor_value_to_micro(&(data->temp));

		/* Copy capture timestamps */
		set->gyro_ts_ns = data->gyro_ts;
		set->accel_ts_ns = data->accel_ts;
		data_put(data);
		return sizeof(*set);
	}

	/* Failed getting data lock */
	return -EBUSY;
}


int network_init(struct avb_sensor_data *sensor_data, int tx_interval)
{

	if (!sensor_data)
		return -EINVAL;
	_data = sensor_data;

	/* Init network */
	return 0;
}

void network_sender(void)
{
	uint8_t seq_num = 0;

	int avb_socket = zsock_socket(AF_PACKET, SOCK_DGRAM, 0x22f0);
	if (avb_socket < 0) {
		printk("Cannot create socket\n%i\n", avb_socket);
		return;
	}

	/* Set destination */
	// const unsigned char ether_broadcast_addr[]={0x00,0x1b,0x21,0xe4,0x67,0x07};
	const unsigned char ether_mcast_addr[] = {0x01, 0x00, 0x5E, 0x01, 0x11, 0x42};

	struct sockaddr_ll addr;
	addr.sll_family = AF_PACKET;
	addr.sll_ifindex = 1;
	addr.sll_protocol = 0xf022;
	addr.sll_halen = sizeof("xx:xx:xx:xx:xx:xx");
	memcpy(addr.sll_addr, ether_mcast_addr, sizeof(ether_mcast_addr));

	struct avtp_stream_pdu *pdu = alloca(PDU_SIZE);
	avtp_stream_pdu_init(pdu);
	char drain_buffer[1500];

	while (1) {
		/* 1. Block until we have 0 or positive credit */
		cbs_credit_get();

		/* 2. Collect data from _data */
		int sz = pdu_add_data(_data, pdu);
		if (sz == DATA_LEN) {

			/* 3. Construct rest of PDU */
			avtp_stream_pdu_set(pdu, AVTP_STREAM_FIELD_STREAM_ID, STREAM_ID);
			avtp_stream_pdu_set(pdu, AVTP_STREAM_FIELD_STREAM_DATA_LEN, sz);
			avtp_stream_pdu_set(pdu, AVTP_STREAM_FIELD_SEQ_NUM, seq_num);

			/* FIXME: validate gptp, is TV valid? */
			uint64_t ptp_time_ns = gptp_ts();
			uint32_t avtptime = (uint32_t)(ptp_time_ns & 0xffffffff);
			avtp_stream_pdu_set(pdu, AVTP_STREAM_FIELD_TV, 1);
			avtp_stream_pdu_set(pdu, AVTP_STREAM_FIELD_TIMESTAMP, avtptime);

			/* 4. Transmit data  */
			zsock_sendto(avb_socket, pdu, PDU_SIZE, 0, (struct sockaddr *)&addr, sizeof(addr));
			seq_num++;

			/* Rip out multiple messages quickly.
			 *
			 * We are sending L2 packets alongside gPTP,
			 * which places the iface in promiscous
			 * mode. This means that packets sent will end
			 * up in the receive queue, so we need to drain
			 * these to avoid filling up the Rx buffers
			 *
			 * https://github.com/zephyrproject-rtos/zephyr/issues/34865 (Einar's)
			 * https://github.com/zephyrproject-rtos/zephyr/issues/34462
			 * https://github.com/zephyrproject-rtos/zephyr/pull/34475
			 */
			int res;
			do {
				res = recv(avb_socket, drain_buffer, sizeof(drain_buffer), MSG_DONTWAIT);
			} while (res > 0);
		}
		/* 5. Signal CBS thread that data has been consumed */
		cbs_credit_put(sz > 0 ? sz : 0);

		/* 6. Goto #1 */
	}
}