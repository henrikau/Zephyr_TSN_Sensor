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
#define PREAMBLE_SZ		7
#define SFD_SZ			1
#define CRC_SZ			4
#define IPG_SZ			12
#define L1_SZ			(PREAMBLE_SZ + SFD_SZ + CRC_SZ + IPG_SZ)
#define L2_SZ			14
#define VLAN_SZ			 4
#define DATA_LEN		sizeof(struct sensor_set)
#define PDU_SIZE		(sizeof(struct avtp_stream_pdu) + DATA_LEN)
#define STREAM_ID		42

struct net_info {
	/* iface related fields
	 */
	int max_mtu;
	int queue;

	/* CBS Settings */
	enum avb_stream_class sc;
	uint64_t portTxRate;
	int64_t idleSlope;
	int64_t sendSlope;

	int credit;
	int maxFrameSize;
	int loCredit;
	int hiCredit;
	uint64_t tx_interval_ns;


	/* Tx Priority */
	struct net_context *avb_ctx;
	/*
	 * Reference to sensor data. The collector-threads will feed data into
	 * this whenever their sensor is ready. When we can transmit, we reserve
	 * this, copy out data and send.
	 */
	struct avb_sensor_data *data;
};
static struct net_info ninfo = {0};


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

/* Dedicated worker that periodically refills the Tx-credit.
 *
 * As we do not have HW assisted CBS, this this is done in SW and to
 * reduce the overhead, the interval is higher than the observation
 * interval for both class A and B.
 *
 */
void network_cbs_refill(void)
{
	/* Wait for network_init() to be called, i.e. setting ->data */
	do {
		k_sleep(K_MSEC(100));
	} while (ninfo.data == NULL);

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

	/* Refill with 100Hz, try to avoid too high overhead whilst
	 * testing and debugging.
	 */
	k_timer_start(&cbs_timer, K_USEC(0), K_USEC(10000));

	uint64_t ts_prev = gptp_ts();
	while (1) {
		/* 1. Replenish credit, adjust for timer being late
		 *
		 * In 802.1Q, idleSlope is used to describe the rate of
		 * replensihing credits for a particular Stream
		 * Class. Class A expects to transmit frames every 125us
		 * (8kHz), B at 4kHz.
		 *
		 * This does not scale particularly well to a SW-only
		 * approach, so we need network_init() to have
		 * calculated idleSlope appropriately.
		 */
		k_sem_take(&cbs_credit_lock, K_FOREVER);
		uint64_t ts_now = gptp_ts();

		/* Given the rate of refill (idleSlope), refill
		 * according to time spent since last refill. This
		 * should scale regardless of cbs_timer
		 */
		uint64_t dt_ns = ts_now - ts_prev;
		int64_t d_credits = (ninfo.idleSlope * dt_ns) / 1e9;
		/* If we're below, we increment regardless of queue */
		if (ninfo.credit < 0 || ninfo.queue > 0)
			ninfo.credit += d_credits;

		/* 2. notify waiters */
		if(ninfo.credit >= 0) {
			if (ninfo.queue > 0) {
				k_sem_give(&cbs_can_tx);
			} else {
				/* no waiters, release excess credits */
				ninfo.credit = 0;
			}
		}

		ts_prev = ts_now;
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
	if (k_sem_take(&cbs_credit_lock, K_FOREVER) == 0) {
		ninfo.queue++;
		k_sem_give(&cbs_credit_lock);
		return k_sem_take(&cbs_can_tx, K_FOREVER);
	}
	return -EBUSY;
}

/* Decrement the credit with the consumed size of the transmitted
 * data. The functino will also release the 'credit-lock'
 */
int cbs_credit_put(int payload_sz)
{
	if (k_sem_take(&cbs_credit_lock, K_FOREVER) == 0) {
		/* We have sent, less pressure on the queue */
		ninfo.queue--;

		/* reduce credit with transmitted data
		 *
		 * data size + avtp headers (PDU_SIZE), ethernet
		 * headers, IPG etc.
		 *
		 * Credits are in bits
		 */
		int tx_sz = payload_sz + sizeof(struct avtp_stream_pdu) + L1_SZ + L2_SZ + VLAN_SZ;
		ninfo.credit -= tx_sz*8;

		k_sem_give(&cbs_credit_lock);
		return 0;
	}
	return -EBUSY;
}

static void clear_data(struct avb_sensor_data *data)
{
	if (!data)
		return;
	memset(data->accel, 0, 3 * sizeof(struct sensor_value));
	memset(data->gyro , 0, 3 * sizeof(struct sensor_value));
	memset(data->magn , 0, 3 * sizeof(struct sensor_value));
	memset(&data->temp , 0,     sizeof(struct sensor_value));
	data->accel_ts = 0;
	data->accel_ctr = 0;
	data->gyro_ts = 0;
	data->gyro_ctr = 0;
}

int pdu_add_data(struct avb_sensor_data *data, struct avtp_stream_pdu *pdu)
{
	if (!data || !pdu)
		return -EINVAL;;

	if (data_get(data) == 0) {
		if (!data->running) {
			/* We are no longer running */
			data_put(data);
			return -EIO;
		}
		else if (!data->ready) {
			/* data not yet ready (still in startup) */
			data_put(data);
			return -ENODATA;
		}
		/*
		 * if (!(data->accel_ctr == 1 && data->gyro_ctr == 1)) :
		 *
		 * We are not completely in synch with polling
		 * Either we:
		 *  - overproduce (ctr > 1)
		 *  - underproduce (ctr == 0)
		 *
		 * For now, let the receiver handle this and send data
		 * packets at the defined rate.
		 *
		 */

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

		/* Avoid sending data more than once (essential part of
		 * a time-triggered architecture)
		 */
		clear_data(data);

		data_put(data);
		return sizeof(*set);
	}

	/* Failed getting data lock */
	return -EBUSY;
}

void gather_net_info(struct net_if *iface, void *user_data)
{
	struct net_info *info = (struct net_info *)user_data;
	if (iface->if_dev->mtu > info->max_mtu)
		info->max_mtu = iface->if_dev->mtu;
}

int network_init(struct avb_sensor_data *sensor_data,
		uint64_t tx_interval_ns,
		enum avb_stream_class sc)
{
	if (!sensor_data)
		return -EINVAL;

	memset(&ninfo, 0, sizeof(ninfo));

	struct net_if *iface = net_if_get_first_by_type(&NET_L2_GET_NAME(ETHERNET));
	if (!iface) {
		printf("Failed getting interface, cannot continue\n");
		return -EINVAL;
	}

	/* If we're using an explicit stream-class, create a vlan and
	 * create the context to go with it.
	 */
	if (sc != CLASS_NONE) {
		int ret = net_eth_vlan_enable(iface, CONFIG_NET_VLAN_TAG_AVB);
		if (ret < 0) {
			printf("Failed activating VLAN:%d (%d)",
				CONFIG_NET_VLAN_TAG_AVB, ret);
			return -EINVAL;
		}
		ret = net_context_get(AF_PACKET, SOCK_DGRAM, IPPROTO_RAW, &ninfo.avb_ctx);
		if (ret != 0) {
			printf("Failed getting context for outgoing frames\n");
			return -EINVAL;
		}

		struct sockaddr_ll addr = {
			.sll_family = AF_PACKET,
			.sll_protocol = htons(0x22f0),
			.sll_ifindex = 1,
			.sll_halen = 6,
		};
		ret = net_context_bind(ninfo.avb_ctx,
				(struct sockaddr *)&addr,
				sizeof(struct sockaddr_ll));
		if (ret < 0) {
			printf("Failed binding to context (%d)\n", ret);
			return -EINVAL;
		}

		/* From 802.1BA and friends
		 * (Unless otherwise configured by a network-admin)
		 */
		uint8_t prio = sc == CLASS_A ? 3 : 2;
		ret = net_context_set_option(ninfo.avb_ctx,
					NET_OPT_PRIORITY,
					&prio, sizeof(prio));
		if (ret < 0) {
			printf("Faield setting prio (%u) for context, %d\n", prio, ret);
			return -EINVAL;
		}
	}

	/* 1. Find port rate (portTransmitRate) and max MTU*/
	ninfo.portTxRate = 100000000;
	ninfo.max_mtu = iface->if_dev->mtu;

	ninfo.data = sensor_data;
	ninfo.tx_interval_ns = tx_interval_ns;

	ninfo.sc = sc;
	ninfo.maxFrameSize = (PDU_SIZE + L2_SZ + VLAN_SZ + L1_SZ)*8;

	/* idleSlope is the rate of refill and is the total size * observation interval.
	 *
	 * The idea being that a stream should send with /at least/ that rate.
	 *
	 * There's nothing wrong by using expected Tx rate instead of
	 * observation interval per. se, but this will interfere with
	 * the transmission guarantees for other streams. Idle slope is
	 * what gives the upper limit of frame sizes.
	 *
	 * However, as a means for refilling credits in a small system
	 * running in /software/ this approach is competely bonkers.
	 *
	 * The "correct" approach would be to take each dataset and
	 * split it into tx_interval_ns / observation_interval. With our
	 * PDU size of 104 bytes, this would lead to 1-2 bytes of
	 * payload *per* frame which is rather ridiculous.
	 *
	 * So we cheat. As long as sensor_data is < max MTU, we send
	 * everything in a single frame and refill credits based on
	 * tx_interval.
	 */
	ninfo.idleSlope = ninfo.maxFrameSize * ((double)1e9 / ninfo.tx_interval_ns);
	ninfo.sendSlope = ninfo.idleSlope - ninfo.portTxRate;

	/* Need to do jump through some hoops to avoid integer overflow */
	ninfo.loCredit = ((int64_t)ninfo.maxFrameSize * ninfo.sendSlope) / ninfo.portTxRate;
	ninfo.hiCredit = ((int64_t)ninfo.max_mtu * 8  * (ninfo.idleSlope) / ninfo.portTxRate);

	printf("Network CBS settings\n");
	printf("  portTxRate          = %10"PRIu64" bps\n", ninfo.portTxRate);
	printf("  idleSlope           = %10"PRId64" bps\n", ninfo.idleSlope);
	printf("  sendSlope           = %10"PRId64" bps\n", ninfo.sendSlope);
	printf("  loCredit            = %10d bits\n", ninfo.loCredit);
	printf("  hiCredit            = %10d bits\n", ninfo.hiCredit);
	printf("  maxFrameSize        = %10d bits\n", ninfo.maxFrameSize);
	printf("  maxInterferenceSize = %10d bytes\n", ninfo.max_mtu);

	return 0;
}

void avb_tx_callback(struct net_context *ctx, int status, void *data)
{
	struct net_info *n = (struct net_info *)data;
	if (ctx == n->avb_ctx) {
		cbs_credit_put(status > 0 ? status : 0);
	}
}

void network_sender(void)
{
	/* Wait for network_init() to be called, i.e. setting ->data */
	do {
		k_sleep(K_MSEC(100));
	} while (ninfo.data == NULL);

	uint8_t seq_num = 0;

	/*
	 * Regardless of context and stream-classes, weneed a convenient
	 * way of draining the receive buffer since gPTP opens the NIC
	 * in promiscous mode.
	 */
	int avb_socket = zsock_socket(AF_PACKET, SOCK_DGRAM, 0x22f0);
	if (avb_socket < 0) {
		printf("[NETWORK] Cannot create socket (%d), aborting\n", avb_socket);
		return;
	}

	/* Set destination */
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

	/* Wait for data to become ready, max 30 sec */
	int ret = data_wait_ready(ninfo.data, 30000);
	if (ret < 0) {
		printf("[NETWORK] Data not available (%d), aborting Tx-thread after 30 sec timeout.\n", ret);
		return;
	}

	while (data_valid(ninfo.data)) {
		/* 1. Block until we have 0 or positive credit */
		cbs_credit_get();

		/* 2. Collect data from _data */
		int sz = pdu_add_data(ninfo.data, pdu);
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
			((struct sensor_set *)pdu->avtp_payload)->sent_ts_ns = gptp_ts();
			if (ninfo.sc == CLASS_NONE) {
				zsock_sendto(avb_socket, pdu, PDU_SIZE, 0, (struct sockaddr *)&addr, sizeof(addr));
				cbs_credit_put(sz > 0 ? sz : 0);
			} else {
				int ret = net_context_sendto(ninfo.avb_ctx,
							pdu,
							PDU_SIZE,
							(struct sockaddr *)&addr,
							sizeof(addr),
							(net_context_send_cb_t)avb_tx_callback, K_NO_WAIT, (void *)&ninfo);
				// cbs_credit_puts() freed in callback
				if (ret < 0) {
					printf("Failed sending data using context, res=%d, stopping.\n", ret);
					ninfo.data->running = false;
				}
			}

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

			seq_num++;
		}

	}
}
