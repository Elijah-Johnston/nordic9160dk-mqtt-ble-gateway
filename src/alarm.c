/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */
#include <string.h>
#include <stdbool.h>
#include <sys/printk.h>
#include <net/mqtt.h>
#include <net/nrf_cloud.h>

#include "alarm.h"

#include "aggregator.h"

/**@brief Function to print strings without null-termination
 */
static void data_print(u8_t *prefix, u8_t *data, size_t len)
{
	char buf[len + 1];

	memcpy(buf, data, len);
	buf[len] = 0;
	printk("%s%s\n", prefix, buf);
}

void send_aggregated_data(struct mqtt_client *c, enum mqtt_qos qos)
{
	static u8_t gps_data_buffer[GPS_NMEA_SENTENCE_MAX_LENGTH];

	static struct nrf_cloud_sensor_data gps_cloud_data = {
		.type = NRF_CLOUD_SENSOR_GPS,
		.data.ptr = gps_data_buffer,
	};

	struct sensor_data aggregator_data;

        struct mqtt_publish_param param;

	param.message.topic.qos = qos;
	param.message.topic.topic.utf8 = CONFIG_MQTT_PUB_TOPIC;
	param.message.topic.topic.size = strlen(CONFIG_MQTT_PUB_TOPIC);
	param.message_id = sys_rand32_get();
	param.dup_flag = 0;
	param.retain_flag = 0;

//	printk("Send data triggered!\n");
	while (1) {
		if (aggregator_get(&aggregator_data) == -ENODATA) {
			break;
		}
		switch (aggregator_data.type) {
		case BEACON_ID:
                        printk("ALARM SEND\n");
//			printk("[%d] Sending Device data.\n",
//			       aggregator_element_count_get());
//			if (aggregator_data.length != 1 ||
//				aggregator_data.data[0] >=
//				ARRAY_SIZE(orientation_strings)) {
//				printk("Unexpected FLIP data format, dropping\n");
//				continue;
//			}
//			flip_cloud_data.data.ptr =
//				orientation_strings[aggregator_data.data[0]];
//			flip_cloud_data.data.len = strlen(
//				orientation_strings[aggregator_data.data[0]]) - 1;
                        param.message.payload.data = aggregator_data.data;
                        param.message.payload.len = aggregator_data.length;
                        data_print("Publishing: ", aggregator_data.data, aggregator_data.length);
//                        printk("to topic: %s len: %u\n",CONFIG_MQTT_PUB_TOPIC, (unsigned int)strlen(CONFIG_MQTT_PUB_TOPIC));
			mqtt_publish(c, &param);
			break;

		case GPS_POSITION:
			printk("%d] Sending GPS data.\n",
			       aggregator_element_count_get());
			gps_cloud_data.data.ptr = &aggregator_data.data[4];
			gps_cloud_data.data.len = aggregator_data.length;
			gps_cloud_data.tag =
			    *((u32_t *)&aggregator_data.data[0]);
                        param.message.payload.data = gps_cloud_data.data.ptr;
                        param.message.payload.len = gps_cloud_data.data.len;
                        data_print("Publishing: ", aggregator_data.data, aggregator_data.length);
                        printk("to topic: %s len: %u\n",CONFIG_MQTT_PUB_TOPIC, (unsigned int)strlen(CONFIG_MQTT_PUB_TOPIC));
			mqtt_publish(c, &param);
			break;

		default:
			printk("Unsupported data type from aggregator: %d.\n",
			       aggregator_data.type);
			continue;
		}
	}
}
