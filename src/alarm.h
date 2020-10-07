/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#ifndef _ALARM_H_
#define _ALARM_H_

void send_aggregated_data(struct mqtt_client *c, enum mqtt_qos qos);

#endif /* _ALARM_H_ */
