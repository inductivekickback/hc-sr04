/*
 * Copyright (c) 2020 Daniel Veilleux
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <string.h>
#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <stdio.h>
#include <sys/__assert.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

K_TIMER_DEFINE(sync_timer, NULL, NULL);


static int measure(const struct device *dev)
{
    int ret;
    struct sensor_value distance;

    ret = sensor_sample_fetch_chan(dev, SENSOR_CHAN_ALL);
    switch (ret) {
    case 0:
        ret = sensor_channel_get(dev, SENSOR_CHAN_DISTANCE, &distance);
        if (ret) {
            LOG_ERR("sensor_channel_get failed ret %d", ret);
            return ret;
        }
        LOG_INF("%s: %d.%03dM", dev->name, (distance.val1 / 1000000), (distance.val2 / 1000));
        break;
    case -EIO:
        LOG_WRN("%s: Could not read device", dev->name);
        break;
    default:
        LOG_ERR("Error when reading device: %s", dev->name);
        break;
    }
    return 0;
}

void main(void)
{
    int ret;
    const struct device *dev;

    if (IS_ENABLED(CONFIG_LOG_BACKEND_RTT)) {
        /* Give RTT log time to be flushed before executing tests */
        k_sleep(K_MSEC(500));
    }

#if CONFIG_HC_SR04
    dev = device_get_binding("HC-SR04_0");
#else
    dev = device_get_binding("HC-SR04_NRFX_0");
#endif

    if (dev == NULL) {
        LOG_ERR("Failed to get dev binding");
        return;
    }
    LOG_INF("dev is %p, name is %s", dev, dev->name);
    k_timer_start(&sync_timer, K_MSEC(25), K_MSEC(25));
    while (1) {
        k_timer_status_sync(&sync_timer);
        ret = measure(dev);
        if (ret) {
            return;
        }
    }
    LOG_INF("exiting");
}
