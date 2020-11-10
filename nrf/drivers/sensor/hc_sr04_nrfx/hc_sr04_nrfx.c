/*
 * Copyright (c) 2020 Daniel Veilleux
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

/*
 * NOTE: Invalid measurements manifest as a 128600us pulse followed by a second pulse of ~6us
 *       about 145us later. This pulse can't be truncated so it effectively reduces the sensor's
 *       working rate.
 */

#define DT_DRV_COMPAT elecfreaks_hc_sr04_nrfx

#include <kernel.h>
#include <drivers/sensor.h>
#include <device.h>
#include <sensor/hc_sr04_nrfx.h>
#include <devicetree.h>

#include <nrfx_timer.h>
#include <nrfx_gpiote.h>
#include <nrfx_ppi.h>
#include <nrfx_egu.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(hc_sr04_nrfx, CONFIG_HC_SR04_NRFX_LOG_LEVEL);

/* Timings defined by spec */
#define T_TRIG_PULSE_US       11
#define T_INVALID_PULSE_US    25000
#define T_MAX_WAIT_MS         130
#define T_SPURIOS_WAIT_US     145
#define METERS_PER_SEC        340

#define EGU_EVENT_POS         0

#define TIMER_TRIG_UP_CHAN    0
#define TIMER_TRIG_DOWN_CHAN  1
#define TIMER_ECHO_START_CHAN 2
#define TIMER_ECHO_END_CHAN   3
#define TIMER_TRIG_UP_COUNT   1
#define TIMER_TRIG_DOWN_COUNT (TIMER_TRIG_UP_COUNT + T_TRIG_PULSE_US)

static struct hc_sr04_nrfx_shared_resources {
    nrfx_timer_t             timer;
    struct k_sem             fetch_sem;
    struct k_mutex           mutex;
    nrfx_gpiote_in_config_t  echo_in;
    nrfx_gpiote_out_config_t trig_out;
    nrf_ppi_channel_group_t  falling_echo_group;
    nrf_ppi_channel_t        trig_up_channel;
    nrf_ppi_channel_t        trig_down_channel;
    nrf_ppi_channel_t        timer_start_channel;
    nrf_ppi_channel_t        rising_group_channel;
    nrf_ppi_channel_t        falling_group_channel;
    nrf_ppi_channel_t        clear_int_channel;
    nrf_ppi_channel_t        capture_stop_channel;    
    bool                     ready; /* The module has been initialized */
} m_shared_resources;

struct hc_sr04_nrfx_data {
    struct sensor_value sensor_value;
};

struct hc_sr04_nrfx_cfg {
    uint32_t trig_pin;
    uint32_t echo_pin;
};


static void egu_handler(uint8_t event_idx, void * p_context)
{
    k_sem_give(&m_shared_resources.fetch_sem);
}

static void timer_handler(nrf_timer_event_t event_type, void * p_context)
{
    /* 
     * Required by the NRFX driver but never called. In fact, it can't be
     * called because it's not registered with IRQ_CONNECT.
     */
}

static void gpiote_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    /* 
     * Required by the NRFX driver but never called. In fact, it can't be
     * called because it's not registered with IRQ_CONNECT.
     */
}

static nrfx_err_t gpiote_pins_init(uint32_t trig_pin, uint32_t echo_pin)
{
    nrfx_err_t nrfx_err = NRFX_SUCCESS;

    nrfx_err = nrfx_gpiote_in_init(echo_pin, &m_shared_resources.echo_in, gpiote_handler);
    if (NRFX_SUCCESS != nrfx_err) {
        return nrfx_err;
    }
    nrfx_err = nrfx_gpiote_out_init(trig_pin, &m_shared_resources.trig_out);
    if (NRFX_SUCCESS != nrfx_err) {
        return nrfx_err;
    }

    nrf_ppi_task_endpoint_setup(NRF_PPI,
        m_shared_resources.trig_up_channel,
        nrfx_gpiote_out_task_addr_get(trig_pin));
    nrf_ppi_task_endpoint_setup(NRF_PPI,
        m_shared_resources.trig_down_channel,
        nrfx_gpiote_out_task_addr_get(trig_pin));
    nrf_ppi_event_endpoint_setup(NRF_PPI,
        m_shared_resources.timer_start_channel,
        nrfx_gpiote_in_event_addr_get(echo_pin));
    nrf_ppi_event_endpoint_setup(NRF_PPI,
        m_shared_resources.rising_group_channel,
        nrfx_gpiote_in_event_addr_get(echo_pin));
    nrf_ppi_event_endpoint_setup(NRF_PPI,
        m_shared_resources.capture_stop_channel,
        nrfx_gpiote_in_event_addr_get(echo_pin));
    nrf_ppi_event_endpoint_setup(NRF_PPI,
        m_shared_resources.clear_int_channel,
        nrfx_gpiote_in_event_addr_get(echo_pin));
    nrf_ppi_event_endpoint_setup(NRF_PPI,
        m_shared_resources.falling_group_channel,
        nrfx_gpiote_in_event_addr_get(echo_pin));

    nrfx_gpiote_in_event_enable(echo_pin, false);
    nrfx_gpiote_out_task_enable(trig_pin);

    return nrfx_err;
}

static void gpiote_pins_uninit(uint32_t trig_pin, uint32_t echo_pin)
{
    nrfx_gpiote_in_uninit(echo_pin);
    nrfx_gpiote_out_uninit(trig_pin);
}

static nrfx_err_t timer_init(void)
{
    nrfx_err_t err;

    m_shared_resources.timer.p_reg       = NRFX_CONCAT_2(NRF_TIMER, CONFIG_HC_SR04_NRFX_TIMER);
    m_shared_resources.timer.instance_id = NRFX_CONCAT_3(NRFX_TIMER,
                                                         CONFIG_HC_SR04_NRFX_TIMER,
                                                         _INST_IDX);
    m_shared_resources.timer.cc_channel_count = NRF_TIMER_CC_CHANNEL_COUNT(CONFIG_HC_SR04_NRFX_TIMER);

    nrfx_timer_config_t cfg = NRFX_TIMER_DEFAULT_CONFIG;
    cfg.bit_width           = NRF_TIMER_BIT_WIDTH_24,
    cfg.frequency           = NRF_TIMER_FREQ_1MHz;
    cfg.interrupt_priority  = DT_IRQ(DT_INST(CONFIG_HC_SR04_NRFX_TIMER, nordic_nrf_timer), priority);

    err = nrfx_timer_init(&m_shared_resources.timer, &cfg, timer_handler);
    if (NRFX_SUCCESS != err) {
        return err;
    }

    nrfx_timer_compare(&m_shared_resources.timer,TIMER_TRIG_UP_CHAN,  TIMER_TRIG_UP_COUNT,  false);
    nrfx_timer_compare(&m_shared_resources.timer,TIMER_TRIG_DOWN_CHAN,TIMER_TRIG_DOWN_COUNT,false);
    return NRFX_SUCCESS;
}

static nrfx_err_t egu_init(NRF_EGU_Type ** p_egu)
{
    nrfx_err_t err;

    IRQ_CONNECT(DT_IRQN(DT_INST(CONFIG_HC_SR04_NRFX_EGU, nordic_nrf_egu)),
            DT_IRQ(DT_INST(CONFIG_HC_SR04_NRFX_EGU, nordic_nrf_egu), priority),
            nrfx_isr,
            NRFX_CONCAT_3(nrfx_egu_, CONFIG_HC_SR04_NRFX_EGU, _irq_handler),
            0);

    nrfx_egu_t egu = NRFX_EGU_INSTANCE(CONFIG_HC_SR04_NRFX_EGU);

    *p_egu = egu.p_reg;

    err = nrfx_egu_init(&egu, 0, egu_handler, NULL);
    if (NRFX_SUCCESS != err) {
        return err;
    }

    nrfx_egu_int_enable(&egu, (1 << EGU_EVENT_POS));
    return NRFX_SUCCESS;
}

static nrfx_err_t ppi_init(NRF_EGU_Type * p_egu, uint32_t echo_pin, uint32_t trig_pin)
{
    nrfx_err_t err;
    nrf_ppi_channel_group_t rising_echo_group;
    nrf_ppi_channel_group_t falling_echo_group;

    err = nrfx_ppi_group_alloc(&rising_echo_group);
    if (NRFX_SUCCESS != err) {
        return err;
    }
    err = nrfx_ppi_group_alloc(&falling_echo_group);
    if (NRFX_SUCCESS != err) {
        return err;
    }

    /*
     * CC[TIMER_TRIG_UP_CHAN] event   -> Trig toggle high
     * CC[TIMER_TRIG_DOWN_CHAN] event -> Trig toggle low
     *                                -> Enable rising edge group
     */
    err = nrfx_ppi_channel_alloc(&m_shared_resources.trig_up_channel);
    if (NRFX_SUCCESS != err) {
        return err;
    }
    err = nrfx_ppi_channel_alloc(&m_shared_resources.trig_down_channel);
    if (NRFX_SUCCESS != err) {
        return err;
    }
    err = nrfx_ppi_channel_assign(m_shared_resources.trig_up_channel,
                nrfx_timer_compare_event_address_get(&m_shared_resources.timer,
                                                     TIMER_TRIG_UP_CHAN),
                nrfx_gpiote_out_task_addr_get(trig_pin));
    if (NRFX_SUCCESS != err) {
        return err;
    }
    err = nrfx_ppi_channel_assign(m_shared_resources.trig_down_channel,
                nrfx_timer_compare_event_address_get(&m_shared_resources.timer,
                                                     TIMER_TRIG_DOWN_CHAN),
                nrfx_gpiote_out_task_addr_get(trig_pin));
    if (NRFX_SUCCESS != err) {
        return err;
    }
    err = nrfx_ppi_channel_fork_assign(m_shared_resources.trig_down_channel,
                nrfx_ppi_task_addr_group_enable_get(rising_echo_group));
    if (NRFX_SUCCESS != err) {
        return err;
    }
    err = nrfx_ppi_channel_enable(m_shared_resources.trig_up_channel);
    if (NRFX_SUCCESS != err) {
        return err;
    }
    err = nrfx_ppi_channel_enable(m_shared_resources.trig_down_channel);
    if (NRFX_SUCCESS != err) {
        return err;
    }

    /*
     * Rising echo event -> Capture TIMER to CC[TIMER_ECHO_START_CHAN]
     *                   -> Disable rising edge group
     *                   -> Enable falling edge group
     */
    err = nrfx_ppi_channel_alloc(&m_shared_resources.timer_start_channel);
    if (NRFX_SUCCESS != err) {
        return err;
    }
    err = nrfx_ppi_channel_alloc(&m_shared_resources.rising_group_channel);
    if (NRFX_SUCCESS != err) {
        return err;
    }
    err = nrfx_ppi_channel_assign(m_shared_resources.timer_start_channel,
                nrfx_gpiote_in_event_addr_get(echo_pin),
                nrfx_timer_capture_task_address_get(&m_shared_resources.timer,
                                                    TIMER_ECHO_START_CHAN));
    if (NRFX_SUCCESS != err) {
        return err;
    }
    err = nrfx_ppi_channel_assign(m_shared_resources.rising_group_channel,
                nrfx_gpiote_in_event_addr_get(echo_pin),
                nrfx_ppi_task_addr_group_disable_get(rising_echo_group));
    if (NRFX_SUCCESS != err) {
        return err;
    }
    err = nrfx_ppi_channel_fork_assign(m_shared_resources.rising_group_channel,
                nrfx_ppi_task_addr_group_enable_get(falling_echo_group));
    if (NRFX_SUCCESS != err) {
        return err;
    }
    err = nrfx_ppi_channel_include_in_group(m_shared_resources.timer_start_channel,
                                            rising_echo_group);
    if (NRFX_SUCCESS != err) {
        return err;
    }
    err = nrfx_ppi_channel_include_in_group(m_shared_resources.rising_group_channel,
                                            rising_echo_group);
    if (NRFX_SUCCESS != err) {
        return err;
    }
    err = nrfx_ppi_channel_enable(m_shared_resources.timer_start_channel);
    if (NRFX_SUCCESS != err) {
        return err;
    }
    err = nrfx_ppi_channel_enable(m_shared_resources.rising_group_channel);
    if (NRFX_SUCCESS != err) {
        return err;
    }
    err = nrfx_ppi_group_disable(rising_echo_group);
    if (NRFX_SUCCESS != err) {
        return err;
    }

    /*
     * Falling echo event -> Capture TIMER to CC[TIMER_ECHO_END_CHAN]
     *                    -> Stop TIMER
     *                    -> Clear TIMER
     *                    -> Trigger EGU interrupt
     *                    -> Disable falling edge group
     */
    err = nrfx_ppi_channel_alloc(&m_shared_resources.capture_stop_channel);
    if (NRFX_SUCCESS != err) {
        return err;
    }
    err = nrfx_ppi_channel_alloc(&m_shared_resources.clear_int_channel);
    if (NRFX_SUCCESS != err) {
        return err;
    }
    err = nrfx_ppi_channel_alloc(&m_shared_resources.falling_group_channel);
    if (NRFX_SUCCESS != err) {
        return err;
    }
    err = nrfx_ppi_channel_assign(m_shared_resources.capture_stop_channel,
                nrfx_gpiote_in_event_addr_get(echo_pin),
                nrfx_timer_capture_task_address_get(&m_shared_resources.timer,
                                                    TIMER_ECHO_END_CHAN));
    if (NRFX_SUCCESS != err) {
        return err;
    }
    err = nrfx_ppi_channel_fork_assign(m_shared_resources.capture_stop_channel,
                nrfx_timer_task_address_get(&m_shared_resources.timer, NRF_TIMER_TASK_STOP));
    if (NRFX_SUCCESS != err) {
        return err;
    }
    err = nrfx_ppi_channel_assign(m_shared_resources.clear_int_channel,
                nrfx_gpiote_in_event_addr_get(echo_pin),
                nrf_egu_task_address_get(p_egu, NRFX_CONCAT_2(NRF_EGU_TASK_TRIGGER,EGU_EVENT_POS)));
    if (NRFX_SUCCESS != err) {
        return err;
    }
    err = nrfx_ppi_channel_fork_assign(m_shared_resources.clear_int_channel,
                nrfx_timer_task_address_get(&m_shared_resources.timer, NRF_TIMER_TASK_CLEAR));
    if (NRFX_SUCCESS != err) {
        return err;
    }
    err = nrfx_ppi_channel_assign(m_shared_resources.falling_group_channel,
                nrfx_gpiote_in_event_addr_get(echo_pin),
                nrfx_ppi_task_addr_group_disable_get(falling_echo_group));
    if (NRFX_SUCCESS != err) {
        return err;
    }
    err = nrfx_ppi_channel_include_in_group(m_shared_resources.capture_stop_channel,
                                            falling_echo_group);
    if (NRFX_SUCCESS != err) {
        return err;
    }
    err = nrfx_ppi_channel_include_in_group(m_shared_resources.clear_int_channel,
                                            falling_echo_group);
    if (NRFX_SUCCESS != err) {
        return err;
    }
    err = nrfx_ppi_channel_include_in_group(m_shared_resources.falling_group_channel,
                                            falling_echo_group);
    if (NRFX_SUCCESS != err) {
        return err;
    }
    err = nrfx_ppi_channel_enable(m_shared_resources.capture_stop_channel);
    if (NRFX_SUCCESS != err) {
        return err;
    }
    err = nrfx_ppi_channel_enable(m_shared_resources.clear_int_channel);
    if (NRFX_SUCCESS != err) {
        return err;
    }
    err = nrfx_ppi_channel_enable(m_shared_resources.falling_group_channel);
    if (NRFX_SUCCESS != err) {
        return err;
    }
    err = nrfx_ppi_group_disable(falling_echo_group);
    if (NRFX_SUCCESS != err) {
        return err;
    }
    return NRFX_SUCCESS;
}

static int hc_sr04_nrfx_init(const struct device *dev)
{
    int           err;
    nrfx_err_t    nrfx_err;
    NRF_EGU_Type *p_egu;

    const struct hc_sr04_nrfx_cfg *p_cfg  = dev->config;
    struct hc_sr04_nrfx_data      *p_data = dev->data;

    p_data->sensor_value.val1 = 0;
    p_data->sensor_value.val2 = 0;

    if (m_shared_resources.ready) {
        /* Already initialized */
        return 0;
    }

    err = k_sem_init(&m_shared_resources.fetch_sem, 0, 1);
    if (0 != err) {
        return err;
    }
    err = k_mutex_init(&m_shared_resources.mutex);
    if (0 != err) {
        return err;
    }

    /* NOTE: This interrupt priority is not used. */
    nrfx_err = nrfx_gpiote_init(0);
    if (NRFX_SUCCESS != nrfx_err) {
        goto ERR_EXIT;
    }

    m_shared_resources.echo_in.sense           = NRF_GPIOTE_POLARITY_TOGGLE;
    m_shared_resources.echo_in.pull            = NRF_GPIO_PIN_NOPULL;
    m_shared_resources.echo_in.is_watcher      = false;
    m_shared_resources.echo_in.hi_accuracy     = true;
    m_shared_resources.echo_in.skip_gpio_setup = false;

    m_shared_resources.trig_out.action     = NRF_GPIOTE_POLARITY_TOGGLE;
    m_shared_resources.trig_out.init_state = false;
    m_shared_resources.trig_out.task_pin   = true;

    nrfx_err = nrfx_gpiote_in_init(p_cfg->echo_pin, &m_shared_resources.echo_in, gpiote_handler);
    if (NRFX_SUCCESS != nrfx_err) {
        goto ERR_EXIT;
    }
    nrfx_err = nrfx_gpiote_out_init(p_cfg->trig_pin, &m_shared_resources.trig_out);
    if (NRFX_SUCCESS != nrfx_err) {
        goto ERR_EXIT;
    }
    nrfx_err = timer_init();
    if (NRFX_SUCCESS != nrfx_err) {
        goto ERR_EXIT;
    }
    nrfx_err = egu_init(&p_egu);
    if (NRFX_SUCCESS != nrfx_err) {
        goto ERR_EXIT;
    }
    nrfx_err = ppi_init(p_egu, p_cfg->echo_pin, p_cfg->trig_pin);
    if (NRFX_SUCCESS != nrfx_err) {
        goto ERR_EXIT;
    }

    /* These will be re-initialized for every fetch. */
    nrfx_gpiote_in_uninit(p_cfg->echo_pin);
    nrfx_gpiote_out_uninit(p_cfg->trig_pin);

    m_shared_resources.ready = true;
    return 0;

ERR_EXIT:
    return -ENXIO;
}

static int hc_sr04_nrfx_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    int        err;
    nrfx_err_t nrfx_err;
    uint32_t   count;

    const struct hc_sr04_nrfx_cfg *p_cfg  = dev->config;
    struct hc_sr04_nrfx_data      *p_data = dev->data;

    if (unlikely((SENSOR_CHAN_ALL != chan) && (SENSOR_CHAN_DISTANCE != chan))) {
        return -ENOTSUP;
    }

    if (unlikely(!m_shared_resources.ready)) {
        LOG_ERR("Driver is not initialized yet");
        return -EBUSY;
    }

    err = k_mutex_lock(&m_shared_resources.mutex, K_FOREVER);
    if (0 != err) {
        return err;
    }

    nrfx_err = gpiote_pins_init(p_cfg->trig_pin, p_cfg->echo_pin);
    if (NRFX_SUCCESS != nrfx_err) {
        LOG_ERR("GPIOTE init failed: %d", nrfx_err);
        (void) k_mutex_unlock(&m_shared_resources.mutex);
        return -ENXIO;
    }

    nrfx_timer_enable(&m_shared_resources.timer);

    if (0 != k_sem_take(&m_shared_resources.fetch_sem, K_MSEC(T_MAX_WAIT_MS))) {
        LOG_DBG("No response from HC-SR04.");
        nrfx_timer_disable(&m_shared_resources.timer);
        gpiote_pins_uninit(p_cfg->trig_pin, p_cfg->echo_pin);
        (void) k_mutex_unlock(&m_shared_resources.mutex);
        return -EIO;
    }

    nrfx_timer_disable(&m_shared_resources.timer);
    gpiote_pins_uninit(p_cfg->trig_pin, p_cfg->echo_pin);

    count  = nrfx_timer_capture_get(&m_shared_resources.timer, TIMER_ECHO_END_CHAN);
    count -= nrfx_timer_capture_get(&m_shared_resources.timer, TIMER_ECHO_START_CHAN);
    if ((T_INVALID_PULSE_US > count) && (T_TRIG_PULSE_US < count)) {
        /* Convert to meters and divide round-trip distance by two */
        count = (count * METERS_PER_SEC / 2);
        p_data->sensor_value.val2 = (count % 1000000);
        p_data->sensor_value.val1 = (count - p_data->sensor_value.val2);
    } else {
        LOG_INF("Invalid measurement");
        p_data->sensor_value.val1 = 0;
        p_data->sensor_value.val2 = 0;
        k_usleep(T_SPURIOS_WAIT_US);
    }

    err = k_mutex_unlock(&m_shared_resources.mutex);
    if (0 != err) {
        return err;
    }
    return 0;
}

static int hc_sr04_nrfx_channel_get(const struct device *dev,
                    enum sensor_channel chan,
                    struct sensor_value *val)
{
    const struct hc_sr04_nrfx_data *p_data = dev->data;

    if (unlikely(!m_shared_resources.ready)) {
        LOG_WRN("Device is not initialized yet");
        return -EBUSY;
    }

    switch (chan) {
    case SENSOR_CHAN_DISTANCE:
        val->val2 = p_data->sensor_value.val2;
        val->val1 = p_data->sensor_value.val1;
        break;
    default:
        return -ENOTSUP;
    }
    return 0;
}

static const struct sensor_driver_api hc_sr04_nrfx_driver_api = {
    .sample_fetch = hc_sr04_nrfx_sample_fetch,
    .channel_get  = hc_sr04_nrfx_channel_get,
};

#define HC_SR04_NRFX_DEVICE(n) \
    static const struct hc_sr04_nrfx_cfg hc_sr04_nrfx_cfg_##n = { \
        .trig_pin = DT_PROP(DT_NODELABEL(us##n##_nrfx), trig_pin), \
        .echo_pin = DT_PROP(DT_NODELABEL(us##n##_nrfx), echo_pin) \
    }; \
    static struct hc_sr04_nrfx_data hc_sr04_nrfx_data_##n; \
    DEVICE_AND_API_INIT(hc_sr04_nrfx_##n, \
                DT_LABEL(DT_NODELABEL(us##n##_nrfx)), \
                hc_sr04_nrfx_init, \
                &hc_sr04_nrfx_data_##n, \
                &hc_sr04_nrfx_cfg_##n, \
                POST_KERNEL, \
                CONFIG_SENSOR_INIT_PRIORITY, \
                &hc_sr04_nrfx_driver_api);

#if DT_NODE_HAS_STATUS(DT_NODELABEL(us0_nrfx), okay)
HC_SR04_NRFX_DEVICE(0)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(us1_nrfx), okay)
HC_SR04_NRFX_DEVICE(1)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(us2_nrfx), okay)
HC_SR04_NRFX_DEVICE(2)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(us3_nrfx), okay)
HC_SR04_NRFX_DEVICE(3)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(us4_nrfx), okay)
HC_SR04_NRFX_DEVICE(4)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(us5_nrfx), okay)
HC_SR04_NRFX_DEVICE(5)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(us6_nrfx), okay)
HC_SR04_NRFX_DEVICE(6)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(us7_nrfx), okay)
HC_SR04_NRFX_DEVICE(7)
#endif

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "HC_SR04_NRFX driver enabled without any devices"
#endif

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) > 8
#warning "Too many HC_SR04_NRFX devices"
#endif
