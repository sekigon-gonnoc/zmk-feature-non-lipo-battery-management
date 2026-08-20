/*
 * Copyright (c) 2025 sekigon-gonnoc
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_non_lipo_battery

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/pm/pm.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>

#include <zmk/pm.h>
#include <zmk/usb.h>

#ifdef CONFIG_ADC_NRFX_SAADC
#include <zephyr/dt-bindings/adc/nrf-adc.h>
#ifndef NRF_SAADC_AIN0
#define NRF_SAADC_AIN0 NRF_ADC_AIN0
#endif
#else
#error "This driver requires the nRF SAADC ADC driver to be enabled."
#endif

LOG_MODULE_REGISTER(zmk_battery_non_lipo, CONFIG_ZMK_LOG_LEVEL);

#if IS_ENABLED(CONFIG_ZMK_NON_LIPO_ADV_SLEEP_TIMEOUT)
static bool is_advertising = true;  // Assume advertising when there's no connection
static bool has_connection = false;
static int64_t advertising_start_time = 0;
static struct k_work_delayable adv_timeout_work;

// Callback to check advertising time and enter sleep mode if needed
static void adv_timeout_handler(struct k_work *work) {
    // Only check if we're advertising, have no connections, and no USB power
    if (is_advertising && !has_connection && !zmk_usb_is_powered()) {
        int64_t now = k_uptime_get();
        int64_t elapsed = now - advertising_start_time;
        
        // If we've been advertising longer than the timeout, enter sleep mode
        if (elapsed >= CONFIG_ZMK_NON_LIPO_ADV_SLEEP_TIMEOUT) {
            LOG_WRN("Advertising timeout reached (%lldms), entering sleep", elapsed);
            
            // Wait for logs to flush
            k_sleep(K_MSEC(100));

            // Power off the system
            zmk_pm_suspend_devices();
            sys_poweroff();
        } else {
            // Not timed out yet, reschedule the timer
            int64_t remaining = CONFIG_ZMK_NON_LIPO_ADV_SLEEP_TIMEOUT - elapsed;
            k_work_schedule(&adv_timeout_work, K_MSEC(MIN(remaining, 10000)));
        }
    }
}

// Bluetooth connection callbacks
static void connected_cb(struct bt_conn *conn, uint8_t err) {
    if (err) {
        LOG_ERR("Connection failed (err %u)", err);
        return;
    }

    LOG_DBG("Connected, stopping advertising timer");
    has_connection = true;
    is_advertising = false;
    k_work_cancel_delayable(&adv_timeout_work);
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason) {
    LOG_DBG("Disconnected (reason %u), starting advertising timer", reason);
    has_connection = false;
    
    // Start tracking advertising time
    is_advertising = true;
    advertising_start_time = k_uptime_get();
    k_work_schedule(&adv_timeout_work, K_MSEC(10000)); // Check after 10 seconds
}

// Bluetooth connection callback structure
static struct bt_conn_cb conn_callbacks = {
    .connected = connected_cb,
    .disconnected = disconnected_cb,
};
#endif

struct io_channel_config {
    uint8_t channel;
};

struct non_lipo_config {
    struct io_channel_config io_channel;
#if DT_INST_NODE_HAS_PROP(0, power_gpios)
    struct gpio_dt_spec power;
#endif
};

struct non_lipo_data {
    const struct device *adc;
    struct adc_channel_cfg acc;
    struct adc_sequence as;
    int16_t adc_raw;
    uint16_t millivolts;
    uint8_t state_of_charge;
};

static uint8_t non_lipo_mv_to_pct(int16_t mv) {
    // Linear approximation based on min/max voltage config
    if (mv >= CONFIG_ZMK_NON_LIPO_MAX_MV) {
        return 100;
    } else if (mv <= CONFIG_ZMK_NON_LIPO_MIN_MV) {
        return 0;
    }

    // Calculate percentage based on min/max range
    return (100 * (mv - CONFIG_ZMK_NON_LIPO_MIN_MV)) / 
           (CONFIG_ZMK_NON_LIPO_MAX_MV - CONFIG_ZMK_NON_LIPO_MIN_MV);
}

static void check_voltage_and_shutdown(uint16_t millivolts) {
    // Check if voltage is below the low threshold
    if (millivolts <= CONFIG_ZMK_NON_LIPO_LOW_MV) {
        // Only shut down if USB power is not connected
        if (!zmk_usb_is_powered()) {
            LOG_WRN("Battery voltage (%dmv) below critical threshold (%dmv) and USB not connected, shutting down",
                    millivolts, CONFIG_ZMK_NON_LIPO_LOW_MV);

            // Wait for logs to flush
            k_sleep(K_MSEC(100));

            // Power off system
            zmk_pm_suspend_devices();
            sys_poweroff();
        } else {
            LOG_WRN("Battery voltage (%dmv) below critical threshold (%dmv) but USB power detected, staying on",
                    millivolts, CONFIG_ZMK_NON_LIPO_LOW_MV);
        }
    }
}

static int non_lipo_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    struct non_lipo_data *drv_data = dev->data;
    struct adc_sequence *as = &drv_data->as;

    // Make sure selected channel is supported
    if (chan != SENSOR_CHAN_GAUGE_VOLTAGE && chan != SENSOR_CHAN_GAUGE_STATE_OF_CHARGE &&
        chan != SENSOR_CHAN_ALL) {
        LOG_DBG("Selected channel is not supported: %d.", chan);
        return -ENOTSUP;
    }

    int rc = 0;

#if DT_INST_NODE_HAS_PROP(0, power_gpios)
    const struct non_lipo_config *drv_cfg = dev->config;
    // Enable power before sampling
    rc = gpio_pin_set_dt(&drv_cfg->power, 1);

    if (rc != 0) {
        LOG_DBG("Failed to enable ADC power GPIO: %d", rc);
        return rc;
    }

    // Wait for stabilization
    k_sleep(K_MSEC(10));
#endif

    // Read ADC
    rc = adc_read(drv_data->adc, as);
    as->calibrate = false;

    if (rc == 0) {
        int32_t val = drv_data->adc_raw;

        adc_raw_to_millivolts(adc_ref_internal(drv_data->adc), drv_data->acc.gain, as->resolution,
                              &val);

        uint16_t millivolts = val;
        LOG_DBG("ADC raw %d ~ %d mV", drv_data->adc_raw, millivolts);
        
        drv_data->millivolts = millivolts;
        drv_data->state_of_charge = non_lipo_mv_to_pct(millivolts);
        
        LOG_DBG("Battery: %d mV, %d%%", millivolts, drv_data->state_of_charge);
        
        // Check if we need to shut down due to low voltage
        check_voltage_and_shutdown(millivolts);
    } else {
        LOG_DBG("Failed to read ADC: %d", rc);
    }

#if DT_INST_NODE_HAS_PROP(0, power_gpios)
    // Disable power GPIO if present
    int rc2 = gpio_pin_set_dt(&drv_cfg->power, 0);

    if (rc2 != 0) {
        LOG_DBG("Failed to disable ADC power GPIO: %d", rc2);
        return rc2;
    }
#endif

    return rc;
}

static int non_lipo_channel_get(const struct device *dev, enum sensor_channel chan,
                               struct sensor_value *val) {
    struct non_lipo_data *drv_data = dev->data;

    switch (chan) {
    case SENSOR_CHAN_GAUGE_VOLTAGE:
        val->val1 = drv_data->millivolts / 1000;
        val->val2 = (drv_data->millivolts % 1000) * 1000;
        break;

    case SENSOR_CHAN_GAUGE_STATE_OF_CHARGE:
        val->val1 = drv_data->state_of_charge;
        val->val2 = 0;
        break;

    default:
        return -ENOTSUP;
    }

    return 0;
}

static const struct sensor_driver_api non_lipo_api = {
    .sample_fetch = non_lipo_sample_fetch,
    .channel_get = non_lipo_channel_get,
};

static int non_lipo_init(const struct device *dev) {
    struct non_lipo_data *drv_data = dev->data;
    const struct non_lipo_config *drv_cfg = dev->config;

    if (!device_is_ready(drv_data->adc)) {
        LOG_ERR("ADC device not ready");
        return -ENODEV;
    }

    int rc = 0;

#if DT_INST_NODE_HAS_PROP(0, power_gpios)
    if (!device_is_ready(drv_cfg->power.port)) {
        LOG_ERR("GPIO port for power control is not ready");
        return -ENODEV;
    }
    rc = gpio_pin_configure_dt(&drv_cfg->power, GPIO_OUTPUT_INACTIVE);
    if (rc != 0) {
        LOG_ERR("Failed to configure power pin %u: %d", drv_cfg->power.pin, rc);
        return rc;
    }
#endif

    drv_data->as = (struct adc_sequence){
        .channels = BIT(0),
        .buffer = &drv_data->adc_raw,
        .buffer_size = sizeof(drv_data->adc_raw),
        .oversampling = 4,
        .calibrate = true,
    };

#ifdef CONFIG_ADC_NRFX_SAADC
    drv_data->acc = (struct adc_channel_cfg){
        .gain = ADC_GAIN_1_6,
        .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
        .input_positive = NRF_SAADC_AIN0 + drv_cfg->io_channel.channel,
    };

    drv_data->as.resolution = 12;
#else
#error Unsupported ADC
#endif

    rc = adc_channel_setup(drv_data->adc, &drv_data->acc);
    LOG_DBG("AIN%u setup returned %d", drv_cfg->io_channel.channel, rc);

#if IS_ENABLED(CONFIG_ZMK_NON_LIPO_ADV_SLEEP_TIMEOUT)
    // Initialize the advertising timeout work
    k_work_init_delayable(&adv_timeout_work, adv_timeout_handler);
    
    // Register Bluetooth connection callbacks
    bt_conn_cb_register(&conn_callbacks);
    
    LOG_INF("Advertising sleep timeout initialized (%d ms)", CONFIG_ZMK_NON_LIPO_ADV_SLEEP_TIMEOUT);
    
    // Start the timer for initial check
    advertising_start_time = k_uptime_get();
    k_work_schedule(&adv_timeout_work, K_MSEC(10000));
#endif

    return rc;
}

static struct non_lipo_data non_lipo_data = {
    .adc = DEVICE_DT_GET(DT_IO_CHANNELS_CTLR(DT_DRV_INST(0)))
};

static const struct non_lipo_config non_lipo_cfg = {
    .io_channel = {
        DT_IO_CHANNELS_INPUT(DT_DRV_INST(0)),
    },
#if DT_INST_NODE_HAS_PROP(0, power_gpios)
    .power = GPIO_DT_SPEC_INST_GET(0, power_gpios),
#endif
};

DEVICE_DT_INST_DEFINE(0, &non_lipo_init, NULL, &non_lipo_data, &non_lipo_cfg, POST_KERNEL,
                      CONFIG_SENSOR_INIT_PRIORITY, &non_lipo_api);
