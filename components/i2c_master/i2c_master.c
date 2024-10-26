/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c.h"
#include "driver/i2c_master.h"

#define ACK_CHECK_EN                        0x1
#define ACK_CHECK_DIS                       0x0

#define WRITE_BIT                           I2C_MASTER_WRITE
#define READ_BIT                            I2C_MASTER_READ

#define ACK_VAL                             I2C_MASTER_ACK
#define NACK_VAL                            I2C_MASTER_NACK
#define LAST_NACK_VAL                       I2C_MASTER_LAST_NACK

typedef enum i2c_trans_type {
    I2C_TRANS_READ = 0,
    I2C_TRANS_WRITE,
    I2C_TRANS_WRITE_READ,
    I2C_TRANS_PROBE,
} i2c_trans_type_t;

typedef struct i2c_trans {
    i2c_trans_type_t type;
    
    i2c_master_bus_handle_t bus_handle;
    uint8_t addr;
    bool check_ack;

    struct {
        size_t array_size;
        i2c_master_transmit_multi_buffer_info_t *buffer_info_array;
    } write;

    struct {
        uint8_t *buffer;
        size_t size;
    } read;

    uint32_t timeout_ms;
} i2c_trans_t;

static esp_err_t i2c_master_trans(i2c_trans_t *trans)
{
    esp_err_t ret = ESP_FAIL;
    i2c_cmd_handle_t cmd;
    uint8_t write_bit;

    xSemaphoreTake(trans->bus_handle->mutex, trans->timeout_ms);

    cmd = i2c_cmd_link_create();
    if (!cmd) {
        goto exit;
    }

    ret = i2c_master_start(cmd);
    if (ret != ESP_OK) {
        goto exit;
    }

    write_bit = (trans->type == I2C_TRANS_WRITE || trans->type == I2C_TRANS_WRITE_READ) ? WRITE_BIT :
                trans->type == I2C_TRANS_READ ? READ_BIT : 0;
    ret = i2c_master_write_byte(cmd, trans->addr << 1 | write_bit, trans->check_ack);
    if (ret != ESP_OK) {
        goto exit;
    }

    if ((trans->type == I2C_TRANS_WRITE) || (trans->type == I2C_TRANS_WRITE_READ)) {
        for (int i = 0; i < trans->write.array_size; i++) {
            ret = i2c_master_write(cmd, trans->write.buffer_info_array[i].write_buffer,
                                   trans->write.buffer_info_array[i].buffer_size, trans->check_ack);
            if (ret != ESP_OK) {
                goto exit;
            }
        }
    } 
    
    if (trans->type == I2C_TRANS_WRITE_READ) {
        ret = i2c_master_start(cmd);
        if (ret != ESP_OK) {
            goto exit;
        }

        ret = i2c_master_write_byte(cmd, trans->addr << 1 | READ_BIT, trans->check_ack);
        if (ret != ESP_OK) {
            goto exit;
        }
    }

    if ((trans->type == I2C_TRANS_READ) || (trans->type == I2C_TRANS_WRITE_READ)) {
        ret = i2c_master_read(cmd, trans->read.buffer, trans->read.size, LAST_NACK_VAL);
        if (ret != ESP_OK) {
            goto exit;
        }
    }

    ret = i2c_master_stop(cmd);
    if (ret != ESP_OK) {
        goto exit;
    }

    ret = i2c_master_cmd_begin(trans->bus_handle->port, cmd, trans->timeout_ms / portTICK_RATE_MS);

exit:
    i2c_cmd_link_delete(cmd);
    xSemaphoreGive(trans->bus_handle->mutex);
    return ret; 
}

/**
 * @brief Allocate an I2C master bus
 *
 * @param[in] bus_config I2C master bus configuration.
 * @param[out] ret_bus_handle I2C bus handle
 * @return
 *      - ESP_OK: I2C master bus initialized successfully.
 *      - ESP_ERR_INVALID_ARG: I2C bus initialization failed because of invalid argument.
 *      - ESP_ERR_NO_MEM: Create I2C bus failed because of out of memory.
 *      - ESP_ERR_NOT_FOUND: No more free bus.
 */
esp_err_t i2c_new_master_bus(const i2c_master_bus_config_t *bus_config, i2c_master_bus_handle_t *ret_bus_handle)
{
    esp_err_t ret;
    i2c_master_bus_t *bus;

    if (!bus_config || !ret_bus_handle) {
        return ESP_ERR_INVALID_ARG;
    }

    if (bus_config->i2c_port >= I2C_NUM_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    bus = heap_caps_malloc(sizeof(i2c_master_bus_t), MALLOC_CAP_8BIT);
    if (!bus) {
        return ESP_ERR_NO_MEM;
    }

    bus->mutex = xSemaphoreCreateMutex();
    if (!bus) {
        heap_caps_free(bus);
        return ESP_ERR_NO_MEM;
    }

    int i2c_master_port = bus_config->i2c_port;
    i2c_config_t conf = {0};
    
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = bus_config->sda_io_num;
    conf.sda_pullup_en = bus_config->flags.enable_internal_pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    conf.scl_io_num = bus_config->scl_io_num;
    conf.scl_pullup_en = bus_config->flags.enable_internal_pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    conf.clk_stretch_tick = 286;

    ret = i2c_driver_install(i2c_master_port, conf.mode);
    if (ret != ESP_OK) {
        vSemaphoreDelete(bus->mutex);
        heap_caps_free(bus);
        return ret;
    }

    ret = i2c_param_config(i2c_master_port, &conf);
    if (ret != ESP_OK) {
        vSemaphoreDelete(bus->mutex);
        heap_caps_free(bus);
        i2c_driver_delete(i2c_master_port);
        return ret;
    }

    bus->port = i2c_master_port;
    *ret_bus_handle = bus;

    return ESP_OK;
}

/**
 * @brief Add I2C master BUS device.
 *
 * @param[in] bus_handle I2C bus handle.
 * @param[in] dev_config device config.
 * @param[out] ret_handle device handle.
 * @return
 *      - ESP_OK: Create I2C master device successfully.
 *      - ESP_ERR_INVALID_ARG: I2C bus initialization failed because of invalid argument.
 *      - ESP_ERR_NO_MEM: Create I2C bus failed because of out of memory.
 */
esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t bus_handle, const i2c_device_config_t *dev_config, i2c_master_dev_handle_t *ret_handle)
{
    struct i2c_master_dev *dev;

    if (!bus_handle || !dev_config || !ret_handle) {
        return ESP_ERR_INVALID_ARG;
    }

    dev = heap_caps_malloc(sizeof(struct i2c_master_dev), MALLOC_CAP_8BIT);
    if (!dev) {
        return ESP_ERR_NO_MEM;
    }

    dev->master_bus = bus_handle;
    dev->device_address = dev_config->device_address;
    dev->ack_check_disable = dev_config->flags.disable_ack_check;

    *ret_handle = dev;

    return ESP_OK;
}

/**
 * @brief Deinitialize the I2C master bus and delete the handle.
 *
 * @param[in] bus_handle I2C bus handle.
 * @return
 *      - ESP_OK: Delete I2C bus success, otherwise, failed.
 *      - Otherwise: Some module delete failed.
 */
esp_err_t i2c_del_master_bus(i2c_master_bus_handle_t bus_handle)
{
    if (!bus_handle) {
        return ESP_ERR_INVALID_ARG;
    }

    vSemaphoreDelete(bus_handle->mutex);
    heap_caps_free(bus_handle);

    return i2c_driver_delete(I2C_NUM_0);
}

/**
 * @brief I2C master bus delete device
 *
 * @param handle i2c device handle
 * @return
 *      - ESP_OK: If device is successfully deleted.
 */
esp_err_t i2c_master_bus_rm_device(i2c_master_dev_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    heap_caps_free(handle);

    return ESP_OK;
}

/**
 * @brief Perform a write transaction on the I2C bus.
 *        The transaction will be undergoing until it finishes or it reaches
 *        the timeout provided.
 *
 * @note If a callback was registered with `i2c_master_register_event_callbacks`, the transaction will be asynchronous, and thus, this function will return directly, without blocking.
 *       You will get finish information from callback. Besides, data buffer should always be completely prepared when callback is registered, otherwise, the data will get corrupt.
 *
 * @param[in] i2c_dev I2C master device handle that created by `i2c_master_bus_add_device`.
 * @param[in] write_buffer Data bytes to send on the I2C bus.
 * @param[in] write_size Size, in bytes, of the write buffer.
 * @param[in] xfer_timeout_ms Wait timeout, in ms. Note: -1 means wait forever.
 * @return
 *      - ESP_OK: I2C master transmit success
 *      - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid.
 *      - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash.
 */
esp_err_t i2c_master_transmit(i2c_master_dev_handle_t i2c_dev, const uint8_t *write_buffer, size_t write_size, int xfer_timeout_ms)
{
    i2c_master_transmit_multi_buffer_info_t buffer_info = {
        .buffer_size = write_size,
        .write_buffer = (uint8_t *)write_buffer
    };
    i2c_trans_t trans = {
        .type = I2C_TRANS_WRITE,
        .bus_handle = i2c_dev->master_bus,
        .addr = i2c_dev->device_address,
        .check_ack = i2c_dev->ack_check_disable ? ACK_CHECK_DIS : ACK_CHECK_EN,
        .write = {
            .array_size = 1,
            .buffer_info_array = &buffer_info
        },
        .timeout_ms = xfer_timeout_ms
    };

    return i2c_master_trans(&trans);
}

/**
 * @brief Transmit multiple buffers of data over an I2C bus.
 *
 * This function transmits multiple buffers of data over an I2C bus using the specified I2C master device handle.
 * It takes in an array of buffer information structures along with the size of the array and a transfer timeout value in milliseconds.
 *
 * @param i2c_dev I2C master device handle that created by `i2c_master_bus_add_device`.
 * @param buffer_info_array Pointer to buffer information array.
 * @param array_size size of buffer information array.
 * @param xfer_timeout_ms Wait timeout, in ms. Note: -1 means wait forever.
 *
 * @return
 *      - ESP_OK: I2C master transmit success
 *      - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid.
 *      - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash.
 */
esp_err_t i2c_master_multi_buffer_transmit(i2c_master_dev_handle_t i2c_dev, i2c_master_transmit_multi_buffer_info_t *buffer_info_array, size_t array_size, int xfer_timeout_ms)
{
    i2c_trans_t trans = {
        .type = I2C_TRANS_WRITE,
        .bus_handle = i2c_dev->master_bus,
        .addr = i2c_dev->device_address,
        .check_ack = i2c_dev->ack_check_disable ? ACK_CHECK_DIS : ACK_CHECK_EN,
        .write = {
            .array_size = array_size,
            .buffer_info_array = buffer_info_array
        },
        .timeout_ms = xfer_timeout_ms
    };

    return i2c_master_trans(&trans);
}

/**
 * @brief Perform a write-read transaction on the I2C bus.
 *        The transaction will be undergoing until it finishes or it reaches
 *        the timeout provided.
 *
 * @note If a callback was registered with `i2c_master_register_event_callbacks`, the transaction will be asynchronous, and thus, this function will return directly, without blocking.
 *       You will get finish information from callback. Besides, data buffer should always be completely prepared when callback is registered, otherwise, the data will get corrupt.
 *
 * @param[in] i2c_dev I2C master device handle that created by `i2c_master_bus_add_device`.
 * @param[in] write_buffer Data bytes to send on the I2C bus.
 * @param[in] write_size Size, in bytes, of the write buffer.
 * @param[out] read_buffer Data bytes received from i2c bus.
 * @param[in] read_size Size, in bytes, of the read buffer.
 * @param[in] xfer_timeout_ms Wait timeout, in ms. Note: -1 means wait forever.
 * @return
 *      - ESP_OK: I2C master transmit-receive success
 *      - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid.
 *      - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash.
 */
esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t i2c_dev, const uint8_t *write_buffer, size_t write_size, uint8_t *read_buffer, size_t read_size, int xfer_timeout_ms)
{
    i2c_master_transmit_multi_buffer_info_t buffer_info = {
        .buffer_size = write_size,
        .write_buffer = (uint8_t *)write_buffer
    };
    i2c_trans_t trans = {
        .type = I2C_TRANS_WRITE_READ,
        .bus_handle = i2c_dev->master_bus,
        .addr = i2c_dev->device_address,
        .check_ack = i2c_dev->ack_check_disable ? ACK_CHECK_DIS : ACK_CHECK_EN,
        .write = {
            .array_size = 1,
            .buffer_info_array = &buffer_info
        },
        .read = {
            .buffer = read_buffer,
            .size = read_size
        },
        .timeout_ms = xfer_timeout_ms
    };

    return i2c_master_trans(&trans);
}

/**
 * @brief Perform a read transaction on the I2C bus.
 *        The transaction will be undergoing until it finishes or it reaches
 *        the timeout provided.
 *
 * @note If a callback was registered with `i2c_master_register_event_callbacks`, the transaction will be asynchronous, and thus, this function will return directly, without blocking.
 *       You will get finish information from callback. Besides, data buffer should always be completely prepared when callback is registered, otherwise, the data will get corrupt.
 *
 * @param[in] i2c_dev I2C master device handle that created by `i2c_master_bus_add_device`.
 * @param[out] read_buffer Data bytes received from i2c bus.
 * @param[in] read_size Size, in bytes, of the read buffer.
 * @param[in] xfer_timeout_ms Wait timeout, in ms. Note: -1 means wait forever.
 * @return
 *      - ESP_OK: I2C master receive success
 *      - ESP_ERR_INVALID_ARG: I2C master receive parameter invalid.
 *      - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash.
 */
esp_err_t i2c_master_receive(i2c_master_dev_handle_t i2c_dev, uint8_t *read_buffer, size_t read_size, int xfer_timeout_ms)
{
    i2c_trans_t trans = {
        .type = I2C_TRANS_READ,
        .bus_handle = i2c_dev->master_bus,
        .addr = i2c_dev->device_address,
        .check_ack = i2c_dev->ack_check_disable ? ACK_CHECK_DIS : ACK_CHECK_EN,
        .read = {
            .buffer = read_buffer,
            .size = read_size
        },
        .timeout_ms = xfer_timeout_ms
    };

    return i2c_master_trans(&trans);
}

/**
 * @brief Probe I2C address, if address is correct and ACK is received, this function will return ESP_OK.
 *
 * @param[in] bus_handle I2C master device handle that created by `i2c_master_bus_add_device`.
 * @param[in] address I2C device address that you want to probe.
 * @param[in] xfer_timeout_ms Wait timeout, in ms. Note: -1 means wait forever (Not recommended in this function).
 *
 * @attention Pull-ups must be connected to the SCL and SDA pins when this function is called. If you get `ESP_ERR_TIMEOUT
 * while `xfer_timeout_ms` was parsed correctly, you should check the pull-up resistors. If you do not have proper resistors nearby.
 * `flags.enable_internal_pullup` is also acceptable.
 *
 * @note The principle of this function is to sent device address with a write command. If the device on your I2C bus, there would be an ACK signal and function
 * returns `ESP_OK`. If the device is not on your I2C bus, there would be a NACK signal and function returns `ESP_ERR_NOT_FOUND`. `ESP_ERR_TIMEOUT` is not an expected
 * failure, which indicated that the i2c probe not works properly, usually caused by pull-up resistors not be connected properly. Suggestion check data on SDA/SCL line
 * to see whether there is ACK/NACK signal is on line when i2c probe function fails.
 *
 * @note There are lots of I2C devices all over the world, we assume that not all I2C device support the behavior like `device_address+nack/ack`.
 * So, if the on line data is strange and no ack/nack got respond. Please check the device datasheet.
 *
 * @return
 *      - ESP_OK: I2C device probe successfully
 *      - ESP_ERR_NOT_FOUND: I2C probe failed, doesn't find the device with specific address you gave.
 *      - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash.
 */
esp_err_t i2c_master_probe(i2c_master_bus_handle_t bus_handle, uint16_t address, int xfer_timeout_ms)
{
    i2c_trans_t trans = {
        .type = I2C_TRANS_PROBE,
        .bus_handle = bus_handle,
        .addr = address,
        .check_ack = ACK_CHECK_EN,
        .timeout_ms = xfer_timeout_ms
    };

    return i2c_master_trans(&trans);
}

/**
 * @brief Register I2C transaction callbacks for a master device
 *
 * @note User can deregister a previously registered callback by calling this function and setting the callback member in the `cbs` structure to NULL.
 * @note When CONFIG_I2C_ISR_IRAM_SAFE is enabled, the callback itself and functions called by it should be placed in IRAM.
 *       The variables used in the function should be in the SRAM as well. The `user_data` should also reside in SRAM.
 * @note If the callback is used for helping asynchronous transaction. On the same bus, only one device can be used for performing asynchronous operation.
 *
 * @param[in] i2c_dev I2C master device handle that created by `i2c_master_bus_add_device`.
 * @param[in] cbs Group of callback functions
 * @param[in] user_data User data, which will be passed to callback functions directly
 * @return
 *      - ESP_OK: Set I2C transaction callbacks successfully
 *      - ESP_ERR_INVALID_ARG: Set I2C transaction callbacks failed because of invalid argument
 *      - ESP_FAIL: Set I2C transaction callbacks failed because of other error
 */
esp_err_t i2c_master_register_event_callbacks(i2c_master_dev_handle_t i2c_dev, const i2c_master_event_callbacks_t *cbs, void *user_data)
{
    return ESP_FAIL;
}

/**
 * @brief Reset the I2C master bus.
 *
 * @param bus_handle I2C bus handle.
 * @return
 *      - ESP_OK: Reset succeed.
 *      - ESP_ERR_INVALID_ARG: I2C master bus handle is not initialized.
 *      - Otherwise: Reset failed.
 */
esp_err_t i2c_master_bus_reset(i2c_master_bus_handle_t bus_handle)
{
    return ESP_OK;
}

/**
 * @brief Wait for all pending I2C transactions done
 *
 * @param[in] bus_handle I2C bus handle
 * @param[in] timeout_ms Wait timeout, in ms. Specially, -1 means to wait forever.
 * @return
 *      - ESP_OK: Flush transactions successfully
 *      - ESP_ERR_INVALID_ARG: Flush transactions failed because of invalid argument
 *      - ESP_ERR_TIMEOUT: Flush transactions failed because of timeout
 *      - ESP_FAIL: Flush transactions failed because of other error
 */
esp_err_t i2c_master_bus_wait_all_done(i2c_master_bus_handle_t bus_handle, int timeout_ms)
{
    return ESP_OK;
}
