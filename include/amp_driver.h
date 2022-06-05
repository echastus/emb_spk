#ifndef __AMP_DRIVER_H__
#define __AMP_DRIVER_H__

#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"

#define AMP_TAG "AMP"

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM 0                        /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 100000               /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0             /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0             /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

/**
 * @brief Read a sequence of bytes from a SSM3582A audio amplifier
 */
esp_err_t ssm3582a_register_read(uint8_t reg_addr, uint8_t *data, size_t len);

/**
 * @brief Write a byte to a SSM3582A audio amplifier
 */
esp_err_t ssm3582a_register_write_byte(uint8_t reg_addr, uint8_t data);

/**
 * @brief SSM3582A initialization
 *
 */
void ssm3582a_init(void);

/**
 * @brief i2c master initialization
 */
esp_err_t i2c_master_init(void);

#endif