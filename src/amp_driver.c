#include "amp_driver.h"
#include "ssm3582.h"

esp_err_t ssm3582a_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, SSM3582_DEVICE_ADDRESS, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}

esp_err_t ssm3582a_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, SSM3582_DEVICE_ADDRESS, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    return ret;
}

void ssm3582a_init(void)
{
    // automatic power down en, temp sensor on, stereo en, right and left channel power on, normal operation
    ssm3582a_register_write_byte(POWER_CTRL, 0x80);
    // dac low power mode disabled, normal behaviour of dac output polarity, edge rate: low emi mode operation, 13 dB gain (6.3v peak)
    ssm3582a_register_write_byte(AMP_DAC_CTRL, 0x88);
    // dac high pass filter enabled, 32-48 khz dac sample rate, soft volume ramping
    ssm3582a_register_write_byte(DAC_CTRL, 0x12);

    // 0dB is 0x40, this is idk -3dB?
    ssm3582a_register_write_byte(VOL_LEFT_CTRL, 0x4A);
    ssm3582a_register_write_byte(VOL_RIGHT_CTRL, 0x4A);

    // rising edge to capture SDATA, tdm 32 bits width, serial interface mode - stereo modes,
    // serial data formas i2s delayed by 1 format, low fsync is left channel
    ssm3582a_register_write_byte(SAI_CTRL1, 0x10);
    // sdata edge nodelay, ((( 24 bits input ))), volume cahnge at all times
    ssm3582a_register_write_byte(SAI_CTRL2, 0x07);
    // dunno, its just at reset value - slot selectrion for TDM?
    ssm3582a_register_write_byte(SLOT_LEFT_CTRL, 0x00);
    // slot selection for right channel, some TDM setting
    ssm3582a_register_write_byte(SLOT_RIGHT_CTRL, 0x01);

    // left limiter: 1200ms/dB release rate | 30us/dB attack rate, limiter disabled though
    ssm3582a_register_write_byte(LIM_LEFT_CTRL1, 0xA0);
    // left limiter: 11v peak limiter treshold i tihnk, 2:1 ratio of threshold
    ssm3582a_register_write_byte(LIM_LEFT_CTRL2, 0x51);
    // left limiter: Battery Voltage Inflection Point at reset value.
    ssm3582a_register_write_byte(LIM_LEFT_CTRL3, 0x22);

    // same attack and release rate, but 0x08 links both channels to the left limiter, still limiters are disabled
    ssm3582a_register_write_byte(LIM_RIGHT_CTRL1, 0xA8);
    ssm3582a_register_write_byte(LIM_RIGHT_CTRL2, 0x51);
    ssm3582a_register_write_byte(LIM_RIGHT_CTRL3, 0x22);

    // DAC high frequency clip value: "clip to 250/256" whatever that means
    ssm3582a_register_write_byte(CLIP_LEFT_CTRL, 0xF9);
    ssm3582a_register_write_byte(CLIP_RIGHT_CTRL, 0xF9);

    // no overtemperature warming gain reduction
    ssm3582a_register_write_byte(FAULT_CTRL1, 0x00);

    // if fault happens, device will try permanently to recover
    ssm3582a_register_write_byte(FAULT_CTRL2, 0x30);

    // if this is set to 1, device will reset
    ssm3582a_register_write_byte(SOFT_RESET, 0x00);
}

esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}