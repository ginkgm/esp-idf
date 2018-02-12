#include "dac.h"

#define DAC_ERR_STR_CHANNEL_ERROR   "DAC channel error"

esp_err_t dac_pad_get_io_num(dac_channel_t channel, gpio_num_t *gpio_num)
{
    RTC_MODULE_CHECK((channel >= DAC_CHANNEL_1) && (channel < DAC_CHANNEL_MAX), DAC_ERR_STR_CHANNEL_ERROR, ESP_ERR_INVALID_ARG);
    RTC_MODULE_CHECK(gpio_num, "Param null", ESP_ERR_INVALID_ARG);

    switch (channel) {
    case DAC_CHANNEL_1:
        *gpio_num = DAC_CHANNEL_1_GPIO_NUM;
        break;
    case DAC_CHANNEL_2:
        *gpio_num = DAC_CHANNEL_2_GPIO_NUM;
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

static esp_err_t dac_rtc_pad_init(dac_channel_t channel)
{
    RTC_MODULE_CHECK((channel >= DAC_CHANNEL_1) && (channel < DAC_CHANNEL_MAX), DAC_ERR_STR_CHANNEL_ERROR, ESP_ERR_INVALID_ARG);
    gpio_num_t gpio_num = 0;
    dac_pad_get_io_num(channel, &gpio_num);
    rtc_gpio_init(gpio_num);
    rtc_gpio_output_disable(gpio_num);
    rtc_gpio_input_disable(gpio_num);
    rtc_gpio_pullup_dis(gpio_num);
    rtc_gpio_pulldown_dis(gpio_num);

    return ESP_OK;
}

esp_err_t dac_output_enable(dac_channel_t channel)
{
    RTC_MODULE_CHECK((channel >= DAC_CHANNEL_1) && (channel < DAC_CHANNEL_MAX), DAC_ERR_STR_CHANNEL_ERROR, ESP_ERR_INVALID_ARG);
    dac_rtc_pad_init(channel);
    portENTER_CRITICAL(&rtc_spinlock);
    if (channel == DAC_CHANNEL_1) {
        SET_PERI_REG_MASK(RTC_IO_PAD_DAC1_REG, RTC_IO_PDAC1_XPD_DAC | RTC_IO_PDAC1_DAC_XPD_FORCE);
    } else if (channel == DAC_CHANNEL_2) {
        SET_PERI_REG_MASK(RTC_IO_PAD_DAC2_REG, RTC_IO_PDAC2_XPD_DAC | RTC_IO_PDAC2_DAC_XPD_FORCE);
    }
    portEXIT_CRITICAL(&rtc_spinlock);

    return ESP_OK;
}

esp_err_t dac_output_disable(dac_channel_t channel)
{
    RTC_MODULE_CHECK((channel >= DAC_CHANNEL_1) && (channel < DAC_CHANNEL_MAX), DAC_ERR_STR_CHANNEL_ERROR, ESP_ERR_INVALID_ARG);
    portENTER_CRITICAL(&rtc_spinlock);
    if (channel == DAC_CHANNEL_1) {
        CLEAR_PERI_REG_MASK(RTC_IO_PAD_DAC1_REG, RTC_IO_PDAC1_XPD_DAC | RTC_IO_PDAC1_DAC_XPD_FORCE);
    } else if (channel == DAC_CHANNEL_2) {
        CLEAR_PERI_REG_MASK(RTC_IO_PAD_DAC2_REG, RTC_IO_PDAC2_XPD_DAC | RTC_IO_PDAC2_DAC_XPD_FORCE);
    }
    portEXIT_CRITICAL(&rtc_spinlock);

    return ESP_OK;
}

esp_err_t dac_output_voltage(dac_channel_t channel, uint8_t dac_value)
{
    RTC_MODULE_CHECK((channel >= DAC_CHANNEL_1) && (channel < DAC_CHANNEL_MAX), DAC_ERR_STR_CHANNEL_ERROR, ESP_ERR_INVALID_ARG);
    portENTER_CRITICAL(&rtc_spinlock);
    //Disable Tone
    CLEAR_PERI_REG_MASK(SENS_SAR_DAC_CTRL1_REG, SENS_SW_TONE_EN);

    //Disable Channel Tone
    if (channel == DAC_CHANNEL_1) {
        CLEAR_PERI_REG_MASK(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_CW_EN1_M);
    } else if (channel == DAC_CHANNEL_2) {
        CLEAR_PERI_REG_MASK(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_CW_EN2_M);
    }

    //Set the Dac value
    if (channel == DAC_CHANNEL_1) {
        SET_PERI_REG_BITS(RTC_IO_PAD_DAC1_REG, RTC_IO_PDAC1_DAC, dac_value, RTC_IO_PDAC1_DAC_S);   //dac_output
    } else if (channel == DAC_CHANNEL_2) {
        SET_PERI_REG_BITS(RTC_IO_PAD_DAC2_REG, RTC_IO_PDAC2_DAC, dac_value, RTC_IO_PDAC2_DAC_S);   //dac_output
    }

    portEXIT_CRITICAL(&rtc_spinlock);

    return ESP_OK;
}

esp_err_t dac_out_voltage(dac_channel_t channel, uint8_t dac_value)
{
    RTC_MODULE_CHECK((channel >= DAC_CHANNEL_1) && (channel < DAC_CHANNEL_MAX), DAC_ERR_STR_CHANNEL_ERROR, ESP_ERR_INVALID_ARG);
    portENTER_CRITICAL(&rtc_spinlock);
    //Disable Tone
    CLEAR_PERI_REG_MASK(SENS_SAR_DAC_CTRL1_REG, SENS_SW_TONE_EN);

    //Disable Channel Tone
    if (channel == DAC_CHANNEL_1) {
        CLEAR_PERI_REG_MASK(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_CW_EN1_M);
    } else if (channel == DAC_CHANNEL_2) {
        CLEAR_PERI_REG_MASK(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_CW_EN2_M);
    }

    //Set the Dac value
    if (channel == DAC_CHANNEL_1) {
        SET_PERI_REG_BITS(RTC_IO_PAD_DAC1_REG, RTC_IO_PDAC1_DAC, dac_value, RTC_IO_PDAC1_DAC_S);   //dac_output
    } else if (channel == DAC_CHANNEL_2) {
        SET_PERI_REG_BITS(RTC_IO_PAD_DAC2_REG, RTC_IO_PDAC2_DAC, dac_value, RTC_IO_PDAC2_DAC_S);   //dac_output
    }

    portEXIT_CRITICAL(&rtc_spinlock);
    //dac pad init
    dac_rtc_pad_init(channel);
    dac_output_enable(channel);

    return ESP_OK;
}

esp_err_t dac_i2s_enable()
{
    portENTER_CRITICAL(&rtc_spinlock);
    SET_PERI_REG_MASK(SENS_SAR_DAC_CTRL1_REG, SENS_DAC_DIG_FORCE_M | SENS_DAC_CLK_INV_M);
    portEXIT_CRITICAL(&rtc_spinlock);
    return ESP_OK;
}

esp_err_t dac_i2s_disable()
{
    portENTER_CRITICAL(&rtc_spinlock);
    CLEAR_PERI_REG_MASK(SENS_SAR_DAC_CTRL1_REG, SENS_DAC_DIG_FORCE_M | SENS_DAC_CLK_INV_M);
    portEXIT_CRITICAL(&rtc_spinlock);
    return ESP_OK;
}

