#include "rtc_io.h"

portMUX_TYPE rtc_spinlock = portMUX_INITIALIZER_UNLOCKED;

//Reg,Mux,Fun,IE,Up,Down,Rtc_number
const rtc_gpio_desc_t rtc_gpio_desc[GPIO_PIN_COUNT] = {
    {RTC_IO_TOUCH_PAD1_REG, RTC_IO_TOUCH_PAD1_MUX_SEL_M, RTC_IO_TOUCH_PAD1_FUN_SEL_S, RTC_IO_TOUCH_PAD1_FUN_IE_M, RTC_IO_TOUCH_PAD1_RUE_M, RTC_IO_TOUCH_PAD1_RDE_M, RTC_IO_TOUCH_PAD1_SLP_SEL_M, RTC_IO_TOUCH_PAD1_SLP_IE_M, RTC_IO_TOUCH_PAD1_HOLD_M, RTC_CNTL_TOUCH_PAD1_HOLD_FORCE_M, RTC_IO_TOUCH_PAD1_DRV_V, RTC_IO_TOUCH_PAD1_DRV_S, RTCIO_GPIO0_CHANNEL}, //0
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //1
    {RTC_IO_TOUCH_PAD2_REG, RTC_IO_TOUCH_PAD2_MUX_SEL_M, RTC_IO_TOUCH_PAD2_FUN_SEL_S, RTC_IO_TOUCH_PAD2_FUN_IE_M, RTC_IO_TOUCH_PAD2_RUE_M, RTC_IO_TOUCH_PAD2_RDE_M, RTC_IO_TOUCH_PAD2_SLP_SEL_M, RTC_IO_TOUCH_PAD2_SLP_IE_M, RTC_IO_TOUCH_PAD2_HOLD_M, RTC_CNTL_TOUCH_PAD2_HOLD_FORCE_M, RTC_IO_TOUCH_PAD2_DRV_V, RTC_IO_TOUCH_PAD2_DRV_S, RTCIO_GPIO2_CHANNEL}, //2
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //3
    {RTC_IO_TOUCH_PAD0_REG, RTC_IO_TOUCH_PAD0_MUX_SEL_M, RTC_IO_TOUCH_PAD0_FUN_SEL_S, RTC_IO_TOUCH_PAD0_FUN_IE_M, RTC_IO_TOUCH_PAD0_RUE_M, RTC_IO_TOUCH_PAD0_RDE_M, RTC_IO_TOUCH_PAD0_SLP_SEL_M, RTC_IO_TOUCH_PAD0_SLP_IE_M, RTC_IO_TOUCH_PAD0_HOLD_M,  RTC_CNTL_TOUCH_PAD0_HOLD_FORCE_M, RTC_IO_TOUCH_PAD0_DRV_V, RTC_IO_TOUCH_PAD0_DRV_S, RTCIO_GPIO4_CHANNEL}, //4
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //5
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //6
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //7
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //8
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //9
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //10
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //11
    {RTC_IO_TOUCH_PAD5_REG, RTC_IO_TOUCH_PAD5_MUX_SEL_M, RTC_IO_TOUCH_PAD5_FUN_SEL_S, RTC_IO_TOUCH_PAD5_FUN_IE_M, RTC_IO_TOUCH_PAD5_RUE_M, RTC_IO_TOUCH_PAD5_RDE_M, RTC_IO_TOUCH_PAD5_SLP_SEL_M, RTC_IO_TOUCH_PAD5_SLP_IE_M, RTC_IO_TOUCH_PAD5_HOLD_M, RTC_CNTL_TOUCH_PAD5_HOLD_FORCE_M, RTC_IO_TOUCH_PAD5_DRV_V, RTC_IO_TOUCH_PAD5_DRV_S, RTCIO_GPIO12_CHANNEL}, //12
    {RTC_IO_TOUCH_PAD4_REG, RTC_IO_TOUCH_PAD4_MUX_SEL_M, RTC_IO_TOUCH_PAD4_FUN_SEL_S, RTC_IO_TOUCH_PAD4_FUN_IE_M, RTC_IO_TOUCH_PAD4_RUE_M, RTC_IO_TOUCH_PAD4_RDE_M, RTC_IO_TOUCH_PAD4_SLP_SEL_M, RTC_IO_TOUCH_PAD4_SLP_IE_M, RTC_IO_TOUCH_PAD4_HOLD_M, RTC_CNTL_TOUCH_PAD4_HOLD_FORCE_M, RTC_IO_TOUCH_PAD4_DRV_V, RTC_IO_TOUCH_PAD4_DRV_S, RTCIO_GPIO13_CHANNEL}, //13
    {RTC_IO_TOUCH_PAD6_REG, RTC_IO_TOUCH_PAD6_MUX_SEL_M, RTC_IO_TOUCH_PAD6_FUN_SEL_S, RTC_IO_TOUCH_PAD6_FUN_IE_M, RTC_IO_TOUCH_PAD6_RUE_M, RTC_IO_TOUCH_PAD6_RDE_M, RTC_IO_TOUCH_PAD6_SLP_SEL_M, RTC_IO_TOUCH_PAD6_SLP_IE_M, RTC_IO_TOUCH_PAD6_HOLD_M, RTC_CNTL_TOUCH_PAD6_HOLD_FORCE_M, RTC_IO_TOUCH_PAD6_DRV_V, RTC_IO_TOUCH_PAD6_DRV_S, RTCIO_GPIO14_CHANNEL}, //14
    {RTC_IO_TOUCH_PAD3_REG, RTC_IO_TOUCH_PAD3_MUX_SEL_M, RTC_IO_TOUCH_PAD3_FUN_SEL_S, RTC_IO_TOUCH_PAD3_FUN_IE_M, RTC_IO_TOUCH_PAD3_RUE_M, RTC_IO_TOUCH_PAD3_RDE_M, RTC_IO_TOUCH_PAD3_SLP_SEL_M, RTC_IO_TOUCH_PAD3_SLP_IE_M, RTC_IO_TOUCH_PAD3_HOLD_M, RTC_CNTL_TOUCH_PAD3_HOLD_FORCE_M, RTC_IO_TOUCH_PAD3_DRV_V, RTC_IO_TOUCH_PAD3_DRV_S, RTCIO_GPIO15_CHANNEL}, //15
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //16
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //17
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //18
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //19
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //20
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //21
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //22
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //23
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //24
    {RTC_IO_PAD_DAC1_REG, RTC_IO_PDAC1_MUX_SEL_M, RTC_IO_PDAC1_FUN_SEL_S, RTC_IO_PDAC1_FUN_IE_M, RTC_IO_PDAC1_RUE_M, RTC_IO_PDAC1_RDE_M, RTC_IO_PDAC1_SLP_SEL_M, RTC_IO_PDAC1_SLP_IE_M, RTC_IO_PDAC1_HOLD_M, RTC_CNTL_PDAC1_HOLD_FORCE_M, RTC_IO_PDAC1_DRV_V, RTC_IO_PDAC1_DRV_S, RTCIO_GPIO25_CHANNEL},                           //25
    {RTC_IO_PAD_DAC2_REG, RTC_IO_PDAC2_MUX_SEL_M, RTC_IO_PDAC2_FUN_SEL_S, RTC_IO_PDAC2_FUN_IE_M, RTC_IO_PDAC2_RUE_M, RTC_IO_PDAC2_RDE_M, RTC_IO_PDAC2_SLP_SEL_M, RTC_IO_PDAC2_SLP_IE_M, RTC_IO_PDAC2_HOLD_M, RTC_CNTL_PDAC2_HOLD_FORCE_M, RTC_IO_PDAC2_DRV_V, RTC_IO_PDAC2_DRV_S, RTCIO_GPIO26_CHANNEL},                           //26
    {RTC_IO_TOUCH_PAD7_REG, RTC_IO_TOUCH_PAD7_MUX_SEL_M, RTC_IO_TOUCH_PAD7_FUN_SEL_S, RTC_IO_TOUCH_PAD7_FUN_IE_M, RTC_IO_TOUCH_PAD7_RUE_M, RTC_IO_TOUCH_PAD7_RDE_M, RTC_IO_TOUCH_PAD7_SLP_SEL_M, RTC_IO_TOUCH_PAD7_SLP_IE_M, RTC_IO_TOUCH_PAD7_HOLD_M, RTC_CNTL_TOUCH_PAD7_HOLD_FORCE_M, RTC_IO_TOUCH_PAD7_DRV_V, RTC_IO_TOUCH_PAD7_DRV_S, RTCIO_GPIO27_CHANNEL}, //27
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //28
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //29
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //30
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //31
    {RTC_IO_XTAL_32K_PAD_REG, RTC_IO_X32P_MUX_SEL_M, RTC_IO_X32P_FUN_SEL_S, RTC_IO_X32P_FUN_IE_M, RTC_IO_X32P_RUE_M, RTC_IO_X32P_RDE_M, RTC_IO_X32P_SLP_SEL_M, RTC_IO_X32P_SLP_IE_M, RTC_IO_X32P_HOLD_M, RTC_CNTL_X32P_HOLD_FORCE_M, RTC_IO_X32P_DRV_V, RTC_IO_X32P_DRV_S, RTCIO_GPIO32_CHANNEL},                            //32
    {RTC_IO_XTAL_32K_PAD_REG, RTC_IO_X32N_MUX_SEL_M, RTC_IO_X32N_FUN_SEL_S, RTC_IO_X32N_FUN_IE_M, RTC_IO_X32N_RUE_M, RTC_IO_X32N_RDE_M, RTC_IO_X32N_SLP_SEL_M, RTC_IO_X32N_SLP_IE_M, RTC_IO_X32N_HOLD_M, RTC_CNTL_X32N_HOLD_FORCE_M, RTC_IO_X32N_DRV_V, RTC_IO_X32N_DRV_S, RTCIO_GPIO33_CHANNEL},                            //33
    {RTC_IO_ADC_PAD_REG, RTC_IO_ADC1_MUX_SEL_M, RTC_IO_ADC1_FUN_SEL_S, RTC_IO_ADC1_FUN_IE_M, 0, 0, RTC_IO_ADC1_SLP_SEL_M, RTC_IO_ADC1_SLP_IE_M, RTC_IO_ADC1_HOLD_M, RTC_CNTL_ADC1_HOLD_FORCE_M, 0, 0, RTCIO_GPIO34_CHANNEL},                                                                //34
    {RTC_IO_ADC_PAD_REG, RTC_IO_ADC2_MUX_SEL_M, RTC_IO_ADC2_FUN_SEL_S, RTC_IO_ADC2_FUN_IE_M, 0, 0, RTC_IO_ADC2_SLP_SEL_M, RTC_IO_ADC2_SLP_IE_M, RTC_IO_ADC1_HOLD_M, RTC_CNTL_ADC2_HOLD_FORCE_M, 0, 0, RTCIO_GPIO35_CHANNEL},                                                                //35
    {RTC_IO_SENSOR_PADS_REG, RTC_IO_SENSE1_MUX_SEL_M, RTC_IO_SENSE1_FUN_SEL_S, RTC_IO_SENSE1_FUN_IE_M, 0, 0, RTC_IO_SENSE1_SLP_SEL_M, RTC_IO_SENSE1_SLP_IE_M, RTC_IO_SENSE1_HOLD_M, RTC_CNTL_SENSE1_HOLD_FORCE_M, 0, 0, RTCIO_GPIO36_CHANNEL},                                                      //36
    {RTC_IO_SENSOR_PADS_REG, RTC_IO_SENSE2_MUX_SEL_M, RTC_IO_SENSE2_FUN_SEL_S, RTC_IO_SENSE2_FUN_IE_M, 0, 0, RTC_IO_SENSE2_SLP_SEL_M, RTC_IO_SENSE2_SLP_IE_M, RTC_IO_SENSE1_HOLD_M, RTC_CNTL_SENSE2_HOLD_FORCE_M, 0, 0, RTCIO_GPIO37_CHANNEL},                                                      //37
    {RTC_IO_SENSOR_PADS_REG, RTC_IO_SENSE3_MUX_SEL_M, RTC_IO_SENSE3_FUN_SEL_S, RTC_IO_SENSE3_FUN_IE_M, 0, 0, RTC_IO_SENSE3_SLP_SEL_M, RTC_IO_SENSE3_SLP_IE_M, RTC_IO_SENSE1_HOLD_M, RTC_CNTL_SENSE3_HOLD_FORCE_M, 0, 0, RTCIO_GPIO38_CHANNEL},                                                       //38
    {RTC_IO_SENSOR_PADS_REG, RTC_IO_SENSE4_MUX_SEL_M, RTC_IO_SENSE4_FUN_SEL_S, RTC_IO_SENSE4_FUN_IE_M, 0, 0, RTC_IO_SENSE4_SLP_SEL_M, RTC_IO_SENSE4_SLP_IE_M, RTC_IO_SENSE1_HOLD_M, RTC_CNTL_SENSE4_HOLD_FORCE_M, 0, 0, RTCIO_GPIO39_CHANNEL},                                                      //39
};

void adc_power_always_on()
{
    portENTER_CRITICAL(&rtc_spinlock);
    SENS.sar_meas_wait2.force_xpd_sar = SENS_FORCE_XPD_SAR_PU;
    portEXIT_CRITICAL(&rtc_spinlock);
}

void adc_power_on()
{
    portENTER_CRITICAL(&rtc_spinlock);
    if (SENS.sar_meas_wait2.force_xpd_sar & SENS_FORCE_XPD_SAR_SW_M) {
        SENS.sar_meas_wait2.force_xpd_sar = SENS_FORCE_XPD_SAR_PU;
    } else {
        SENS.sar_meas_wait2.force_xpd_sar = SENS_FORCE_XPD_SAR_FSM;
    }
    portEXIT_CRITICAL(&rtc_spinlock);
}

void adc_power_off()
{
    portENTER_CRITICAL(&rtc_spinlock);
    //Bit1  0:Fsm  1: SW mode
    //Bit0  0:SW mode power down  1: SW mode power on
    SENS.sar_meas_wait2.force_xpd_sar = SENS_FORCE_XPD_SAR_PD;
    portEXIT_CRITICAL(&rtc_spinlock);
}




/*---------------------------------------------------------------
                        INTERRUPT HANDLER
---------------------------------------------------------------*/

typedef struct rtc_isr_handler_ {
    uint32_t mask;
    intr_handler_t handler;
    void* handler_arg;
    SLIST_ENTRY(rtc_isr_handler_) next;
} rtc_isr_handler_t;

static SLIST_HEAD(rtc_isr_handler_list_, rtc_isr_handler_) s_rtc_isr_handler_list =
        SLIST_HEAD_INITIALIZER(s_rtc_isr_handler_list);
portMUX_TYPE s_rtc_isr_handler_list_lock = portMUX_INITIALIZER_UNLOCKED;
static intr_handle_t s_rtc_isr_handle;

static void rtc_isr(void* arg)
{
    uint32_t status = REG_READ(RTC_CNTL_INT_ST_REG);
    rtc_isr_handler_t* it;
    portENTER_CRITICAL(&s_rtc_isr_handler_list_lock);
    SLIST_FOREACH(it, &s_rtc_isr_handler_list, next) {
        if (it->mask & status) {
            portEXIT_CRITICAL(&s_rtc_isr_handler_list_lock);
            (*it->handler)(it->handler_arg);
            portENTER_CRITICAL(&s_rtc_isr_handler_list_lock);
        }
    }
    portEXIT_CRITICAL(&s_rtc_isr_handler_list_lock);
    REG_WRITE(RTC_CNTL_INT_CLR_REG, status);
}

static esp_err_t rtc_isr_ensure_installed()
{
    esp_err_t err = ESP_OK;
    portENTER_CRITICAL(&s_rtc_isr_handler_list_lock);
    if (s_rtc_isr_handle) {
        goto out;
    }

    REG_WRITE(RTC_CNTL_INT_ENA_REG, 0);
    REG_WRITE(RTC_CNTL_INT_CLR_REG, UINT32_MAX);
    err = esp_intr_alloc(ETS_RTC_CORE_INTR_SOURCE, 0, &rtc_isr, NULL, &s_rtc_isr_handle);
    if (err != ESP_OK) {
        goto out;
    }

out:
    portEXIT_CRITICAL(&s_rtc_isr_handler_list_lock);
    return err;
}


esp_err_t rtc_isr_register(intr_handler_t handler, void* handler_arg, uint32_t rtc_intr_mask)
{
    esp_err_t err = rtc_isr_ensure_installed();
    if (err != ESP_OK) {
        return err;
    }

    rtc_isr_handler_t* item = malloc(sizeof(*item));
    if (item == NULL) {
        return ESP_ERR_NO_MEM;
    }
    item->handler = handler;
    item->handler_arg = handler_arg;
    item->mask = rtc_intr_mask;
    portENTER_CRITICAL(&s_rtc_isr_handler_list_lock);
    SLIST_INSERT_HEAD(&s_rtc_isr_handler_list, item, next);
    portEXIT_CRITICAL(&s_rtc_isr_handler_list_lock);
    return ESP_OK;
}

esp_err_t rtc_isr_deregister(intr_handler_t handler, void* handler_arg)
{
    rtc_isr_handler_t* it;
    rtc_isr_handler_t* prev = NULL;
    bool found = false;
    portENTER_CRITICAL(&s_rtc_isr_handler_list_lock);
    SLIST_FOREACH(it, &s_rtc_isr_handler_list, next) {
        if (it->handler == handler && it->handler_arg == handler_arg) {
            if (it == SLIST_FIRST(&s_rtc_isr_handler_list)) {
                SLIST_REMOVE_HEAD(&s_rtc_isr_handler_list, next);
            } else {
                SLIST_REMOVE_AFTER(prev, next);
            }
            found = true;
            break;
        }
        prev = it;
    }
    portEXIT_CRITICAL(&s_rtc_isr_handler_list_lock);
    return found ? ESP_OK : ESP_ERR_INVALID_STATE;
}


/*---------------------------------------------------------------
                        RTC IO
---------------------------------------------------------------*/
esp_err_t rtc_gpio_init(gpio_num_t gpio_num)
{
    RTC_MODULE_CHECK(rtc_gpio_is_valid_gpio(gpio_num), "RTC_GPIO number error", ESP_ERR_INVALID_ARG);
    portENTER_CRITICAL(&rtc_spinlock);
    // 0: GPIO connected to digital GPIO module. 1: GPIO connected to analog RTC module.
    SET_PERI_REG_MASK(rtc_gpio_desc[gpio_num].reg, (rtc_gpio_desc[gpio_num].mux));
    //0:RTC FUNCIOTN 1,2,3:Reserved
    SET_PERI_REG_BITS(rtc_gpio_desc[gpio_num].reg, RTC_IO_TOUCH_PAD1_FUN_SEL_V, 0x0, rtc_gpio_desc[gpio_num].func);
    portEXIT_CRITICAL(&rtc_spinlock);

    return ESP_OK;
}

esp_err_t rtc_gpio_deinit(gpio_num_t gpio_num)
{
    RTC_MODULE_CHECK(rtc_gpio_is_valid_gpio(gpio_num), "RTC_GPIO number error", ESP_ERR_INVALID_ARG);
    portENTER_CRITICAL(&rtc_spinlock);
    //Select Gpio as Digital Gpio
    CLEAR_PERI_REG_MASK(rtc_gpio_desc[gpio_num].reg, (rtc_gpio_desc[gpio_num].mux));
    portEXIT_CRITICAL(&rtc_spinlock);

    return ESP_OK;
}

static esp_err_t rtc_gpio_output_enable(gpio_num_t gpio_num)
{
    int rtc_gpio_num = rtc_gpio_desc[gpio_num].rtc_num;
    RTC_MODULE_CHECK(rtc_gpio_num != -1, "RTC_GPIO number error", ESP_ERR_INVALID_ARG);
    SET_PERI_REG_MASK(RTC_GPIO_ENABLE_W1TS_REG, (1 << (rtc_gpio_num + RTC_GPIO_ENABLE_W1TS_S)));
    CLEAR_PERI_REG_MASK(RTC_GPIO_ENABLE_W1TC_REG, (1 << (rtc_gpio_num + RTC_GPIO_ENABLE_W1TC_S)));

    return ESP_OK;
}

static esp_err_t rtc_gpio_output_disable(gpio_num_t gpio_num)
{
    int rtc_gpio_num = rtc_gpio_desc[gpio_num].rtc_num;
    RTC_MODULE_CHECK(rtc_gpio_num != -1, "RTC_GPIO number error", ESP_ERR_INVALID_ARG);
    CLEAR_PERI_REG_MASK(RTC_GPIO_ENABLE_W1TS_REG, (1 << (rtc_gpio_num + RTC_GPIO_ENABLE_W1TS_S)));
    SET_PERI_REG_MASK(RTC_GPIO_ENABLE_W1TC_REG, (1 << ( rtc_gpio_num + RTC_GPIO_ENABLE_W1TC_S)));

    return ESP_OK;
}

static esp_err_t rtc_gpio_input_enable(gpio_num_t gpio_num)
{
    RTC_MODULE_CHECK(rtc_gpio_is_valid_gpio(gpio_num), "RTC_GPIO number error", ESP_ERR_INVALID_ARG);
    portENTER_CRITICAL(&rtc_spinlock);
    SET_PERI_REG_MASK(rtc_gpio_desc[gpio_num].reg, rtc_gpio_desc[gpio_num].ie);
    portEXIT_CRITICAL(&rtc_spinlock);

    return ESP_OK;
}

static esp_err_t rtc_gpio_input_disable(gpio_num_t gpio_num)
{
    RTC_MODULE_CHECK(rtc_gpio_is_valid_gpio(gpio_num), "RTC_GPIO number error", ESP_ERR_INVALID_ARG);
    portENTER_CRITICAL(&rtc_spinlock);
    CLEAR_PERI_REG_MASK(rtc_gpio_desc[gpio_num].reg, rtc_gpio_desc[gpio_num].ie);
    portEXIT_CRITICAL(&rtc_spinlock);

    return ESP_OK;
}

esp_err_t rtc_gpio_set_level(gpio_num_t gpio_num, uint32_t level)
{
    int rtc_gpio_num = rtc_gpio_num = rtc_gpio_desc[gpio_num].rtc_num;;
    RTC_MODULE_CHECK(rtc_gpio_is_valid_gpio(gpio_num), "RTC_GPIO number error", ESP_ERR_INVALID_ARG);

    if (level) {
        WRITE_PERI_REG(RTC_GPIO_OUT_W1TS_REG, (1 << (rtc_gpio_num + RTC_GPIO_OUT_DATA_W1TS_S)));
    } else {
        WRITE_PERI_REG(RTC_GPIO_OUT_W1TC_REG, (1 << (rtc_gpio_num + RTC_GPIO_OUT_DATA_W1TC_S)));
    }

    return ESP_OK;
}

uint32_t rtc_gpio_get_level(gpio_num_t gpio_num)
{
    uint32_t level = 0;
    int rtc_gpio_num = rtc_gpio_desc[gpio_num].rtc_num;
    RTC_MODULE_CHECK(rtc_gpio_is_valid_gpio(gpio_num), "RTC_GPIO number error", ESP_ERR_INVALID_ARG);

    portENTER_CRITICAL(&rtc_spinlock);
    level = READ_PERI_REG(RTC_GPIO_IN_REG);
    portEXIT_CRITICAL(&rtc_spinlock);
    return ((level >> (RTC_GPIO_IN_NEXT_S + rtc_gpio_num)) & 0x01);
}

esp_err_t rtc_gpio_set_drive_capability(gpio_num_t gpio_num, gpio_drive_cap_t strength)
{
    RTC_MODULE_CHECK(rtc_gpio_is_valid_gpio(gpio_num), "RTC_GPIO number error", ESP_ERR_INVALID_ARG);
    RTC_MODULE_CHECK(GPIO_IS_VALID_OUTPUT_GPIO(gpio_num), "Output pad only", ESP_ERR_INVALID_ARG);
    RTC_MODULE_CHECK(strength < GPIO_DRIVE_CAP_MAX, "GPIO drive capability error", ESP_ERR_INVALID_ARG);

    portENTER_CRITICAL(&rtc_spinlock);
    SET_PERI_REG_BITS(rtc_gpio_desc[gpio_num].reg, rtc_gpio_desc[gpio_num].drv_v, strength, rtc_gpio_desc[gpio_num].drv_s);
    portEXIT_CRITICAL(&rtc_spinlock);
    return ESP_OK;
}

esp_err_t rtc_gpio_get_drive_capability(gpio_num_t gpio_num, gpio_drive_cap_t* strength)
{
    RTC_MODULE_CHECK(rtc_gpio_is_valid_gpio(gpio_num), "RTC_GPIO number error", ESP_ERR_INVALID_ARG);
    RTC_MODULE_CHECK(GPIO_IS_VALID_OUTPUT_GPIO(gpio_num), "Output pad only", ESP_ERR_INVALID_ARG);
    RTC_MODULE_CHECK(strength != NULL, "GPIO drive pointer error", ESP_ERR_INVALID_ARG);

    *strength = GET_PERI_REG_BITS2(rtc_gpio_desc[gpio_num].reg, rtc_gpio_desc[gpio_num].drv_v, rtc_gpio_desc[gpio_num].drv_s);
    return ESP_OK;
}

esp_err_t rtc_gpio_set_direction(gpio_num_t gpio_num, rtc_gpio_mode_t mode)
{
    RTC_MODULE_CHECK(rtc_gpio_is_valid_gpio(gpio_num), "RTC_GPIO number error", ESP_ERR_INVALID_ARG);

    switch (mode) {
    case RTC_GPIO_MODE_INPUT_ONLY:
        rtc_gpio_output_disable(gpio_num);
        rtc_gpio_input_enable(gpio_num);
        break;
    case RTC_GPIO_MODE_OUTPUT_ONLY:
        rtc_gpio_output_enable(gpio_num);
        rtc_gpio_input_disable(gpio_num);
        break;
    case RTC_GPIO_MODE_INPUT_OUTPUT:
        rtc_gpio_output_enable(gpio_num);
        rtc_gpio_input_enable(gpio_num);
        break;
    case RTC_GPIO_MODE_DISABLED:
        rtc_gpio_output_disable(gpio_num);
        rtc_gpio_input_disable(gpio_num);
        break;
    }

    return ESP_OK;
}

esp_err_t rtc_gpio_pullup_en(gpio_num_t gpio_num)
{
    //this is a digital pad
    if (rtc_gpio_desc[gpio_num].pullup == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    //this is a rtc pad
    portENTER_CRITICAL(&rtc_spinlock);
    SET_PERI_REG_MASK(rtc_gpio_desc[gpio_num].reg, rtc_gpio_desc[gpio_num].pullup);
    portEXIT_CRITICAL(&rtc_spinlock);

    return ESP_OK;
}

esp_err_t rtc_gpio_pulldown_en(gpio_num_t gpio_num)
{
    //this is a digital pad
    if (rtc_gpio_desc[gpio_num].pulldown == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    //this is a rtc pad
    portENTER_CRITICAL(&rtc_spinlock);
    SET_PERI_REG_MASK(rtc_gpio_desc[gpio_num].reg, rtc_gpio_desc[gpio_num].pulldown);
    portEXIT_CRITICAL(&rtc_spinlock);

    return ESP_OK;
}

esp_err_t rtc_gpio_pullup_dis(gpio_num_t gpio_num)
{
    //this is a digital pad
    if ( rtc_gpio_desc[gpio_num].pullup == 0 ) {
        return ESP_ERR_INVALID_ARG;
    }

    //this is a rtc pad
    portENTER_CRITICAL(&rtc_spinlock);
    CLEAR_PERI_REG_MASK(rtc_gpio_desc[gpio_num].reg, rtc_gpio_desc[gpio_num].pullup);
    portEXIT_CRITICAL(&rtc_spinlock);

    return ESP_OK;
}

esp_err_t rtc_gpio_pulldown_dis(gpio_num_t gpio_num)
{
    //this is a digital pad
    if (rtc_gpio_desc[gpio_num].pulldown == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    //this is a rtc pad
    portENTER_CRITICAL(&rtc_spinlock);
    CLEAR_PERI_REG_MASK(rtc_gpio_desc[gpio_num].reg, rtc_gpio_desc[gpio_num].pulldown);
    portEXIT_CRITICAL(&rtc_spinlock);

    return ESP_OK;
}

esp_err_t rtc_gpio_hold_en(gpio_num_t gpio_num)
{
    // check if an RTC IO
    if (rtc_gpio_desc[gpio_num].pullup == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&rtc_spinlock);
    SET_PERI_REG_MASK(rtc_gpio_desc[gpio_num].reg, rtc_gpio_desc[gpio_num].hold);
    portEXIT_CRITICAL(&rtc_spinlock);
    return ESP_OK;
}

esp_err_t rtc_gpio_hold_dis(gpio_num_t gpio_num)
{
    // check if an RTC IO
    if (rtc_gpio_desc[gpio_num].pullup == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&rtc_spinlock);
    CLEAR_PERI_REG_MASK(rtc_gpio_desc[gpio_num].reg, rtc_gpio_desc[gpio_num].hold);
    portEXIT_CRITICAL(&rtc_spinlock);
    return ESP_OK;
}


void rtc_gpio_force_hold_dis_all()
{
    for (int gpio = 0; gpio < GPIO_PIN_COUNT; ++gpio) {
        const rtc_gpio_desc_t* desc = &rtc_gpio_desc[gpio];
        if (desc->hold_force != 0) {
            REG_CLR_BIT(RTC_CNTL_HOLD_FORCE_REG, desc->hold_force);
        }
    }
}

#include "rtc_io.h"
