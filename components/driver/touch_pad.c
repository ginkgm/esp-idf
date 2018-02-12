#include "touch_pad.h"

#define TOUCH_PAD_FILTER_FACTOR_DEFAULT   (16)
#define TOUCH_PAD_SHIFT_DEFAULT           (4)

static SemaphoreHandle_t rtc_touch_mux = NULL;

typedef struct {
    TimerHandle_t timer;
    uint32_t filtered_val[TOUCH_PAD_MAX];
    uint32_t filter_period;
    uint32_t period;
    bool enable;
} touch_pad_filter_t;
static touch_pad_filter_t *s_touch_pad_filter = NULL;

//Some register bits of touch sensor 8 and 9 are mismatched, we need to swap the bits.
#define BITSWAP(data, n, m)   (((data >> n) &  0x1)  == ((data >> m) & 0x1) ? (data) : ((data) ^ ((0x1 <<n) | (0x1 << m))))
#define TOUCH_BITS_SWAP(v)  BITSWAP(v, TOUCH_PAD_NUM8, TOUCH_PAD_NUM9)

//Some registers of touch sensor 8 and 9 are mismatched, we need to swap register index
inline static touch_pad_t touch_pad_num_wrap(touch_pad_t touch_num)
{
    if (touch_num == TOUCH_PAD_NUM8) {
        return TOUCH_PAD_NUM9;
    } else if (touch_num == TOUCH_PAD_NUM9) {
        return TOUCH_PAD_NUM8;
    }
    return touch_num;
}

esp_err_t touch_pad_isr_handler_register(void (*fn)(void *), void *arg, int no_use, intr_handle_t *handle_no_use)
{
    RTC_MODULE_CHECK(fn, "Touch_Pad ISR null", ESP_ERR_INVALID_ARG);
    return rtc_isr_register(fn, arg, RTC_CNTL_TOUCH_INT_ST_M);
}

esp_err_t touch_pad_isr_register(intr_handler_t fn, void* arg)
{
    RTC_MODULE_CHECK(fn, "Touch_Pad ISR null", ESP_ERR_INVALID_ARG);
    return rtc_isr_register(fn, arg, RTC_CNTL_TOUCH_INT_ST_M);
}

esp_err_t touch_pad_isr_deregister(intr_handler_t fn, void *arg)
{
    return rtc_isr_deregister(fn, arg);
}

static esp_err_t touch_pad_get_io_num(touch_pad_t touch_num, gpio_num_t *gpio_num)
{
    switch (touch_num) {
    case TOUCH_PAD_NUM0:
        *gpio_num = TOUCH_PAD_NUM0_GPIO_NUM;
        break;
    case TOUCH_PAD_NUM1:
        *gpio_num = TOUCH_PAD_NUM1_GPIO_NUM;
        break;
    case TOUCH_PAD_NUM2:
        *gpio_num = TOUCH_PAD_NUM2_GPIO_NUM;
        break;
    case TOUCH_PAD_NUM3:
        *gpio_num = TOUCH_PAD_NUM3_GPIO_NUM;
        break;
    case TOUCH_PAD_NUM4:
        *gpio_num = TOUCH_PAD_NUM4_GPIO_NUM;
        break;
    case TOUCH_PAD_NUM5:
        *gpio_num = TOUCH_PAD_NUM5_GPIO_NUM;
        break;
    case TOUCH_PAD_NUM6:
        *gpio_num = TOUCH_PAD_NUM6_GPIO_NUM;
        break;
    case TOUCH_PAD_NUM7:
        *gpio_num = TOUCH_PAD_NUM7_GPIO_NUM;
        break;
    case TOUCH_PAD_NUM8:
        *gpio_num = TOUCH_PAD_NUM8_GPIO_NUM;
        break;
    case TOUCH_PAD_NUM9:
        *gpio_num = TOUCH_PAD_NUM9_GPIO_NUM;
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

static uint32_t _touch_filter_iir(uint32_t in_now, uint32_t out_last, uint32_t k)
{
    if (k == 0) {
        return in_now;
    } else {
        uint32_t out_now = (in_now + (k - 1) * out_last) / k;
        return out_now;
    }
}

static void touch_pad_filter_cb(void *arg)
{
    if (s_touch_pad_filter == NULL) {
        return;
    }
    uint16_t val;
    for (int i = 0; i < TOUCH_PAD_MAX; i++) {
        touch_pad_read(i, &val);
        s_touch_pad_filter->filtered_val[i] = s_touch_pad_filter->filtered_val[i] == 0 ? (val << TOUCH_PAD_SHIFT_DEFAULT) : s_touch_pad_filter->filtered_val[i];
        s_touch_pad_filter->filtered_val[i] = _touch_filter_iir((val << TOUCH_PAD_SHIFT_DEFAULT),
                s_touch_pad_filter->filtered_val[i], TOUCH_PAD_FILTER_FACTOR_DEFAULT);
    }
}

esp_err_t touch_pad_set_meas_time(uint16_t sleep_cycle, uint16_t meas_cycle)
{
    xSemaphoreTake(rtc_touch_mux, portMAX_DELAY);
    portENTER_CRITICAL(&rtc_spinlock);
    //touch sensor sleep cycle Time = sleep_cycle / RTC_SLOW_CLK( can be 150k or 32k depending on the options)
    SENS.sar_touch_ctrl2.touch_sleep_cycles = sleep_cycle;
    //touch sensor measure time= meas_cycle / 8Mhz
    SENS.sar_touch_ctrl1.touch_meas_delay = meas_cycle;
    portEXIT_CRITICAL(&rtc_spinlock);
    xSemaphoreGive(rtc_touch_mux);
    return ESP_OK;
}

esp_err_t touch_pad_get_meas_time(uint16_t *sleep_cycle, uint16_t *meas_cycle)
{
    portENTER_CRITICAL(&rtc_spinlock);
    if (sleep_cycle) {
        *sleep_cycle = SENS.sar_touch_ctrl2.touch_sleep_cycles;
    }
    if (meas_cycle) {
        *meas_cycle = SENS.sar_touch_ctrl1.touch_meas_delay;
    }
    portEXIT_CRITICAL(&rtc_spinlock);
    return ESP_OK;
}

esp_err_t touch_pad_set_voltage(touch_high_volt_t refh, touch_low_volt_t refl, touch_volt_atten_t atten)
{
    RTC_MODULE_CHECK(((refh < TOUCH_HVOLT_MAX) && (refh >= (int )TOUCH_HVOLT_KEEP)), "touch refh error",
            ESP_ERR_INVALID_ARG);
    RTC_MODULE_CHECK(((refl < TOUCH_LVOLT_MAX) && (refh >= (int )TOUCH_LVOLT_KEEP)), "touch refl error",
            ESP_ERR_INVALID_ARG);
    RTC_MODULE_CHECK(((atten < TOUCH_HVOLT_ATTEN_MAX) && (refh >= (int )TOUCH_HVOLT_ATTEN_KEEP)), "touch atten error",
            ESP_ERR_INVALID_ARG);

    portENTER_CRITICAL(&rtc_spinlock);
    if (refh > TOUCH_HVOLT_KEEP) {
        RTCIO.touch_cfg.drefh = refh;
    }
    if (refl > TOUCH_LVOLT_KEEP) {
        RTCIO.touch_cfg.drefl = refl;
    }
    if (atten > TOUCH_HVOLT_ATTEN_KEEP) {
        RTCIO.touch_cfg.drange = atten;
    }
    portEXIT_CRITICAL(&rtc_spinlock);
    return ESP_OK;
}

esp_err_t touch_pad_get_voltage(touch_high_volt_t *refh, touch_low_volt_t *refl, touch_volt_atten_t *atten)
{
    portENTER_CRITICAL(&rtc_spinlock);
    if (refh) {
        *refh = RTCIO.touch_cfg.drefh;
    }
    if (refl) {
        *refl = RTCIO.touch_cfg.drefl;
    }
    if (atten) {
        *atten = RTCIO.touch_cfg.drange;
    }
    portEXIT_CRITICAL(&rtc_spinlock);
    return ESP_OK;
}

esp_err_t touch_pad_set_cnt_mode(touch_pad_t touch_num, touch_cnt_slope_t slope, touch_tie_opt_t opt)
{
    RTC_MODULE_CHECK((slope < TOUCH_PAD_SLOPE_MAX), "touch slope error", ESP_ERR_INVALID_ARG);
    RTC_MODULE_CHECK((opt < TOUCH_PAD_TIE_OPT_MAX), "touch opt error", ESP_ERR_INVALID_ARG);
    
    touch_pad_t touch_pad_wrap = touch_pad_num_wrap(touch_num);
    portENTER_CRITICAL(&rtc_spinlock);
    RTCIO.touch_pad[touch_pad_wrap].tie_opt = opt;
    RTCIO.touch_pad[touch_num].dac = slope;
    portEXIT_CRITICAL(&rtc_spinlock);
    return ESP_OK;
}

esp_err_t touch_pad_get_cnt_mode(touch_pad_t touch_num, touch_cnt_slope_t *slope, touch_tie_opt_t *opt)
{
    RTC_MODULE_CHECK((touch_num < TOUCH_PAD_MAX), "touch IO error", ESP_ERR_INVALID_ARG);
    
    touch_pad_t touch_pad_wrap = touch_pad_num_wrap(touch_num);
    portENTER_CRITICAL(&rtc_spinlock);
    if(opt) {
        *opt = RTCIO.touch_pad[touch_pad_wrap].tie_opt;
    }
    if(slope) {
        *slope = RTCIO.touch_pad[touch_num].dac;
    }
    portEXIT_CRITICAL(&rtc_spinlock);
    return ESP_OK;
}

esp_err_t touch_pad_io_init(touch_pad_t touch_num)
{
    RTC_MODULE_CHECK((touch_num < TOUCH_PAD_MAX), "touch IO error", ESP_ERR_INVALID_ARG);
    gpio_num_t gpio_num = GPIO_NUM_0;
    touch_pad_get_io_num(touch_num, &gpio_num);
    rtc_gpio_init(gpio_num);
    rtc_gpio_set_direction(gpio_num, RTC_GPIO_MODE_DISABLED);
    rtc_gpio_pulldown_dis(gpio_num);
    rtc_gpio_pullup_dis(gpio_num);
    return ESP_OK;
}

esp_err_t touch_pad_set_fsm_mode(touch_fsm_mode_t mode)
{
    RTC_MODULE_CHECK((mode < TOUCH_FSM_MODE_MAX), "touch fsm mode error", ESP_ERR_INVALID_ARG);
    portENTER_CRITICAL(&rtc_spinlock);
    SENS.sar_touch_ctrl2.touch_start_en = 0;
    SENS.sar_touch_ctrl2.touch_start_force = mode;
    RTCCNTL.state0.touch_slp_timer_en = (mode == TOUCH_FSM_MODE_TIMER ? 1 : 0);
    portEXIT_CRITICAL(&rtc_spinlock);
    return ESP_OK;
}

esp_err_t touch_pad_get_fsm_mode(touch_fsm_mode_t *mode)
{
    if (mode) {
        *mode = SENS.sar_touch_ctrl2.touch_start_force;
    }
    return ESP_OK;
}

esp_err_t touch_pad_sw_start()
{
    portENTER_CRITICAL(&rtc_spinlock);
    SENS.sar_touch_ctrl2.touch_start_en = 0;
    SENS.sar_touch_ctrl2.touch_start_en = 1;
    portEXIT_CRITICAL(&rtc_spinlock);
    return ESP_OK;
}

esp_err_t touch_pad_set_thresh(touch_pad_t touch_num, uint16_t threshold)
{
    RTC_MODULE_CHECK((touch_num < TOUCH_PAD_MAX), "touch IO error", ESP_ERR_INVALID_ARG);
    touch_pad_t tp_wrap = touch_pad_num_wrap(touch_num);
    portENTER_CRITICAL(&rtc_spinlock);
    if (tp_wrap & 0x1) {
        SENS.touch_thresh[tp_wrap / 2].l_thresh = threshold;
    } else {
        SENS.touch_thresh[tp_wrap / 2].h_thresh = threshold;
    }
    portEXIT_CRITICAL(&rtc_spinlock);
    return ESP_OK;
}

esp_err_t touch_pad_get_thresh(touch_pad_t touch_num, uint16_t *threshold)
{
    RTC_MODULE_CHECK((touch_num < TOUCH_PAD_MAX), "touch IO error", ESP_ERR_INVALID_ARG);
    touch_pad_t tp_wrap = touch_pad_num_wrap(touch_num);
    if (threshold) {
        *threshold = (tp_wrap & 0x1 )? \
                SENS.touch_thresh[tp_wrap / 2].l_thresh : \
                SENS.touch_thresh[tp_wrap / 2].h_thresh;
    }
    return ESP_OK;
}

esp_err_t touch_pad_set_trigger_mode(touch_trigger_mode_t mode)
{
    RTC_MODULE_CHECK((mode < TOUCH_TRIGGER_MAX), "touch trigger mode error", ESP_ERR_INVALID_ARG);
    portENTER_CRITICAL(&rtc_spinlock);
    SENS.sar_touch_ctrl1.touch_out_sel = mode;
    portEXIT_CRITICAL(&rtc_spinlock);
    return ESP_OK;
}

esp_err_t touch_pad_get_trigger_mode(touch_trigger_mode_t *mode)
{
    if (mode) {
        *mode = SENS.sar_touch_ctrl1.touch_out_sel;
    }
    return ESP_OK;
}

esp_err_t touch_pad_set_trigger_source(touch_trigger_src_t src)
{
    RTC_MODULE_CHECK((src < TOUCH_TRIGGER_SOURCE_MAX), "touch trigger source error", ESP_ERR_INVALID_ARG);
    portENTER_CRITICAL(&rtc_spinlock);
    SENS.sar_touch_ctrl1.touch_out_1en = src;
    portEXIT_CRITICAL(&rtc_spinlock);
    return ESP_OK;
}

esp_err_t touch_pad_get_trigger_source(touch_trigger_src_t *src)
{
    if (src) {
        *src = SENS.sar_touch_ctrl1.touch_out_1en;
    }
    return ESP_OK;
}

esp_err_t touch_pad_set_group_mask(uint16_t set1_mask, uint16_t set2_mask, uint16_t en_mask)
{
    RTC_MODULE_CHECK((set1_mask <= TOUCH_PAD_BIT_MASK_MAX), "touch set1 bitmask error", ESP_ERR_INVALID_ARG);
    RTC_MODULE_CHECK((set2_mask <= TOUCH_PAD_BIT_MASK_MAX), "touch set2 bitmask error", ESP_ERR_INVALID_ARG);
    RTC_MODULE_CHECK((en_mask <= TOUCH_PAD_BIT_MASK_MAX), "touch work_en bitmask error", ESP_ERR_INVALID_ARG);

    portENTER_CRITICAL(&rtc_spinlock);
    SENS.sar_touch_enable.touch_pad_outen1 |= TOUCH_BITS_SWAP(set1_mask);
    SENS.sar_touch_enable.touch_pad_outen2 |= TOUCH_BITS_SWAP(set2_mask);
    SENS.sar_touch_enable.touch_pad_worken |= TOUCH_BITS_SWAP(en_mask);
    portEXIT_CRITICAL(&rtc_spinlock);

    return ESP_OK;
}

esp_err_t touch_pad_get_group_mask(uint16_t *set1_mask, uint16_t *set2_mask, uint16_t *en_mask)
{
    portENTER_CRITICAL(&rtc_spinlock);
    if (set1_mask) {
        *set1_mask = TOUCH_BITS_SWAP(SENS.sar_touch_enable.touch_pad_outen1);
    }
    if (set2_mask) {
        *set2_mask = TOUCH_BITS_SWAP(SENS.sar_touch_enable.touch_pad_outen2);
    }
    if (en_mask) {
        *en_mask = TOUCH_BITS_SWAP(SENS.sar_touch_enable.touch_pad_worken);
    }
    portEXIT_CRITICAL(&rtc_spinlock);

    return ESP_OK;
}

esp_err_t touch_pad_clear_group_mask(uint16_t set1_mask, uint16_t set2_mask, uint16_t en_mask)
{
    RTC_MODULE_CHECK((set1_mask <= TOUCH_PAD_BIT_MASK_MAX), "touch set1 bitmask error", ESP_ERR_INVALID_ARG);
    RTC_MODULE_CHECK((set2_mask <= TOUCH_PAD_BIT_MASK_MAX), "touch set2 bitmask error", ESP_ERR_INVALID_ARG);
    RTC_MODULE_CHECK((en_mask <= TOUCH_PAD_BIT_MASK_MAX), "touch work_en bitmask error", ESP_ERR_INVALID_ARG);

    portENTER_CRITICAL(&rtc_spinlock);
    SENS.sar_touch_enable.touch_pad_outen1 &= TOUCH_BITS_SWAP(~set1_mask);
    SENS.sar_touch_enable.touch_pad_outen2 &= TOUCH_BITS_SWAP(~set2_mask);
    SENS.sar_touch_enable.touch_pad_worken &= TOUCH_BITS_SWAP(~en_mask);
    portEXIT_CRITICAL(&rtc_spinlock);
    return ESP_OK;
}

uint32_t IRAM_ATTR touch_pad_get_status()
{
    uint32_t status = SENS.sar_touch_ctrl2.touch_meas_en;
    return TOUCH_BITS_SWAP(status);
}

esp_err_t IRAM_ATTR touch_pad_clear_status()
{
    portENTER_CRITICAL(&rtc_spinlock);
    SENS.sar_touch_ctrl2.touch_meas_en_clr = 1;
    portEXIT_CRITICAL(&rtc_spinlock);
    return ESP_OK;
}

esp_err_t touch_pad_intr_enable()
{
    portENTER_CRITICAL(&rtc_spinlock);
    RTCCNTL.int_ena.rtc_touch = 1;
    portEXIT_CRITICAL(&rtc_spinlock);
    return ESP_OK;
}

esp_err_t touch_pad_intr_disable()
{
    portENTER_CRITICAL(&rtc_spinlock);
    RTCCNTL.int_ena.rtc_touch = 0;
    portEXIT_CRITICAL(&rtc_spinlock);
    return ESP_OK;
}

esp_err_t touch_pad_config(touch_pad_t touch_num, uint16_t threshold)
{
    RTC_MODULE_CHECK(rtc_touch_mux != NULL, "Touch pad not initialized", ESP_FAIL);
    RTC_MODULE_CHECK(touch_num < TOUCH_PAD_MAX, "Touch_Pad Num Err", ESP_ERR_INVALID_ARG);
    touch_pad_set_thresh(touch_num, threshold);
    touch_pad_io_init(touch_num);
    touch_pad_set_cnt_mode(touch_num, TOUCH_PAD_SLOPE_7, TOUCH_PAD_TIE_OPT_HIGH);
    touch_pad_set_group_mask((1 << touch_num), (1 << touch_num), (1 << touch_num));
    return ESP_OK;
}

esp_err_t touch_pad_init()
{
    if (rtc_touch_mux == NULL) {
        rtc_touch_mux = xSemaphoreCreateMutex();
    }
    if (rtc_touch_mux == NULL) {
        return ESP_FAIL;
    }
    touch_pad_intr_disable();
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_DEFAULT);
    touch_pad_set_trigger_mode(TOUCH_TRIGGER_MODE_DEFAULT);
    touch_pad_set_trigger_source(TOUCH_TRIGGER_SOURCE_DEFAULT);
    touch_pad_clear_status();
    touch_pad_set_meas_time(TOUCH_PAD_SLEEP_CYCLE_DEFAULT, TOUCH_PAD_MEASURE_CYCLE_DEFAULT);
    return ESP_OK;
}

esp_err_t touch_pad_deinit()
{
    if (rtc_touch_mux == NULL) {
        return ESP_FAIL;
    }
    touch_pad_filter_delete();
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_SW);
    touch_pad_clear_status();
    touch_pad_intr_disable();
    vSemaphoreDelete(rtc_touch_mux);
    rtc_touch_mux = NULL;
    return ESP_OK;
}

esp_err_t touch_pad_read(touch_pad_t touch_num, uint16_t *touch_value)
{
    RTC_MODULE_CHECK(touch_num < TOUCH_PAD_MAX, "Touch_Pad Num Err", ESP_ERR_INVALID_ARG);
    RTC_MODULE_CHECK(touch_value != NULL, "touch_value", ESP_ERR_INVALID_ARG);
    RTC_MODULE_CHECK(rtc_touch_mux != NULL, "Touch pad not initialized", ESP_FAIL);

    touch_pad_t tp_wrap = touch_pad_num_wrap(touch_num);
    xSemaphoreTake(rtc_touch_mux, portMAX_DELAY);
    while (SENS.sar_touch_ctrl2.touch_meas_done == 0) {};
    *touch_value = (tp_wrap & 0x1) ? \
                        SENS.touch_meas[tp_wrap / 2].l_val: \
                        SENS.touch_meas[tp_wrap / 2].h_val;
    xSemaphoreGive(rtc_touch_mux);
    return ESP_OK;
}

IRAM_ATTR esp_err_t touch_pad_read_filtered(touch_pad_t touch_num, uint16_t *touch_value)
{
    RTC_MODULE_CHECK(rtc_touch_mux != NULL, "Touch pad not initialized", ESP_FAIL);
    RTC_MODULE_CHECK(touch_num < TOUCH_PAD_MAX, "Touch_Pad Num Err", ESP_ERR_INVALID_ARG);
    RTC_MODULE_CHECK(touch_value != NULL, "touch_value", ESP_ERR_INVALID_ARG);
    RTC_MODULE_CHECK(s_touch_pad_filter != NULL, "Touch pad filter not initialized", ESP_ERR_INVALID_STATE);
    *touch_value = (s_touch_pad_filter->filtered_val[touch_num] >> TOUCH_PAD_SHIFT_DEFAULT);
    return ESP_OK;
}

esp_err_t touch_pad_set_filter_period(uint32_t new_period_ms)
{
    RTC_MODULE_CHECK(s_touch_pad_filter != NULL, "Touch pad filter not initialized", ESP_ERR_INVALID_STATE);
    RTC_MODULE_CHECK(new_period_ms > 0, "Touch pad filter period error", ESP_ERR_INVALID_ARG);
    RTC_MODULE_CHECK(rtc_touch_mux != NULL, "Touch pad not initialized", ESP_ERR_INVALID_STATE);

    esp_err_t ret = ESP_OK;
    xSemaphoreTake(rtc_touch_mux, portMAX_DELAY);
    if (s_touch_pad_filter != NULL) {
        xTimerChangePeriod(s_touch_pad_filter->timer, new_period_ms / portTICK_PERIOD_MS, portMAX_DELAY);
        s_touch_pad_filter->period = new_period_ms;
    } else {
        ESP_LOGE(RTC_MODULE_TAG, "Touch pad filter deleted");
        ret = ESP_ERR_INVALID_STATE;
    }
    xSemaphoreGive(rtc_touch_mux);
    return ret;
}

esp_err_t touch_pad_get_filter_period(uint32_t* p_period_ms)
{
    RTC_MODULE_CHECK(s_touch_pad_filter != NULL, "Touch pad filter not initialized", ESP_ERR_INVALID_STATE);
    RTC_MODULE_CHECK(p_period_ms != NULL, "Touch pad period pointer error", ESP_ERR_INVALID_ARG);
    RTC_MODULE_CHECK(rtc_touch_mux != NULL, "Touch pad not initialized", ESP_ERR_INVALID_STATE);

    esp_err_t ret = ESP_OK;
    xSemaphoreTake(rtc_touch_mux, portMAX_DELAY);
    if (s_touch_pad_filter != NULL) {
        *p_period_ms = s_touch_pad_filter->period;
    } else {
        ESP_LOGE(RTC_MODULE_TAG, "Touch pad filter deleted");
        ret = ESP_ERR_INVALID_STATE;
    }
    xSemaphoreGive(rtc_touch_mux);
    return ret;
}

esp_err_t touch_pad_filter_start(uint32_t filter_period_ms)
{
    RTC_MODULE_CHECK(filter_period_ms >= portTICK_PERIOD_MS, "Touch pad filter period error", ESP_ERR_INVALID_ARG);
    RTC_MODULE_CHECK(rtc_touch_mux != NULL, "Touch pad not initialized", ESP_ERR_INVALID_STATE);

    esp_err_t ret = ESP_OK;
    xSemaphoreTake(rtc_touch_mux, portMAX_DELAY);
    if (s_touch_pad_filter == NULL) {
        s_touch_pad_filter = (touch_pad_filter_t *) calloc(1, sizeof(touch_pad_filter_t));
        if (s_touch_pad_filter == NULL) {
            ret = ESP_ERR_NO_MEM;
        }
    }
    if (s_touch_pad_filter->timer == NULL) {
        s_touch_pad_filter->timer = xTimerCreate("filter_tmr", filter_period_ms / portTICK_PERIOD_MS, pdTRUE,
        NULL, touch_pad_filter_cb);
        if (s_touch_pad_filter->timer == NULL) {
            ret = ESP_ERR_NO_MEM;
        }
        xTimerStart(s_touch_pad_filter->timer, portMAX_DELAY);
        s_touch_pad_filter->enable = true;
    } else {
        xTimerChangePeriod(s_touch_pad_filter->timer, filter_period_ms / portTICK_PERIOD_MS, portMAX_DELAY);
        s_touch_pad_filter->period = filter_period_ms;
        xTimerStart(s_touch_pad_filter->timer, portMAX_DELAY);
    }
    xSemaphoreGive(rtc_touch_mux);
    return ret;
}

esp_err_t touch_pad_filter_stop()
{
    RTC_MODULE_CHECK(s_touch_pad_filter != NULL, "Touch pad filter not initialized", ESP_ERR_INVALID_STATE);

    esp_err_t ret = ESP_OK;
    xSemaphoreTake(rtc_touch_mux, portMAX_DELAY);
    if (s_touch_pad_filter != NULL) {
        xTimerStop(s_touch_pad_filter->timer, portMAX_DELAY);
        s_touch_pad_filter->enable = false;
    } else {
        ESP_LOGE(RTC_MODULE_TAG, "Touch pad filter deleted");
        ret = ESP_ERR_INVALID_STATE;
    }
    xSemaphoreGive(rtc_touch_mux);
    return ret;
}

esp_err_t touch_pad_filter_delete()
{
    RTC_MODULE_CHECK(s_touch_pad_filter != NULL, "Touch pad filter not initialized", ESP_ERR_INVALID_STATE);
    xSemaphoreTake(rtc_touch_mux, portMAX_DELAY);
    if (s_touch_pad_filter != NULL) {
        if (s_touch_pad_filter->timer != NULL) {
            xTimerStop(s_touch_pad_filter->timer, portMAX_DELAY);
            xTimerDelete(s_touch_pad_filter->timer, portMAX_DELAY);
            s_touch_pad_filter->timer = NULL;
        }
        free(s_touch_pad_filter);
        s_touch_pad_filter = NULL;
    }
    xSemaphoreGive(rtc_touch_mux);
    return ESP_OK;
}

