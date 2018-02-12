// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <esp_types.h>
#include <stdlib.h>
#include <ctype.h>
#include "rom/ets_sys.h"
#include "esp_log.h"
#include "soc/rtc_io_reg.h"
#include "soc/rtc_io_struct.h"
#include "soc/sens_reg.h"
#include "soc/sens_struct.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_cntl_struct.h"
#include "soc/syscon_reg.h"
#include "soc/syscon_struct.h"
#include "rtc_io.h"
#include "touch_pad.h"
#include "adc.h"
#include "dac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/xtensa_api.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "esp_intr_alloc.h"
#include "sys/lock.h"
#include "driver/rtc_cntl.h"
#include "driver/gpio.h"
#include "adc1_i2s_private.h"

#ifndef NDEBUG
// Enable built-in checks in queue.h in debug builds
#define INVARIANTS
#endif
#include "rom/queue.h"


#define ADC_FSM_RSTB_WAIT_DEFAULT     (8)
#define ADC_FSM_START_WAIT_DEFAULT    (5)
#define ADC_FSM_STANDBY_WAIT_DEFAULT  (100)
#define ADC_FSM_TIME_KEEP             (-1)
#define ADC_MAX_MEAS_NUM_DEFAULT      (255)
#define ADC_MEAS_NUM_LIM_DEFAULT      (1)
#define SAR_ADC_CLK_DIV_DEFUALT       (2)
#define ADC_PATT_LEN_MAX              (16)

static const char *RTC_MODULE_TAG = "RTC_MODULE";

#define RTC_MODULE_CHECK(a, str, ret_val) if (!(a)) {                                \
    ESP_LOGE(RTC_MODULE_TAG,"%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str); \
    return (ret_val);                                                              \
}

#define RTC_RES_CHECK(res, ret_val) if ( (a) != ESP_OK) {                           \
        ESP_LOGE(RTC_MODULE_TAG,"%s:%d (%s)", __FILE__, __LINE__, __FUNCTION__); \
        return (ret_val);                                                              \
}

#define ADC_CHECK_UNIT(unit) RTC_MODULE_CHECK(adc_unit < ADC_UNIT_2, "ADC unit error, only support ADC1 for now", ESP_ERR_INVALID_ARG)

#define ADC1_CHECK_FUNCTION_RET(fun_ret) if(fun_ret!=ESP_OK){\
    ESP_LOGE(RTC_MODULE_TAG,"%s:%d\n",__FUNCTION__,__LINE__);\
    return ESP_FAIL;\
}

#define ADC2_CHECK_FUNCTION_RET(fun_ret) do { if(fun_ret!=ESP_OK){\
    ESP_LOGE(RTC_MODULE_TAG,"%s:%d\n",__FUNCTION__,__LINE__);\
    return ESP_FAIL;\
} }while (0)

/*
In ADC2, there're two locks used for different cases:
1. lock shared with app and WIFI: 
   when wifi using the ADC2, we assume it will never stop, 
   so app checks the lock and returns immediately if failed.

2. lock shared between tasks: 
   when several tasks sharing the ADC2, we want to guarantee 
   all the requests will be handled.
   Since conversions are short (about 31us), app returns the lock very soon, 
   we use a spinlock to stand there waiting to do conversions one by one.

adc2_spinlock should be acquired first, then adc2_wifi_lock or rtc_spinlock.
*/
//prevent ADC2 being used by wifi and other tasks at the same time.
static _lock_t adc2_wifi_lock = NULL;
//prevent ADC2 being used by tasks (regardless of WIFI)
portMUX_TYPE adc2_spinlock = portMUX_INITIALIZER_UNLOCKED;

//prevent ADC1 being used by I2S dma and other tasks at the same time.
static _lock_t adc1_i2s_lock = NULL;

/*---------------------------------------------------------------
                    ADC Common
---------------------------------------------------------------*/
static esp_err_t adc_set_fsm_time(int rst_wait, int start_wait, int standby_wait, int sample_cycle)
{
    portENTER_CRITICAL(&rtc_spinlock);
    // Internal FSM reset wait time
    if (rst_wait >= 0) {
        SYSCON.saradc_fsm.rstb_wait = rst_wait;
    }
    // Internal FSM start wait time
    if (start_wait >= 0) {
        SYSCON.saradc_fsm.start_wait = start_wait;
    }
    // Internal FSM standby wait time
    if (standby_wait >= 0) {
        SYSCON.saradc_fsm.standby_wait = standby_wait;
    }
    // Internal FSM standby sample cycle
    if (sample_cycle >= 0) {
        SYSCON.saradc_fsm.sample_cycle = sample_cycle;
    }
    portEXIT_CRITICAL(&rtc_spinlock);
    return ESP_OK;
}

static esp_err_t adc_set_data_format(adc_i2s_encode_t mode)
{
    portENTER_CRITICAL(&rtc_spinlock);
    //data format:
    //0: ADC_ENCODE_12BIT  [15:12]-channel [11:0]-12 bits ADC data
    //1: ADC_ENCODE_11BIT  [15]-1 [14:11]-channel [10:0]-11 bits ADC data, the resolution should not be larger than 11 bits in this case.
    SYSCON.saradc_ctrl.data_sar_sel = mode;
    portEXIT_CRITICAL(&rtc_spinlock);
    return ESP_OK;
}

static esp_err_t adc_set_measure_limit(uint8_t meas_num, bool lim_en)
{
    portENTER_CRITICAL(&rtc_spinlock);
    // Set max measure number
    SYSCON.saradc_ctrl2.max_meas_num = meas_num;
    // Enable max measure number limit
    SYSCON.saradc_ctrl2.meas_num_limit = lim_en;
    portEXIT_CRITICAL(&rtc_spinlock);
    return ESP_OK;
}

static esp_err_t adc_set_work_mode(adc_unit_t adc_unit)
{
    portENTER_CRITICAL(&rtc_spinlock);
    if (adc_unit == ADC_UNIT_1) {
        // saradc mode sel : 0--single saradc;  1--double saradc;  2--alternative saradc
        SYSCON.saradc_ctrl.work_mode = 0;
        //ENABLE ADC  0: ADC1  1: ADC2, only work for single SAR mode
        SYSCON.saradc_ctrl.sar_sel = 0;
    } else if (adc_unit == ADC_UNIT_2) {
        // saradc mode sel : 0--single saradc;  1--double saradc;  2--alternative saradc
        SYSCON.saradc_ctrl.work_mode = 0;
        //ENABLE ADC1  0: SAR1  1: SAR2  only work for single SAR mode
        SYSCON.saradc_ctrl.sar_sel = 1;
    } else if (adc_unit == ADC_UNIT_BOTH) {
        // saradc mode sel : 0--single saradc;  1--double saradc;  2--alternative saradc
        SYSCON.saradc_ctrl.work_mode = 1;
    } else if (adc_unit == ADC_UNIT_ALTER) {
        // saradc mode sel : 0--single saradc;  1--double saradc;  2--alternative saradc
        SYSCON.saradc_ctrl.work_mode = 2;
    }
    portEXIT_CRITICAL(&rtc_spinlock);
    return ESP_OK;
}

static esp_err_t adc_set_atten(adc_unit_t adc_unit, adc_channel_t channel, adc_atten_t atten)
{
    ADC_CHECK_UNIT(adc_unit);
    if (adc_unit & ADC_UNIT_1) {
        RTC_MODULE_CHECK((adc1_channel_t)channel < ADC1_CHANNEL_MAX, "ADC Channel Err", ESP_ERR_INVALID_ARG);
    }
    RTC_MODULE_CHECK(atten < ADC_ATTEN_MAX, "ADC Atten Err", ESP_ERR_INVALID_ARG);

    portENTER_CRITICAL(&rtc_spinlock);
    if (adc_unit & ADC_UNIT_1) {
        //SAR1_atten
        SET_PERI_REG_BITS(SENS_SAR_ATTEN1_REG, SENS_SAR1_ATTEN_VAL_MASK, atten, (channel * 2));
    }
    if (adc_unit & ADC_UNIT_2) {
        //SAR2_atten
        SET_PERI_REG_BITS(SENS_SAR_ATTEN2_REG, SENS_SAR2_ATTEN_VAL_MASK, atten, (channel * 2));
    }
    portEXIT_CRITICAL(&rtc_spinlock);
    return ESP_OK;
}

esp_err_t adc_set_clk_div(uint8_t clk_div)
{
    portENTER_CRITICAL(&rtc_spinlock);
    // ADC clock devided from APB clk, 80 / 2 = 40Mhz,
    SYSCON.saradc_ctrl.sar_clk_div = clk_div;
    portEXIT_CRITICAL(&rtc_spinlock);
    return ESP_OK;
}

esp_err_t adc_set_i2s_data_source(adc_i2s_source_t src)
{
    RTC_MODULE_CHECK(src < ADC_I2S_DATA_SRC_MAX, "ADC i2s data source error", ESP_ERR_INVALID_ARG);
    portENTER_CRITICAL(&rtc_spinlock);
    // 1: I2S input data is from SAR ADC (for DMA)  0: I2S input data is from GPIO matrix
    SYSCON.saradc_ctrl.data_to_i2s = src;
    portEXIT_CRITICAL(&rtc_spinlock);
    return ESP_OK;
}

esp_err_t adc_gpio_init(adc_unit_t adc_unit, adc_channel_t channel)
{
    ADC_CHECK_UNIT(adc_unit);
    gpio_num_t gpio_num = 0;
    if (adc_unit & ADC_UNIT_1) {
        RTC_MODULE_CHECK((adc1_channel_t) channel < ADC1_CHANNEL_MAX, "ADC1 channel error", ESP_ERR_INVALID_ARG);
        ADC1_CHECK_FUNCTION_RET(adc1_pad_get_io_num((adc1_channel_t) channel, &gpio_num));
        ADC1_CHECK_FUNCTION_RET(rtc_gpio_init(gpio_num));
        ADC1_CHECK_FUNCTION_RET(rtc_gpio_output_disable(gpio_num));
        ADC1_CHECK_FUNCTION_RET(rtc_gpio_input_disable(gpio_num));
        ADC1_CHECK_FUNCTION_RET(gpio_set_pull_mode(gpio_num, GPIO_FLOATING));
    }
    return ESP_OK;
}

esp_err_t adc_set_data_inv(adc_unit_t adc_unit, bool inv_en)
{
    portENTER_CRITICAL(&rtc_spinlock);
    if (adc_unit & ADC_UNIT_1) {
        // Enable ADC data invert
        SENS.sar_read_ctrl.sar1_data_inv = inv_en;
    }
    if (adc_unit & ADC_UNIT_2) {
        // Enable ADC data invert
        SENS.sar_read_ctrl2.sar2_data_inv = inv_en;
    }
    portEXIT_CRITICAL(&rtc_spinlock);
    return ESP_OK;
}

esp_err_t adc_set_data_width(adc_unit_t adc_unit, adc_bits_width_t bits)
{
    ADC_CHECK_UNIT(adc_unit);
    RTC_MODULE_CHECK(bits < ADC_WIDTH_MAX, "ADC bit width error", ESP_ERR_INVALID_ARG);
    portENTER_CRITICAL(&rtc_spinlock);
    if (adc_unit & ADC_UNIT_1) {
        SENS.sar_start_force.sar1_bit_width = bits;
        SENS.sar_read_ctrl.sar1_sample_bit = bits;
    }
    if (adc_unit & ADC_UNIT_2) {
        SENS.sar_start_force.sar2_bit_width = bits;
        SENS.sar_read_ctrl2.sar2_sample_bit = bits;
    }
    portEXIT_CRITICAL(&rtc_spinlock);
    return ESP_OK;
}

/*-------------------------------------------------------------------------------------
 *                      ADC I2S
 *------------------------------------------------------------------------------------*/
static esp_err_t adc_set_i2s_data_len(adc_unit_t adc_unit, int patt_len)
{
    ADC_CHECK_UNIT(adc_unit);
    RTC_MODULE_CHECK((patt_len < ADC_PATT_LEN_MAX) && (patt_len > 0), "ADC pattern length error", ESP_ERR_INVALID_ARG);
    portENTER_CRITICAL(&rtc_spinlock);
    if(adc_unit & ADC_UNIT_1) {
        SYSCON.saradc_ctrl.sar1_patt_len = patt_len - 1;
    }
    if(adc_unit & ADC_UNIT_2) {
        SYSCON.saradc_ctrl.sar2_patt_len = patt_len - 1;
    }
    portEXIT_CRITICAL(&rtc_spinlock);
    return ESP_OK;
}

static esp_err_t adc_set_i2s_data_pattern(adc_unit_t adc_unit, int seq_num, adc_channel_t channel, adc_bits_width_t bits, adc_atten_t atten)
{
    ADC_CHECK_UNIT(adc_unit);
    if (adc_unit & ADC_UNIT_1) {
        RTC_MODULE_CHECK((adc1_channel_t) channel < ADC1_CHANNEL_MAX, "ADC1 channel error", ESP_ERR_INVALID_ARG);
    }
    RTC_MODULE_CHECK(bits < ADC_WIDTH_MAX, "ADC bit width error", ESP_ERR_INVALID_ARG);
    RTC_MODULE_CHECK(atten < ADC_ATTEN_MAX, "ADC Atten Err", ESP_ERR_INVALID_ARG);

    portENTER_CRITICAL(&rtc_spinlock);
    //Configure pattern table, each 8 bit defines one channel
    //[7:4]-channel [3:2]-bit width [1:0]- attenuation
    //BIT WIDTH: 3: 12BIT  2: 11BIT  1: 10BIT  0: 9BIT
    //ATTEN: 3: ATTEN = 11dB 2: 6dB 1: 2.5dB 0: 0dB
    uint8_t val = (channel << 4) | (bits << 2) | (atten << 0);
    if (adc_unit & ADC_UNIT_1) {
        SYSCON.saradc_sar1_patt_tab[seq_num / 4] &= (~(0xff << ((3 - (seq_num % 4)) * 8)));
        SYSCON.saradc_sar1_patt_tab[seq_num / 4] |= (val << ((3 - (seq_num % 4)) * 8));
    }
    if (adc_unit & ADC_UNIT_2) {
        SYSCON.saradc_sar2_patt_tab[seq_num / 4] &= (~(0xff << ((3 - (seq_num % 4)) * 8)));
        SYSCON.saradc_sar2_patt_tab[seq_num / 4] |= (val << ((3 - (seq_num % 4)) * 8));
    }
    portEXIT_CRITICAL(&rtc_spinlock);
    return ESP_OK;
}

esp_err_t adc_i2s_mode_init(adc_unit_t adc_unit, adc_channel_t channel)
{
    ADC_CHECK_UNIT(adc_unit);
    if (adc_unit & ADC_UNIT_1) {
        RTC_MODULE_CHECK((adc1_channel_t) channel < ADC1_CHANNEL_MAX, "ADC1 channel error", ESP_ERR_INVALID_ARG);
    }

    uint8_t table_len = 1;
    //POWER ON SAR
    adc_power_always_on();
    adc_gpio_init(adc_unit, channel);
    adc_set_i2s_data_len(adc_unit, table_len);
    adc_set_i2s_data_pattern(adc_unit, 0, channel, ADC_WIDTH_BIT_12, ADC_ATTEN_DB_11);
    portENTER_CRITICAL(&rtc_spinlock);
    if (adc_unit & ADC_UNIT_1) {
        //switch SARADC into DIG channel
        SENS.sar_read_ctrl.sar1_dig_force = 1;
    }
    if (adc_unit & ADC_UNIT_2) {
        //switch SARADC into DIG channel
        SENS.sar_read_ctrl2.sar2_dig_force = 1;
        //1: SAR ADC2 is controlled by DIG ADC2 CTRL 0: SAR ADC2 is controlled by PWDET CTRL
        SYSCON.saradc_ctrl.sar2_mux = 1;
    }
    portEXIT_CRITICAL(&rtc_spinlock);
    adc_set_i2s_data_source(ADC_I2S_DATA_SRC_ADC);
    adc_set_clk_div(SAR_ADC_CLK_DIV_DEFUALT);
    // Set internal FSM wait time.
    adc_set_fsm_time(ADC_FSM_RSTB_WAIT_DEFAULT, ADC_FSM_START_WAIT_DEFAULT, ADC_FSM_STANDBY_WAIT_DEFAULT,
            ADC_FSM_TIME_KEEP);
    adc_set_work_mode(adc_unit);
    adc_set_data_format(ADC_ENCODE_12BIT);
    adc_set_measure_limit(ADC_MAX_MEAS_NUM_DEFAULT, ADC_MEAS_NUM_LIM_DEFAULT);
    //Invert The Level, Invert SAR ADC1 data
    adc_set_data_inv(adc_unit, true);
    return ESP_OK;
 }

/*-------------------------------------------------------------------------------------
 *                      ADC1
 *------------------------------------------------------------------------------------*/
esp_err_t adc1_pad_get_io_num(adc1_channel_t channel, gpio_num_t *gpio_num)
{
    RTC_MODULE_CHECK(channel < ADC1_CHANNEL_MAX, "ADC1 Channel Err", ESP_ERR_INVALID_ARG);

    switch (channel) {
    case ADC1_CHANNEL_0:
        *gpio_num = ADC1_CHANNEL_0_GPIO_NUM;
        break;
    case ADC1_CHANNEL_1:
        *gpio_num = ADC1_CHANNEL_1_GPIO_NUM;
        break;
    case ADC1_CHANNEL_2:
        *gpio_num = ADC1_CHANNEL_2_GPIO_NUM;
        break;
    case ADC1_CHANNEL_3:
        *gpio_num = ADC1_CHANNEL_3_GPIO_NUM;
        break;
    case ADC1_CHANNEL_4:
        *gpio_num = ADC1_CHANNEL_4_GPIO_NUM;
        break;
    case ADC1_CHANNEL_5:
        *gpio_num = ADC1_CHANNEL_5_GPIO_NUM;
        break;
    case ADC1_CHANNEL_6:
        *gpio_num = ADC1_CHANNEL_6_GPIO_NUM;
        break;
    case ADC1_CHANNEL_7:
        *gpio_num = ADC1_CHANNEL_7_GPIO_NUM;
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

esp_err_t adc1_config_channel_atten(adc1_channel_t channel, adc_atten_t atten)
{
    RTC_MODULE_CHECK(channel < ADC1_CHANNEL_MAX, "ADC Channel Err", ESP_ERR_INVALID_ARG);
    RTC_MODULE_CHECK(atten < ADC_ATTEN_MAX, "ADC Atten Err", ESP_ERR_INVALID_ARG);
    adc_gpio_init(ADC_UNIT_1, channel);
    adc_set_atten(ADC_UNIT_1, channel, atten);
    return ESP_OK;
}

esp_err_t adc1_config_width(adc_bits_width_t width_bit)
{
    RTC_MODULE_CHECK(width_bit < ADC_WIDTH_MAX, "ADC bit width error", ESP_ERR_INVALID_ARG);
    adc_set_data_width(ADC_UNIT_1, width_bit);
    adc_set_data_inv(ADC_UNIT_1, true);
    return ESP_OK;
}

esp_err_t adc1_i2s_mode_acquire()
{
    //lazy initialization
    //for i2s, block until acquire the lock
    _lock_acquire( &adc1_i2s_lock );
    ESP_LOGD( RTC_MODULE_TAG, "i2s mode takes adc1 lock." );
    portENTER_CRITICAL(&rtc_spinlock);
    SENS.sar_meas_wait2.force_xpd_sar = SENS_FORCE_XPD_SAR_PU;
    //switch SARADC into DIG channel
    SENS.sar_read_ctrl.sar1_dig_force = 1;
    portEXIT_CRITICAL(&rtc_spinlock);
    return ESP_OK;
}

esp_err_t adc1_adc_mode_acquire()
{
    //lazy initialization
    //for adc1, block until acquire the lock
    _lock_acquire( &adc1_i2s_lock );
    ESP_LOGD( RTC_MODULE_TAG, "adc mode takes adc1 lock." );
    portENTER_CRITICAL(&rtc_spinlock);
    // for now the WiFi would use ADC2 and set xpd_sar force on.
    // so we can not reset xpd_sar to fsm mode directly.
    // We should handle this after the synchronization mechanism is established.

    //switch SARADC into RTC channel
    SENS.sar_read_ctrl.sar1_dig_force = 0;
    portEXIT_CRITICAL(&rtc_spinlock);
    return ESP_OK;
}

esp_err_t adc1_lock_release()
{
    RTC_MODULE_CHECK((uint32_t*)adc1_i2s_lock != NULL, "adc1 lock release called before acquire", ESP_ERR_INVALID_STATE );
    // for now the WiFi would use ADC2 and set xpd_sar force on.
    // so we can not reset xpd_sar to fsm mode directly.
    // We should handle this after the synchronization mechanism is established.

    _lock_release( &adc1_i2s_lock );
    ESP_LOGD( RTC_MODULE_TAG, "returns adc1 lock." );
    return ESP_OK;
}

int adc1_get_raw(adc1_channel_t channel)
{
    uint16_t adc_value;
    RTC_MODULE_CHECK(channel < ADC1_CHANNEL_MAX, "ADC Channel Err", ESP_ERR_INVALID_ARG);
    adc1_adc_mode_acquire();
    adc_power_on();

    portENTER_CRITICAL(&rtc_spinlock);
    //Adc Controler is Rtc module,not ulp coprocessor
    SENS.sar_meas_start1.meas1_start_force = 1;
    //Disable Amp Bit1=0:Fsm  Bit1=1(Bit0=0:PownDown Bit10=1:Powerup)
    SENS.sar_meas_wait2.force_xpd_amp = 0x2;
    //Open the ADC1 Data port Not ulp coprocessor
    SENS.sar_meas_start1.sar1_en_pad_force = 1;
    //Select channel
    SENS.sar_meas_start1.sar1_en_pad = (1 << channel);
    SENS.sar_meas_ctrl.amp_rst_fb_fsm = 0;
    SENS.sar_meas_ctrl.amp_short_ref_fsm = 0;
    SENS.sar_meas_ctrl.amp_short_ref_gnd_fsm = 0;
    SENS.sar_meas_wait1.sar_amp_wait1 = 1;
    SENS.sar_meas_wait1.sar_amp_wait2 = 1;
    SENS.sar_meas_wait2.sar_amp_wait3 = 1;
    while (SENS.sar_slave_addr1.meas_status != 0);
    SENS.sar_meas_start1.meas1_start_sar = 0;
    SENS.sar_meas_start1.meas1_start_sar = 1;
    while (SENS.sar_meas_start1.meas1_done_sar == 0);
    adc_value = SENS.sar_meas_start1.meas1_data_sar;
    portEXIT_CRITICAL(&rtc_spinlock);
    adc1_lock_release();
    return adc_value;
}

int adc1_get_voltage(adc1_channel_t channel)    //Deprecated. Use adc1_get_raw() instead
{
    return adc1_get_raw(channel);
}

void adc1_ulp_enable(void)
{
    adc_power_on();

    portENTER_CRITICAL(&rtc_spinlock);
    SENS.sar_meas_start1.meas1_start_force = 0;
    SENS.sar_meas_start1.sar1_en_pad_force = 0;
    SENS.sar_meas_wait2.force_xpd_amp = 0x2;
    SENS.sar_meas_ctrl.amp_rst_fb_fsm = 0;
    SENS.sar_meas_ctrl.amp_short_ref_fsm = 0;
    SENS.sar_meas_ctrl.amp_short_ref_gnd_fsm = 0;
    SENS.sar_meas_wait1.sar_amp_wait1 = 0x1;
    SENS.sar_meas_wait1.sar_amp_wait2 = 0x1;
    SENS.sar_meas_wait2.sar_amp_wait3 = 0x1;
    portEXIT_CRITICAL(&rtc_spinlock);
}

/*---------------------------------------------------------------
                    ADC2
---------------------------------------------------------------*/
esp_err_t adc2_pad_get_io_num(adc2_channel_t channel, gpio_num_t *gpio_num)
{
    RTC_MODULE_CHECK(channel < ADC2_CHANNEL_MAX, "ADC2 Channel Err", ESP_ERR_INVALID_ARG);

    switch (channel) {
    case ADC2_CHANNEL_0:
        *gpio_num = ADC2_CHANNEL_0_GPIO_NUM;
        break;
    case ADC2_CHANNEL_1:
        *gpio_num = ADC2_CHANNEL_1_GPIO_NUM;
        break;
    case ADC2_CHANNEL_2:
        *gpio_num = ADC2_CHANNEL_2_GPIO_NUM;
        break;
    case ADC2_CHANNEL_3:
        *gpio_num = ADC2_CHANNEL_3_GPIO_NUM;
        break;
    case ADC2_CHANNEL_4:
        *gpio_num = ADC2_CHANNEL_4_GPIO_NUM;
        break;
    case ADC2_CHANNEL_5:
        *gpio_num = ADC2_CHANNEL_5_GPIO_NUM;
        break;
    case ADC2_CHANNEL_6:
        *gpio_num = ADC2_CHANNEL_6_GPIO_NUM;
        break;
    case ADC2_CHANNEL_7:
        *gpio_num = ADC2_CHANNEL_7_GPIO_NUM;
        break;
    case ADC2_CHANNEL_8:
        *gpio_num = ADC2_CHANNEL_8_GPIO_NUM;
        break;
    case ADC2_CHANNEL_9:
        *gpio_num = ADC2_CHANNEL_9_GPIO_NUM;
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

esp_err_t adc2_wifi_acquire()
{
    //lazy initialization
    //for wifi, block until acquire the lock
    _lock_acquire( &adc2_wifi_lock );
    ESP_LOGD( RTC_MODULE_TAG, "Wi-Fi takes adc2 lock." );
    return ESP_OK;
}

esp_err_t adc2_wifi_release()
{
    RTC_MODULE_CHECK((uint32_t*)adc2_wifi_lock != NULL, "wifi release called before acquire", ESP_ERR_INVALID_STATE );

    _lock_release( &adc2_wifi_lock );
    ESP_LOGD( RTC_MODULE_TAG, "Wi-Fi returns adc2 lock." );
    return ESP_OK;
}

static esp_err_t adc2_pad_init(adc2_channel_t channel)
{
    gpio_num_t gpio_num = 0;
    ADC2_CHECK_FUNCTION_RET(adc2_pad_get_io_num(channel, &gpio_num));
    ADC2_CHECK_FUNCTION_RET(rtc_gpio_init(gpio_num));
    ADC2_CHECK_FUNCTION_RET(rtc_gpio_output_disable(gpio_num));
    ADC2_CHECK_FUNCTION_RET(rtc_gpio_input_disable(gpio_num));
    ADC2_CHECK_FUNCTION_RET(gpio_set_pull_mode(gpio_num, GPIO_FLOATING));
    return ESP_OK;
}

esp_err_t adc2_config_channel_atten(adc2_channel_t channel, adc_atten_t atten)
{
    RTC_MODULE_CHECK(channel < ADC2_CHANNEL_MAX, "ADC2 Channel Err", ESP_ERR_INVALID_ARG);
    RTC_MODULE_CHECK(atten <= ADC_ATTEN_11db, "ADC2 Atten Err", ESP_ERR_INVALID_ARG);

    adc2_pad_init(channel);
    portENTER_CRITICAL( &adc2_spinlock );

    //lazy initialization
    //avoid collision with other tasks
    if ( _lock_try_acquire( &adc2_wifi_lock ) == -1 ) {
        //try the lock, return if failed (wifi using).
        portEXIT_CRITICAL( &adc2_spinlock );
        return ESP_ERR_TIMEOUT;
    }
    SENS.sar_atten2 = ( SENS.sar_atten2 & ~(3<<(channel*2)) ) | ((atten&3) << (channel*2));
    _lock_release( &adc2_wifi_lock );
    
    portEXIT_CRITICAL( &adc2_spinlock );
    return ESP_OK;
}

static inline void adc2_config_width(adc_bits_width_t width_bit)
{
    portENTER_CRITICAL(&rtc_spinlock);
    //sar_start_force shared with ADC1
    SENS.sar_start_force.sar2_bit_width = width_bit;
    portEXIT_CRITICAL(&rtc_spinlock);

    //Invert the adc value,the Output value is invert
    SENS.sar_read_ctrl2.sar2_data_inv = 1;
    //Set The adc sample width,invert adc value,must digital sar2_bit_width[1:0]=3
    SENS.sar_read_ctrl2.sar2_sample_bit = width_bit;
    //Take the control from WIFI    
    SENS.sar_read_ctrl2.sar2_pwdet_force = 0;
}

//registers in critical section with adc1:
//SENS_SAR_START_FORCE_REG, 
esp_err_t adc2_get_raw(adc2_channel_t channel, adc_bits_width_t width_bit, int* raw_out)
{
    uint16_t adc_value = 0;
    RTC_MODULE_CHECK(channel < ADC2_CHANNEL_MAX, "ADC Channel Err", ESP_ERR_INVALID_ARG);

    //in critical section with whole rtc module
    adc_power_on();

    //avoid collision with other tasks
    portENTER_CRITICAL(&adc2_spinlock); 
    //lazy initialization
    //try the lock, return if failed (wifi using).
    if ( _lock_try_acquire( &adc2_wifi_lock ) == -1 ) {
        portEXIT_CRITICAL( &adc2_spinlock );
        return ESP_ERR_TIMEOUT;
    }
    //in critical section with whole rtc module
    adc2_config_width( width_bit );
    
    //Adc Controler is Rtc module,not ulp coprocessor
    SENS.sar_meas_start2.meas2_start_force = 1; //force pad mux and force start
    //Open the ADC2 Data port Not ulp coprocessor
    SENS.sar_meas_start2.sar2_en_pad_force = 1; //open the ADC2 data port
    //Select channel
    SENS.sar_meas_start2.sar2_en_pad = 1 << channel; //pad enable
    SENS.sar_meas_start2.meas2_start_sar = 0; //start force 0
    SENS.sar_meas_start2.meas2_start_sar = 1; //start force 1
    while (SENS.sar_meas_start2.meas2_done_sar == 0) {}; //read done
    adc_value = SENS.sar_meas_start2.meas2_data_sar;
    _lock_release( &adc2_wifi_lock );
    portEXIT_CRITICAL(&adc2_spinlock);

    *raw_out = (int)adc_value;
    return ESP_OK;
}

esp_err_t adc2_vref_to_gpio(gpio_num_t gpio)
{
    int channel;
    if(gpio == GPIO_NUM_25){
        channel = 8;    //Channel 8 bit
    }else if (gpio == GPIO_NUM_26){
        channel = 9;    //Channel 9 bit
    }else if (gpio == GPIO_NUM_27){
        channel = 7;    //Channel 7 bit
    }else{
        return ESP_ERR_INVALID_ARG;
    }

    //Configure RTC gpio
    rtc_gpio_init(gpio);
    rtc_gpio_output_disable(gpio);
    rtc_gpio_input_disable(gpio);
    rtc_gpio_pullup_dis(gpio);
    rtc_gpio_pulldown_dis(gpio);
    //force fsm
    adc_power_always_on();               //Select power source of ADC

    RTCCNTL.bias_conf.dbg_atten = 0;     //Check DBG effect outside sleep mode
    //set dtest (MUX_SEL : 0 -> RTC; 1-> vdd_sar2)
    RTCCNTL.test_mux.dtest_rtc = 1;      //Config test mux to route v_ref to ADC2 Channels
    //set ent
    RTCCNTL.test_mux.ent_rtc = 1;
    //set sar2_en_test
    SENS.sar_start_force.sar2_en_test = 1;
    //set sar2 en force
    SENS.sar_meas_start2.sar2_en_pad_force = 1;      //Pad bitmap controlled by SW
    //set en_pad for channels 7,8,9 (bits 0x380)
    SENS.sar_meas_start2.sar2_en_pad = 1<<channel;

    return ESP_OK;
}

/*---------------------------------------------------------------
                        HALL SENSOR
---------------------------------------------------------------*/
static int hall_sensor_get_value()    //hall sensor without LNA
{
    int Sens_Vp0;
    int Sens_Vn0;
    int Sens_Vp1;
    int Sens_Vn1;
    int hall_value;
    
    adc_power_on();

    portENTER_CRITICAL(&rtc_spinlock);
    SENS.sar_touch_ctrl1.xpd_hall_force = 1;     // hall sens force enable
    RTCIO.hall_sens.xpd_hall = 1;      // xpd hall
    SENS.sar_touch_ctrl1.hall_phase_force = 1;   // phase force

    RTCIO.hall_sens.hall_phase = 0;      // hall phase
    Sens_Vp0 = adc1_get_raw(ADC1_CHANNEL_0);
    Sens_Vn0 = adc1_get_raw(ADC1_CHANNEL_3);
    RTCIO.hall_sens.hall_phase = 1;
    Sens_Vp1 = adc1_get_raw(ADC1_CHANNEL_0);
    Sens_Vn1 = adc1_get_raw(ADC1_CHANNEL_3);

    SENS.sar_touch_ctrl1.xpd_hall_force = 0;
    SENS.sar_touch_ctrl1.hall_phase_force = 0;
    portEXIT_CRITICAL(&rtc_spinlock);
    hall_value = (Sens_Vp1 - Sens_Vp0) - (Sens_Vn1 - Sens_Vn0);

    return hall_value;
}

int hall_sensor_read()
{
    adc_gpio_init(ADC_UNIT_1, ADC1_CHANNEL_0);
    adc_gpio_init(ADC_UNIT_1, ADC1_CHANNEL_3);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_0);
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_0);
    return hall_sensor_get_value();
}


