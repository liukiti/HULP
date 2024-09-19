#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "soc/rtc.h"

#include "hulp.h"
#include "hulp_i2cbb.h"

#define PIN_SDA_GPIO GPIO_NUM_27
#define PIN_SCL_GPIO GPIO_NUM_26

#define SCL_PIN PIN_SCL_GPIO
#define SDA_PIN PIN_SDA_GPIO

#define SLAVE_ADDR 0x18U

// Set for 8-bit read:
#define SLAVE_READ8_SUBADDR0 0x28U
#define SLAVE_READ8_SUBADDR1 0x29U
#define SLAVE_READ8_SUBADDR2 0x2AU
#define SLAVE_READ8_SUBADDR3 0x2BU
#define SLAVE_READ8_SUBADDR4 0x2CU
#define SLAVE_READ8_SUBADDR5 0x2DU
// // Set for 16-bit read:
// #define SLAVE_READ16_SUBADDR 0x28U

// Set subaddress and value for write:
#define SLAVE_WRITE_SUBADDR 0x20U
#define SLAVE_WRITE_VALUE 0x04U

RTC_DATA_ATTR ulp_var_t ulp_data8_0;
RTC_DATA_ATTR ulp_var_t ulp_data8_1;
RTC_DATA_ATTR ulp_var_t ulp_data8_2;
RTC_DATA_ATTR ulp_var_t ulp_data8_3;
RTC_DATA_ATTR ulp_var_t ulp_data8_4;
RTC_DATA_ATTR ulp_var_t ulp_data8_5;
RTC_DATA_ATTR ulp_var_t ulp_data16;
RTC_DATA_ATTR ulp_var_t ulp_nacks;
RTC_DATA_ATTR ulp_var_t ulp_buserrors;

void init_ulp()
{
    enum {
        LBL_WRITE_RETURN,
        LBL_READ8_0_RETURN,
        LBL_READ8_1_RETURN,
        LBL_READ8_2_RETURN,
        LBL_READ8_3_RETURN,
        LBL_READ8_4_RETURN,
        LBL_READ8_5_RETURN,
        LBL_READ16_RETURN,

        LBL_HALT,
        
        LBL_I2C_READ_ENTRY,
        LBL_I2C_WRITE_ENTRY,
        LBL_I2C_NACK,
        LBL_I2C_ARBLOST,
    };

    const ulp_insn_t program[] = {
        I_MOVI(R2,0),

    #ifdef SLAVE_WRITE_SUBADDR
        M_I2CBB_WR(LBL_WRITE_RETURN, LBL_I2C_WRITE_ENTRY, SLAVE_WRITE_SUBADDR, SLAVE_WRITE_VALUE),
    #endif
    
    #ifdef SLAVE_READ8_SUBADDR0
        M_I2CBB_RD(LBL_READ8_0_RETURN, LBL_I2C_READ_ENTRY, SLAVE_READ8_SUBADDR0),
        I_PUT(R0, R2, ulp_data8_0),
        M_I2CBB_RD(LBL_READ8_1_RETURN, LBL_I2C_READ_ENTRY, SLAVE_READ8_SUBADDR1),
        I_PUT(R0, R2, ulp_data8_1),
        M_I2CBB_RD(LBL_READ8_2_RETURN, LBL_I2C_READ_ENTRY, SLAVE_READ8_SUBADDR2),
        I_PUT(R0, R2, ulp_data8_2),
        M_I2CBB_RD(LBL_READ8_3_RETURN, LBL_I2C_READ_ENTRY, SLAVE_READ8_SUBADDR3),
        I_PUT(R0, R2, ulp_data8_3),
        M_I2CBB_RD(LBL_READ8_4_RETURN, LBL_I2C_READ_ENTRY, SLAVE_READ8_SUBADDR4),
        I_PUT(R0, R2, ulp_data8_4),
        M_I2CBB_RD(LBL_READ8_5_RETURN, LBL_I2C_READ_ENTRY, SLAVE_READ8_SUBADDR5),
        I_PUT(R0, R2, ulp_data8_5),
    #endif

#ifdef TEMP_SENS
        I_TSENS(R0, 1000),
        I_MOVI(R2,0),
        I_PUT(R0, R2, ulp_tsens_val),
#endif
        I_WAKE(),

    #ifdef SLAVE_READ16_SUBADDR
        M_I2CBB_RD(LBL_READ16_RETURN, LBL_I2C_READ_ENTRY, SLAVE_READ16_SUBADDR),
        I_PUT(R0, R2, ulp_data16),
        I_WAKE(),
    #endif


        I_HALT(),

        M_LABEL(LBL_I2C_NACK),
            I_GET(R0, R2, ulp_nacks),
            I_ADDI(R0,R0,1),
            I_PUT(R0,R2, ulp_nacks),
            I_WAKE(),
            I_BXR(R3),

        M_LABEL(LBL_I2C_ARBLOST),
            I_GET(R0, R2, ulp_buserrors),
            I_ADDI(R0,R0,1),
            I_PUT(R0,R2, ulp_buserrors),
            I_WAKE(),
            I_BXR(R3),

        M_INCLUDE_I2CBB(LBL_I2C_READ_ENTRY, LBL_I2C_WRITE_ENTRY, LBL_I2C_ARBLOST, LBL_I2C_NACK, SCL_PIN, SDA_PIN, SLAVE_ADDR),
    };

    ESP_ERROR_CHECK(hulp_configure_pin(SCL_PIN, RTC_GPIO_MODE_INPUT_ONLY, GPIO_FLOATING, 0));
    ESP_ERROR_CHECK(hulp_configure_pin(SDA_PIN, RTC_GPIO_MODE_INPUT_ONLY, GPIO_FLOATING, 0));

    hulp_peripherals_on();

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(hulp_ulp_load(program, sizeof(program), 1ULL * 1000 * 1000, 0));
    ESP_ERROR_CHECK(hulp_ulp_run(0));
}

void ulp_isr(void *task_handle_ptr)
{
    xTaskNotifyFromISR(*(TaskHandle_t*)task_handle_ptr, 0, eNoAction, NULL);
}

extern "C" void app_main()
{
    // ULP will trigger an interrupt when there's new data or an error
    TaskHandle_t main_handle =  xTaskGetCurrentTaskHandle();
    hulp_ulp_isr_register(&ulp_isr, &main_handle);
    hulp_ulp_interrupt_en();

    init_ulp();

    for(;;)
    {
        // Wait for interrupt
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
        printf("x: %d | y: %d | z: %d |, NACK Errors: %u, Bus Errors: %u\n", 
                   (int16_t)((ulp_data8_1.val<<8)+ulp_data8_0.val),
                   (int16_t)((ulp_data8_3.val<<8)+ulp_data8_2.val),
                   (int16_t)((ulp_data8_5.val<<8)+ulp_data8_4.val),
                    ulp_nacks.val, ulp_buserrors.val);
    }
}