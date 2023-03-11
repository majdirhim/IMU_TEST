#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

#include "lsm6dso_reg.h"
#include "string.h"

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME      10

#define I2C_MASTER_NUM          I2C_NUM_0 // I2C_NUM_1
#define I2C_MASTER_SCL_IO       21 // 27
#define I2C_MASTER_SDA_IO       22 //26
#define I2C_MASTER_FREQ_HZ      100000

#define SENSOR_BUS I2C_MASTER_NUM
#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */
/* Private variables ---------------------------------------------------------*/
static uint8_t whoamI, rst;
static uint8_t tx_buffer[1000];

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(i2c_port_t handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(i2c_port_t handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);


/**
 * @brief i2c master initialization
 */
void i2c_master_init()
{
    i2c_config_t conf;
    conf.clk_flags = 0;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void app_main(void)
{
	  i2c_master_init();


	  stmdev_ctx_t dev_ctx;
	  lsm6dso_pin_int1_route_t int1_route;
	  /* Initialize mems driver interface */
	  dev_ctx.write_reg = platform_write;
	  dev_ctx.read_reg = platform_read;
	  dev_ctx.handle = I2C_NUM_0;
	  /* Wait sensor boot time */
	  platform_delay(BOOT_TIME);
	  /* Check device ID */
	  lsm6dso_device_id_get(&dev_ctx, &whoamI);
	  printf("state : %d\n\r",whoamI);
	  if (whoamI != LSM6DSO_ID)
	    while (1);
	  printf("debug after while 1\n\r");
	  /* Restore default configuration */
	  lsm6dso_reset_set(&dev_ctx, PROPERTY_ENABLE);

	  do {
	    lsm6dso_reset_get(&dev_ctx, &rst);
	  } while (rst);

	  /* Disable I3C interface */
	  lsm6dso_i3c_disable_set(&dev_ctx, LSM6DSO_I3C_DISABLE);
	  /* Set XL and Gyro Output Data Rate */
	  lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_208Hz);
	  lsm6dso_gy_data_rate_set(&dev_ctx, LSM6DSO_GY_ODR_104Hz);
	  /* Set 2g full XL scale and 250 dps full Gyro */
	  lsm6dso_xl_full_scale_set(&dev_ctx, LSM6DSO_2g);
	  lsm6dso_gy_full_scale_set(&dev_ctx, LSM6DSO_250dps);
	  /* Set duration for Activity detection to 9.62 ms (= 2 * 1 / ODR_XL) */
	  lsm6dso_wkup_dur_set(&dev_ctx, 0x02);
	  /* Set duration for Inactivity detection to 4.92 s (= 2 * 512 / ODR_XL) */
	  lsm6dso_act_sleep_dur_set(&dev_ctx, 0x02);
	  /* Set Activity/Inactivity threshold to 62.5 mg */
	  lsm6dso_wkup_threshold_set(&dev_ctx, 0x02);
	  /* Inactivity configuration: XL to 12.5 in LP, gyro to Power-Down */
	  lsm6dso_act_mode_set(&dev_ctx, LSM6DSO_XL_12Hz5_GY_PD);
	  /* Enable interrupt generation on Inactivity INT1 pin */
	  lsm6dso_pin_int1_route_get(&dev_ctx, &int1_route);
	  int1_route.sleep_change = PROPERTY_ENABLE;
	  lsm6dso_pin_int1_route_set(&dev_ctx, int1_route);

	  /* Wait Events */
	  while (1) {
	    lsm6dso_all_sources_t all_source;
	    /* Check if Activity/Inactivity events */
	    lsm6dso_all_sources_get(&dev_ctx, &all_source);

	    if (all_source.sleep_state) {
	      sprintf((char *)tx_buffer, "Inactivity Detected\r\n");
	      tx_com(tx_buffer, strlen((char const *)tx_buffer));
	    }

	    if (all_source.wake_up) {
	      sprintf((char *)tx_buffer, "Activity Detected\r\n");
	      tx_com(tx_buffer, strlen((char const *)tx_buffer));
	    }
	  }
}



/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 * __________________________________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write 1 byte (register address) + ack | write n bytes + ack  | stop |
 * --------|---------------------------|---------------------------------------|----------------------|------|
 *
 */
static int32_t platform_write(i2c_port_t handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);  // Start bit
    i2c_master_write_byte(cmd, (uint8_t) (LSM6DSO_I2C_ADD_H <<1 ) | WRITE_BIT, ACK_CHECK_EN); //slave_addr + wr_bit + ack
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN); //  write 1 byte (register address) + ack
    i2c_master_write(cmd, bufp, len, ACK_CHECK_EN); // write n bytes + ack
    i2c_master_stop(cmd); //Stop
    esp_err_t ret = i2c_master_cmd_begin(handle, cmd, 1000 / portTICK_PERIOD_MS); //Send the Tram
    i2c_cmd_link_delete(cmd);
    return ret;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 * _____________________________________________________________________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write 1 byte (register address) + ack | start | slave_addr + rd_bit + ack | read n bytes + nack | stop |
 * --------|---------------------------|---------------------------------------|-------|---------------------------|---------------------|------|
 *
 */
static int32_t platform_read(i2c_port_t handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
	   if (len == 0) {
	        return ESP_OK;
	    }
	    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	    i2c_master_start(cmd);
	    i2c_master_write_byte(cmd, (uint8_t) (LSM6DSO_I2C_ADD_H <<1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
	    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);

	    i2c_master_start(cmd);
	    i2c_master_write_byte(cmd, (uint8_t) (LSM6DSO_I2C_ADD_H <<1 ) | I2C_MASTER_READ, ACK_CHECK_EN);
	    if (len > 1) {
	        i2c_master_read(cmd, bufp, len - 1, ACK_VAL); // read len-1 bytes + Ack
	    }
	    i2c_master_read_byte(cmd, bufp + len - 1, NACK_VAL); // read the last byte + NACK to end  transcations
	    i2c_master_stop(cmd);
	    esp_err_t ret = i2c_master_cmd_begin(handle, cmd, 1000 / portTICK_PERIOD_MS);
	    i2c_cmd_link_delete(cmd);
	    return ret;
}

/*
 * @brief  Send buffer to console (platform dependent)
 *
 * @param  tx_buffer     buffer to transmit
 * @param  len           number of byte to send
 *
 */
static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
	/*for(uint16_t i = 0 ; i<len ;i++)
		printf("%c",tx_buffer[i]);
	printf("\n\r");*/
}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
	vTaskDelay(ms/portTICK_PERIOD_MS);
}
