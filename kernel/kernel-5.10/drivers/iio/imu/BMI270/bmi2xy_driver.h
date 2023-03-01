/**
 * @section LICENSE
 * Copyright (c) 2019~2020 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http://www.fsf.org/copyleft/gpl.html
 *
 * @filename bmi2xy_driver.h
 * @date	 30/09/2021
 * @version	 2.0.0
 *
 * @brief	 BMI2xy Linux Driver
 */

#ifndef BMI2XY_DRIVER_H
#define BMI2XY_DRIVER_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************/
/* System header files */
/*********************************************************************/
#include <linux/types.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/firmware.h>

/*********************************************************************/
/* Own header files */
/*********************************************************************/
/* BMI2xy variants. Only one should be enabled */
#include "bmi270.h"

/*********************************************************************/
/* Macro definitions */
/*********************************************************************/
/** Name of the device driver and accel input device*/
#define SENSOR_NAME "bmi2xy"
/** Name of the feature input device*/
#define SENSOR_NAME_FEAT "bmi2xy_feat"

/* Generic */
#define ANY_NO_MOTION_WORKAROUND		(1)
#define BMI2XY_ENABLE_INT1			(1)
#define BMI2XY_MAX_RETRY_I2C_XFER		(10)
#define BMI2XY_I2C_WRITE_DELAY_TIME		(1)
#define REL_FEAT_STATUS				(1)
#define REL_HW_STATUS				(2)

/*fifo definition*/
#define A_BYTES_FRM		 (6)
#define G_BYTES_FRM		 (6)
#define M_BYTES_FRM		 (8)
#define MA_BYTES_FRM	 (14)
#define MG_BYTES_FRM	 (14)
#define AG_BYTES_FRM	 (12)
#define AMG_BYTES_FRM	 (20)
#define FIFO_ACC_EN_MSK				(0x40)
#define FIFO_GYRO_EN_MSK			(0x80)
#define FIFO_MAG_EN_MSK				(0x20)

#if defined(BMM150)
#define BMI2_AUX_CHIP_ID_ADDR			(0x40)
#define BMI2_AUX_PWR_MODE_SUSPEND		(0x02)
#define BMI2_AUX_PWR_MODE_NORMAL		(0x00)
#endif


#define BMI2_FIFO_ACC_FRAME_COUNT     UINT8_C(50)

/*! Number of gyro frames to be extracted from FIFO. */
#define BMI2_FIFO_GYRO_FRAME_COUNT      UINT8_C(50)

/*! Macro to read sensortime byte in FIFO. */
#define SENSORTIME_OVERHEAD_BYTE        UINT8_C(220)
/*! Buffer size allocated to store raw FIFO data. */
#define BMI2_FIFO_RAW_DATA_BUFFER_SIZE  UINT16_C(2048)

/*! Length of data to be read from FIFO. */
#define BMI2_FIFO_RAW_DATA_USER_LENGTH  UINT16_C(2048)

/**
 * enum bmi2xy_config_func - Enumerations to select the sensors
 */
enum bmi2xy_config_func {
	BMI2XY_SIG_MOTION_SENSOR = 0,
	BMI2XY_STEP_DETECTOR_SENSOR = 1,
	BMI2XY_STEP_COUNTER_SENSOR = 2,
	BMI2XY_ACTIVITY_SENSOR = 3,
	BMI2XY_WRIST_WAKEUP_SENSOR = 4,
	BMI2XY_WRIST_GESTURE_SENSOR = 5,
	BMI2XY_ANY_MOTION_SENSOR = 6,
	BMI2XY_NO_MOTION_SENSOR = 7
};

/**
 * enum bmi2xy_int_status0 - Enumerations corresponding to status0 registers
 */
enum bmi2xy_int_status0 {
	SIG_MOTION_OUT = 0x01,
	STEP_DET_OUT = 0x02,
	ACTIVITY_OUT = 0x04,
	WRIST_WAKEUP_OUT = 0x08,
	WRIST_GESTURE_OUT = 0x10,
	NO_MOTION_OUT = 0x20,
	ANY_MOTION_OUT = 0x40
};

/**
 * enum bmi2xy_int_status1 - Enumerations corresponding to status1 registers
 */
enum bmi2xy_int_status1 {
	FIFOFULL_OUT = 0x01,
	FIFOWATERMARK_OUT = 0x02,
	MAG_DRDY_OUT = 0x20,
	ACC_DRDY_OUT = 0x80
};

/**
 * enum bmi2xy_self_test_rslt - Enumerations for Self-test feature
 */
enum bmi2xy_self_test_rslt {
	SELF_TEST_PASS,
	SELF_TEST_FAIL,
	SELF_TEST_NOT_RUN
};

/**
 * struct pw_mode - Structure for sensor power modes.
 */
struct pw_mode {
	u8 acc_pm;
	u8 gyr_pm;
};

/**
 *	struct bmi2xy_client_data - Client structure which holds sensor-specific
 *	information.
 */
struct bmi2xy_client_data {
	struct bmi2_dev device;
	struct device *dev;
	struct input_dev *acc_input;
	struct input_dev *feat_input;
	u8 config_file_loaded;
	u8 fifo_gyr_enable;
	u8 fifo_acc_enable;
	u32 fifo_bytecount;
	struct pw_mode pw;
	u8 acc_odr;
	u8 gyr_odr;
	struct mutex lock;
	int IRQ;
	u8 gpio_pin;
	struct work_struct irq_work;
	u16 fw_version;
	char *config_stream_name;
	int reg_sel;
	int reg_len;
	int feat_page_sel;
	int feat_page_len;
	struct delayed_work delay_work_sig;
	atomic_t in_suspend;
	u8 acc_selftest;
	u8 gyr_selftest;
	u8 sigmotion_enable;
	u8 stepdet_enable;
	u8 stepcounter_enable;
	u8 anymotion_enable;
	u8 nomotion_enable;
	u8 wrist_wakeup_enable;
	u8 wrist_gesture_enable;
	u8 activity_enable;
	u8 err_int_trigger_num;
	u32 step_counter_val;
	u32 step_counter_temp;
};

/*********************************************************************/
/* Function prototype declarations */
/*********************************************************************/

#if defined(BMM150)
/**
 * user_aux_read - Reads data from auxillary sensor in manual mode.
 *
 * @reg_addr : Register address to read the data.
 * @aux_data : Aux data pointer to store the read data.
 * @len : No of bytes to read from the register.
 *
 * Return : Result of execution status
 * * 0 - Success
 * * negative value -> Error
 */
s8 user_aux_read(u8 reg_addr, u8 *aux_data, u16 len);

/**
 * user_aux_write - Writes data to the auxillary sensor in manual mode.
 *
 * @reg_addr : Register address to write the data.
 * @aux_data : Aux data pointer to store the data being written.
 * @len : No of bytes to write to the register.
 *
 * Return : Result of execution status
 * * 0 - Success
 * * negative value -> Error
 */
s8 user_aux_write(u8 reg_addr, u8 *aux_data, u16 len);
#endif

/**
 * bmi2xy_probe - This is the probe function for bmi2xy sensor.
 * Called from the I2C driver probe function to initialize the sensor.
 *
 * @client_data : Structure instance of client data.
 * @dev : Structure instance of device.
 *
 * Return : Result of execution status
 * * 0 - Success
 * * negative value -> Error
 */
int bmi2xy_probe(struct bmi2xy_client_data *client_data, struct device *dev);

/**
 * bmi2xy_remove - This function removes the driver from the device.
 *
 * @dev : Structure instance of device.
 *
 * Return : Result of execution status
 * * 0 - Success
 * * negative value -> Error
 */
int bmi2xy_remove(struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* BMI2XY_DRIVER_H_	*/
