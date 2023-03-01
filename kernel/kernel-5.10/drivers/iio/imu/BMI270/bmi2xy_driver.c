/*
 * @section LICENSE
 * Copyright (c) 2019~2020 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http://www.fsf.org/copyleft/gpl.html
 *
 * @filename bmi2xy_driver.c
 * @date	 30/09/2021
 * @version	 2.0.0
 *
 * @brief	 BMI2xy Linux Driver
 */

/*********************************************************************/
/* System header files */
/*********************************************************************/
#include <linux/types.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

/*********************************************************************/
/* Own header files */
/*********************************************************************/
#include "bmi2xy_driver.h"
#include "bs_log.h"

/*********************************************************************/
/* Local macro definitions */
/*********************************************************************/
#define DRIVER_VERSION "2.0.0"
/*********************************************************************/
/* Global data */
/*********************************************************************/
/* Macros to select low_g and free_fall */

#define MS_TO_US(msec)		UINT32_C((msec) * 1000)

/**
 * soft_reset_store - sysfs write callback which performs the
 * sensor soft reset.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t soft_reset_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int err;
	unsigned long soft_reset;
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);

	/* Base of decimal number system is 10 */
	err = kstrtoul(buf, 10, &soft_reset);

	if (err) {
		PERR("Soft reset: invalid input");
		return -EIO;
	}

	if (soft_reset)
		/* Perform soft reset */
		err = bmi2_soft_reset(&client_data->device);
	else
		return -EINVAL;
	if (err) {
		PERR("Soft Reset failed");
		return -EIO;
	}

	return count;
}

/**
 * bmi2xy_check_error - check error code and return -EIO if err is not 0.
 *
 * @print_msg	: print message to print on if err is not 0.
 * @err			: error return from execution function.
 */
static void bmi2xy_check_error(char *print_msg, int err)
{
	if (err)
		PERR("%s failed with error code: %d", print_msg, err);
}

/**
 * bmi2xy_i2c_delay_us - Adds a delay in units of microsecs.
 *
 * @usec		: Delay value in microsecs.
 * @intf_ptr	: Void pointer that can enable the linking of descriptors
 *									for interface related call backs
 */
static void bmi2xy_i2c_delay_us(u32 usec, void *intf_ptr)
{

	if (usec <= (MS_TO_US(20)))

		/* Delay range of usec to usec + 1 millisecs
		 * required due to kernel limitation
		 */
		usleep_range(usec, usec + 1000);
	else
		msleep(usec/1000);
}

/**
 * chip_id_show - sysfs callback for reading the chip id of the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t chip_id_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	u8 chip_id[2] = {0};
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);

	err = bmi2_get_regs(BMI2_CHIP_ID_ADDR, chip_id,
					2, &client_data->device);
	bmi2xy_check_error("chip id read", err);

	return scnprintf(buf, 96, "chip_id=0x%x rev_id=0x%x\n",
		chip_id[0], chip_id[1]);
}

/**
 * acc_enable_show - sysfs callback which tells whether accelerometer is
 * enabled or disabled.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t acc_enable_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	int err;
	u8 acc_enable = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);

	err = bmi2_get_regs(BMI2_PWR_CTRL_ADDR, &acc_enable, 1,
							&client_data->device);
	/* Get the accel enable bit */
	acc_enable = (acc_enable & 0x04) >> 2;
	bmi2xy_check_error("acc power reg read", err);

	return scnprintf(buf, 96, "%d\n", acc_enable);
}

/**
 * acc_enable_store - sysfs callback which enables or disables the
 * accelerometer.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 * Accelerometer will not be disabled unless all the features related to
 * accelerometer are disabled.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t acc_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int err;
	unsigned long op_mode;
	u8 sens_list = BMI2_ACCEL;
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);

	err = kstrtoul(buf, 10, &op_mode);
	bmi2xy_check_error("get user option", err);
	if (op_mode == 0 &&
		((client_data->sigmotion_enable +
		client_data->stepdet_enable +
		client_data->stepcounter_enable +
		client_data->activity_enable +
		client_data->anymotion_enable +
		client_data->nomotion_enable +
		client_data->wrist_wakeup_enable +
		client_data->wrist_gesture_enable) == 0)) {
		mutex_lock(&client_data->lock);
		err = bmi270_sensor_disable(&sens_list, 1,
						&client_data->device);
		mutex_unlock(&client_data->lock);
		PDEBUG("acc_enable %ld", op_mode);
	} else if (op_mode == 1) {
		mutex_lock(&client_data->lock);
		err = bmi270_sensor_enable(&sens_list, 1, &client_data->device);
		mutex_unlock(&client_data->lock);
		PDEBUG("acc_enable %ld", op_mode);
	} else {
		PERR("Disable all features before disabling acc");
		return -EIO;
	}
	bmi2xy_check_error("set sensor en/disable", err);

	mutex_lock(&client_data->lock);
	client_data->pw.acc_pm = op_mode;
	mutex_unlock(&client_data->lock);

	return count;
}

/**
 * gyr_enable_show - sysfs callback which tells whether gyroscope is
 * enabled or disabled.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t gyr_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int err;
	u8 gyr_enable = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);

	err = bmi2_get_regs(BMI2_PWR_CTRL_ADDR, &gyr_enable, 1,
							&client_data->device);
	gyr_enable = (gyr_enable & 0x02) >> 1;
	bmi2xy_check_error("read pwr reg", err);

	return scnprintf(buf, 96, "%d\n", gyr_enable);
}

/**
 * gyr_enable_store - sysfs callback which enables or disables the
 * gyroscope.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t gyr_enable_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	int err;
	unsigned long op_mode;
	u8 sens_list = BMI2_GYRO;
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);

	err = kstrtoul(buf, 10, &op_mode);
	bmi2xy_check_error("get user option", err);

	if (op_mode == 0) {
		mutex_lock(&client_data->lock);
		err = bmi270_sensor_disable(&sens_list, 1, &client_data->device);
		mutex_unlock(&client_data->lock);
		PDEBUG("gyr_enable %ld", op_mode);
	} else if (op_mode == 1) {
		mutex_lock(&client_data->lock);
		err = bmi270_sensor_enable(&sens_list, 1, &client_data->device);
		mutex_unlock(&client_data->lock);
		PDEBUG("gyr_enable %ld", op_mode);
	} else {
		err = -EINVAL;
	}
	bmi2xy_check_error("set sensor en/disable", err);

	mutex_lock(&client_data->lock);
	client_data->pw.gyr_pm = op_mode;
	mutex_unlock(&client_data->lock);

	return count;
}

/**
 * acc_value_show - sysfs read callback which gives the
 * raw accelerometer value from the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t acc_value_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);
	struct bmi2_sens_data sensor_data;

	if (client_data->pw.acc_pm) {
		mutex_lock(&client_data->lock);
		err = bmi2_get_sensor_data(&sensor_data, &client_data->device);
		bmi2xy_check_error("get acc sensor data", err);
		mutex_unlock(&client_data->lock);
		return scnprintf(buf, 48, "%hd %hd %hd\n",
				sensor_data.acc.x,
				sensor_data.acc.y,
				sensor_data.acc.z);
	} else {
		PERR("sensor not enabled");
		return -EIO;
	}
}

/**
 * gyr_value_show - sysfs read callback which gives the
 * raw gyroscope value from the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t gyr_value_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);
	struct bmi2_sens_data sensor_data;

	if (client_data->pw.gyr_pm) {
		mutex_lock(&client_data->lock);
		err = bmi2_get_sensor_data(&sensor_data, &client_data->device);
		mutex_unlock(&client_data->lock);
		bmi2xy_check_error("get gyro sensor data", err);

		return scnprintf(buf, 48, "%hd %hd %hd\n",
				sensor_data.gyr.x,
				sensor_data.gyr.y,
				sensor_data.gyr.z);
	} else {
		PERR("sensor not enabled");
		return -EIO;
	}
}

/**
 * acc_range_show - sysfs read callback which gives the
 * accelerometer range which is set in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t acc_range_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);
	struct bmi2_sens_config config;

	mutex_lock(&client_data->lock);
	config.type = BMI2_ACCEL;
	err = bmi270_get_sensor_config(&config, 1, &client_data->device);
	mutex_unlock(&client_data->lock);
	bmi2xy_check_error("get sensor config", err);

	return scnprintf(buf, 16, "%d\n", config.cfg.acc.range);
}

/**
 * acc_range_store - sysfs write callback which sets the
 * accelerometer range to be set in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t acc_range_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int err;
	unsigned long acc_range;
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);
	struct bmi2_sens_config config;

	err = kstrtoul(buf, 10, &acc_range);
	if (err)
		return -EINVAL;
	mutex_lock(&client_data->lock);
	config.type = BMI2_ACCEL;
	err = bmi270_get_sensor_config(&config, 1, &client_data->device);
	config.cfg.acc.range = (u8)(acc_range);
	err += bmi270_set_sensor_config(&config, 1, &client_data->device);
	mutex_unlock(&client_data->lock);
	bmi2xy_check_error("set acc range", err);

	return count;
}

/**
 * acc_odr_show - sysfs read callback which gives the
 * accelerometer output data rate of the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t acc_odr_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);
	struct bmi2_sens_config config;

	mutex_lock(&client_data->lock);
	config.type = BMI2_ACCEL;
	err = bmi270_get_sensor_config(&config, 1, &client_data->device);
	bmi2xy_check_error("get sensor config", err);
	client_data->acc_odr = config.cfg.acc.odr;
	mutex_unlock(&client_data->lock);

	return scnprintf(buf, 16, "%d\n", client_data->acc_odr);
}

/**
 * acc_odr_store - sysfs write callback which sets the
 * accelerometer output data rate in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t acc_odr_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	int err;
	u8 acc_odr;
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);
	struct bmi2_sens_config config;

	err = kstrtou8(buf, 10, &acc_odr);
	bmi2xy_check_error("get user input", err);

	mutex_lock(&client_data->lock);
	config.type = BMI2_ACCEL;
	err = bmi270_get_sensor_config(&config, 1, &client_data->device);
	config.cfg.acc.odr = acc_odr;
	if (acc_odr < BMI2_ACC_ODR_12_5HZ) {
		config.cfg.acc.filter_perf = BMI2_POWER_OPT_MODE;
		config.cfg.acc.bwp = BMI2_ACC_RES_AVG16;
	} else {
		config.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
		config.cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
	}
	err = bmi270_set_sensor_config(&config, 1, &client_data->device);
	bmi2xy_check_error("acc ODR config", err);

	client_data->acc_odr = acc_odr;
	mutex_unlock(&client_data->lock);
	return count;
}

/**
 * gyr_range_show - sysfs read callback which gives the
 * gyroscope range of the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t gyr_range_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);
	struct bmi2_sens_config config;

	mutex_lock(&client_data->lock);
	config.type = BMI2_GYRO;
	err = bmi270_get_sensor_config(&config, 1, &client_data->device);
	mutex_unlock(&client_data->lock);
	bmi2xy_check_error("get sensor config", err);

	return scnprintf(buf, 16, "%d\n", config.cfg.gyr.range);
}

/**
 * gyr_range_store - sysfs write callback which sets the
 * gyroscope range in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t gyr_range_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int err;
	unsigned long gyr_range;
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);
	struct bmi2_sens_config config;

	err = kstrtoul(buf, 10, &gyr_range);
	bmi2xy_check_error("get user input", err);

	mutex_lock(&client_data->lock);
	config.type = BMI2_GYRO;
	err = bmi270_get_sensor_config(&config, 1, &client_data->device);
	config.cfg.gyr.range = (u8)(gyr_range);
	err += bmi270_set_sensor_config(&config, 1, &client_data->device);
	mutex_unlock(&client_data->lock);
	bmi2xy_check_error("set sensor config", err);

	return count;
}

/**
 * gyr_odr_show - sysfs read callback which gives the
 * gyroscope output data rate of the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t gyr_odr_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);
	struct bmi2_sens_config config;

	mutex_lock(&client_data->lock);
	config.type = BMI2_GYRO;
	err = bmi270_get_sensor_config(&config, 1, &client_data->device);
	bmi2xy_check_error("get sensor config", err);

	client_data->gyr_odr = config.cfg.gyr.odr;
	mutex_unlock(&client_data->lock);

	return scnprintf(buf, 16, "%d\n", client_data->gyr_odr);
}

/**
 * gyr_odr_store - sysfs write callback which sets the
 * gyroscope output data rate in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t gyr_odr_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	int err;
	unsigned long gyr_odr;
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);
	struct bmi2_sens_config config;

	err = kstrtoul(buf, 10, &gyr_odr);
	if (err)
		return err;
	mutex_lock(&client_data->lock);
	config.type = BMI2_GYRO;
	err = bmi270_get_sensor_config(&config, 1, &client_data->device);
	bmi2xy_check_error("get sensor config", err);
	config.cfg.gyr.odr = (u8)(gyr_odr);
	err += bmi270_set_sensor_config(&config, 1, &client_data->device);
	bmi2xy_check_error("set sensor config", err);
	client_data->gyr_odr = gyr_odr;
	mutex_unlock(&client_data->lock);

	return count;
}

/**
 * acc_selftest_show - sysfs read callback which gives the
 * accelerometer self test result of the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t acc_selftest_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);

	return scnprintf(buf, 64, "Pass => 0\tFail => 1\tNot run => 2 %d\n",
			client_data->acc_selftest);
}

/**
 * acc_selftest_store - sysfs write callback which performs the
 * accelerometer self test in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t acc_selftest_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int err;
	unsigned long selftest;
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);

	err = kstrtoul(buf, 10, &selftest);
	bmi2xy_check_error("get user option", err);

	if (selftest == 1) {
		mutex_lock(&client_data->lock);
		/* Perform accelerometer self-test */
		err = bmi2_perform_accel_self_test(&client_data->device);
		if (err)
			client_data->acc_selftest = SELF_TEST_FAIL;
		else
			client_data->acc_selftest = SELF_TEST_PASS;
		mutex_unlock(&client_data->lock);
	} else {
		return -EINVAL;
	}

	return count;
}

/**
 * acc_foc_show - sysfs read callback which notifies the format which
 * is to be used to perform accelerometer FOC in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t acc_foc_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, 64,
		"Use echo x_axis y_axis z_axis sign > acc_foc\n");
}

/**
 * acc_foc_store - sysfs write callback which performs the
 * accelerometer FOC in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t acc_foc_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct bmi2_accel_foc_g_value g_value = {0};
	unsigned int data[4];
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);
	int err = 0;

	err = sscanf(buf, "%d %d %d %d",
		&data[0], &data[1], &data[2], &data[3]);

	if (err != 4) {
		PERR("Invalid argument");
		return -EINVAL;
	}

	g_value.x = data[0];
	g_value.y = data[1];
	g_value.z = data[2];
	g_value.sign = data[3];
	PDEBUG("g_value.x=%d, g_value.y=%d, g_value.z=%d g_value.sign=%d",
		g_value.x, g_value.y, g_value.z, g_value.sign);
	err = bmi2_perform_accel_foc(&g_value, &client_data->device);
	bmi2xy_check_error("accel FOC", err);

	PINFO("FOC Accel successfully done\n");
	return count;
}

/**
 * gyr_foc_show - sysfs read callback which performs the gyroscope
 * FOC in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t gyr_foc_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);
	int err = 0;

	err = bmi2_perform_gyro_foc(&client_data->device);
	bmi2xy_check_error("gyro FOC", err);

	return scnprintf(buf, 64,
		"FOC Gyro successfully done\n");
}

/**
 * fifo_data_frame_show - sysfs read callback which reads the fifo data
 * according to the fifo length.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t fifo_data_frame_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);
	int err = 0;
	u16 index = 0;
	u16 fifo_length = 0;
	u16 acc_frame_length = BMI2_FIFO_ACC_FRAME_COUNT;
	u16 gyr_frame_length = BMI2_FIFO_GYRO_FRAME_COUNT;
	u8 fifo_data[2268] = { 0 };
	struct bmi2_sens_axes_data
					fifo_acc_data[BMI2_FIFO_ACC_FRAME_COUNT] = { { 0 } };
	struct bmi2_sens_axes_data
						fifo_gyr_data[BMI2_FIFO_GYRO_FRAME_COUNT] = { { 0 } };
	struct bmi2_fifo_frame fifoframe = { 0 };

	if (!client_data->fifo_acc_enable
					&& !client_data->fifo_gyr_enable) {
		PERR("sensor not enabled for fifo");
		return -EINVAL;
	}
	mutex_lock(&client_data->lock);
	fifoframe.data = fifo_data;
	err = bmi2_get_fifo_length(&fifo_length, &client_data->device);
	bmi2xy_check_error("get fifo length", err);

	fifoframe.length = fifo_length;
	PDEBUG("FIFO data bytes available : %d", fifo_length);
	err = bmi2_read_fifo_data(&fifoframe, &client_data->device);
	bmi2xy_check_error("read fifo data", err);

	if (client_data->fifo_acc_enable) {
		PDEBUG("FIFO accel frames requested : %d", acc_frame_length);

		err = bmi2_extract_accel(fifo_acc_data, &acc_frame_length,
											&fifoframe, &client_data->device);
		if (err == 2)
			PDEBUG("fifo data is not fully read and extrated");
		else if (err != 0)
			PDEBUG("acc fifo extract failed with err: %d", err);
		PDEBUG("FIFO accel frames extracted : %d", acc_frame_length);
		for (index = 0; index < acc_frame_length; index++) {
			PDEBUG("ACCEL[%d] X:%d\t Y:%d\t Z:%d\n", index,
					fifo_acc_data[index].x,
					fifo_acc_data[index].y,
					fifo_acc_data[index].z);
		}

	}

	if (client_data->fifo_gyr_enable) {
		PDEBUG("FIFO gyro frames requested : %d", gyr_frame_length);
		err = bmi2_extract_gyro(fifo_gyr_data, &gyr_frame_length, &fifoframe,
														&client_data->device);
		if (err == 2)
			PDEBUG("fifo data is not fully read and extrated");
		else if (err != 0)
			PDEBUG("gyro fifo extract failed with err: %d", err);
		PDEBUG("FIFO gyro frames extracted : %d", gyr_frame_length);
		for (index = 0; index < gyr_frame_length; index++) {
			PDEBUG("GYRO[%d] X:%d\t Y:%d\t Z:%d\n",
			   index,
			   fifo_gyr_data[index].x,
			   fifo_gyr_data[index].y,
			   fifo_gyr_data[index].z);
		}
	}
	mutex_unlock(&client_data->lock);
	PDEBUG("Skipped frame count = %d\n", fifoframe.skipped_frame_count);
	return 0;
}

/**
 * acc_fifo_enable_show - sysfs read callback which shows the enable or
 * disable status of the accelerometer FIFO in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t acc_fifo_enable_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	int err;
	u8 fifo_acc_enable;
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);

	mutex_lock(&client_data->lock);
	/* Get the fifo config */
	err = bmi2_get_regs(0x49, &fifo_acc_enable, 1, &client_data->device);
	mutex_unlock(&client_data->lock);
	fifo_acc_enable = (fifo_acc_enable & 0x40) >> 6;
	bmi2xy_check_error("get user option", err);
	if (err) {
		PERR("acc fifo enable failed with err: %d", err);
		return err;
	}
	return scnprintf(buf, 16, "%x\n", fifo_acc_enable);
}

/**
 * acc_fifo_enable_store - sysfs write callback enables or
 * disables the accelerometer FIFO in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t acc_fifo_enable_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	int err;
	unsigned long data;
	u8 fifo_acc_enable;
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);

	mutex_lock(&client_data->lock);
	err = kstrtoul(buf, 10, &data);
	bmi2xy_check_error("get user option", err);

	fifo_acc_enable = (unsigned char)(data & 0x01);
	/* Disable advance power save to use FIFO */
	if (fifo_acc_enable) {
		err = bmi2_set_adv_power_save(0, &client_data->device);
		if (err) {
			mutex_unlock(&client_data->lock);
			return -EINVAL;
		}
	}
	err = bmi2_set_fifo_config(BMI2_FIFO_ACC_EN, fifo_acc_enable,
							&client_data->device);
	bmi2xy_check_error("set fifo config", err);
	client_data->fifo_acc_enable = fifo_acc_enable;
	mutex_unlock(&client_data->lock);

	return count;
}

/**
 * gyr_fifo_enable_store - sysfs write callback enables or
 * disables the gyroscope FIFO in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t gyr_fifo_enable_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	int err;
	unsigned long data;
	unsigned char fifo_gyr_enable;
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);

	err = kstrtoul(buf, 10, &data);
	bmi2xy_check_error("get user option", err);

	fifo_gyr_enable = (unsigned char)(data & 0x01);

	mutex_lock(&client_data->lock);
	/* Disable advance power save to use FIFO */
	if (fifo_gyr_enable) {
		err = bmi2_set_adv_power_save(0, &client_data->device);
		bmi2xy_check_error("set aps mode", err);
	}

	err = bmi2_set_fifo_config(BMI2_FIFO_GYR_EN, fifo_gyr_enable,
							&client_data->device);
	bmi2xy_check_error("set fifo config", err);
	client_data->fifo_gyr_enable = fifo_gyr_enable;
	mutex_unlock(&client_data->lock);

	return count;
}

/**
 * gyr_fifo_enable_show - sysfs read callback which shows the enable or
 * disable status of the gyroscope FIFO in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t gyr_fifo_enable_show(struct device *dev,
					 struct device_attribute *attr, char *buf)
{
	int err;
	u8 fifo_gyr_enable;
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);

	mutex_lock(&client_data->lock);
	/* Get the fifo config */
	err = bmi2_get_regs(BMI2_FIFO_CONFIG_1_ADDR, &fifo_gyr_enable, 1,
			&client_data->device);
	mutex_unlock(&client_data->lock);
	fifo_gyr_enable = (fifo_gyr_enable & 0x80) >> 7;
	bmi2xy_check_error("get fifo config", err);

	return scnprintf(buf, 16, "%x\n", fifo_gyr_enable);
}

/**
 * fifo_flush_store - sysfs write callback which flushes the fifo data
 * in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t fifo_flush_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int err;
	unsigned long enable;
	/* Command for fifo flush */
	u8 fifo_flush = 0xB0;
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);

	err = kstrtoul(buf, 10, &enable);
	bmi2xy_check_error("get user option", err);

	if (enable == 0x01) {
		mutex_lock(&client_data->lock);
		err = bmi2_set_regs(BMI2_CMD_REG_ADDR, &fifo_flush, 1,
							&client_data->device);
		mutex_unlock(&client_data->lock);
	} else {
		return -EINVAL;
	}
	bmi2xy_check_error("fifo flush", err);

	return count;
}

/**
 * aps_enable_show - sysfs callback which tells whether the advanced power
 * save mode is enabled or disabled.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t aps_enable_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	int err;
	u8 aps_status = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);

	/* Get the status of advance power save mode */
	err = bmi2_get_adv_power_save(&aps_status, &client_data->device);
	bmi2xy_check_error("get aps mode", err);


	/* Maximum number of bytes used to prevent overflow */
	return scnprintf(buf, 96, "%d\n", aps_status);
}

/**
 * aps_enable_store - sysfs callback which enables or disables the
 * advanced power save mode.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t aps_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int err;
	u8 aps_status;
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);

	err = kstrtou8(buf, 10, &aps_status);
	bmi2xy_check_error("get user option", err);


	if (aps_status) {

		/* Enable advance power save mode */
		err = bmi2_set_adv_power_save(BMI2_ENABLE,
							&client_data->device);
	} else {

		/* Disable advance power save mode */
		err = bmi2_set_adv_power_save(BMI2_DISABLE,
							&client_data->device);
	}
	bmi2xy_check_error("set aps mode", err);

	return count;
}

/**
 * load_config_stream_show - sysfs read callback which gives the loaded
 * config stream in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t load_config_stream_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);

	return scnprintf(buf, 48, "config stream %s\n",
					client_data->config_stream_name);
}

/**
 * bmi2xy_initialize_interrupt_settings - Initialize the interrupt settings of
 * the sensor.
 *
 * @client_data: Instance of client data structure.
 *
 * Return: Status of the function.
 * * 0			- OK
 * * Negative value	- Failed
 */
static int bmi2xy_initialize_interrupt_settings(
					struct bmi2xy_client_data *client_data)
{
	int err = 0;
	/* Input enable and open drain */
	u8 int_enable = 0x0A;
	/* Permanent latched */
	u8 latch_enable = 0x01;
	/* Map all feature interrupts */
	u8 int1_map = 0xff;

	mutex_lock(&client_data->lock);
	err = bmi2_set_regs(BMI2_INT1_MAP_FEAT_ADDR, &int1_map, 1,
			&client_data->device);
	bmi2xy_i2c_delay_us(MS_TO_US(10), &client_data->device.intf_ptr);
	err += bmi2_set_regs(BMI2_INT1_IO_CTRL_ADDR, &int_enable, 1,
			&client_data->device);
	bmi2xy_i2c_delay_us(MS_TO_US(10), &client_data->device.intf_ptr);
	err += bmi2_set_regs(BMI2_INT_LATCH_ADDR, &latch_enable, 1,
			&client_data->device);
	mutex_unlock(&client_data->lock);
	bmi2xy_i2c_delay_us(MS_TO_US(10), &client_data->device.intf_ptr);
	bmi2xy_check_error("INT enable and mapping", err);

	return err;
}

/**
 * bmi2xy_init_fifo_config - Initializes the fifo configuration of the sensor.
 *
 * @client_data: Instance of client data structure.
 *
 * Return: Status of the function.
 * * 0	-	OK
 * * Negative value	-	Failed
 */
static int bmi2xy_init_fifo_config(struct bmi2xy_client_data *client_data)
{
	int err = 0;
	u8 reg_data;

	mutex_lock(&client_data->lock);
	err = bmi2_get_regs(BMI2_FIFO_CONFIG_0_ADDR, &reg_data, 1,
							&client_data->device);
	/* Enable fifo time */
	reg_data |= 0x02;
	err = bmi2_set_regs(BMI2_FIFO_CONFIG_0_ADDR, &reg_data, 1,
							&client_data->device);
	mutex_unlock(&client_data->lock);
	bmi2xy_check_error("set fifo config", err);

	mutex_lock(&client_data->lock);
	err = bmi2_get_regs(BMI2_FIFO_CONFIG_1_ADDR, &reg_data, 1,
							&client_data->device);
	/* Enable fifo header */
	reg_data |= 0x10;
	err = bmi2_set_regs(BMI2_FIFO_CONFIG_1_ADDR, &reg_data, 1,
							&client_data->device);
	mutex_unlock(&client_data->lock);
	bmi2xy_check_error("set fifo config", err);

	return 0;
}

/**
 * bmi2xy_update_config_stream - Loads the config stream in the sensor.
 *
 * @client_data: Instance of client data structure.
 * @option: Option to choose different tbin images.
 *
 * Return: Status of the function.
 * * 0	-	OK
 * * Negative value	-	Failed
 */
int bmi2xy_update_config_stream(struct bmi2xy_client_data *client_data,
				int option)
{
	int err = 0;
	u8 crc_check = 0;

	/* Delay to read the configuration load status */
	/* Get the status */
	if (option == 1) {
		err = bmi270_init(&client_data->device);
		bmi2xy_check_error("config stream download", err);

		client_data->config_stream_name = "bmi270_config_stream";
		bmi2xy_i2c_delay_us(MS_TO_US(200),
						&client_data->device.intf_ptr);
		err = bmi2_get_regs(BMI2_INTERNAL_STATUS_ADDR,
		&crc_check, 1, &client_data->device);
		bmi2xy_check_error("CRC check", err);

		if (crc_check != BMI2_CONFIG_LOAD_SUCCESS)
			PERR("crc check error %x", crc_check);
	} else {
		PERR("Invalid input use: echo 1 > load_config_stream");
		return -EINVAL;
	}
	return err;
}

/**
 * load_config_stream_store - sysfs write callback which loads the
 * config stream in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t load_config_stream_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned long choose = 0;
	int err = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);
	struct bmi2_remap remapped_axis;

	err = kstrtoul(buf, 10, &choose);
	bmi2xy_check_error("get user option", err);


	/* Load config file if not loaded in init stage */
	if (!client_data->config_file_loaded) {
		err = bmi2xy_update_config_stream(client_data, choose);
		bmi2xy_check_error("load config stream", err);

	}
	err = bmi2xy_initialize_interrupt_settings(client_data);
	err += bmi2xy_init_fifo_config(client_data);
	err += bmi2_get_remap_axes(&remapped_axis, &client_data->device);
	remapped_axis.x = BMI2_NEG_X;
	remapped_axis.y = BMI2_Y;
	remapped_axis.z = BMI2_NEG_Z;

	err = bmi2_set_remap_axes(&remapped_axis, &client_data->device);
	bmi2xy_check_error("axis remap", err);

	return count;
}

/**
 * reg_sel_show - sysfs read callback which provides the register
 * address selected.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t reg_sel_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);

	return scnprintf(buf, 64, "reg=0X%02X, len=%d\n",
		client_data->reg_sel, client_data->reg_len);
}

/**
 * reg_sel_store - sysfs write callback which stores the register
 * address to be selected.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t reg_sel_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);
	int err;

	mutex_lock(&client_data->lock);
	err = sscanf(buf, "%11X %11d",
		&client_data->reg_sel, &client_data->reg_len);
	if ((err != 2) || (client_data->reg_len > 128)
		|| (client_data->reg_sel > 127)) {
		PERR("Invalid argument");
		mutex_unlock(&client_data->lock);
		return -EINVAL;
	}
	mutex_unlock(&client_data->lock);
	return count;
}

/**
 * reg_val_show - sysfs read callback which shows the register
 * value which is read from the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t reg_val_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);
	int err;
	u8 reg_data[128];
	int i;
	int pos;

	mutex_lock(&client_data->lock);
	if ((client_data->reg_len > 128) || (client_data->reg_sel > 127)) {
		PERR("Invalid argument");
		mutex_unlock(&client_data->lock);
		return -EINVAL;
	}
	if (client_data->reg_len == 0) {
		PERR("error reg_sel len is 0");
		return -EINVAL;
	}
	err = bmi2_get_regs(client_data->reg_sel, reg_data,
				client_data->reg_len, &client_data->device);
	bmi2xy_check_error("get register", err);

	pos = 0;
	for (i = 0; i < client_data->reg_len; ++i) {
		pos += scnprintf(buf + pos, 16, "%02X", reg_data[i]);
		buf[pos++] = (i + 1) % 16 == 0 ? '\n' : ' ';
	}
	mutex_unlock(&client_data->lock);
	if (buf[pos - 1] == ' ')
		buf[pos - 1] = '\n';
	return pos;
}

/**
 * reg_val_store - sysfs write callback which stores the register
 * value which is to be written in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t reg_val_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);
	int err;
	u8 reg_data[128] = {0,};
	int i, j, status, digit;

	if (client_data->reg_len == 0) {
		PERR("error reg_sel len is 0");
		return -EINVAL;
	}
	status = 0;
	mutex_lock(&client_data->lock);
	for (i = j = 0; i < count && j < client_data->reg_len; ++i) {
		if (buf[i] == ' ' || buf[i] == '\n' || buf[i] == '\t' ||
			buf[i] == '\r') {
			status = 0;
			++j;
			continue;
		}
		digit = buf[i] & 0x10 ? (buf[i] & 0xF) : ((buf[i] & 0xF) + 9);
		PDEBUG("digit is %d", digit);
		switch (status) {
		case 2:
			++j; /* Fall thru */
		case 0:
			reg_data[j] = digit;
			status = 1;
			break;
		case 1:
			reg_data[j] = reg_data[j] * 16 + digit;
			status = 2;
			break;
		}
	}
	if (status > 0)
		++j;
	if (j > client_data->reg_len)
		j = client_data->reg_len;
	else if (j < client_data->reg_len) {
		PERR("Invalid argument");
		mutex_unlock(&client_data->lock);
		return -EINVAL;
	}
	PDEBUG("Reg data read as");
	for (i = 0; i < j; ++i)
		PDEBUG("%d", reg_data[i]);
	err = client_data->device.write(client_data->reg_sel, reg_data,
					client_data->reg_len,
					client_data->device.intf_ptr);
	mutex_unlock(&client_data->lock);
	bmi2xy_check_error("register read/write operation", err);

	return count;
}

/**
 * driver_version_show - sysfs read callback which provides the driver
 * version.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t driver_version_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, 128,
		"Driver version: %s\n", DRIVER_VERSION);
}

/**
 * avail_sensor_show - sysfs read callback which provides the sensor-id
 * to the user.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t avail_sensor_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	u16 avail_sensor = 270;

	return scnprintf(buf, 32, "%d\n", avail_sensor);
}

/**
 * execute_crt_store - sysfs write callback which triggers the
 * CRT feature in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t execute_crt_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	int err = 0;
	unsigned long rd_wr_len, rd_wr_len_prev;
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);

	err = kstrtoul(buf, 10, &rd_wr_len);
	bmi2xy_check_error("get user option", err);

	rd_wr_len_prev = client_data->device.read_write_len;

	client_data->device.read_write_len = (unsigned int) rd_wr_len;
	mutex_lock(&client_data->lock);
	err = bmi2_do_crt(&client_data->device);
	mutex_unlock(&client_data->lock);
	client_data->device.read_write_len = rd_wr_len_prev;

	PDEBUG("CRT execution status %d", err);

	return count;
}


/**
 * gyr_usr_gain_show - sysfs callback which returns the gyro
 * user gain values from the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 * This should be called after executing the CRT operation.
 *
 * Return: Number of characters returned.
 */
static ssize_t gyr_usr_gain_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int err;
	u8 gyr_usr_gain[3];
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);

	err = bmi2_get_regs(BMI2_GYR_USR_GAIN_0_ADDR, gyr_usr_gain,
				3, &client_data->device);
	bmi2xy_check_error("get gyro user gain register", err);


	return scnprintf(buf, 48, "x = %hd\t y = %hd\t z= %hd\n",
					gyr_usr_gain[0],
					gyr_usr_gain[1],
					gyr_usr_gain[2]);
}

/**
 * gyr_offset_comp_show - sysfs callback which returns the gyro
 * offset compensation values from the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t gyr_offset_comp_show(struct device *dev,
					 struct device_attribute *attr, char *buf)
{
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);
	/* Structure to store gyroscope offset compensated data */
	struct bmi2_sens_axes_data gyr_off_comp = {0};

	/* Read the offset compensation registers */
	err =  bmi2_read_gyro_offset_comp_axes(&gyr_off_comp,
							&client_data->device);

	bmi2xy_check_error("read gyro offset comp", err);

	return scnprintf(buf, 48, "%hd %hd %hd\n", gyr_off_comp.x,
						gyr_off_comp.y,
						gyr_off_comp.z);
}

/**
 * gyr_offset_comp_store - sysfs write callback which performs the
 * gyroscope offset compensation in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t gyr_offset_comp_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	int err;
	unsigned long gyr_off_comp;
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);
	u8	sensor_sel[3];

	sensor_sel[0] = BMI2_ACCEL;
	sensor_sel[1] = BMI2_GYRO;
	sensor_sel[2] = BMI2_GYRO_SELF_OFF;

	err = kstrtoul(buf, 10, &gyr_off_comp);
	bmi2xy_check_error("get user option", err);


	if (gyr_off_comp) {
		/* Enable self offset correction */
		err = bmi270_sensor_enable(&sensor_sel[2], 1,
							&client_data->device);
		/* Enable gyroscope offset compensation */
		err = bmi2_set_gyro_offset_comp(BMI2_ENABLE,
							&client_data->device);
	} else {
		/* Disable self offset correction */
		err = bmi270_sensor_disable(&sensor_sel[2], 1,
							&client_data->device);
	}
	bmi2xy_check_error("gyro offset comp set", err);

	if (!err)
		err = bmi270_sensor_enable(&sensor_sel[0], 2,
							&client_data->device);
	bmi2xy_check_error("sensor enable", err);

	PDEBUG("Gyro offset compenstion successfully done\n");
	return count;
}


/**
 * gyr_selftest_show - sysfs read callback which gives the
 * gyroscope self test result of the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t gyr_selftest_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);

	return scnprintf(buf, 64, "Pass => 0\tFail => 1\tNot run => 2 %d\n",
			client_data->gyr_selftest);
}

/**
 * gyr_selftest_store - sysfs write callback which performs the
 * gyroscope self test in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t gyr_selftest_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int err;
	unsigned long selftest;
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);

	err = kstrtoul(buf, 10, &selftest);
	bmi2xy_check_error("get user option", err);

	if (selftest == 1) {
		mutex_lock(&client_data->lock);
		/* Perform accelerometer self-test */
		err = bmi2_do_gyro_st(&client_data->device);
		if (err)
			client_data->gyr_selftest = SELF_TEST_FAIL;
		else
			client_data->gyr_selftest = SELF_TEST_PASS;
		mutex_unlock(&client_data->lock);
	} else {
		return -EINVAL;
	}

	return count;
}

/**
 * nvm_prog_show - sysfs read callback which gives the performs NVM
 * back up of offset compensation values for accelerometer.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t nvm_prog_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);

	err = bmi2_nvm_prog(&client_data->device);
	bmi2xy_check_error("nvm programming", err);


	return scnprintf(buf, 64, "nvm programming successful\n");

}

/**
 * config_function_show - sysfs read callback which gives the list of
 * enabled features in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t config_function_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);

	return scnprintf(buf, PAGE_SIZE,
		"sig_motion0=%d\nstep_detector1=%d\nstep_counter2=%d\n"
		"activity3=%d\nwrist_wakeup4=%d\nwrist_gesture5=%d\n"
		"any_motion6=%d\nnomotion7=%d\n",
		client_data->sigmotion_enable, client_data->stepdet_enable,
		client_data->stepcounter_enable, client_data->activity_enable,
		client_data->wrist_wakeup_enable, client_data->wrist_gesture_enable,
		client_data->anymotion_enable, client_data->nomotion_enable);
}

/**
 * config_function_store - sysfs write callback which enable or disable
 * the features in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t config_function_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	int ret;
	int config_func = 0;
	int enable = 0;
	u8 sens_list = 0;
	struct bmi2_sens_config config[2];
	struct bmi2_sens_int_config sens_int = { 0 };
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);

	ret = sscanf(buf, "%11d %11d", &config_func, &enable);
	PDEBUG("config_func = %d, enable=%d", config_func, enable);
	if (ret != 2) {
		PERR("Invalid argument");
		return -EINVAL;
	}
	if (config_func < 0 || config_func > 16)
		return -EINVAL;

	mutex_lock(&client_data->lock);
	switch (config_func) {
	case BMI2XY_SIG_MOTION_SENSOR:
		sens_list = BMI2_SIG_MOTION;
		sens_int.type = BMI2_SIG_MOTION;
		client_data->sigmotion_enable = enable;
		break;
	case BMI2XY_NO_MOTION_SENSOR:
		config[0].type = BMI2_NO_MOTION;
		sens_list = BMI2_NO_MOTION;
		sens_int.type = BMI2_NO_MOTION;
		ret = bmi270_get_sensor_config(&config[0], 1,
							&client_data->device);
		if (enable) {
			/* Enable the x, y and z axis of nomotion */
			config[0].cfg.no_motion.select_x = BMI2_ENABLE;
			config[0].cfg.no_motion.select_y = BMI2_ENABLE;
			config[0].cfg.no_motion.select_z = BMI2_ENABLE;

		} else {
			/* Disable the x, y and z axis of nomotion */
			config[0].cfg.no_motion.select_x = BMI2_DISABLE;
			config[0].cfg.no_motion.select_y = BMI2_DISABLE;
			config[0].cfg.no_motion.select_z = BMI2_DISABLE;
		}
		ret += bmi270_set_sensor_config(&config[0], 1, &client_data->device);
		if (ret == 0)
			client_data->nomotion_enable =	enable;
		break;
	case BMI2XY_STEP_DETECTOR_SENSOR:
		sens_list = BMI2_STEP_DETECTOR;
		sens_int.type = BMI2_STEP_DETECTOR;
		client_data->stepdet_enable = enable;
		break;
	case BMI2XY_STEP_COUNTER_SENSOR:
		sens_list = BMI2_STEP_COUNTER;
		sens_int.type = BMI2_STEP_COUNTER;
		client_data->stepcounter_enable = enable;
		break;
	case BMI2XY_ANY_MOTION_SENSOR:
		config[0].type = BMI2_ANY_MOTION;
		sens_list = BMI2_ANY_MOTION;
		sens_int.type = BMI2_ANY_MOTION;
		ret = bmi270_get_sensor_config(&config[0], 1,
						&client_data->device);

		if (enable) {
			/* Enable the x, y and z axis of anymotion */
			config[0].cfg.any_motion.select_x = BMI2_ENABLE;
			config[0].cfg.any_motion.select_y = BMI2_ENABLE;
			config[0].cfg.any_motion.select_z = BMI2_ENABLE;

		} else {
			/* Disable the x, y and z axis of anymotion */
			config[0].cfg.any_motion.select_x = BMI2_DISABLE;
			config[0].cfg.any_motion.select_y = BMI2_DISABLE;
			config[0].cfg.any_motion.select_z = BMI2_DISABLE;
		}
		ret += bmi270_set_sensor_config(&config[0], 1, &client_data->device);
		if (ret == 0)
			client_data->anymotion_enable = enable;
		break;
	case BMI2XY_ACTIVITY_SENSOR:
		sens_list = BMI2_STEP_ACTIVITY;
		sens_int.type = BMI2_STEP_ACTIVITY;
		client_data->activity_enable = enable;
		break;
	case BMI2XY_WRIST_WAKEUP_SENSOR:
		sens_list = BMI2_WRIST_WEAR_WAKE_UP;
		sens_int.type = BMI2_WRIST_WEAR_WAKE_UP;
		client_data->wrist_wakeup_enable = enable;
		break;
	case BMI2XY_WRIST_GESTURE_SENSOR:
		if (enable) {
			config[0].type = BMI2_WRIST_GESTURE;
			ret = bmi270_get_sensor_config(&config[0], 1,
							&client_data->device);
			config[0].cfg.wrist_gest.wearable_arm = BMI2_ARM_LEFT;
			ret += bmi270_set_sensor_config(&config[0], 1,
														&client_data->device);
		}
		sens_list = BMI2_WRIST_GESTURE;
		sens_int.type = BMI2_WRIST_GESTURE;
		client_data->wrist_gesture_enable = enable;
		break;

	default:
		PERR("Invalid sensor handle: %d", config_func);
		mutex_unlock(&client_data->lock);
		return -EINVAL;
	}
	if (enable == 1) {
		sens_int.hw_int_pin = BMI2_INT1;
		ret = bmi270_sensor_enable(&sens_list, 1, &client_data->device);
		ret = bmi270_map_feat_int(&sens_int, 1, &client_data->device);
	}
	if (enable == 0) {
		sens_int.hw_int_pin = BMI2_INT_NONE;
		ret = bmi270_sensor_disable(&sens_list, 1, &client_data->device);
		ret = bmi270_map_feat_int(&sens_int, 1, &client_data->device);
	}
	mutex_unlock(&client_data->lock);
	bmi2xy_check_error("config feature", ret);

	return count;
}

/**
 * feat_page_sel_show - sysfs read callback which shows the feature
 * address being selected.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t feat_page_sel_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);

	return scnprintf(buf, 64, "reg=0X%02X, len=%d\n",
		client_data->feat_page_sel, client_data->feat_page_len);
}

/**
 * feat_page_sel_store - sysfs write callback which stores the feature
 * address to be used for reading the data.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t feat_page_sel_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);
	int err;

	mutex_lock(&client_data->lock);
	err = sscanf(buf, "%11X %11d",
		&client_data->feat_page_sel, &client_data->feat_page_len);
	mutex_unlock(&client_data->lock);
	if (err != 2) {
		PERR("Invalid argument");
		return -EINVAL;
	}
	return count;
}

/**
 * feat_page_val_show - sysfs read callback which provides the feature
 * data which is read from the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t feat_page_val_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);
	int err = 0;
	u8 reg_data[128];
	int pos, i;
	u8 page;

	if (client_data->feat_page_len == 0) {
		PERR("error feat_page_sel len is 0");
		return -EINVAL;
	}
	mutex_lock(&client_data->lock);
	page = (u8)client_data->feat_page_sel;

	err = bmi2_set_regs(BMI2_FEAT_PAGE_ADDR, &page, 1,
							&client_data->device);
	/* Get the configuration from the page */
	err += bmi2_get_regs(BMI2_FEATURES_REG_ADDR,
		reg_data, BMI2_FEAT_SIZE_IN_BYTES, &client_data->device);
	bmi2xy_check_error("get register", err);

	pos = 0;
	for (i = 0; i < client_data->feat_page_len; ++i) {
		pos += scnprintf(buf + pos, 16, "%02X", reg_data[i]);
		buf[pos++] = (i + 1) % 16 == 0 ? '\n' : ' ';
	}
	mutex_unlock(&client_data->lock);
	if (buf[pos - 1] == ' ')
		buf[pos - 1] = '\n';
	return pos;
}

/**
 * feat_page_val_store - sysfs write callback which stores the feature
 * data which is to be written in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t feat_page_val_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);
	int err;
	u8 aps_status = 0;
	u8 reg_data[128] = {0,};
	int i, j, status, digit;
	u8 page;

	if (client_data->feat_page_len == 0) {
		PERR("error feat_page_sel len is 0");
		return -EINVAL;
	}
	mutex_lock(&client_data->lock);
	/* Get the status of advance power save mode */
	err = bmi2_get_adv_power_save(&aps_status, &client_data->device);
	if (aps_status) {
		/* Disable advance power save mode */
		err = bmi2_set_adv_power_save(BMI2_DISABLE,
							&client_data->device);
	}
	page = (u8)client_data->feat_page_sel;

	status = 0;
	/* Lint -save -e574 */
	for (i = j = 0; i < count && j < client_data->feat_page_len; ++i) {
		/* Lint -restore */
		if (buf[i] == ' ' || buf[i] == '\n' || buf[i] == '\t' ||
			buf[i] == '\r') {
			status = 0;
			++j;
			continue;
		}
		digit = buf[i] & 0x10 ? (buf[i] & 0xF) : ((buf[i] & 0xF) + 9);
		PDEBUG("digit is %d", digit);
		switch (status) {
		case 2:
			++j; /* Fall thru */
		case 0:
			reg_data[j] = digit;
			status = 1;
			break;
		case 1:
			reg_data[j] = reg_data[j] * 16 + digit;
			status = 2;
			break;
		}
	}
	if (status > 0)
		++j;
	if (j > client_data->feat_page_len)
		j = client_data->feat_page_len;
	else if (j < client_data->feat_page_len) {
		PERR("Invalid argument");
		mutex_unlock(&client_data->lock);
		return -EINVAL;
	}
	PDEBUG("Reg data read as");
	for (i = 0; i < j; ++i)
		PDEBUG("%d", reg_data[i]);
	/* Switch page */
	err = bmi2_set_regs(BMI2_FEAT_PAGE_ADDR, &page, 1,
							&client_data->device);
	/* Set the configuration back to the page */
	err += bmi2_set_regs(BMI2_FEATURES_REG_ADDR,
	reg_data, BMI2_FEAT_SIZE_IN_BYTES, &client_data->device);
	bmi2xy_check_error("set register data", err);

	if (aps_status) {
		/* Disable advance power save mode */
		err = bmi2_set_adv_power_save(BMI2_ENABLE,
							&client_data->device);
	}
	mutex_unlock(&client_data->lock);
	bmi2xy_check_error("set aps mode", err);

	return count;
}

/**
 * step_counter_val_show - sysfs read callback which reads and provide
 * output value of step-counter sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t step_counter_val_show(struct device *dev,
					 struct device_attribute *attr, char *buf)
{
	int err = 0;
	u32 step_counter_val = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);
	struct bmi2_feat_sensor_data step_counter_data;

	step_counter_data.type = BMI2_STEP_COUNTER;
	mutex_lock(&client_data->lock);
	err = bmi270_get_feature_data(&step_counter_data, 1, &client_data->device);
	mutex_unlock(&client_data->lock);
	bmi2xy_check_error("get feature data", err);

	step_counter_val = step_counter_data.sens_data.step_counter_output;
	PDEBUG("val %u", step_counter_val);
	mutex_lock(&client_data->lock);
	if (client_data->err_int_trigger_num == 0) {
		client_data->step_counter_val = step_counter_val;
		PDEBUG("report val %u", client_data->step_counter_val);
		err = scnprintf(buf, 96, "%u\n", client_data->step_counter_val);
		client_data->step_counter_temp = client_data->step_counter_val;
	} else {
		PDEBUG("after err report val %u",
			client_data->step_counter_val + step_counter_val);
		err = scnprintf(buf, 96, "%u\n",
			client_data->step_counter_val + step_counter_val);
		client_data->step_counter_temp =
			client_data->step_counter_val + step_counter_val;
	}
	mutex_unlock(&client_data->lock);
	return err;
}

/**
 * step_counter_watermark_show - sysfs read callback which reads and
 * provide the watermark level of step-counter sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t step_counter_watermark_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	int err = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);
	struct bmi2_sens_config config;

	mutex_lock(&client_data->lock);
	config.type = BMI2_STEP_COUNTER;
	err = bmi270_get_sensor_config(&config, 1, &client_data->device);
	mutex_unlock(&client_data->lock);
	bmi2xy_check_error("get sensor config", err);

	return scnprintf(buf, 32, "%d\n",
				config.cfg.step_counter.watermark_level);
}

/**
 * step_counter_watermark_store - sysfs write callback which stores the
 * watermark level of step-counter in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t step_counter_watermark_store(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	int err = 0;
	unsigned long step_watermark;
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);
	struct bmi2_sens_config config;

	err = kstrtoul(buf, 10, &step_watermark);
	bmi2xy_check_error("get user option", err);

	PDEBUG("watermark step_counter %ld", step_watermark);
	mutex_lock(&client_data->lock);
	config.type = BMI2_STEP_COUNTER;
	err = bmi270_get_sensor_config(&config, 1, &client_data->device);
	config.cfg.step_counter.watermark_level = (u16)step_watermark;
	err += bmi270_set_sensor_config(&config, 1, &client_data->device);
	mutex_unlock(&client_data->lock);
	bmi2xy_check_error("get/set sensor config", err);

	return count;
}

/**
 * step_counter_reset_store - sysfs write callback which resets the
 * step-counter value in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t step_counter_reset_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int err = 0;
	unsigned long reset_counter;
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);
	struct bmi2_sens_config config;

	err = kstrtoul(buf, 10, &reset_counter);
	bmi2xy_check_error("get user option", err);

	PDEBUG("reset_counter %ld", reset_counter);
	mutex_lock(&client_data->lock);
	config.type = BMI2_STEP_COUNTER;
	err = bmi270_get_sensor_config(&config, 1, &client_data->device);
	config.cfg.step_counter.reset_counter = (u16)reset_counter;
	err += bmi270_set_sensor_config(&config, 1, &client_data->device);
	bmi2xy_check_error("get/set sensor config", err);

	client_data->step_counter_val = 0;
	client_data->step_counter_temp = 0;
	mutex_unlock(&client_data->lock);
	return count;
}

/**
 * any_motion_config_show - sysfs read callback which reads the
 * any-motion configuration from the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t any_motion_config_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);
	struct bmi2_sens_config config;

	mutex_lock(&client_data->lock);
	config.type = BMI2_ANY_MOTION;
	err = bmi270_get_sensor_config(&config, 1, &client_data->device);
	mutex_unlock(&client_data->lock);
	bmi2xy_check_error("get sensor config", err);

	return scnprintf(buf, PAGE_SIZE, "duration =0x%x threshold= 0x%x\n",
					config.cfg.any_motion.duration,
					config.cfg.any_motion.threshold);

}

/**
 * any_motion_config_store - sysfs write callback which writes the
 * any-motion configuration in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t any_motion_config_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	int err = 0;
	unsigned int data[2] = {0};
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);
	struct bmi2_sens_config config;

	err = sscanf(buf, "%11x %11x", &data[0], &data[1]);
	if (err != 2) {
		PERR("Invalid argument");
		return -EINVAL;
	}
	mutex_lock(&client_data->lock);
	config.type = BMI2_ANY_MOTION;
	err = bmi270_get_sensor_config(&config, 1, &client_data->device);
	config.cfg.any_motion.duration = (u16)data[0];
	config.cfg.any_motion.threshold = (u16)data[1];
	err += bmi270_set_sensor_config(&config, 1, &client_data->device);
	mutex_unlock(&client_data->lock);
	bmi2xy_check_error("get/set sensor config", err);

	return count;
}

/**
 * any_motion_axis_select_store - sysfs write callback which enable or
 * disable the x,y and z axis of any-motion sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t any_motion_axis_select_store(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	int err = 0;
	unsigned int data[3] = {0};
	struct bmi2_sens_config sens;
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);

	err = sscanf(buf, "%11x %11x %11x", &data[0], &data[1], &data[2]);
	if (err != 3) {
		PERR("Invalid argument");
		return -EINVAL;
	}
	/* Configure the axis for any/no-motion */
	mutex_lock(&client_data->lock);
	sens.type = BMI2_ANY_MOTION;

	err = bmi270_get_sensor_config(&sens, 1, &client_data->device);
	sens.cfg.any_motion.select_x = data[0] & 0x01;
	sens.cfg.any_motion.select_y = data[1] & 0x01;
	sens.cfg.any_motion.select_z = data[2] & 0x01;
	err = bmi270_set_sensor_config(&sens, 1, &client_data->device);
	bmi2xy_check_error("get/set asensor config failed", err);

	/* If user disabled all axis,then anymotion/no-motion is disabled
	 * For re-enabling anymotion/nomotion, config_function sysnode or
	 * any_no_motion_axis_select(with atleast 1 axis enabled) can be used.
	 */
	if ((data[0] == 0) && (data[1] == 0) && (data[2] == 0))
		client_data->anymotion_enable = 0;

	mutex_unlock(&client_data->lock);

	return count;
}

/**
 * wrist_gest_config_show - sysfs read callback which shows values for the
 * list of configurations for wrist gesture feature.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t wrist_gest_config_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);
	struct bmi2_sens_config config;

	mutex_lock(&client_data->lock);
	config.type = BMI2_WRIST_GESTURE;
	err = bmi270_get_sensor_config(&config, 1, &client_data->device);
	mutex_unlock(&client_data->lock);
	bmi2xy_check_error("get sensor config", err);

	return scnprintf(buf, PAGE_SIZE,
		"wearable_arm0=0x%x\nmin_flick_peak1=0x%x\nmin_flick_samples2=0x%x\n"
		"max_duration3=0x%x\n",
		config.cfg.wrist_gest.wearable_arm,
		config.cfg.wrist_gest.min_flick_peak,
		config.cfg.wrist_gest.min_flick_samples,
		config.cfg.wrist_gest.max_duration);
}

/**
 * wrist_gest_config_store - sysfs write callback which set wrist gesture
 * configurations.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t wrist_gest_config_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	int err = 0;
	unsigned int data[4] = {0};
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);
	struct bmi2_sens_config config;

	if (client_data == NULL) {
		PERR("Invalid client_data pointer");
		return -ENODEV;
	}
	err = sscanf(buf, "%11x %11x %11x %11x", &data[0], &data[1], &data[2],
																	&data[3]);
	if (err != 4) {
		PERR("Invalid argument");
		return -EINVAL;
	}

	mutex_lock(&client_data->lock);
	config.type = BMI2_WRIST_GESTURE;
	err = bmi270_get_sensor_config(&config, 1, &client_data->device);
	config.cfg.wrist_gest.wearable_arm = (u16)data[0];
	config.cfg.wrist_gest.min_flick_peak = (u16)data[1];
	config.cfg.wrist_gest.min_flick_samples = (u16)data[2];
	config.cfg.wrist_gest.max_duration = (u16)data[3];

	err += bmi270_set_sensor_config(&config, 1, &client_data->device);
	mutex_unlock(&client_data->lock);
	bmi2xy_check_error("set sensor config", err);

	return count;
}

/**
 * wrist_wakeup_config_show - sysfs read callback which gives the list of
 * configurations for wrist wakeup feature.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t wrist_wakeup_config_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);
	struct bmi2_sens_config config;

	mutex_lock(&client_data->lock);
	config.type = BMI2_WRIST_WEAR_WAKE_UP;
	err = bmi270_get_sensor_config(&config, 1, &client_data->device);
	mutex_unlock(&client_data->lock);
	bmi2xy_check_error("get sensor config", err);
	return scnprintf(buf, PAGE_SIZE,
		"min_angle_focus0=0x%x\nmin_angle_nonfocus1=0x%x\nmax_tilt_lr2=0x%x\n"
		"max_tilt_ll3=0x%x\nmax_tilt_pd4=0x%x\nmax_tilt_pu5=0x%x\n",
		config.cfg.wrist_wear_wake_up.min_angle_focus,
		config.cfg.wrist_wear_wake_up.min_angle_nonfocus,
		config.cfg.wrist_wear_wake_up.max_tilt_lr,
		config.cfg.wrist_wear_wake_up.max_tilt_ll,
		config.cfg.wrist_wear_wake_up.max_tilt_pd,
		config.cfg.wrist_wear_wake_up.max_tilt_pu);
}

/**
 * wrist_wakeup_config_store - sysfs write callback which set wrist wakeup
 * configurations.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t wrist_wakeup_config_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	int err = 0;
	unsigned int data[6] = {0};
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);
	struct bmi2_sens_config config;

	err = sscanf(buf, "%11x %11x %11x %11x %11x %11x", &data[0], &data[1],
		&data[2], &data[3], &data[4], &data[5]);
	if (err != 6) {
		PERR("Invalid argument");
		return -EINVAL;
	}

	mutex_lock(&client_data->lock);
	config.type = BMI2_WRIST_WEAR_WAKE_UP;
	err = bmi270_get_sensor_config(&config, 1, &client_data->device);
	config.cfg.wrist_wear_wake_up.min_angle_focus = (u16) data[0];
	config.cfg.wrist_wear_wake_up.min_angle_nonfocus = (u16) data[1];
	config.cfg.wrist_wear_wake_up.max_tilt_lr = (u16)data[2];
	config.cfg.wrist_wear_wake_up.max_tilt_ll = (u16)data[3];
	config.cfg.wrist_wear_wake_up.max_tilt_pd = (u16)data[4];
	config.cfg.wrist_wear_wake_up.max_tilt_pu = (u16)data[5];
	err += bmi270_set_sensor_config(&config, 1, &client_data->device);
	mutex_unlock(&client_data->lock);
	bmi2xy_check_error("get sensor config", err);
	return count;
}

/**
 * nomotion_config_show - sysfs read callback which reads the
 * no-motion configuration from the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as output.
 *
 * Return: Number of characters returned.
 */
static ssize_t no_motion_config_show(struct device *dev,
					 struct device_attribute *attr, char *buf)
{
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);
	struct bmi2_sens_config config;

	mutex_lock(&client_data->lock);
	config.type = BMI2_NO_MOTION;
	err = bmi270_get_sensor_config(&config, 1, &client_data->device);
	mutex_unlock(&client_data->lock);
	bmi2xy_check_error("get sensor config", err);
	return scnprintf(buf, PAGE_SIZE, "duration =0x%x threshold= 0x%x\n",
						config.cfg.no_motion.duration,
						config.cfg.no_motion.threshold);

}

/**
 * no_motion_config_store - sysfs write callback which writes the
 * no-motion configuration in the sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t no_motion_config_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	int err = 0;
	unsigned int data[2] = {0};
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);
	struct bmi2_sens_config config;

	err = sscanf(buf, "%11x %11x", &data[0], &data[1]);
	if (err != 2) {
		PERR("Invalid argument");
		return -EINVAL;
	}
	mutex_lock(&client_data->lock);
	config.type = BMI2_NO_MOTION;
	err = bmi270_get_sensor_config(&config, 1, &client_data->device);
	config.cfg.no_motion.duration = (u16)data[0];
	config.cfg.no_motion.threshold = (u16)data[1];
	err += bmi270_set_sensor_config(&config, 1, &client_data->device);
	mutex_unlock(&client_data->lock);
	bmi2xy_check_error("set sensor config", err);
	return count;
}

/**
 * no_motion_axis_select_store - sysfs write callback which enable or
 * disable the x,y and z axis of no-motion sensor.
 *
 * @dev: Device instance
 * @attr: Instance of device attribute file
 * @buf: Instance of the data buffer which serves as input.
 * @count: Number of characters in the buffer `buf`.
 *
 * Return: Number of characters used from buffer `buf`, which equals count.
 */
static ssize_t no_motion_axis_select_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	int err = 0;
	unsigned int data[3] = {0};
	struct bmi2_sens_config sens;
	struct input_dev *input = to_input_dev(dev);
	struct bmi2xy_client_data *client_data = input_get_drvdata(input);

	err = sscanf(buf, "%11x %11x %11x", &data[0], &data[1], &data[2]);
	if (err != 3) {
		PERR("Invalid argument");
		return -EINVAL;
	}
	/* Configure the axis for any/no-motion */
	mutex_lock(&client_data->lock);

	sens.type = BMI2_NO_MOTION;
	err = bmi270_get_sensor_config(&sens, 1, &client_data->device);
	sens.cfg.no_motion.select_x = data[0] & 0x01;
	sens.cfg.no_motion.select_y = data[1] & 0x01;
	sens.cfg.no_motion.select_z = data[2] & 0x01;

	err = bmi270_set_sensor_config(&sens, 1, &client_data->device);
	bmi2xy_check_error("set sensor config", err);
	/* If user disabled all axis,then anymotion/no-motion is disabled
	 * For re-enabling anymotion/nomotion, config_function sysnode or
	 * any_no_motion_axis_select(with atleast 1 axis enabled) can be used.
	 */
	if ((data[0] == 0) && (data[1] == 0) && (data[2] == 0))
		client_data->nomotion_enable = 0;

	mutex_unlock(&client_data->lock);

	return count;
}

static DEVICE_ATTR_RO(chip_id);
static DEVICE_ATTR_RW(acc_enable);
static DEVICE_ATTR_RO(acc_value);
static DEVICE_ATTR_RW(acc_range);
static DEVICE_ATTR_RW(acc_odr);
static DEVICE_ATTR_RW(gyr_enable);
static DEVICE_ATTR_RO(gyr_value);
static DEVICE_ATTR_RW(gyr_range);
static DEVICE_ATTR_RW(gyr_odr);
static DEVICE_ATTR_RW(acc_selftest);
static DEVICE_ATTR_RO(avail_sensor);
static DEVICE_ATTR_RW(load_config_stream);
static DEVICE_ATTR_RW(aps_enable);
static DEVICE_ATTR_RW(reg_sel);
static DEVICE_ATTR_RW(reg_val);
static DEVICE_ATTR_RO(driver_version);
static DEVICE_ATTR_RW(acc_foc);
static DEVICE_ATTR_RO(gyr_foc);
static DEVICE_ATTR_WO(fifo_flush);
static DEVICE_ATTR_RO(fifo_data_frame);
static DEVICE_ATTR_RW(acc_fifo_enable);
static DEVICE_ATTR_RW(gyr_fifo_enable);
static DEVICE_ATTR_WO(soft_reset);
static DEVICE_ATTR_RW(feat_page_sel);
static DEVICE_ATTR_RW(feat_page_val);
static DEVICE_ATTR_RW(config_function);
static DEVICE_ATTR_RO(nvm_prog);
static DEVICE_ATTR_RW(gyr_selftest);
static DEVICE_ATTR_RW(gyr_offset_comp);
static DEVICE_ATTR_RW(any_motion_config);
static DEVICE_ATTR_WO(any_motion_axis_select);
static DEVICE_ATTR_RO(step_counter_val);
static DEVICE_ATTR_RW(step_counter_watermark);
static DEVICE_ATTR_WO(step_counter_reset);
static DEVICE_ATTR_RO(gyr_usr_gain);
static DEVICE_ATTR_WO(execute_crt);
static DEVICE_ATTR_RW(no_motion_config);
static DEVICE_ATTR_WO(no_motion_axis_select);
static DEVICE_ATTR_RW(wrist_gest_config);
static DEVICE_ATTR_RW(wrist_wakeup_config);

static struct attribute *bmi2xy_attributes[] = {
	&dev_attr_chip_id.attr,
	&dev_attr_acc_enable.attr,
	&dev_attr_acc_value.attr,
	&dev_attr_acc_range.attr,
	&dev_attr_acc_odr.attr,
	&dev_attr_acc_fifo_enable.attr,
	&dev_attr_gyr_fifo_enable.attr,
	&dev_attr_gyr_enable.attr,
	&dev_attr_gyr_value.attr,
	&dev_attr_gyr_range.attr,
	&dev_attr_gyr_odr.attr,
	&dev_attr_acc_selftest.attr,
	&dev_attr_avail_sensor.attr,
	&dev_attr_driver_version.attr,
	&dev_attr_load_config_stream.attr,
	&dev_attr_aps_enable.attr,
	&dev_attr_reg_sel.attr,
	&dev_attr_reg_val.attr,
	&dev_attr_fifo_flush.attr,
	&dev_attr_fifo_data_frame.attr,
	&dev_attr_acc_foc.attr,
	&dev_attr_gyr_foc.attr,
	&dev_attr_soft_reset.attr,
	&dev_attr_feat_page_sel.attr,
	&dev_attr_feat_page_val.attr,
	&dev_attr_config_function.attr,
	&dev_attr_nvm_prog.attr,
	&dev_attr_gyr_selftest.attr,
	&dev_attr_gyr_offset_comp.attr,
	&dev_attr_step_counter_val.attr,
	&dev_attr_step_counter_watermark.attr,
	&dev_attr_step_counter_reset.attr,
	&dev_attr_any_motion_config.attr,
	&dev_attr_any_motion_axis_select.attr,
	&dev_attr_gyr_usr_gain.attr,
	&dev_attr_execute_crt.attr,
	&dev_attr_no_motion_config.attr,
	&dev_attr_no_motion_axis_select.attr,
	&dev_attr_wrist_gest_config.attr,
	&dev_attr_wrist_wakeup_config.attr,
	NULL
};

static struct attribute_group bmi2xy_attribute_group = {
	.attrs = bmi2xy_attributes
};

#if defined(BMI2XY_ENABLE_INT1) || defined(BMI2XY_ENABLE_INT2)
/**
 *	bmi2xy_feat_function_handle - Reports the interrupt events to HAL.
 *	@client_data : Pointer to client data structure.
 *	@status : Interrupt status information.
 */
static void bmi2xy_feat_function_handle(struct bmi2xy_client_data *client_data,
					u32 status)
{
	input_event(client_data->feat_input, EV_MSC, REL_FEAT_STATUS,
		status);
	input_sync(client_data->feat_input);
}

/**
 *	bmi2xy_irq_work_func - Bottom half handler for feature interrupts.
 *	@work : Work data for the workqueue handler.
 */
static void bmi2xy_irq_work_func(struct work_struct *work)
{
	struct bmi2xy_client_data *client_data = container_of(work,
		struct bmi2xy_client_data, irq_work);
	u8 int_status[2] = {0, 0};
	int err = 0;
	int in_suspend_copy;
	u32 status;
#ifdef ANY_NO_MOTION_WORKAROUND
	u8 feature = 0;
#endif

	in_suspend_copy = atomic_read(&client_data->in_suspend);
	err = bmi2_get_regs(BMI2_INT_STATUS_0_ADDR, int_status, 2,
							&client_data->device);

	bmi2xy_check_error("get INT status register", err);
	PDEBUG("int_status0 = 0x%x int_status1 =0x%x",
		int_status[0], int_status[1]);
	if (in_suspend_copy &&
		((int_status[0] & STEP_DET_OUT) == 0x02)) {
		return;
	}

	if (int_status[0]) {
		mutex_lock(&client_data->lock);

		if (int_status[0] & BMI270_SIG_MOT_STATUS_MASK)
			PDEBUG("Sig Motion Interrupt occurred");
		if (int_status[0] & BMI270_STEP_CNT_STATUS_MASK) {
			struct bmi2_feat_sensor_data sensor_data = { 0 };

			PDEBUG("step counter Interrupt occurred");
			sensor_data.type = BMI2_STEP_COUNTER;
			err = bmi270_get_feature_data(&sensor_data, 1,
												&client_data->device);
			bmi2xy_check_error("get feature data", err);
			PDEBUG("Steps counted : %u\n",
				sensor_data.sens_data.step_counter_output);
		}
		if (int_status[0] & BMI270_STEP_ACT_STATUS_MASK) {
			struct bmi2_feat_sensor_data sensor_data = { 0 };
			const char *activity_output[4] = { "BMI2_STILL", "BMI2_WALK",
												"BMI2_RUN", "BMI2_UNKNOWN" };

			PDEBUG("Step Activity Interrupt Occurred");
			sensor_data.type = BMI2_STEP_ACTIVITY;
			err = bmi270_get_feature_data(&sensor_data, 1,
												&client_data->device);
			bmi2xy_check_error("get feature data", err);
			PDEBUG("Step activity : %s\n",
						activity_output[sensor_data.sens_data.activity_output]);
		}
		if (int_status[0] & BMI270_WRIST_WAKE_UP_STATUS_MASK)
			PDEBUG("wrist wake up interrupt occurred");
		if (int_status[0] & BMI270_WRIST_GEST_STATUS_MASK) {
			struct bmi2_feat_sensor_data sens_data = { 0 };
			const char *gesture_output[6] = { "unknown_gesture",
							"push_arm_down", "pivot_up", "wrist_shake_jiggle",
							"flick_in", "flick_out" };

			sens_data.type = BMI2_WRIST_GESTURE;
			err = bmi270_get_feature_data(&sens_data, 1, &client_data->device);
			PDEBUG("Wrist gesture = %d\r\n",
								sens_data.sens_data.wrist_gesture_output);
			PDEBUG("Gesture output = %s\n",
				gesture_output[sens_data.sens_data.wrist_gesture_output]);
		}
#ifdef ANY_NO_MOTION_WORKAROUND
		if ((int_status[0] & BMI270_ANY_MOT_STATUS_MASK) &&
					(client_data->anymotion_enable == 1)) {
			feature = BMI2_ANY_MOTION;
			err = bmi270_sensor_disable(&feature, 1,
							&client_data->device);
			if (err == 0)
				client_data->anymotion_enable = 0;
		} else {
			/*lint -e502*/
			int_status[0] &= ~BMI270_ANY_MOT_STATUS_MASK;
			/*lint +e502*/
		}

		if ((int_status[0] & BMI270_NO_MOT_STATUS_MASK) &&
					(client_data->nomotion_enable == 1)) {
			feature = BMI2_NO_MOTION;
			err = bmi270_sensor_disable(&feature, 1,
							&client_data->device);
			if (err == 0)
				client_data->nomotion_enable = 0;
		} else {
			/*lint -e502*/
			int_status[0] &= ~BMI270_NO_MOT_STATUS_MASK;
			/*lint +e502*/
		}
#endif
		status = int_status[0] | (int_status[1] << 8);
		bmi2xy_feat_function_handle(client_data, status);
		mutex_unlock(&client_data->lock);
	}
}

/**
 * bmi2xy_irq_handle - IRQ handler function.
 * @irq : Number of irq line.
 * @handle : Instance of client data.
 *
 * Return : Status of IRQ function.
 */
static irqreturn_t bmi2xy_irq_handle(int irq, void *handle)
{
	struct bmi2xy_client_data *client_data = handle;
	int in_suspend_copy;
	int err;

	PERR("In IRQ handle");
	in_suspend_copy = atomic_read(&client_data->in_suspend);
	if ((in_suspend_copy == 1) &&
		(client_data->sigmotion_enable == 1)) {
		err = schedule_work(&client_data->irq_work);
	} else
		err = schedule_work(&client_data->irq_work);

	bmi2xy_check_error("irq work schedule", err);

	return IRQ_HANDLED;
}

/**
 * bmi2xy_request_irq - Allocates interrupt resources and enables the
 * interrupt line and IRQ handling.
 *
 * @client_data: Instance of the client data.
 *
 * Return : Status of the function.
 * * 0 - OK
 * * Any Negative value - Error.
 */
static int bmi2xy_request_irq(struct bmi2xy_client_data *client_data)
{
	int err = 0;

	PDEBUG("Interrupt mapped to GPIO Pin: %d\n", client_data->IRQ);
	err = request_irq(client_data->IRQ, bmi2xy_irq_handle,
			IRQF_TRIGGER_RISING,
			SENSOR_NAME, client_data);
	bmi2xy_check_error("request IRQ", err);
	INIT_WORK(&client_data->irq_work, bmi2xy_irq_work_func);
	return err;
}
#endif

/**
 * bmi2xy_acc_input_init - Register the accelerometer input device in the
 * system.
 * @client_data : Instance of client data.
 *
 * Return : Status of the function.
 * * 0 - OK
 * * Any Negative value - Error.
 */
static int bmi2xy_acc_input_init(struct bmi2xy_client_data *client_data)
{
	struct input_dev *dev;
	int err = 0;

	dev = input_allocate_device();
	if (dev == NULL)
		return -ENOMEM;

	dev->name = SENSOR_NAME;
	dev->id.bustype = BUS_SPI;
	input_set_capability(dev, EV_ABS, ABS_MISC);
	input_set_capability(dev, EV_MSC, REL_HW_STATUS);
	input_set_drvdata(dev, client_data);
	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		return -EIO;
	}
	client_data->acc_input = dev;
	return 0;
}

/**
 * bmi2xy_acc_input_destroy - Un-register the Accelerometer input device from
 * the system.
 *
 * @client_data :Instance of client data.
 */
static void bmi2xy_acc_input_destroy(struct bmi2xy_client_data *client_data)
{
	struct input_dev *dev = client_data->acc_input;

	input_unregister_device(dev);
}

/**
 * bmi2xy_feat_input_init - Register the feature input device in the
 * system.
 * @client_data : Instance of client data.
 *
 * Return : Status of the function.
 * * 0 - OK.
 * * Any negative value - Error.
 */
static int bmi2xy_feat_input_init(struct bmi2xy_client_data *client_data)
{
	struct input_dev *dev;
	int err = 0;

	dev = input_allocate_device();
	if (dev == NULL)
		return -ENOMEM;
	dev->name = SENSOR_NAME_FEAT;
	dev->id.bustype = BUS_SPI;

	input_set_capability(dev, EV_MSC, REL_FEAT_STATUS);
	input_set_drvdata(dev, client_data);
	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		return -EIO;
	}
	client_data->feat_input = dev;
	return 0;
}

/**
 * bmi2xy_feat_input_destroy - Un-register the feature input device from the
 * system.
 * @client_data : Instance of client data.
 */
static void bmi2xy_feat_input_destroy(struct bmi2xy_client_data *client_data)
{
	struct input_dev *dev = client_data->acc_input;

	input_unregister_device(dev);
}


int bmi2xy_probe(struct bmi2xy_client_data *client_data, struct device *dev)
{
	int err = 0;

	PINFO("bmi2xy sensor probe");

	dev_set_drvdata(dev, client_data);

	client_data->dev = dev;
	client_data->device.delay_us = bmi2xy_i2c_delay_us;
	/*lint -e86*/
	mutex_init(&client_data->lock);
	/*lint +e86*/
	/* Accel input device init */
	err = bmi2xy_acc_input_init(client_data);
	/* Sysfs node creation */
	err += sysfs_create_group(&client_data->acc_input->dev.kobj,
			&bmi2xy_attribute_group);
	err += bmi2xy_feat_input_init(client_data);
	bmi2xy_i2c_delay_us(MS_TO_US(10), &client_data->device.intf_ptr);
	/* Request irq and config*/
	#if defined(BMI2XY_ENABLE_INT1) || defined(BMI2XY_ENABLE_INT2)
	err += bmi2xy_request_irq(client_data);
	#endif

	#ifdef BMI2XY_LOAD_CONFIG_FILE_IN_INIT
	err += bmi2xy_update_config_stream(client_data, 1);

	client_data->config_file_loaded = 1;
	#endif
	/* Set the self test status */
	client_data->acc_selftest = SELF_TEST_NOT_RUN;
	client_data->gyr_selftest = SELF_TEST_NOT_RUN;
	bmi2xy_check_error("sensor probe", err);
	if (err) {
		kfree(client_data);
		return -EIO;
	}
	PINFO("sensor %s probed successfully", SENSOR_NAME);
	return 0;
}

/**
 * bmi2xy_remove - This function removes the driver from the device.
 * @dev : Instance of the device.
 *
 * Return : Status of the suspend function.
 * * 0 - OK.
 * * Negative value : Error.
 */
int bmi2xy_remove(struct device *dev)
{
	int err = 0;
	struct bmi2xy_client_data *client_data = dev_get_drvdata(dev);

	if (client_data != NULL) {
		bmi2xy_i2c_delay_us(MS_TO_US(BMI2XY_I2C_WRITE_DELAY_TIME),
						&client_data->device.intf_ptr);
		sysfs_remove_group(&client_data->acc_input->dev.kobj,
				&bmi2xy_attribute_group);
		bmi2xy_acc_input_destroy(client_data);
		bmi2xy_feat_input_destroy(client_data);
		kfree(client_data);
	}
	return err;
}
/* Lint -save -e19 */
EXPORT_SYMBOL(bmi2xy_remove);
/* Lint -restore */
