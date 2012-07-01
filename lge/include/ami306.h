/* include/linux/ami306.h - AMI306 compass driver
 *
 * Copyright (C) 2009 AMIT Technology Inc.
 * Author: Kyle Chen <sw-support@amit-inc.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*
 * Definitions for ami306 compass chip.
 */
#ifndef AMI306_H
#define AMI306_H

#include <linux/ioctl.h>
//#include <asm-arm/arch/regs-gpio.h>

#define AMI306_I2C_ADDRESS 			0x0E  //new Addr=0x0E(Low), old Addr=0x0F(High)

#define STM_GYRO		//20110630 for STM Gyro sensor S/W support

/* AMI306 Internal Register Address  (Please refer to AMI306 Specifications) */
#define AMI306_REG_CTRL1			0x1B
#define AMI306_REG_CTRL2			0x1C
#define AMI306_REG_CTRL3			0x1D
#define AMI306_REG_CTRL4			0x5C
#define AMI306_REG_DATAXH			0x10
#define AMI306_REG_DATAXL			0x11
#define AMI306_REG_DATAYH			0x12
#define AMI306_REG_DATAYL			0x13
#define AMI306_REG_DATAZH			0x14
#define AMI306_REG_DATAZL			0x15
#define AMI306_REG_WIA				0x0F

/* AMI306 Control Bit  (Please refer to AMI306 Specifications) */
#define AMI306_CTRL1_PC1			0x80
#define AMI306_CTRL1_FS1_NORMAL			0x00 //Normal
#define AMI306_CTRL1_FS1_FORCE			0x02 //Force
#define AMI306_CTRL1_ODR1			0x10 //20SPS(20HZ)
#define AMI306_CTRL2_DREN			0x08
#define AMI306_CTRL2_DRP			0x04
#define AMI306_CTRL3_NOFORCE_BIT		0x00
#define AMI306_CTRL3_FORCE_BIT			0x40
#define AMI306_CTRL3_B0_LO_CLR			0x00
#define AMI306_CTRL4_COMPASS_MODE	0x00
#define AMI306_CTRL4_HIGHSPEED_MODE 0x80
#define AMI304_WIA_VALUE			0x47
#define AMI306_WIA_VALUE			0x46

/* IOCTLs for ami306 misc. device library */
#define AMI306IO			   0x83
#define AMI306_IOCTL_INIT                  _IO(AMI306IO, 0x01)
#define AMI306_IOCTL_READ_CHIPINFO         _IOR(AMI306IO, 0x02, int)
#define AMI306_IOCTL_READ_SENSORDATA       _IOR(AMI306IO, 0x03, int)
#define AMI306_IOCTL_READ_POSTUREDATA      _IOR(AMI306IO, 0x04, int)
#define AMI306_IOCTL_WRITE_POSTUREDATA     _IOW(AMI306IO, 0x05, int)
#define AMI306_IOCTL_READ_CALIDATA         _IOR(AMI306IO, 0x06, int)
#define AMI306_IOCTL_WRITE_CALIDATA        _IOW(AMI306IO, 0x07, int)
#define AMI306_IOCTL_READ_GYRODATA         _IOR(AMI306IO, 0x08, int)
#define AMI306_IOCTL_WRITE_GYRODATA        _IOW(AMI306IO, 0x09, int)
#define AMI306_IOCTL_READ_PEDODATA         _IOR(AMI306IO, 0x0A, long)
#define AMI306_IOCTL_WRITE_PEDODATA        _IOW(AMI306IO, 0x0B, long)
#define AMI306_IOCTL_READ_PEDOPARAM        _IOR(AMI306IO, 0x0C, int)
#define AMI306_IOCTL_WRITE_PEDOPARAM       _IOW(AMI306IO, 0x0D, int)
#define AMI306_IOCTL_READ_CONTROL          _IOR(AMI306IO, 0x0E, int)
#define AMI306_IOCTL_WRITE_CONTROL         _IOW(AMI306IO, 0x0F, int)
#define AMI306_IOCTL_WRITE_MODE            _IOW(AMI306IO, 0x10, int)
#define AMI306_IOCTL_WRITE_REPORT          _IOW(AMI306IO, 0x11, int)
#define AMI306_IOCTL_READ_WIA			   _IOR(AMI306IO, 0x12, int)
#define AMI306_IOCTL_READ_SENSORDATA2      _IOR(AMI306IO, 0x13, int)

/* IOCTLs for AMI306 middleware misc. device library */
#define AMI306DAEIO						   0x84
#define AMI306DAE_IOCTL_GET_SENSORDATA     _IOR(AMI306DAEIO, 0x01, int)
#define AMI306DAE_IOCTL_SET_POSTURE        _IOW(AMI306DAEIO, 0x02, int)
#define AMI306DAE_IOCTL_SET_CALIDATA       _IOW(AMI306DAEIO, 0x03, int)
#define AMI306DAE_IOCTL_SET_GYRODATA       _IOW(AMI306DAEIO, 0x04, int)
#define AMI306DAE_IOCTL_SET_PEDODATA       _IOW(AMI306DAEIO, 0x05, long)
#define AMI306DAE_IOCTL_GET_PEDOPARAM      _IOR(AMI306DAEIO, 0x06, int)
#define AMI306DAE_IOCTL_SET_PEDOPARAM      _IOR(AMI306DAEIO, 0x07, int)
#define AMI306DAE_IOCTL_SET_CONTROL        _IOW(AMI306DAEIO, 0x08, int)
#define AMI306DAE_IOCTL_GET_CONTROL        _IOR(AMI306DAEIO, 0x09, int)
#define AMI306DAE_IOCTL_SET_MODE           _IOW(AMI306DAEIO, 0x0A, int)
#define AMI306DAE_IOCTL_SET_REPORT         _IOW(AMI306DAEIO, 0x0B, int)
#define AMI306DAE_IOCTL_GET_WIA			   _IOR(AMI306DAEIO, 0x0C, int)
#define AMI306DAE_IOCTL_SET_I2CDATA         _IOW(AMI306DAEIO, 0x0d, int)
#define AMI306DAE_IOCTL_SET_I2CADDR         _IOW(AMI306DAEIO, 0x0e, int)
#define AMI306DAE_IOCTL_GET_I2CDATA         _IOR(AMI306DAEIO, 0x0f, int)
#define AMI306DAE_IOCTL_GET_SENSORDATA2    _IOR(AMI306DAEIO, 0x10, int)

/* IOCTLs for AMI306 HAL misc. device library */
#define AMI306HALIO						   0x85
#define AMI306HAL_IOCTL_GET_SENSORDATA     _IOR(AMI306HALIO, 0x01, int)
#define AMI306HAL_IOCTL_GET_POSTURE        _IOR(AMI306HALIO, 0x02, int)
#define AMI306HAL_IOCTL_GET_CALIDATA       _IOR(AMI306HALIO, 0x03, int)
#define AMI306HAL_IOCTL_GET_GYRODATA       _IOR(AMI306HALIO, 0x04, int)
#define AMI306HAL_IOCTL_GET_PEDODATA       _IOR(AMI306HALIO, 0x05, long)
#define AMI306HAL_IOCTL_GET_PEDOPARAM      _IOR(AMI306HALIO, 0x06, int)
#define AMI306HAL_IOCTL_SET_PEDOPARAM      _IOW(AMI306HALIO, 0x07, int)
#define AMI306HAL_IOCTL_GET_CONTROL        _IOR(AMI306HALIO, 0x08, int)
#define AMI306HAL_IOCTL_SET_CONTROL        _IOW(AMI306HALIO, 0x09, int)
#define AMI306HAL_IOCTL_GET_WIA			   _IOR(AMI306HALIO, 0x0A, int)

#define AMI304_CHIPSET				0
#define AMI306_CHIPSET				1
#define AMI306_BUFSIZE				256
#define AMI306_NORMAL_MODE			0
#define AMI306_FORCE_MODE			1
#define AMI306_IRQ				IRQ_EINT9

/* Define items in Control-Byte */
#define AMI306_CB_LENGTH			10
#define AMI306_CB_LOOPDELAY			 0
#define AMI306_CB_RUN				 1
#define AMI306_CB_ACCCALI			 2
#define AMI306_CB_MAGCALI			 3
#define AMI306_CB_ACTIVESENSORS		 4
#define AMI306_CB_PD_RESET			 5
#define AMI306_CB_PD_EN_PARAM		 6
#define AMI306_CB_QWERTY		 	7
#define AMI306_CB_CHANGE_WINDOW		 8
#define AMI306_CB_MDIR		         9

/* Pedometer Parameters */
#define AMI306_PD_LENGTH			10
#define AMI306_PD_PRARM_IIR1		 0
#define AMI306_PD_PRARM_IIR2		 1
#define AMI306_PD_PRARM_IIR3		 2
#define AMI306_PD_PRARM_IIR4		 3
#define AMI306_PD_PRARM_TH1			 4
#define AMI306_PD_PRARM_TH2			 5
#define AMI306_PD_PRARM_TH3			 6
#define AMI306_PD_PRARM_TH4			 7
#define AMI306_PD_UNDEFINE_1		 8
#define AMI306_PD_UNDEFINE_2		 9

/* Define AMIT Sensor Type */
#define AMIT_ACCELEROMETER_SENSOR	0
#define AMIT_MAGNETIC_FIELD_SENSOR	1
#define AMIT_ORIENTATION_SENSOR		2
#define AMIT_GYROSCOPE_SENSOR	    3
#define AMIT_PEDOMETER_SENSOR	    4

#define AMIT_BIT_ACCELEROMETER		(1<<AMIT_ACCELEROMETER_SENSOR)
#define AMIT_BIT_MAGNETIC_FIELD		(1<<AMIT_MAGNETIC_FIELD_SENSOR)
#define AMIT_BIT_ORIENTATION		(1<<AMIT_ORIENTATION_SENSOR)
#define AMIT_BIT_GYROSCOPE			(1<<AMIT_GYROSCOPE_SENSOR)
#define AMIT_BIT_PEDOMETER			(1<<AMIT_PEDOMETER_SENSOR)

#endif
