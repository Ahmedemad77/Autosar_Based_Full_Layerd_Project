/******************************************************************************
*
* Module: Dio
*
* File Name: Port_Cfg.h
*
* Description: Pre-compile Configuration Header file for TM4C123GH6PM Microcontroller - Port Driver 
*
* Author: Ahmed Emad
******************************************************************************/

#ifndef PORT_CFG_H
#define PORT_CFG_H


/*
 * Module Version 1.0.0
 */
#define PORT_CFG_SW_MAJOR_VERSION           (1U)
#define PORT_CFG_SW_MINOR_VERSION           (0U)
#define PORT_CFG_SW_PATCH_VERSION           (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_CFG_AR_RELEASE_MAJOR_VERSION   (4U)
#define PORT_CFG_AR_RELEASE_MINOR_VERSION   (0U)
#define PORT_CFG_AR_RELEASE_PATCH_VERSION   (3U)


/* Pre-compile option for Development Error Detect */    
#define PORT_DEV_ERROR_DETECT               (STD_ON)


/* Pre-compile option for enable/disable use of Function Port_SetPinDirection() */    
#define PORT_SET_PIN_DIRECTION_API           (STD_ON)


/* Pre-compile option for enable/disable use of Function Port_SetPinMode() */    
#define PORT_SET_PIN_MODE_API                (STD_ON)


/* Pre-compile option for enable/disable use of Function Port_GetVersionInfo */    
#define PORT_VERSION_INFO_API                (STD_ON)

/* Configration of configured Pins */

/* LED1 Configration */
#define PortConf_LED1_PIN_NUM                (Port_PinType)PORT_F_PIN_1 /* Pin 1 in PORTF */
#define PortConf_LED1_DIRECTION               PORT_PIN_OUT   /* output pin */
#define PortConf_LED1_RESISTOR                OFF           /* disable internal resistor */
#define PortConf_LED1_MODE                    PORT_PIN_MODE_DIO /* dio pin */
#define PortConf_LED1_INITIAL_VALUE           PORT_PIN_LEVEL_LOW /* initially led is off*/
#define PortConf_LED1_DIRECTION_CHANGE        STD_OFF /* disable direction change during runtime */
#define portConf_LED1_MODE_CHANGE             STD_OFF /* disable mode change during runtime */

/* SW1 Configration */
#define PortConf_BUTTON_PIN_NUM                (Port_PinType)PORT_F_PIN_4 /* Pin 4 in PORTF */
#define PortConf_BUTTON_DIRECTION               PORT_PIN_IN   /* input pin */
#define PortConf_BUTTON_RESISTOR                OFF           /* disable internal resistor */
#define PortConf_BUTTON_MODE                    PORT_PIN_MODE_DIO /* dio pin */
#define PortConf_BUTTON_INITIAL_VALUE           PORT_PIN_LEVEL_LOW /* doesn't matter as it input pin */
#define PortConf_BUTTON_DIRECTION_CHANGE        STD_OFF /* disable direction change during runtime */
#define portConf_BUTTON_MODE_CHANGE             STD_OFF /* disable mode change during runtime */


#endif /* PORT_CFG_H */



