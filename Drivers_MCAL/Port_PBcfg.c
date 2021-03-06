/******************************************************************************
*
* Module: Port 
*       
* File Name: Port_PBcfg.c 
*
* Description: Post Build Configuration Source file for TM4C123GH6PM Microcontroller - Port Driver
*
* Author: Ahmed Emad
******************************************************************************/

#include "Port.h"

/*
 * Module Version 1.0.0
 */
#define Port_PBCFG_SW_MAJOR_VERSION              (1U)
#define Port_PBCFG_SW_MINOR_VERSION              (0U)
#define Port_PBCFG_SW_PATCH_VERSION              (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define Port_PBCFG_AR_RELEASE_MAJOR_VERSION     (4U)
#define Port_PBCFG_AR_RELEASE_MINOR_VERSION     (0U)
#define Port_PBCFG_AR_RELEASE_PATCH_VERSION     (3U)

/* AUTOSAR Version checking between Port_PBcfg.c and Port.h files */
#if ((Port_PBCFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (Port_PBCFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (Port_PBCFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of PBcfg.c does not match the expected version"
#endif

/* Software Version checking between Port_PBcfg.c and Port.h files */
#if ((Port_PBCFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
 ||  (Port_PBCFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
 ||  (Port_PBCFG_SW_PATCH_VERSION != Port_SW_PATCH_VERSION))
  #error "The SW version of PBcfg.c does not match the expected version"
#endif


/* PB structure used with Port_Init API */

const Port_ConfigType Port_configration ={
  /* Port A */
    PORT_A_PIN_0,PORT_PIN_IN,OFF,PORT_PIN_MODE_DIO,PORT_PIN_LEVEL_LOW,STD_OFF,STD_OFF,
    
    PORT_A_PIN_1,PORT_PIN_IN,OFF,PORT_PIN_MODE_DIO,PORT_PIN_LEVEL_LOW,STD_OFF,STD_OFF,
    
    PORT_A_PIN_2,PORT_PIN_IN,OFF,PORT_PIN_MODE_DIO,PORT_PIN_LEVEL_LOW,STD_OFF,STD_OFF,
    
    PORT_A_PIN_3,PORT_PIN_IN,OFF,PORT_PIN_MODE_DIO,PORT_PIN_LEVEL_LOW,STD_OFF,STD_OFF,
    
    PORT_A_PIN_4,PORT_PIN_IN,OFF,PORT_PIN_MODE_DIO,PORT_PIN_LEVEL_LOW,STD_OFF,STD_OFF,
    
    PORT_A_PIN_5,PORT_PIN_IN,OFF,PORT_PIN_MODE_DIO,PORT_PIN_LEVEL_LOW,STD_OFF,STD_OFF,
    
    PORT_A_PIN_6,PORT_PIN_IN,OFF,PORT_PIN_MODE_DIO,PORT_PIN_LEVEL_LOW,STD_OFF,STD_OFF,

    PORT_A_PIN_7,PORT_PIN_IN,OFF,PORT_PIN_MODE_DIO,PORT_PIN_LEVEL_LOW,STD_OFF,STD_OFF,

  /* Port B */
    PORT_B_PIN_0,PORT_PIN_IN,OFF,PORT_PIN_MODE_DIO,PORT_PIN_LEVEL_LOW,STD_OFF,STD_OFF,
    
    PORT_B_PIN_1,PORT_PIN_IN,OFF,PORT_PIN_MODE_DIO,PORT_PIN_LEVEL_LOW,STD_OFF,STD_OFF,
    
    PORT_B_PIN_2,PORT_PIN_IN,OFF,PORT_PIN_MODE_DIO,PORT_PIN_LEVEL_LOW,STD_OFF,STD_OFF,
    
    PORT_B_PIN_3,PORT_PIN_IN,OFF,PORT_PIN_MODE_DIO,PORT_PIN_LEVEL_LOW,STD_OFF,STD_OFF,
    
    PORT_B_PIN_4,PORT_PIN_IN,OFF,PORT_PIN_MODE_DIO,PORT_PIN_LEVEL_LOW,STD_OFF,STD_OFF,
    
    PORT_B_PIN_5,PORT_PIN_IN,OFF,PORT_PIN_MODE_DIO,PORT_PIN_LEVEL_LOW,STD_OFF,STD_OFF,
    
    PORT_B_PIN_6,PORT_PIN_IN,OFF,PORT_PIN_MODE_DIO,PORT_PIN_LEVEL_LOW,STD_OFF,STD_OFF,

    PORT_B_PIN_7,PORT_PIN_IN,OFF,PORT_PIN_MODE_DIO,PORT_PIN_LEVEL_LOW,STD_OFF,STD_OFF,
    
    /* Port C */
    PORT_C_PIN_0,PORT_PIN_IN,OFF,PORT_PIN_MODE_DIO,PORT_PIN_LEVEL_LOW,STD_OFF,STD_OFF,
    
    PORT_C_PIN_1,PORT_PIN_IN,OFF,PORT_PIN_MODE_DIO,PORT_PIN_LEVEL_LOW,STD_OFF,STD_OFF,
    
    PORT_C_PIN_2,PORT_PIN_IN,OFF,PORT_PIN_MODE_DIO,PORT_PIN_LEVEL_LOW,STD_OFF,STD_OFF,
    
    PORT_C_PIN_3,PORT_PIN_IN,OFF,PORT_PIN_MODE_DIO,PORT_PIN_LEVEL_LOW,STD_OFF,STD_OFF,
    
    PORT_C_PIN_4,PORT_PIN_IN,OFF,PORT_PIN_MODE_DIO,PORT_PIN_LEVEL_LOW,STD_OFF,STD_OFF,
    
    PORT_C_PIN_5,PORT_PIN_IN,OFF,PORT_PIN_MODE_DIO,PORT_PIN_LEVEL_LOW,STD_OFF,STD_OFF,
    
    PORT_C_PIN_6,PORT_PIN_IN,OFF,PORT_PIN_MODE_DIO,PORT_PIN_LEVEL_LOW,STD_OFF,STD_OFF,

    PORT_C_PIN_7,PORT_PIN_IN,OFF,PORT_PIN_MODE_DIO,PORT_PIN_LEVEL_LOW,STD_OFF,STD_OFF,

      /* Port D */
    PORT_D_PIN_0,PORT_PIN_IN,OFF,PORT_PIN_MODE_DIO,PORT_PIN_LEVEL_LOW,STD_OFF,STD_OFF,
    
    PORT_D_PIN_1,PORT_PIN_IN,OFF,PORT_PIN_MODE_DIO,PORT_PIN_LEVEL_LOW,STD_OFF,STD_OFF,
    
    PORT_D_PIN_2,PORT_PIN_IN,OFF,PORT_PIN_MODE_DIO,PORT_PIN_LEVEL_LOW,STD_OFF,STD_OFF,
    
    PORT_D_PIN_3,PORT_PIN_IN,OFF,PORT_PIN_MODE_DIO,PORT_PIN_LEVEL_LOW,STD_OFF,STD_OFF,
    
    PORT_D_PIN_4,PORT_PIN_IN,OFF,PORT_PIN_MODE_DIO,PORT_PIN_LEVEL_LOW,STD_OFF,STD_OFF,
    
    PORT_D_PIN_5,PORT_PIN_IN,OFF,PORT_PIN_MODE_DIO,PORT_PIN_LEVEL_LOW,STD_OFF,STD_OFF,
    
    PORT_D_PIN_6,PORT_PIN_IN,OFF,PORT_PIN_MODE_DIO,PORT_PIN_LEVEL_LOW,STD_OFF,STD_OFF,

    PORT_D_PIN_7,PORT_PIN_IN,OFF,PORT_PIN_MODE_DIO,PORT_PIN_LEVEL_LOW,STD_OFF,STD_OFF,

    /* Port E */
    PORT_E_PIN_0,PORT_PIN_IN,OFF,PORT_PIN_MODE_DIO,PORT_PIN_LEVEL_LOW,STD_OFF,STD_OFF,
    
    PORT_E_PIN_1,PORT_PIN_IN,OFF,PORT_PIN_MODE_DIO,PORT_PIN_LEVEL_LOW,STD_OFF,STD_OFF,
    
    PORT_E_PIN_2,PORT_PIN_IN,OFF,PORT_PIN_MODE_DIO,PORT_PIN_LEVEL_LOW,STD_OFF,STD_OFF,
    
    PORT_E_PIN_3,PORT_PIN_IN,OFF,PORT_PIN_MODE_DIO,PORT_PIN_LEVEL_LOW,STD_OFF,STD_OFF,
    
    PORT_E_PIN_4,PORT_PIN_IN,OFF,PORT_PIN_MODE_DIO,PORT_PIN_LEVEL_LOW,STD_OFF,STD_OFF,
    
    PORT_E_PIN_5,PORT_PIN_IN,OFF,PORT_PIN_MODE_DIO,PORT_PIN_LEVEL_LOW,STD_OFF,STD_OFF,

    /* Port F */
    PORT_F_PIN_0,PORT_PIN_IN,OFF,PORT_PIN_MODE_DIO,PORT_PIN_LEVEL_LOW,STD_OFF,STD_OFF,
    
    /* LED1 Configration */
    PortConf_LED1_PIN_NUM,PortConf_LED1_DIRECTION,PortConf_LED1_RESISTOR,PortConf_LED1_MODE
    ,PortConf_LED1_INITIAL_VALUE,PortConf_LED1_DIRECTION_CHANGE,portConf_LED1_MODE_CHANGE,
    
    PORT_F_PIN_2,PORT_PIN_IN,OFF,PORT_PIN_MODE_DIO,PORT_PIN_LEVEL_LOW,STD_OFF,STD_OFF,
    
    PORT_F_PIN_3,PORT_PIN_IN,OFF,PORT_PIN_MODE_DIO,PORT_PIN_LEVEL_LOW,STD_OFF,STD_OFF,

    /* SW1 Configration */
    PortConf_BUTTON_PIN_NUM,PortConf_BUTTON_DIRECTION,PortConf_BUTTON_RESISTOR,PortConf_BUTTON_MODE
    ,PortConf_BUTTON_INITIAL_VALUE,PortConf_BUTTON_DIRECTION_CHANGE,portConf_BUTTON_MODE_CHANGE,
    

};





















