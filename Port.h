 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.h
 *
 * Description: Header file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Ahmed emad
 ******************************************************************************/


#ifndef PORT_H
#define PORT_H

/* Vendor  ID */
#define PORT_VENDOR_ID (1000U)

/* Port Module ID */
#define PORT_MODULE_ID (124U)

/* Port Instance ID */
#define PORT_INSTANCE_ID (0U)

/*
 * Module Version 1.0.0
 */
#define PORT_SW_MAJOR_VERSION           (1U)
#define PORT_SW_MINOR_VERSION           (0U)
#define PORT_SW_PATCH_VERSION           (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_AR_RELEASE_MAJOR_VERSION   (4U)
#define PORT_AR_RELEASE_MINOR_VERSION   (0U)
#define PORT_AR_RELEASE_PATCH_VERSION   (3U)

/*
 * Macros for Port Status
 */
#define PORT_INITIALIZED                (1U)
#define PORT_NOT_INITIALIZED            (0U)


/*
 * Macros for Port pin level
 */
#define PORT_PIN_LEVEL_HIGH             (1U)
#define PORT_PIN_LEVEL_LOW              (0U)

/* Macro for defining number of pins of MCU */
#define PORT_NUMBER_OF_MCU_PINS         (43U)

/* Macros for defining indicator values for each port */
#define PORT_A                          (0U)
#define PORT_B                          (1U)
#define PORT_C                          (2U)
#define PORT_D                          (3U)
#define PORT_E                          (4U)
#define PORT_F                          (5U)

/* Macros for defining indicator values for each pin */
#define PORT_A_PIN_0                           (0U)
#define PORT_A_PIN_1                           (1U)
#define PORT_A_PIN_2                           (2U)
#define PORT_A_PIN_3                           (3U)
#define PORT_A_PIN_4                           (4U)
#define PORT_A_PIN_5                           (5U)
#define PORT_A_PIN_6                           (6U)
#define PORT_A_PIN_7                           (7U)    

#define PORT_B_PIN_0                           (8U)
#define PORT_B_PIN_1                           (9U)
#define PORT_B_PIN_2                           (10U)
#define PORT_B_PIN_3                           (11U)
#define PORT_B_PIN_4                           (12U)
#define PORT_B_PIN_5                           (13U)
#define PORT_B_PIN_6                           (14U)
#define PORT_B_PIN_7                           (15U)

#define PORT_C_PIN_0                           (16U)
#define PORT_C_PIN_1                           (17U)
#define PORT_C_PIN_2                           (18U)
#define PORT_C_PIN_3                           (19U)
#define PORT_C_PIN_4                           (20U)
#define PORT_C_PIN_5                           (21U)
#define PORT_C_PIN_6                           (22U)
#define PORT_C_PIN_7                           (23U)    

#define PORT_D_PIN_0                           (24U)
#define PORT_D_PIN_1                           (25U)
#define PORT_D_PIN_2                           (26U)
#define PORT_D_PIN_3                           (27U)
#define PORT_D_PIN_4                           (28U)
#define PORT_D_PIN_5                           (29U)
#define PORT_D_PIN_6                           (30U)
#define PORT_D_PIN_7                           (31U)    

#define PORT_E_PIN_0                           (32U)
#define PORT_E_PIN_1                           (33U)
#define PORT_E_PIN_2                           (34U)
#define PORT_E_PIN_3                           (35U)
#define PORT_E_PIN_4                           (36U)
#define PORT_E_PIN_5                           (37U)   

#define PORT_F_PIN_0                           (38U)
#define PORT_F_PIN_1                           (39U)
#define PORT_F_PIN_2                           (40U)
#define PORT_F_PIN_3                           (41U)
#define PORT_F_PIN_4                           (42U)

#include "Std_Types.h"

/* checking AUTOSAR versions between std_Types.h and Port.h */
#if (PORT_AR_RELEASE_MAJOR_VERSION != STD_TYPES_AR_RELEASE_MAJOR_VERSION) \
    ||(PORT_AR_RELEASE_MINOR_VERSION != STD_TYPES_AR_RELEASE_MINOR_VERSION)\
    ||(PORT_AR_RELEASE_PATCH_VERSION != STD_TYPES_AR_RELEASE_PATCH_VERSION)
#error "The AR version of Std_Types.h does not match the expected version" 
#endif


/* Non AUTOSAR file */
#include "Common_Macros.h"

/* Pre-compile configration file for port driver */
#include "Port_Cfg.h"

/* checking AUTOSAR versions between Port_Cfg.h  and Port.h */
#if (PORT_AR_RELEASE_MAJOR_VERSION != PORT_CFG_AR_RELEASE_MAJOR_VERSION) \
    ||(PORT_AR_RELEASE_MINOR_VERSION != PORT_CFG_AR_RELEASE_MINOR_VERSION)\
    ||(PORT_AR_RELEASE_PATCH_VERSION != PORT_CFG_AR_RELEASE_PATCH_VERSION)
#error "The AR version of Port_Cfg.h does not match the expected version" 
#endif

/* checking Software versions between Port_Cfg.h  and Port.h */
#if   (PORT_SW_MAJOR_VERSION != PORT_CFG_SW_MAJOR_VERSION) \
    ||(PORT_SW_MINOR_VERSION != PORT_CFG_SW_MINOR_VERSION)\
    ||(PORT_SW_PATCH_VERSION != PORT_CFG_SW_PATCH_VERSION)
#error "The AR version of Port_Cfg.h does not match the expected version" 
#endif



/******************************************************************************
 *                      API Service Id Macros                                 *
 ******************************************************************************/

/* service ID for port Init  */
#define PORT_INIT_SID                    (uint8)0X00 

/* service ID for port Set pin direction  */
#define PORT_SET_PIN_DIRECTION_SID       (uint8)0X01 

/* service ID for Refersh port direction  */
#define PORT_REFERSH_PORT_DIRECTION_SID  (uint8)0X02

/* service ID for port get version info  */
#define PORT_GET_VERSION_INFO_SID        (uint8)0X03

/* service ID for port Set pin Mode  */
#define PORT_SET_PIN_MODE_SID            (uint8)0X04 

/*******************************************************************************
 *                      DET Error Codes                                        *
 *******************************************************************************/

/* DET code to report Invalid Port Pin ID requested */
#define PORT_E_PARAM_PIN              (uint8)0x0A

/* DET code to report error for request change direction of unchangable pin */
#define PORT_E_DIRECTION_UNCHANGEABLE (uint8)0x0B 

/* API Port_Init service called with wrong parameter */
#define PORT_E_PARAM_CONFIG           (uint8)0x0C 

/* DET code to report error for invalid mode */
#define PORT_E_PARAM_INVALID_MODE     (uint8)0x0D 

/* DET code to report error for request change mode of unchangable pin */
#define PORT_E_MODE_UNCHANGEABLE      (uint8)0x0E

/* API service called  without module initialization */
#define PORT_E_UNINIT                 (uint8)0x0F

/* APIs called with a Null  Pointer  */
#define PORT_E_PARAM_POINTER          (uint8)0x10 

/*******************************************************************************
 *                              Module Data Types                              *
 *******************************************************************************/

/* Type definition for Port_PinType used by the Port APIs which hold pin number*/
typedef uint8 Port_PinType;

/* Description: Enum to hold PIN direction */
typedef enum
{
    PORT_PIN_IN,PORT_PIN_OUT
}Port_PinDirectionType;


/* Description: Enum to declare values for modes of pin */
typedef enum
{
    PORT_PIN_MODE_ADC , PORT_PIN_MODE_CAN ,PORT_PIN_MODE_DIO,
    PORT_PIN_MODE_DIO_GPT ,PORT_PIN_MODE_DIO_WDG,PORT_PIN_MODE_FLEXRAY ,
    PORT_PIN_MODE_ICU ,PORT_PIN_MODE_LIN ,PORT_PIN_MODE_MEM,
    PORT_PIN_MODE_PWM,PORT_PIN_MODE_SPI
}Port_pinModes;


/* Type definition for Port_PinMode used by the Port APIs which hold pin mpde */
typedef uint8 Port_PinModeType;

/* Description: Enum to hold internal resistor type for PIN */
typedef enum
{
    OFF,PULL_UP,PULL_DOWN
}Port_InternalResistor;


/* Description: Structure to configure each individual PIN:
 *	1. ID for the pin 
 *  2. the direction of pin --> INPUT or OUTPUT
 *  3. the internal resistor --> Disable, Pull up or Pull down
 *  4. the mode of pin --> ADC ,SPI ..
 *  5. initial value of the pin --> HIGH ,LOW 
 *  6. if pin allowed to change its direction --> STD_ON ,std_off
 *  7. if pin allowed to change its mode    --> STD_ON ,std_off
 */
typedef struct 
{
    Port_PinType pin_num; 
    Port_PinDirectionType direction;
    Port_InternalResistor resistor;
    Port_PinModeType mode;
    uint8 initial_value;
    uint8 direction_change;
    uint8 mode_change;
}Port_configPin;

typedef struct 
{
    Port_configPin Pins_Configration [PORT_NUMBER_OF_MCU_PINS];
}Port_ConfigType;





/*******************************************************************************
 *                      Function Prototypes                                    *
 *******************************************************************************/

/************************************************************************************
* Service Name: Port_Init
* Sync/Async: Synchronous
* Reentrancy: Non reentrant
* Parameters (in): ConfigPtr - Pointer to post-build configuration data
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Initialize ALL ports and port pins with the configuration set pointed 
*              to by the parameter ConfigPtr
************************************************************************************/
void Port_Init( const Port_ConfigType* ConfigPtr );


/* The function Port_SetPinDirection shall only be available to the 
 * user if the pre-compile parameter PORT_SET_PIN_DIRECTION_API is set to TRUE */

#if PORT_SET_PIN_DIRECTION_API
/************************************************************************************
* Service Name: Port_SetPinDirection
* Sync/Async: Synchronous
* Reentrancy: Reentrant
* Parameters (in): Pin - Port pin Id number
*                  Direction  - Port pin direction 
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: set the port pin direction during runtime 
************************************************************************************/
void Port_SetPinDirection( Port_PinType Pin, Port_PinDirectionType Direction ); 

#endif

/************************************************************************************
* Service Name: Port_RefreshPortDirection
* Sync/Async: Synchronous
* Reentrancy: Non reentrant
* Parameters (in): None 
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: refresh the direction of all configured ports to the configured
*              direction (PortPinDirection)
************************************************************************************/
void Port_RefreshPortDirection( void ); 


/* The function Port_SetPinDirection shall only be available to the 
 * user if the pre-compile parameter PORT_VERSION_INFO_API is set to TRUE */
#if PORT_VERSION_INFO_API
/************************************************************************************
* Service Name: Port_GetVersionInfo
* Sync/Async: Synchronous
* Reentrancy: Non reentrant
* Parameters (in): None 
* Parameters (inout): None
* Parameters (out): versioninfo - Pointer to where to store the version information 
*                   of this module. 
* Return value: None
* Description: Returns the version information of this module
************************************************************************************/
void Port_GetVersionInfo(  Std_VersionInfoType* versioninfo ); 
#endif


/* The function Port_SetPinDirection shall only be available to the 
 * user if the pre-compile parameter PORT_SET_PIN_MODE_API is set to TRUE */

#if PORT_SET_PIN_MODE_API
/************************************************************************************
* Service Name: Port_SetPinMode
* Sync/Async: Synchronous
* Reentrancy: Reentrant
* Parameters (in): Pin - Port pin Id number
*                  Mode  - New Port Pin mode to be set on port pin. 
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin mode.  
************************************************************************************/
void Port_SetPinMode( Port_PinType Pin, Port_PinModeType Mode ); 
#endif


/*******************************************************************************
 *                       External Variables                                    *
 *******************************************************************************/

/* Extern PB structures to be used by Dio and other modules */
extern const Port_ConfigType Port_configration;

#endif /* PORT_H */
