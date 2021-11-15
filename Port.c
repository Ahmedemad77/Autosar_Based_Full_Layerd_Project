 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.c
 *
 * Description: Source file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Mohamed Tarek
 ******************************************************************************/

#include "Port.h"
#include "Port_Regs.h"
#include "tm4c123gh6pm_registers.h"
#include "Det.h"


#if (PORT_DEV_ERROR_DETECT == STD_ON)

#include "Det.h"
/* AUTOSAR Version checking between Det and Dio Modules */
#if ((DET_AR_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 || (DET_AR_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 || (DET_AR_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Det.h does not match the expected version"
#endif

#endif

STATIC uint8 Port_Status = PORT_NOT_INITIALIZED;
/* pointer to point to the first element in the array of structrue of configration*/
STATIC const Port_configPin* Port_PinsConfigPtr =  NULL_PTR;


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
void Port_Init( const Port_ConfigType* ConfigPtr )
{
    /* parameters to hold the port number and pin Number of PIN ID */
    uint8 PortNum = 0;
    uint8 PinNum  = 0;
    
    volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
    volatile uint32 delay = 0; /* used to waste cycles for proper operation of clock*/
    

#if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* check if the input configuration pointer is not a NULL_PTR */
	if (NULL_PTR == ConfigPtr)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_INIT_SID,
		     PORT_E_PARAM_CONFIG);
	}
	else
#endif
	{
		/*
		 * Set the module state to initialized and point to the PB configuration structure using a global pointer.
		 * This global pointer is global to be used by other functions to read the PB configuration structures
		 */
		Port_Status       = PORT_INITIALIZED;
		Port_PinsConfigPtr = ConfigPtr->Pins_Configration; /* address of the first Channels structure --> Channels[0] */
	}
    /* enable the clock for all ports*/
    /* Edit 1 */
    
    
    /* loop over MCU pins and init them as configration */
    for (uint8 pin = 0; pin < PORT_NUMBER_OF_MCU_PINS; pin++)
    {
        /* getting Port number  & Pin number from pin ID  */ 
        if (Port_PinsConfigPtr[pin].pin_num <=37){/* if the pin falls in first four ports (37 pins)*/
        
            PortNum = Port_PinsConfigPtr[pin].pin_num / 8;/* 8 --> number of pins for port */
            PinNum  = Port_PinsConfigPtr[pin].pin_num % 8;

        }else/* In port F */
        {
            PortNum = PORT_F;
            PinNum  = Port_PinsConfigPtr[pin].pin_num - 38; /* 38 -- > number of the past 4 ports  */
        }
        
        /* Enable clock for PORT and allow time for clock to start*/
        SYSCTL_REGCGC2_REG |= (1<<PortNum);
        delay = SYSCTL_REGCGC2_REG;


        /* getting Base Adress corresponding to PortNum */
        switch(PortNum)
         {
            case  0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
	       	       break;
            case  1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
	                 break;
	        case  2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
	            	 break;
            case  3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
	            	 break;
            case  4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
	                 break;
             case  5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
	                 break;
         }

        /* checking if the pin needs unlocking (PD7 or PF0) */
        if( ((PortNum == PORT_D) && (PinNum == 7)) || ((PortNum == PORT_F) && (PinNum == 0)) ) 
        {
            /* Unlock the GPIOCR register */   
            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;     
            /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) , PinNum);  
        }else
        {
            /* Do Nothing */
        }
        

        /* configuring pin   */
        if( (PortNum == PORT_C) && (PinNum <= 3) ) /* PC0 to PC3 */
        {
            /* Do Nothing ...  this is the JTAG pins */
        }else  
        {
         /* if mode not Analog i.e digital functionality */
          if (Port_PinsConfigPtr[pin].mode != PORT_PIN_MODE_ADC)
          {
             /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
             CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , PinNum);     
            
            /* setting alternate function and port control register for the corresponding mode */
            switch (Port_PinsConfigPtr[pin].mode)
            {
            case PORT_PIN_MODE_DIO:
            case PORT_PIN_MODE_DIO_WDG:
            case PORT_PIN_MODE_FLEXRAY:
            case PORT_PIN_MODE_LIN:
            case PORT_PIN_MODE_MEM:
                    /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , PinNum);             
                    /* Clear the PMCx bits for this pin */
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (PinNum*4));     
                break;
            case PORT_PIN_MODE_DIO_GPT:
                    /* Enable Alternative function for this pin by set the corresponding bit in GPIOAFSEL register */
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , PinNum);             
                    /* Set the PMCx bits for this pin by 7 "the value corresponding to GPT"*/
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x00000007 << (PinNum*4));     
                break;
            case PORT_PIN_MODE_CAN:
                    /* Enable Alternative function for this pin by set the corresponding bit in GPIOAFSEL register */
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , PinNum);             
                    /* Set the PMCx bits */
                    if (PinNum == PORT_F_PIN_0 ||PinNum ==PORT_F_PIN_3){
                    /* Set the PMCx bits for this pin by 3 "the value corresponding to CAN for PF0 & PF3"*/
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x00000007 << (PinNum*4));
                    }else
                    {
                     /* Set the PMCx bits for this pin by 8 "the value corresponding to CAN for other pins"*/
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x00000008 << (PinNum*4));
                    }
                break;
            case PORT_PIN_MODE_PWM:
                    /* Enable Alternative function for this pin by set the corresponding bit in GPIOAFSEL register */
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , PinNum);  
                    /* Set the PMCx bits */        
                    if ( PinNum == PORT_D_PIN_0 ||PinNum ==PORT_D_PIN_1 ||PinNum == PORT_E_PIN_4 \
                          || PinNum == PORT_E_PIN_5 ||PortNum ==PORT_B || PortNum ==PORT_C)
                    {
                    /* Set the PMCx bits for those pin by 4 "the value corresponding to PWM" */
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x00000004 << (PinNum*4));
                    }else
                    {
                     /* Set the PMCx bits for this pin by 8 "the value corresponding to CAN for other pins"*/
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x00000005 << (PinNum*4));
                    }
                break;
            case PORT_PIN_MODE_SPI:
                    /* Enable Alternative function for this pin by set the corresponding bit in GPIOAFSEL register */
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , PinNum);             
                    /* Set the PMCx bits */
                    if (PortNum == PORT_D ){
                    /* Set the PMCx bits for this pin by 1 "the value corresponding to SSI(SPI) to module 3" */
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x00000001 << (PinNum*4));
                    }else
                    {
                     /* Set the PMCx bits for this pin by 2 "the value corresponding to SSI (SPI) to module 0,1,2"*/
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x00000002 << (PinNum*4));
                    }
            default:
                break;
            }

            /* setting direction & initial value if output and  pull up,down if input  */    
            if(Port_PinsConfigPtr[pin].direction == PORT_PIN_OUT){
                /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
	            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , PinNum);               
        
                    if(Port_PinsConfigPtr[pin].initial_value == STD_HIGH)
                    {
                        /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
                        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , PinNum);          
                    }
                     else
                    {
                         /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
                        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , PinNum);       
                    }
            }
            else if(Port_PinsConfigPtr[pin].direction == PORT_PIN_IN){       
                /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , PinNum);             
        
                 if(Port_PinsConfigPtr[pin].resistor == PULL_UP)
                    {
                        /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
                        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , PinNum);       
                    }
                else if(Port_PinsConfigPtr[pin].resistor == PULL_DOWN)
                    {
                        /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
                       SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , PinNum);     
                    }
                else
                   {
                       /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
                      CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , PinNum);     
                      /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
                      CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , PinNum);   
                   }
            }
            else
            {
                 /* Do Nothing */
            }
          
            /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , PinNum);

          }else /* mode is analog   "ADC" */
          {
            /* clear the corresponding bit in the GPIODEN register to disable digital functionality on this pin */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , PinNum);

            /* Enable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , PinNum);  
                               
             /* Set the corresponding bit in the GPIOAMSEL register to enable analog functionality on this pin */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , PinNum);     
            
          }
          
        }
    }
}

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
void Port_SetPinDirection( Port_PinType Pin, Port_PinDirectionType Direction ){

    /* parameters to hold the port number and pin Number of PIN ID */
    uint8 PortNum = 0;
    uint8 PinNum  = 0;

	volatile uint32 * PortGpio_Ptr = NULL_PTR;
	boolean error = FALSE;
    

#if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* Check if the Driver is initialized before using this function */
	if (PORT_NOT_INITIALIZED == Port_Status)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_SET_PIN_DIRECTION_SID, PORT_E_UNINIT);
		error = TRUE;
	}
	else
	{
		/* No Action Required */
	}

	/* Check if the used channel is within the valid range */
	if ( Pin >= PORT_NUMBER_OF_MCU_PINS)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_SET_PIN_DIRECTION_SID, PORT_E_PARAM_PIN);
		error = TRUE;
	}
	else
	{
		/* No Action Required */
	}

    /* ckeck if pin direction is changable */
    if (Port_PinsConfigPtr[Pin].direction_change !=STD_ON)
    {
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_SET_PIN_DIRECTION_SID, PORT_E_DIRECTION_UNCHANGEABLE);
		error = TRUE;

    }
    else
    {
        /* No Action Required */   
    }
#endif

    if (error == FALSE)
    {
             /* getting Port number  & Pin number from pin ID  */ 
        if (Port_PinsConfigPtr[Pin].pin_num <=37){/* if the pin falls in first four ports (37 pins)*/
        
            PortNum = Port_PinsConfigPtr[Pin].pin_num / 8;/* 8 --> number of pins for port */
            PinNum  = Port_PinsConfigPtr[Pin].pin_num % 8;

        }else/* In port F */
        {
            PortNum = PORT_F;
            PinNum  = Port_PinsConfigPtr[Pin].pin_num - 38; /* 38 -- > number of the past 4 ports  */
        }
        
        /* getting Base Adress corresponding to PortNum */
        switch(PortNum)
         {
            case  0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
	       	       break;
            case  1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
	                 break;
	        case  2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
	            	 break;
            case  3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
	            	 break;
            case  4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
	                 break;
             case  5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
	                 break;
         }    

        if(Direction == PORT_PIN_OUT){
            /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
	        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , PinNum); 
        }else
        {
            /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , PinNum);             
        }
            
    }
    else
    {
      /* No action Required */
    }
}
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
void Port_RefreshPortDirection( void ){

    /* parameters to hold the port number and pin Number of PIN ID */
    uint8 PortNum = 0;
    uint8 PinNum  = 0;

    volatile uint32 * PortGpio_Ptr = NULL_PTR;
    boolean error = FALSE;

#if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* Check if the Driver is initialized before using this function */
	if (PORT_NOT_INITIALIZED == Port_Status)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_REFERSH_PORT_DIRECTION_SID, PORT_E_UNINIT);
		error = TRUE;
	}
	else
	{
		/* No Action Required */
	}

#endif

    if (error == FALSE)
    {
        /* loop over the pins and refresh only pins which configured as unchangable-direction  */
        for (uint8 pin = 0; pin < PORT_NUMBER_OF_MCU_PINS; pin++)
        {
            if (Port_PinsConfigPtr[pin].direction_change == STD_OFF)
            {
                /* Refresh direction */

                /* getting Port number  & Pin number from pin ID  */ 
                if (Port_PinsConfigPtr[pin].pin_num <=37){/* if the pin falls in first four ports (37 pins)*/
        
                     PortNum = Port_PinsConfigPtr[pin].pin_num / 8;/* 8 --> number of pins for port */
                     PinNum  = Port_PinsConfigPtr[pin].pin_num % 8;

                }else/* In port F */
                {
                    PortNum = PORT_F;
                    PinNum  = Port_PinsConfigPtr[pin].pin_num - 38; /* 38 -- > number of the past 4 ports  */
                }
        
                /* getting Base Adress corresponding to PortNum */
                switch(PortNum)
                 {
                    case  0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
	       	             break;
                    case  1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
	                     break;
	                case  2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
	                  	 break;
                    case  3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
	                	 break;
                    case  4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
	                     break;
                    case  5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
	                     break;
                }

                /* refreh direction of pins  */
                if( Port_PinsConfigPtr[pin].direction == PORT_PIN_OUT){
                     /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
	                 SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , PinNum); 
                }else
                {
                    /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , PinNum);             
                }
        
            }else
            {
                /* No action required */
            }
        
        }
        
        
    }else
    {
        /* No action required */
    }
    
}

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
void Port_GetVersionInfo(  Std_VersionInfoType* versioninfo ){

#if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* Check if input pointer is not Null pointer */
	if(NULL_PTR == versioninfo)
	{
		/* Report to DET  */
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_GET_VERSION_INFO_SID, PORT_E_PARAM_POINTER);
	}
	else
#endif /* (DIO_DEV_ERROR_DETECT == STD_ON) */
	{
		/* Copy the vendor Id */
		versioninfo->vendorID = (uint16)PORT_VENDOR_ID;
		/* Copy the module Id */
		versioninfo->moduleID = (uint16)PORT_MODULE_ID;
		/* Copy Software Major Version */
		versioninfo->sw_major_version = (uint8)PORT_SW_MAJOR_VERSION;
		/* Copy Software Minor Version */
		versioninfo->sw_minor_version = (uint8)PORT_SW_MINOR_VERSION;
		/* Copy Software Patch Version */
		versioninfo->sw_patch_version = (uint8)PORT_SW_PATCH_VERSION;
	}
}
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
void Port_SetPinMode( Port_PinType Pin, Port_PinModeType Mode ){

    /* parameters to hold the port number and pin Number of PIN ID */
    uint8 PortNum = 0;
    uint8 PinNum  = 0;

	volatile uint32 * PortGpio_Ptr = NULL_PTR;
	boolean error = FALSE;
    

#if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* Check if the Driver is initialized before using this function */
	if (PORT_NOT_INITIALIZED == Port_Status)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_SET_PIN_MODE_SID, PORT_E_UNINIT);
		error = TRUE;
	}
	else
	{
		/* No Action Required */
	}

	/* Check if the used channel is within the valid range */
	if ( Pin >= PORT_NUMBER_OF_MCU_PINS)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_SET_PIN_MODE_SID, PORT_E_PARAM_PIN);
		error = TRUE;
	}
	else
	{
		/* No Action Required */
	}

    /* ckeck if pin mode is changable */
    if (Port_PinsConfigPtr[Pin].mode_change !=STD_ON)
    {
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_SET_PIN_MODE_SID, PORT_E_MODE_UNCHANGEABLE);
		error = TRUE;

    }
    else
    {
        /* No Action Required */   
    }
    /* check if the mode is valid by comparing the modes range*/
    if (Mode<PORT_PIN_MODE_ADC || Mode> PORT_PIN_MODE_SPI)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_SET_PIN_MODE_SID, PORT_E_PARAM_INVALID_MODE);
		error = TRUE;
    }
    else
    {
        /* No Action Required */
    }
    
#endif

    if (error == FALSE)
    {
             /* getting Port number  & Pin number from pin ID  */ 
        if (Port_PinsConfigPtr[Pin].pin_num <=37){/* if the pin falls in first four ports (37 pins)*/
        
            PortNum = Port_PinsConfigPtr[Pin].pin_num / 8;/* 8 --> number of pins for port */
            PinNum  = Port_PinsConfigPtr[Pin].pin_num % 8;

        }else/* In port F */
        {
            PortNum = PORT_F;
            PinNum  = Port_PinsConfigPtr[Pin].pin_num - 38; /* 38 -- > number of the past 4 ports  */
        }
        
        /* getting Base Adress corresponding to PortNum */
        switch(PortNum)
         {
            case  0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
	       	       break;
            case  1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
	                 break;
	        case  2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
	            	 break;
            case  3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
	            	 break;
            case  4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
	                 break;
             case  5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
	                 break;
         }    
        /* if mode not Analog i.e digital functionality */
          if (Mode != PORT_PIN_MODE_ADC)
          {
             /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
             CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , PinNum);     
            
            /* setting alternate function and port control register for the corresponding mode */
            switch (Mode)
            {
            case PORT_PIN_MODE_DIO:
            case PORT_PIN_MODE_DIO_WDG:
            case PORT_PIN_MODE_FLEXRAY:
            case PORT_PIN_MODE_LIN:
            case PORT_PIN_MODE_MEM:
                    /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , PinNum);             
                    /* Clear the PMCx bits for this pin */
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (PinNum*4));     
                break;
            case PORT_PIN_MODE_DIO_GPT:
                    /* Enable Alternative function for this pin by set the corresponding bit in GPIOAFSEL register */
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , PinNum);             
                    /* Set the PMCx bits for this pin by 7 "the value corresponding to GPT"*/
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x00000007 << (PinNum*4));     
                break;
            case PORT_PIN_MODE_CAN:
                    /* Enable Alternative function for this pin by set the corresponding bit in GPIOAFSEL register */
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , PinNum);             
                    /* Set the PMCx bits */
                    if (PinNum == PORT_F_PIN_0 ||PinNum ==PORT_F_PIN_3){
                    /* Set the PMCx bits for this pin by 3 "the value corresponding to CAN for PF0 & PF3"*/
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x00000007 << (PinNum*4));
                    }else
                    {
                     /* Set the PMCx bits for this pin by 8 "the value corresponding to CAN for other pins"*/
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x00000008 << (PinNum*4));
                    }
                break;
            case PORT_PIN_MODE_PWM:
                    /* Enable Alternative function for this pin by set the corresponding bit in GPIOAFSEL register */
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , PinNum);  
                    /* Set the PMCx bits */        
                    if ( PinNum == PORT_D_PIN_0 ||PinNum ==PORT_D_PIN_1 ||PinNum == PORT_E_PIN_4 \
                          || PinNum == PORT_E_PIN_5 ||PortNum ==PORT_B || PortNum ==PORT_C)
                    {
                    /* Set the PMCx bits for those pin by 4 "the value corresponding to PWM" */
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x00000004 << (PinNum*4));
                    }else
                    {
                     /* Set the PMCx bits for this pin by 8 "the value corresponding to CAN for other pins"*/
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x00000005 << (PinNum*4));
                    }
                break;
            case PORT_PIN_MODE_SPI:
                    /* Enable Alternative function for this pin by set the corresponding bit in GPIOAFSEL register */
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , PinNum);             
                    /* Set the PMCx bits */
                    if (PortNum == PORT_D ){
                    /* Set the PMCx bits for this pin by 1 "the value corresponding to SSI(SPI) to module 3" */
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x00000001 << (PinNum*4));
                    }else
                    {
                     /* Set the PMCx bits for this pin by 2 "the value corresponding to SSI (SPI) to module 0,1,2"*/
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x00000002 << (PinNum*4));
                    }
            default:
                break;
            }

          }else{ /* mode is analog   "ADC" */
          
            /* clear the corresponding bit in the GPIODEN register to disable digital functionality on this pin */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , PinNum);

            /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , PinNum);  
                               
             /* Set the corresponding bit in the GPIOAMSEL register to enable analog functionality on this pin */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , PinNum);     
            
          }


            
    }
    else
    {
      /* No action Required */
    }


} 
#endif
