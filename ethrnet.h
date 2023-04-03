/**
  ******************************************************************************
* @file    ethrnet.h
* @author  zhy
* @brief   ethrnet function
  *          
  ******************************************************************************
  * @attention   None
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ETHRNET_H
#define ETHRNET_H

#ifdef __cplusplus
 extern "C" {
#endif 
/* Includes ------------------------------------------------------------------*/  
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/
/** @defgroup   eth buff type
* @{
*/  
struct eth_fifo
{
  uint8_t *buf;
  uint16_t tail;
  uint16_t head;
  uint16_t len;
  uint8_t  empty;
};

 /**
* @}
*/    
/*MACRO-----------------------------------------------------------------------*/ 


/* Peripheral Control functions  ************************************************/
void Eth_Init(void);
void Eth_Tcp_Server_Thread(void);
void Eth_Tcp_Client_Thread(void);
void Eth_Udp_Thread(void);

#ifdef __cplusplus
}
#endif



#endif 
/********************************END OF FILE***********************************/
