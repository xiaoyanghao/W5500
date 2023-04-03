/**
  ******************************************************************************
  * @file    periph_serial_mxuart.h
  * @author  zhy
  * @brief   This file contains all the functions prototypes for the extend uart
  *          config driver.
  ******************************************************************************
  * @attention   None
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PERIPH_SERIAL_W5500_H
#define PERIPH_SERIAL_W5500_H

//#define USEMXUART


#ifdef __cplusplus
 extern "C" {
#endif
   
/* Includes ------------------------------------------------------------------*/
#include "bsp_spi_drv.h" 
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/

/** @defgroup 
* @{
*/  
enum W55XX_PROTOCOL
{
  PROTOCOL_TCP,
  PROTOCOL_UDP,
  PROTOCOL_MACRAW, //only can use sock 0
};

enum W55XX_ERRO
{
    W55XX_NO_ERRO,
    W55XX_OPEN_ERRO = 0x05,
    W55XX_CONNET_ERRO,
    W55XX_ACCEPT_ERRO,
};
/**
* @}
*/

/* Peripheral Control functions  ************************************************/
int8_t Periph_W55xx_Init(void);
int8_t Periph_W55xx_Get_Link(void);
int8_t Periph_W55xx_Set_Mac(uint8_t *mac);
int8_t Periph_W55xx_Set_IP(uint8_t *ip, uint8_t *subnet, uint8_t *gateway);
int8_t Periph_W55xx_Mem_Alloc(uint8_t *tx_mem, uint8_t *rx_mem);
uint8_t  Periph_W55xx_Get_Socket_Status(uint8_t sockfd);
int8_t Periph_W55xx_Socket_Socket(uint8_t sockfd, uint8_t protocol,uint16_t local_port);
int8_t Periph_W55xx_Socket_Bind(uint8_t sockfd, uint16_t local_port);
int8_t Periph_W55xx_Socket_Listen(uint8_t sockfd);
int8_t Periph_W55xx_Socket_Accept(uint8_t  sockfd, uint8_t *connect_ip, uint16_t *connect_port);
int8_t Periph_W55xx_Socket_Connect(uint8_t  sockfd, uint8_t *dest_ip,uint16_t dest_port);
int8_t Periph_W55xx_Socket_Disconnect(uint8_t sockfd);
int8_t Periph_W55xx_Socket_Close(uint8_t sockfd);
uint16_t Periph_W55xx_Socket_Revlen(uint8_t sockfd);
uint16_t Periph_W55xx_Socket_Send(uint8_t sockfd,uint8_t *value, uint16_t len);
uint16_t Periph_W55xx_Socket_Rev(uint8_t sockfd,uint8_t *value, uint16_t len);
uint16_t Periph_W55xx_Socket_Sendto(uint8_t sockfd,uint8_t *value, uint16_t len, \
                                            uint8_t *dest_ip, uint16_t dest_port);
uint16_t Periph_W55xx_Socket_Recvfrom(uint8_t sockfd,uint8_t *value, uint16_t len, \
                                            uint8_t *from_ip, uint16_t *from_port);
void tcp_server_demo(void);
void tcp_client_demo(void);
void udp_demo(void);
#ifdef __cplusplus
}
#endif


#endif


/********************************END OF FILE***********************************/
