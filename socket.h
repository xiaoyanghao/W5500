/**
  ******************************************************************************
  * @file    socket.h
  * @author  zhy
  * @brief   socket functions
  *          
  ******************************************************************************
  * @attention   None
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SOCKET_H
#define SOCKET_H

#ifdef __cplusplus
 extern "C" {
#endif 
/* Includes ------------------------------------------------------------------*/  
#include "stm32f1xx_hal.h"
/* Exported types ------------------------------------------------------------*/
/** @defgroup   socket type
* @{
*/  

 /**
  * @brief  domain
*/ 
 typedef enum
 {              
  AF_INET,
  AF_INET6,
  AF_LOCAL,
}DOMAIN; 

 /**
  * @brief  type
*/
 typedef enum
 {              
  SOCK_STREAM,
  SOCK_DGRAM,
  SOCK_RAW,  //MAC only socket 0 use
}TYPE; 

 /**
  * @brief  sa_family_t
*/ 
 typedef struct
 {              
  uint8_t ip[4];
  uint16_t port;
}sa_family_t;

 /**
  * @brief  sockaddr
*/ 
struct sockaddr
{
  uint8_t sa_len;
  sa_family_t sa_family;
  char sa_data[14];

};

typedef uint32_t socklen_t;
typedef int ssize_t;

 /**
* @}
*/    
/*MACRO-----------------------------------------------------------------------*/ 
#define SOCKETMAXNUM       8
#define SOCKCLOSED        0x00
#define SOCKINIT          0x13
#define SOCKLISTEN        0x14
#define SOCKESTABLISHED   0x17
#define SOCKCLOSE_WAIT    0x1C
#define SOCKUDP           0x22
#define SOCKMACRAW        0x42
#define SOCKSYNSENT       0x15
#define SOCKSYNRECV       0x16
#define SOCKFIN_WAIT      0x18
#define SOCKFIN_CLOSING   0x1A
#define SOCKFINTIME_WAIT  0x1B
#define SOCKLAST_ACK      0x1D

/* Peripheral Control functions  ************************************************/
int socket(int domain, int type, int protocol);
int bind(int sockfd, const struct sockaddr *addr, socklen_t  addrlen);
int listen(int sockfd, int backlog);
int accept(int sockfd, struct sockaddr *addr, socklen_t  addrlen);
int connect(int sockfd, const struct sockaddr *addr, socklen_t  addrlen);
int close(int sockfd);
ssize_t send(int sockfd, const void *buf, size_t len, int flags);
ssize_t sendto(int sockfd, const void *buf, size_t len, int flags,\
                                     const struct sockaddr *addr, socklen_t  addrlen);
ssize_t recv(int sockfd, const void *buf, size_t len, int flags);                                   
ssize_t recvfrom(int sockfd, const void *buf, size_t len, int flags,\
                                     const struct sockaddr *addr, socklen_t  addrlen);
ssize_t socket_get_recvlen(int sockfd);
int8_t socket_get_eth_linkstatus(void);
int8_t sock_set_eth_mac(uint8_t *mac);                                  
int8_t sock_set_eth_ip(uint8_t *ip, uint8_t *subnet, uint8_t *gateway);
uint8_t socket_get_status(uint8_t sockfd);

#ifdef __cplusplus
}
#endif



#endif 
/********************************END OF FILE***********************************/
