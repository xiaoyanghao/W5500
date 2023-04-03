/**
******************************************************************************
* @file    socket.c
* @author  zhy
* @brief   socket function
*         
*
******************************************************************************
* @attention None
*
******************************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include "socket.h"
#include "periph_serial_w5500.h"

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function ----------------------------------------------------------*/
/**
* @brief  socket
* @param  domain enum DOMAIN
          type   enum TYPE
          
          protocol - local_port
                   - use socketfd 
                  ( socketfd << 16)|local_port
* @retval sockfd  successful
          -1    failed  
*/
int socket(int domain, int type, int protocol)
{
   int8_t ret = -1;
   int sockfd = -1; 
   if(domain != AF_INET && domain != AF_INET6 && domain != AF_LOCAL){
    return -1;
   }
   if((type != SOCK_DGRAM) && (type != SOCK_STREAM ) && (type != SOCK_RAW )){
    return -1;
   }
   
   if(((protocol & 0xffff0000) >> 16) > SOCKETMAXNUM){
    return -1;
   }
   sockfd = ((protocol & 0xffff0000) >> 16);
   switch(type){
        case SOCK_DGRAM:
               ret = Periph_W55xx_Socket_Socket((uint8_t)sockfd, PROTOCOL_UDP, (uint16_t)(protocol & 0xffff));  
        break;
        case SOCK_STREAM:
               ret = Periph_W55xx_Socket_Socket((uint8_t)sockfd, PROTOCOL_TCP, (uint16_t)(protocol & 0xffff)); 
        break; 
        case SOCK_RAW:
              if(sockfd == 0){
               ret =  Periph_W55xx_Socket_Socket((uint8_t)sockfd, PROTOCOL_MACRAW, (uint16_t)(protocol & 0xffff)); 
              }
        break; 
        default:
        break;            
    }
    if(ret == W55XX_OPEN_ERRO || ret == -1 ){
        return -1;
    }
    return sockfd;
}

/**
* @brief  bind
* @param  sockfd socket fd
          addr  address of struct sockaddr
          addrlen sizeof  struct sockaddr
* @retval 0  successful
          -1    failed  
*/
int bind(int sockfd, const struct sockaddr *addr, socklen_t  addrlen)
{
    if(sockfd >= SOCKETMAXNUM||sizeof(struct sockaddr)!=addrlen){
        return -1;
    }
    return 0;
}

/**
* @brief  listen
* @param  sockfd socket fd
          backlog max connet num
* @retval 0  successful
          -1    failed 
* @note backlog must be 1
*/
int listen(int sockfd, int backlog)
{
    if(sockfd >= SOCKETMAXNUM || backlog!=1){
       return -1; 
    }
    if(Periph_W55xx_Socket_Listen((uint8_t)sockfd) == -1){
       return -1; 
    }
    return 0;
}

/**
* @brief  accept
* @param  sockfd socket fd
          addr  address of struct sockaddr
          addrlen sizeof  struct sockaddr
* @retval sockfd  successful
          -1    failed 
*/
int accept(int sockfd, struct sockaddr *addr, socklen_t  addrlen)
{
    if(sockfd >= SOCKETMAXNUM||sizeof(struct sockaddr)!=addrlen){
        return -1;
    } 
   if(Periph_W55xx_Socket_Accept((uint8_t)sockfd, (uint8_t *)addr->sa_family.ip, \
                                 (uint16_t *) &addr->sa_family.port)!=0){
        return -1;
   }
   return sockfd;
}

/**
* @brief  connect
* @param  sockfd socket fd
          addr  address of struct sockaddr
          addrlen sizeof  struct sockaddr
* @retval 0     successful
          -1    failed 
*/
int connect(int sockfd, const struct sockaddr *addr, socklen_t  addrlen)
{
    if(sockfd >= SOCKETMAXNUM||sizeof(struct sockaddr)!=addrlen){
        return -1;
    } 
    if(Periph_W55xx_Socket_Connect((uint8_t)sockfd, (uint8_t *)addr->sa_family.ip, (uint16_t)addr->sa_family.port) != 0){
        return -1;
    }
    return 0;
}

/**
* @brief  close
* @param  sockfd socket fd
          backlog max connet num
* @retval 0  successful
          -1    failed 
* @note backlog must be 1
*/
int close(int sockfd)
{
    if(sockfd >= SOCKETMAXNUM){
        return -1;
    } 
    if(Periph_W55xx_Socket_Close((uint8_t)sockfd) != 0){
        return -1;
    }
    return 0;
}

/**
* @brief  send.
* @param  s: socket number.
*         buf: data buffer to send.
*	      len: data length.
*         flags: noblock
                 block
* @retval  data length -successful  
          -1-failed    
*/
ssize_t send(int sockfd, const void *buf, size_t len, int flags)
{
    uint16_t ret = 0;
    if(sockfd >= SOCKETMAXNUM||len<=0){
        return -1;
    } 
    ret = Periph_W55xx_Socket_Send((uint8_t)sockfd, (uint8_t *)buf, len);
    if( ret == 0){
        return -1;
    }
    return (ssize_t)ret;
}

/**
* @brief  sendto.
* @param  s: socket number.
*         buf: data buffer to send.
*	      len: data length.
*         flags: noblock
                 block
          addr  address of struct sockaddr
          addrlen sizeof  struct sockaddr                 
* @retval  data length -successful  
          -1-failed    
*/
ssize_t sendto(int sockfd, const void *buf, size_t len, int flags,\
                                     const struct sockaddr *addr, socklen_t  addrlen)
{
    uint16_t ret = 0;
    if(sockfd >= SOCKETMAXNUM||len<=0 || sizeof(struct sockaddr)!=addrlen){
        return -1;
    } 
    ret = Periph_W55xx_Socket_Sendto((uint8_t)sockfd, (uint8_t *)buf, len, (uint8_t *)addr->sa_family.ip, (uint16_t)addr->sa_family.port);
    if( ret == 0){
        return -1;
    }
    return (ssize_t)ret;  
}

/**
* @brief  recv.
* @param  s: socket number.
*         buf: data buffer to send.
*	      len: data length.
*         flags: noblock
                 block
* @retval  data length -successful  
          -1-failed    
*/
ssize_t recv(int sockfd, const void *buf, size_t len, int flags)
{
    uint16_t ret = 0;
    if(sockfd >= SOCKETMAXNUM||len<=0){
        return -1;
    } 
    ret = Periph_W55xx_Socket_Rev((uint8_t)sockfd, (uint8_t *)buf, (uint16_t)len);
    if( ret == 0){
        return -1;
    }
    return (ssize_t)ret;  
}

/**
* @brief  recvfrom.
* @param  s: socket number.
*         buf: data buffer to send.
*	      len: data length.
*         flags: noblock
                 block
          addr  address of struct sockaddr
          addrlen sizeof  struct sockaddr                 
* @retval  data length -successful  
          -1-failed    
*/
ssize_t recvfrom(int sockfd, const void *buf, size_t len, int flags,\
                                     const struct sockaddr *addr, socklen_t  addrlen)
{
    uint16_t ret = 0;
    if(sockfd >= SOCKETMAXNUM||len<=0 || sizeof(struct sockaddr)!=addrlen){
        return -1;
    } 
    ret = Periph_W55xx_Socket_Recvfrom((uint8_t)sockfd, (uint8_t *)buf, len, (uint8_t *)addr->sa_family.ip, (uint16_t *)&addr->sa_family.port);
    if( ret == 0){
        return -1;
    }
    return (ssize_t)ret;  
}

/**
* @brief  socket get recv data len .
* @param  s: socket number.              
* @retval  data length -successful  
          -1-failed    
*/
ssize_t socket_get_recvlen(int sockfd)
{ 
    if(sockfd >= SOCKETMAXNUM){
        return -1;
    } 
    return (ssize_t)Periph_W55xx_Socket_Revlen((uint8_t)sockfd);
}

/**
* @brief  socket get network link status.           
* @retval  0 -successful  
          -1-failed    
*/
int8_t socket_get_eth_linkstatus(void)
{
    return Periph_W55xx_Get_Link();
}

/**
* @brief   set mac.  
* @retval  0-successful  
          -1-failed    
*/
int8_t sock_set_eth_mac(uint8_t *mac)
{
   return Periph_W55xx_Set_Mac(mac);
}
/**
* @brief   set IP.  
* @retval  0-successful  
          -1-failed    
*/
int8_t sock_set_eth_ip(uint8_t *ip, uint8_t *subnet, uint8_t *gateway)
{
    return Periph_W55xx_Set_IP(ip, subnet, gateway);
}

/**
* @brief  socket get status.
* @param  sockfd            
* @retval socket status 
*/
uint8_t socket_get_status(uint8_t sockfd)
{
    if(sockfd >= SOCKETMAXNUM){
        return 0;
    }    
   return  Periph_W55xx_Get_Socket_Status(sockfd);
}



/********************************END OF FILE***********************************/
