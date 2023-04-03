/**
******************************************************************************
* @file    ethrnet.c
* @author  zhy
* @brief   ethrnet function
*         
*
******************************************************************************
* @attention None
*
******************************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include "ethrnet.h"
#include "socket.h"

/* Private typedef -----------------------------------------------------------*/
 
/* Private variables ---------------------------------------------------------*/
static uint8_t eth_default_subnet[4]={255,255,255,0};
static uint8_t eth_default_ip[4]={192,168,1,88};
static uint8_t eth_default_gateway[4]={192,168,1,1};
static uint8_t eth_default_mac[6]={0x00,0x08,0xdc,0x11,0x11,0x11};
/*static uint8_t tcpc_buf[256] = {0};
static uint8_t tcps_buf[256] = {0};
static uint8_t udp_buf[256] = {0};
static struct  eth_fifo eth_tcps_fifo = {.empty = 1};
static struct  eth_fifo eth_tcpc_fifo = {.empty = 1};
static struct  eth_fifo eth_udp_fifo = {.empty = 1};
*/
/* Private function ----------------------------------------------------------*/
/**
* @brief  ethrnet init
* @retval None
*/
void Eth_Init(void)
{
    sock_set_eth_mac(eth_default_mac);
    sock_set_eth_ip(eth_default_ip, eth_default_subnet, eth_default_gateway);
}

/**
* @brief  Ethrnet TCP Server
* @retval None
*/
uint8_t tcp_server_buf[256] = {0};
struct sockaddr client_addr = {0};
void Eth_Tcp_Server_Thread(void)
{
   static int sockfd = -1;
   int sever_port = (0 << 16)|6601;
   static uint8_t sockfd_creat = 0;
   ssize_t recvlen = 0;
   if(socket_get_eth_linkstatus() == -1){
    return;
   } 
   if(sockfd_creat == 0){
     sockfd = socket(AF_INET,SOCK_STREAM,sever_port);
     if(sockfd == -1){
      return;
    }
   }
   sockfd_creat = 1;
   switch(socket_get_status(sockfd)){
        case SOCKCLOSED:
            sockfd_creat = 0;
        break;
        case SOCKINIT:
             bind(sockfd, &client_addr, sizeof(struct sockaddr));
             listen(sockfd,1);
        break;
        case SOCKESTABLISHED: 
             accept(sockfd, &client_addr, sizeof(struct sockaddr));
             recvlen = socket_get_recvlen(sockfd);
             if(recvlen > 0){
                recv(sockfd, tcp_server_buf, recvlen, 0);
                send(sockfd, tcp_server_buf, recvlen, 0);
             }
        break; 
        case SOCKCLOSE_WAIT:
            close(sockfd);
        break;
        default:
        break;
   }
}

/**
* @brief  Ethrnet TCP client
* @retval None
*/
uint8_t tcp_client_buf[256] = {0};
void Eth_Tcp_Client_Thread(void)
{
   static int sockfd = -1;
   int client_port = (1 << 16)| 5501;
   static uint8_t sockfd_creat = 0;
   ssize_t recvlen = 0;
   struct sockaddr server_addr = {.sa_family.ip[0] = 192,.sa_family.ip[1] = 168,  
                                        .sa_family.ip[2] = 1,.sa_family.ip[3] = 100,   
                                        .sa_family.port = 5502};
   if(socket_get_eth_linkstatus() == -1){
    return;
   } 
   if(sockfd_creat == 0){
     sockfd = socket(AF_INET,SOCK_STREAM,client_port);
     if(sockfd == -1){
      return;
    }
   }
   sockfd_creat = 1;
   switch(socket_get_status(sockfd)){
        case SOCKCLOSED:
            sockfd_creat = 0;
        break;
        case SOCKINIT:
             connect(sockfd, &server_addr, sizeof(struct sockaddr));
        break;
        case SOCKESTABLISHED: 
             recvlen = socket_get_recvlen(sockfd);
             if(recvlen > 0){
                recv(sockfd, tcp_client_buf, recvlen, 0);
                send(sockfd, tcp_client_buf, recvlen, 0);
             }
        break; 
        case SOCKCLOSE_WAIT:
            close(sockfd);
        break;
        default:
        break;
   }
}

/**
* @brief  Ethrnet UDP
* @retval None
*/
uint8_t udp_buf[256] = {0};
void Eth_Udp_Thread(void)
{
   static int sockfd = -1;
   int udp_port = (2 << 16)|8801;
   static uint8_t sockfd_creat = 0;
   ssize_t recvlen = 0;
   const struct sockaddr udp_addr = {.sa_family.ip[0] = 192,.sa_family.ip[1] = 168,  
                                     .sa_family.ip[2] = 1,.sa_family.ip[3] = 100, 
                                     .sa_family.port = 8802};
   if(socket_get_eth_linkstatus() == -1){
    return;
   } 
   if(sockfd_creat == 0){
     sockfd = socket(AF_INET,SOCK_DGRAM,udp_port);
     if(sockfd == -1){
      return;
    }
   }
   sockfd_creat = 1;
   switch(socket_get_status(sockfd)){
        case SOCKCLOSED:
            sockfd_creat = 0;
        break;
        case SOCKUDP: 
             recvlen = socket_get_recvlen(sockfd);
             if(recvlen > 0){
                recvfrom(sockfd, udp_buf, recvlen, 0, &udp_addr, sizeof(struct sockaddr));
                sendto(sockfd, udp_buf, recvlen-8, 0, &udp_addr, sizeof(struct sockaddr));
             }
        break; 
        case SOCKCLOSE_WAIT:
            close(sockfd);
        break;
        default:
        break;
   }
}





/********************************END OF FILE***********************************/
