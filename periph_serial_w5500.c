/**
  ******************************************************************************
  * @file    periph_serial_W5500.c
  * @author  zhy
  * @brief   W55OO config.
  *          This is the common part of the extend  uart periph initialization
  *
  @verbatim
  ==============================================================================
                     ##### How to use this driver #####
  ==============================================================================
    [..]
    The common HAL driver contains a set of generic and common APIs that can be
    used by the PPP peripheral drivers and the user to start using the HAL.
    [..]
    The HAL contains two APIs' categories:
         (+) Common HAL APIs
         (+) Services HAL APIs

  @endverbatim
  ******************************************************************************
  * @attention   1) SPI CLOCK MAX 33.3MHz (36MHZ ok)
   
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "periph_serial_w5500.h"  
#include <stdlib.h>   
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/** @defgroup extend uart Types
* @{
*/  

typedef struct 
{
  
  SpiClient_t *client;
  volatile uint32_t spiFlag;
  volatile uint64_t lastUpdate;
  uint8_t error;
  uint8_t enabled;
  
} W55XX_T; 
 
/**
* @}
*/  
/* Private define ------------------------------------------------------------*/
#define SOCKETNUM                   8

/* --------------------------Global register define --------------------------*/
#define MR                          (0x000000)
/**brief Gateway IP Register address*/
#define GAR0                        (0x000100)
#define GAR1                        (0x000200)
#define GAR2                        (0x000300)
#define GAR3                        (0x000400)
/**brief Subnet mask Register address*/
#define SUBR0                       (0x000500)
#define SUBR1                       (0x000600)
#define SUBR2                       (0x000700)
#define SUBR3                       (0x000800)
/**brief Source MAC Register address*/
#define SHAR0                       (0x000900)
#define SHAR1                       (0x000A00)
#define SHAR2                       (0x000B00)
#define SHAR3                       (0x000C00)
#define SHAR4                       (0x000D00)
#define SHAR5                       (0x000E00)
/**@brief Source IP Register address*/
#define SIPR0                       (0x000F00)
#define SIPR1                       (0x001000)
#define SIPR2                       (0x001100)
#define SIPR3                       (0x001200)
/**@brief set Interrupt low level timer register address*/
#define INTLEVEL0                   (0x001300)
#define INTLEVEL1                   (0x001400)
/**@brief Interrupt Register*/
#define IR                          (0x001500)
/**@brief Interrupt mask register*/
#define IMR                         (0x001600)
/**@brief Socket Interrupt Register*/
#define SIR                         (0x001700) 
/**@brief Socket Interrupt Mask Register*/
#define SIMR                        (0x001800)
/**@brief Timeout register address( 1 is 100us )*/
#define RTR0                        (0x001900)
#define RTR1                        (0x001A00)
/**@brief Retry count reigster*/
#define WIZ_RCR                      (0x001B00)
/**@brief PPP LCP Request Timer register  in PPPoE mode*/
#define PTIMER                      (0x001C00)
/**@brief PPP LCP Magic number register  in PPPoE mode*/
#define PMAGIC                      (0x001D00)
/**@brief PPP Destination MAC Register address*/
#define PDHAR0                      (0x001E00)
#define PDHAR1                      (0x001F00)
#define PDHAR2                      (0x002000)
#define PDHAR3                      (0x002100)
#define PDHAR4                      (0x002200)
#define PDHAR5                      (0x002300)
/**
 @brief PPP Session Identification Register
 */
#define PSID0                       (0x002400)
#define PSID1                       (0x002500)
/**@brief PPP Maximum Segment Size(MSS) register*/
#define PMR0                        (0x002600)
#define PMR1                        (0x002700)
/**@brief Unreachable IP register address in UDP mode*/
#define UIPR0                       (0x002800)
#define UIPR1                       (0x002900)
#define UIPR2                       (0x002A00)
#define UIPR3                       (0x002B00)
/**@brief Unreachable Port register address in UDP mode*/
#define UPORT0                      (0x002C00)
#define UPORT1                      (0x002D00)
/**@brief PHY Configuration Register*/
#define PHYCFGR                      (0x002E00)
/**@brief chip version register address*/
#define VERSIONR                    (0x003900)  

/* --------------Global register  bitdefines ---------------------------------*/
#define W55XX_VER                    0x04


/* --------------------------socket register defines -------------------------*/
/**@brief socket Mode register*/
#define Sn_MR(ch)                       (0x000008 + (ch<<5))
/**@brief channel Sn_CR register*/
#define Sn_CR(ch)                       (0x000108 + (ch<<5))
/**@brief channel interrupt register*/
#define Sn_IR(ch)                       (0x000208 + (ch<<5))
/**@brief channel status register*/
#define Sn_SR(ch)                       (0x000308 + (ch<<5))
/**@brief source port register*/
#define Sn_PORT0(ch)                    (0x000408 + (ch<<5))
#define Sn_PORT1(ch)                    (0x000508 + (ch<<5))
/**@brief Peer MAC register address*/
#define Sn_DHAR0(ch)                    (0x000608 + (ch<<5))
#define Sn_DHAR1(ch)                    (0x000708 + (ch<<5))
#define Sn_DHAR2(ch)                    (0x000808 + (ch<<5))
#define Sn_DHAR3(ch)                    (0x000908 + (ch<<5))
#define Sn_DHAR4(ch)                    (0x000A08 + (ch<<5))
#define Sn_DHAR5(ch)                    (0x000B08 + (ch<<5))
/**@brief Peer IP register address*/
#define Sn_DIPR0(ch)                    (0x000C08 + (ch<<5))
#define Sn_DIPR1(ch)                    (0x000D08 + (ch<<5))
#define Sn_DIPR2(ch)                    (0x000E08 + (ch<<5))
#define Sn_DIPR3(ch)                    (0x000F08 + (ch<<5))
/**@brief Peer port register address*/
#define Sn_DPORT0(ch)                   (0x001008 + (ch<<5))
#define Sn_DPORT1(ch)                   (0x001108 + (ch<<5))
/**@brief Maximum Segment Size(Sn_MSSR0) register address*/
#define Sn_MSSR0(ch)                    (0x001208 + (ch<<5))
#define Sn_MSSR1(ch)                    (0x001308 + (ch<<5))

#define Sn_PROTO(ch)                    (0x001408 + (ch<<5))
/** @brief IP Type of Service(TOS) Register */
#define Sn_TOS(ch)                      (0x001508 + (ch<<5))
/**@brief IP Time to live(TTL) Register */
#define Sn_TTL(ch)                      (0x001608 + (ch<<5))
/**@brief Receive memory size reigster*/
#define Sn_RXMEM_SIZE(ch)               (0x001E08 + (ch<<5))
/**@brief Transmit memory size reigster*/
#define Sn_TXMEM_SIZE(ch)               (0x001F08 + (ch<<5))
/**@brief Transmit free memory size register*/
#define Sn_TX_FSR0(ch)                  (0x002008 + (ch<<5))
#define Sn_TX_FSR1(ch)                  (0x002108 + (ch<<5))
/**
 @brief Transmit memory read pointer register address
 */
#define Sn_TX_RD0(ch)                   (0x002208 + (ch<<5))
#define Sn_TX_RD1(ch)                   (0x002308 + (ch<<5))
/**@brief Transmit memory write pointer register address*/
#define Sn_TX_WR0(ch)                   (0x002408 + (ch<<5))
#define Sn_TX_WR1(ch)                   (0x002508 + (ch<<5))
/**@brief Received data size register*/
#define Sn_RX_RSR0(ch)                  (0x002608 + (ch<<5))
#define Sn_RX_RSR1(ch)                  (0x002708 + (ch<<5))
/**@brief Read point of Receive memory*/
#define Sn_RX_RD0(ch)                   (0x002808 + (ch<<5))
#define Sn_RX_RD1(ch)                   (0x002908 + (ch<<5))
/**@brief Write point of Receive memory*/
#define Sn_RX_WR0(ch)                   (0x002A08 + (ch<<5))
#define Sn_RX_WR1(ch)                   (0x002B08 + (ch<<5))
/**@brief socket interrupt mask register*/
#define Sn_IMR(ch)                      (0x002C08 + (ch<<5))
/**@brief frag field value in IP header register*/
#define Sn_FRAG(ch)                     (0x002D08 + (ch<<5))
/**@brief Keep Timer register*/
#define Sn_KPALVTR(ch)                  (0x002F08 + (ch<<5))

/* ---------socket register bitdefines ---------------------------------------*/
/* Sn_MR values */
#define Sn_MR_CLOSE                  0x00     /**< unused socket */
#define Sn_MR_TCP                    0x01     /**< TCP */
#define Sn_MR_UDP                    0x02     /**< UDP */
#define Sn_MR_IPRAW                  0x03      /**< IP LAYER RAW SOCK */
#define Sn_MR_MACRAW                 0x04      /**< MAC LAYER RAW SOCK */
#define Sn_MR_PPPOE                  0x05     /**< PPPoE */
#define Sn_MR_UCASTB                 0x10     /**< Unicast Block in UDP Multicating*/
#define Sn_MR_ND                     0x20     /**< No Delayed Ack(TCP) flag */
#define Sn_MR_MC                     0x20     /**< Multicast IGMP (UDP) flag */
#define Sn_MR_BCASTB                 0x40     /**< Broadcast blcok in UDP Multicating */
#define Sn_MR_MULTI                  0x80     /**< support UDP Multicating */

 /* Sn_MR values on MACRAW MODE */
#define Sn_MR_MIP6N                  0x10     /**< IPv6 packet Block */
#define Sn_MR_MMB                    0x20     /**< IPv4 Multicasting Block */
//#define Sn_MR_BCASTB                 0x40     /**< Broadcast blcok */
#define Sn_MR_MFEN                   0x80     /**< support MAC filter enable */

/* Sn_CR values */
#define Sn_CR_OPEN                   0x01     /**< initialize or open socket */
#define Sn_CR_LISTEN                 0x02     /**< wait connection request in tcp mode(Server mode) */
#define Sn_CR_CONNECT                0x04     /**< send connection request in tcp mode(Client mode) */
#define Sn_CR_DISCON                 0x08     /**< send closing reqeuset in tcp mode */
#define Sn_CR_CLOSE                  0x10     /**< close socket */
#define Sn_CR_SEND                   0x20     /**< update txbuf pointer, send data */
#define Sn_CR_SEND_MAC               0x21     /**< send data with MAC address, so without ARP process */
#define Sn_CR_SEND_KEEP              0x22     /**<  send keep alive message */
#define Sn_CR_RECV                   0x40     /**< update rxbuf pointer, recv data */

#define Sn_IR_SEND_OK                0x10     /**< complete sending */
#define Sn_IR_TIMEOUT                0x08     /**< assert timeout */
#define Sn_IR_RECV                   0x04     /**< receiving data */
#define Sn_IR_DISCON                 0x02     /**< closed socket */
#define Sn_IR_CON                    0x01     /**< established connection */

/* Sn_SR values */
#define SOCK_CLOSED                  0x00     /**< closed */
#define SOCK_INIT                    0x13     /**< init state */
#define SOCK_LISTEN                  0x14     /**< listen state */
#define SOCK_SYNSENT                 0x15     /**< connection state */
#define SOCK_SYNRECV                 0x16     /**< connection state */
#define SOCK_ESTABLISHED             0x17     /**< success to connect */
#define SOCK_FIN_WAIT                0x18     /**< closing state */
#define SOCK_CLOSING                 0x1A     /**< closing state */
#define SOCK_TIME_WAIT               0x1B     /**< closing state */
#define SOCK_CLOSE_WAIT              0x1C     /**< closing state */
#define SOCK_LAST_ACK                0x1D     /**< closing state */
#define SOCK_UDP                     0x22     /**< udp socket */
#define SOCK_IPRAW                   0x32     /**< ip raw mode socket */
#define SOCK_MACRAW                  0x42     /**< mac raw mode socket */
#define SOCK_PPPOE                   0x5F     /**< pppoe socket */

/* --------------------------Connect gpio define -----------------------------*/
#define W55xx_CS_CLK_ENABLE()           __HAL_RCC_GPIOA_CLK_ENABLE()
#define W55xx_CS_PORT                   GPIOA
#define W55xx_CS_PIN                    GPIO_PIN_4
#define W55xx_RST_CLK_ENABLE()          __HAL_RCC_GPIOC_CLK_ENABLE()
#define W55xx_RST_PORT                  GPIOC
#define W55xx_RST_PIN                   GPIO_PIN_5
#define W55xx_INT_CLK_ENABLE()          __HAL_RCC_GPIOC_CLK_ENABLE()
#define W55xx_INT_PORT                  GPIOC
#define W55xx_INT_PIN                   GPIO_PIN_4

/* Private variables ---------------------------------------------------------*/
static W55XX_T w55xx_t = {.error = 1};
static uint8_t w55xx_default_tmem[8]={2,2,2,2,2,2,2,2};
static uint8_t w55xx_default_rmem[8]={2,2,2,2,2,2,2,2};
/* Private function prototypes -----------------------------------------------*/
static void periph_w55xx_gpio_init(void);
static void periph_w55xx_rst(void);
static void periph_w55xx_setreg(uint32_t reg, uint8_t *value, uint16_t len);
static void periph_w55xx_getreg(uint32_t reg, uint8_t *value, uint16_t len);
static int8_t periph_w55xx_verify_setreg(uint32_t reg, uint8_t *value, uint16_t len);
static int8_t periph_w55xx_write_mem(uint8_t sockfd, uint8_t *value, uint16_t len);
static int8_t periph_w55xx_read_mem(uint8_t sockfd, uint8_t *value, uint16_t len);

/* Private functions ---------------------------------------------------------*/
/**
* @brief  W5500 init.  
* @retval  0-successful  
          -1-failed    
*/
int8_t Periph_W55xx_Init(void)
{
  
  uint8_t w55xx_ver = 0x00;
  int8_t ret = -1;
  periph_w55xx_gpio_init();
  w55xx_t.client = Bsp_Spi_Serial_Open(SPI1_ID, SPI_BAUDRATEPRESCALER_2, \
                                   SPI_MO, W55xx_CS_PORT, W55xx_CS_PIN, \
                                                &w55xx_t.spiFlag, NULL);
  if(w55xx_t.client != NULL){
    w55xx_t.enabled = 1;
    periph_w55xx_rst();
    periph_w55xx_getreg(VERSIONR, &w55xx_ver, 1);
    if(w55xx_ver == W55XX_VER){
    Periph_W55xx_Mem_Alloc(w55xx_default_tmem,w55xx_default_rmem);
     w55xx_t.error = 0;
     ret = 0;
    }  
  } 
  return ret;
}
/**
* @brief  W5500 set mac.  
* @retval  0-successful  
          -1-failed    
*/
int8_t Periph_W55xx_Get_Link(void)
{
   int8_t ret = -1;
   uint8_t phy_value;
   periph_w55xx_getreg(PHYCFGR, &phy_value, 1);
   if(phy_value & (0x01 << 0)){
     ret = 0;
   }
   return ret;
}

/**
* @brief  W5500 set mac.  
* @retval  0-successful  
          -1-failed    
*/
int8_t Periph_W55xx_Set_Mac(uint8_t *mac)
{
   return periph_w55xx_verify_setreg(SHAR0, mac, 6);
}

/**
* @brief  W5500 set IP.  
* @retval  0-successful  
          -1-failed    
*/
int8_t Periph_W55xx_Set_IP(uint8_t *ip, uint8_t *subnet, uint8_t *gateway)
{
  int8_t ret = -1;
  ret = periph_w55xx_verify_setreg(SIPR0, ip, 4);
  if(ret == -1){
    goto w55xx_ip_erro;
  }
  ret = periph_w55xx_verify_setreg(SUBR0, subnet, 4);
  if(ret == -1){
    goto w55xx_ip_erro;
  }
  ret = periph_w55xx_verify_setreg(GAR0, gateway, 4);
  if(ret == -1){
    goto w55xx_ip_erro;
  } 
w55xx_ip_erro:
  return ret;
}

/**
* @brief  W5500 alloc memory. 
* @param  tx_mem   addr of tx buff size (unit kbyte) 0/2/4/8/16 
          rx_mem   addr of tx buff size (unit kbyte) 0/2/4/8/16
* @retval 0-successful  
          -1-failed   
* @note  TMSR and RMSR bits are as followsMaximum memory size for Tx, Rx in the W5500 is 16K Bytes,
	 In the range of 16KBytes, the memory size could be allocated dynamically by each channel.
	 Be attentive to sum of memory size shouldn't exceed 16Kbytes
	 and to data transmission and receiption from non-allocated channel may cause some problems.
	 If the 16KBytes memory is already  assigned to centain channel, 
	 other 3 channels couldn't be used, for there's no available memory.
	 If two 4KBytes memory are assigned to two each channels, 
	 other 2 channels couldn't be used, for there's no available memory.
*/
int8_t Periph_W55xx_Mem_Alloc(uint8_t *tx_mem, uint8_t *rx_mem)
{
   uint8_t tx_mem_sum = 0, rx_mem_sum = 0;
   uint8_t index = 0;
   int8_t tret = -1,rret = -1;
   int8_t ret = -1;
   for(index = 0; index < SOCKETNUM; index++){
     tx_mem_sum += tx_mem[index];
     rx_mem_sum += rx_mem[index];
   }
   if(tx_mem_sum <= 16 && rx_mem_sum <= 16){
     for(index = 0;index < SOCKETNUM; index++){
       tret = periph_w55xx_verify_setreg(Sn_TXMEM_SIZE(index), &tx_mem[index], 1);
       rret = periph_w55xx_verify_setreg(Sn_RXMEM_SIZE(index), &rx_mem[index], 1);
       if(tret == -1 || rret == -1){
         break;
       }
     }
     if(tret == 0 && rret == 0){
       ret = 0;
     }
   }
   return ret;
}

/**
* @brief  W5500 socket.
* @param  sockfd            
* @retval socket status 
*/
uint8_t  Periph_W55xx_Get_Socket_Status(uint8_t sockfd)
{
    uint8_t sr_value = 0;
    periph_w55xx_getreg(Sn_SR(sockfd), &sr_value, 1);
    return sr_value;
}

/**
* @brief  W5500 socket.
* @param  protocol enum W55XX_PROTOCOL 
          local_port local_port             
* @retval  sockfd-successful  
          -1-failed    
*/
int8_t Periph_W55xx_Socket_Socket(uint8_t sockfd, uint8_t protocol,uint16_t local_port)
{
  int8_t ret = -1;
  uint8_t w_protocol = 0,dec = 5;
  uint8_t value = Sn_CR_OPEN;
  uint8_t w_sr = 0;
  uint8_t tmp_port[2] = {0};
  if(sockfd < SOCKETNUM){
    switch(protocol){
    case PROTOCOL_TCP :
      w_protocol = Sn_MR_TCP|Sn_MR_ND;
      w_sr = SOCK_INIT;
      break;
    case PROTOCOL_UDP :
      w_protocol = Sn_MR_UDP;
      w_sr = SOCK_UDP;
      break; 
    case PROTOCOL_MACRAW :
      w_protocol = Sn_MR_MACRAW;
      w_sr = SOCK_MACRAW;
      break;
    default:
      break;
    }
    tmp_port[0] = (uint8_t)((local_port & 0xff00) >> 8);
    tmp_port[1] = (uint8_t)(local_port & 0x00ff);
    ret = periph_w55xx_verify_setreg(Sn_PORT0(sockfd), &tmp_port[0], 2);
    if(ret != -1){
      ret = periph_w55xx_verify_setreg(Sn_MR(sockfd), &w_protocol, 1);
      if(ret != -1){
        periph_w55xx_setreg(Sn_CR(sockfd), &value, 1);
        do{
          dec--;
          periph_w55xx_getreg(Sn_SR(sockfd), &value, 1);
        }while((value != w_sr) && (dec != 0));
        if(dec != 0){
          ret = 0;
          value = 0xff;
          periph_w55xx_setreg(Sn_IR(sockfd), &value, 1);
        }
        else{
          ret = W55XX_OPEN_ERRO; 
        }
      }
    }
  }
  return ret;
}

/**
* @brief  W5500 Bind.
* @param  sockfd  
          local_port addr of local port
* @retval  sockfd-successful  
          -1-failed    
*/
int8_t Periph_W55xx_Socket_Bind(uint8_t sockfd, uint16_t local_port)
{
  uint8_t tmp_port[2] = {0};
  tmp_port[0] = (uint8_t)((local_port & 0xff00) >> 8);
  tmp_port[1] = (uint8_t)(local_port & 0x00ff);
  return periph_w55xx_verify_setreg(Sn_PORT0(sockfd), &tmp_port[0], 2);
}

/**
* @brief  W5500 Listen.
* @param  sockfd  
          local_port addr of local port
* @retval  sockfd-successful  
          -1-failed    
*/
int8_t Periph_W55xx_Socket_Listen(uint8_t sockfd)
{
   int8_t ret = -1;
   uint8_t value = Sn_CR_LISTEN, dec = 5;
   periph_w55xx_setreg(Sn_CR(sockfd), &value, 1);
   do{
     dec--;
     periph_w55xx_getreg(Sn_SR(sockfd), &value, 1);
   }while( (value != SOCK_LISTEN) && (dec != 0));
   if(dec != 0){
     ret = 0;
     value = 0xff;
     periph_w55xx_setreg(Sn_IR(sockfd), &value, 1);
   }
   return ret;
}

/**
* @brief  W5500 Accept.
* @param  sockfd  
          local_port addr of local port
* @retval  sockfd-successful  
          -1-failed
            
*/
int8_t Periph_W55xx_Socket_Accept(uint8_t  sockfd, uint8_t *connect_ip, uint16_t *connect_port)
{
   int8_t ret = -1;
   uint8_t sr_value = 0x00;
   uint8_t temp_buf[2] = {0};
   periph_w55xx_getreg(Sn_SR(sockfd), &sr_value, 1);
   if(sr_value == SOCK_ESTABLISHED){
     ret = 0;
     periph_w55xx_getreg(Sn_DIPR0(sockfd), connect_ip, 4);
     periph_w55xx_getreg(Sn_DPORT0(sockfd), &temp_buf[0], 2);
     *connect_port = (temp_buf[0] << 8) | temp_buf[1];
   }
   if(sr_value == SOCK_CLOSED){
     ret = W55XX_ACCEPT_ERRO;
   }
   return ret;
}

/**
* @brief  W5500 Connect.
* @param  sockfd  
          dest_ip   addr of  destination ip
          dest_port addr of  destination port   
          local_port addr of local port
* @retval  sockfd-successful  
          -1-failed    
*/
uint8_t dest[4] = {0};
uint16_t dest_p = 0;
int8_t Periph_W55xx_Socket_Connect(uint8_t  sockfd, uint8_t *dest_ip,uint16_t dest_port)
{
  int8_t ret = -1;
  uint8_t tmp_port[2] = {0};
  uint8_t sr_value = 0,ir_value = Sn_CR_CONNECT;
  uint8_t dec = 0x10;
  memcpy(dest, dest_ip,4);
  dest_p = dest_port;
  periph_w55xx_setreg(Sn_DIPR0(sockfd), &dest_ip[0], 4);
  tmp_port[0] = (uint8_t)((dest_port & 0xff00) >> 8);
  tmp_port[1] = (uint8_t)(dest_port & 0x00ff);
  periph_w55xx_setreg(Sn_DPORT0(sockfd), &tmp_port[0], 2);
  periph_w55xx_setreg(Sn_CR(sockfd), &ir_value, 1);
  do{
     dec--;
     periph_w55xx_getreg(Sn_SR(sockfd), &sr_value, 1);
     periph_w55xx_getreg(Sn_IR(sockfd), &ir_value, 1);
     if((ir_value & Sn_IR_TIMEOUT) != 0 ||sr_value == SOCK_CLOSED){
       ir_value = Sn_IR_TIMEOUT;
       periph_w55xx_setreg(Sn_IR(sockfd), &ir_value, 1);
       ret = W55XX_CONNET_ERRO; 
       goto w55xx_connet_erro;
     }
  }while(sr_value != SOCK_ESTABLISHED && dec!= 0);
  if(dec == 0){
    ret = 0;
  }
w55xx_connet_erro:
  return ret;
}

/**
* @brief  W5500 Disconnect.
* @param  sockfd  
* @retval  sockfd-successful  
          -1-failed    
*/
int8_t Periph_W55xx_Socket_Disconnect(uint8_t sockfd)
{
  int8_t ret = -1;
  uint8_t value = Sn_CR_DISCON, dec = 0x10;
  periph_w55xx_setreg(Sn_CR(sockfd), &value, 1);
  do{
     dec--;
     periph_w55xx_getreg(Sn_SR(sockfd), &value, 1);
  }while( (value != SOCK_CLOSED) && (dec != 0));
  if(dec != 0){
     ret = 0;
     value = 0xff;
     periph_w55xx_setreg(Sn_IR(sockfd), &value, 1);
  }
  return ret;
}

/**
* @brief  W5500  close.
* @retval  0-successful  
          -1-failed    
*/
int8_t Periph_W55xx_Socket_Close(uint8_t sockfd)
{
   int8_t ret = -1;
   uint8_t value = Sn_CR_CLOSE, dec = 5;
   periph_w55xx_setreg(Sn_CR(sockfd), &value, 1);
   do{
     dec--;
     periph_w55xx_getreg(Sn_SR(sockfd), &value, 1);
   }while( (value != SOCK_CLOSED) && (dec != 0));
   if(dec != 0){
     ret = 0;
     value = 0xff;
     periph_w55xx_setreg(Sn_IR(sockfd), &value, 1);
   }
   return ret;
}

/**
* @brief  W5500  Send.
* @param  s: socket number.
*         value: data buffer to send.
*	  len: data length.
* @retval  data length -successful    
*/
uint16_t Periph_W55xx_Socket_Send(uint8_t sockfd,uint8_t *value, uint16_t len)
{
   uint16_t freesize = 0,dec = 0xfff,cr_dec = 30;
   uint8_t tmp_buf[2] = {0};
   uint8_t sr_value = 0x00,ir_value = 0x00;
   uint8_t cr_value = Sn_CR_SEND;
   uint16_t ret = 0;
   do{
     dec--;
     periph_w55xx_getreg(Sn_TX_FSR0(sockfd), tmp_buf, 2);
     freesize = (tmp_buf[0] << 8)| tmp_buf[1];
     periph_w55xx_getreg(Sn_SR(sockfd), &sr_value, 1);
    if((sr_value != SOCK_ESTABLISHED) && (sr_value != SOCK_CLOSE_WAIT) && \
       (sr_value != SOCK_UDP) && (sr_value != SOCK_MACRAW)|| dec == 0){   
      goto w55xx_send_erro;
    }
   }while (freesize < len);
   periph_w55xx_write_mem(sockfd, value, len);
   periph_w55xx_setreg(Sn_CR(sockfd), &cr_value, 1);
   do{
     cr_dec--;
     periph_w55xx_getreg(Sn_SR(sockfd), &sr_value, 1);
     periph_w55xx_getreg(Sn_IR(sockfd), &ir_value, 1);
    if ((sr_value != SOCK_ESTABLISHED) && (sr_value != SOCK_CLOSE_WAIT) && \
        (sr_value != SOCK_UDP) && (sr_value != SOCK_MACRAW)){
      goto w55xx_send_erro;     
    }  
   }while(!(ir_value & Sn_IR_SEND_OK) && (cr_dec != 0));
   if(cr_dec!=0){
     ir_value = Sn_IR_SEND_OK;
     periph_w55xx_setreg(Sn_IR(sockfd), &ir_value, 1);
     ret = len;
   }
w55xx_send_erro:
  return ret;
}

/**
* @brief  W5500  Rev len.
* @param  s: socket number.
* @retval  data length 
*/
uint16_t  Periph_W55xx_Socket_Revlen(uint8_t sockfd)
{
   uint16_t currnt_len = 0,next_len = 0;
   uint8_t tmep_buf[2] = {0};
   uint16_t dec = 0xfff;
   do{
      dec--;
      periph_w55xx_getreg(Sn_RX_RSR0(sockfd), &tmep_buf[0], 2);
      currnt_len = (tmep_buf[0] << 8)|tmep_buf[1];	  
      if (currnt_len != 0){
        periph_w55xx_getreg(Sn_RX_RSR0(sockfd), &tmep_buf[0], 2);
        next_len = (tmep_buf[0] << 8)|tmep_buf[1];
      }
   }while (currnt_len != next_len);
   return currnt_len;
}

/**
* @brief  W5500  Rev.
* @param  s: socket number.
*         value: data buffer to send.
*	  len: data length.
* @retval  data length    
*/
uint16_t Periph_W55xx_Socket_Rev(uint8_t sockfd,uint8_t *value, uint16_t len)
{
   uint8_t sr_value = 0x00;
   uint8_t cr_value = Sn_CR_RECV, cr_dec = 10;
   uint16_t ret = 0;
   periph_w55xx_read_mem(sockfd, value, len);
   periph_w55xx_setreg(Sn_CR(sockfd), &cr_value, 1);
   do{
     cr_dec--;
     periph_w55xx_getreg(Sn_SR(sockfd), &sr_value, 1);
     periph_w55xx_getreg(Sn_CR(sockfd), &cr_value, 1);
    if ((sr_value != SOCK_ESTABLISHED) && (sr_value != SOCK_CLOSE_WAIT) ){
      goto w55xx_read_erro;     
    }  
   }while((cr_value) && (cr_dec != 0));
   if(cr_dec!=0){
     ret = len;
   }
w55xx_read_erro:
  return ret;
}

/**
* @brief  W5500  Sendto.
* @param  s: socket number.
*         value: data buffer to send.
*	  len: data length.
*	  addr: IP address to send.
*	  port: IP port to send.
* @retval  data length  
*/
uint16_t Periph_W55xx_Socket_Sendto(uint8_t sockfd,uint8_t *value, uint16_t len, \
                                            uint8_t *dest_ip, uint16_t dest_port)
{
  uint16_t ret = 0;
  uint8_t temp_buf[2] = {0};
  periph_w55xx_setreg(Sn_DIPR0(sockfd), dest_ip, 4);
  temp_buf[0] = (uint8_t)((dest_port & 0xff00) >> 8);
  temp_buf[1] = (uint8_t)(dest_port & 0x00ff);
  periph_w55xx_setreg(Sn_DPORT0(sockfd), temp_buf, 2);
  ret = Periph_W55xx_Socket_Send(sockfd, value, len);
  return ret;
}

/**
*@brief   This function is an application I/F function which is used to receive the data in other then
	   TCP mode. This function is used to receive UDP, IP_RAW and MAC_RAW mode, and handle the header as well.
*@param		s: socket number.
*@param		buf: data buffer to receive.
*@param		len: data length.
*@param		addr: IP address to receive.
*@param		port: IP port to receive.
*@return	This function return received data size for success else 0.
*/
uint16_t Periph_W55xx_Socket_Recvfrom(uint8_t sockfd,uint8_t *value, uint16_t len, \
                                            uint8_t *from_ip, uint16_t *from_port)
{
  uint8_t head[8] = {0};
  uint8_t  temp_buf[2] = {0},mr_value = 0,ir_value = 0xff;
  uint16_t read_ptr = 0,ret = 0;
  uint32_t addr = 0;
  uint16_t datalen = 0;
  uint8_t sr_value = 0x00;
  uint8_t cr_value = Sn_CR_RECV, cr_dec = 10;
  periph_w55xx_getreg(Sn_RX_RD0(sockfd), temp_buf, 2);
  read_ptr = (temp_buf[0] << 8) | temp_buf[1];
  addr = (uint32_t)(read_ptr<<8) + (sockfd<<5) + 0x18;
  periph_w55xx_getreg(Sn_MR(sockfd), &mr_value, 1);
  switch (mr_value & 0x07){
   case Sn_MR_UDP:
     periph_w55xx_getreg(addr, head, 0x08);  
     read_ptr += 8;
     memcpy(from_ip, head, 4);
     *from_port  = (head[4] << 8) + head[5];
     datalen = (head[6] << 8) + head[7];
     addr = (uint32_t)(read_ptr<<8) +  (sockfd<<5) + 0x18;
     periph_w55xx_getreg(addr, value, datalen); 
     periph_w55xx_setreg(Sn_IR(sockfd), &ir_value, 1);
     read_ptr += datalen;
     temp_buf[0] = (uint8_t)((read_ptr & 0xff00) >> 8);
     temp_buf[1] = (uint8_t)(read_ptr & 0x00ff);
     periph_w55xx_setreg(Sn_RX_RD0(sockfd), temp_buf, 2);
     break;
   case Sn_MR_MACRAW:
     periph_w55xx_getreg(addr, head, 0x02);  
     read_ptr += 2;
     datalen = (head[0]<<8) + head[1] - 2;
     addr = (uint32_t)(read_ptr<<8) +  (sockfd<<5) + 0x18;
     periph_w55xx_getreg(addr, value, datalen); 
     periph_w55xx_setreg(Sn_IR(sockfd), &ir_value, 1);
     read_ptr += datalen;
     temp_buf[0] = (uint8_t)((read_ptr & 0xff00) >> 8);
     temp_buf[1] = (uint8_t)(read_ptr & 0x00ff); 
     periph_w55xx_setreg(Sn_RX_RD0(sockfd), temp_buf, 2);    
     break;  
   default:
     break;
  }
   periph_w55xx_setreg(Sn_CR(sockfd), &cr_value, 1);
   do{
     cr_dec--;
     periph_w55xx_getreg(Sn_SR(sockfd), &sr_value, 1);
     periph_w55xx_getreg(Sn_CR(sockfd), &cr_value, 1);
    if ((sr_value != SOCK_UDP) && (sr_value != SOCK_CLOSE_WAIT) && (sr_value != SOCK_MACRAW)){
      goto w55xx_recvfrom_erro;     
    }  
   }while((cr_value) && (cr_dec != 0));
   if(cr_dec!=0){
     ret = len;
   }  
 w55xx_recvfrom_erro:
  return ret; 
}

/**
* @brief  w5500 gpio init.  
*/
static void periph_w55xx_gpio_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  W55xx_CS_CLK_ENABLE();
  W55xx_RST_CLK_ENABLE();
  W55xx_INT_CLK_ENABLE();
  
  GPIO_InitStruct.Pin = W55xx_RST_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(W55xx_RST_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = W55xx_CS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(W55xx_CS_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = W55xx_INT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(W55xx_INT_PORT, &GPIO_InitStruct);
  
  HAL_GPIO_WritePin(W55xx_RST_PORT, W55xx_RST_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(W55xx_CS_PORT, W55xx_CS_PIN, GPIO_PIN_SET);
}

/**
* @brief  w5500 rst.    
*/
static void periph_w55xx_rst(void)
{

   HAL_GPIO_WritePin(W55xx_RST_PORT, W55xx_RST_PIN, GPIO_PIN_RESET);					
   HAL_Delay(50);
   HAL_GPIO_WritePin(W55xx_RST_PORT, W55xx_RST_PIN, GPIO_PIN_SET);	
   HAL_Delay(1500);
     
}

/**
* @brief  W5500  write register.  
* @param  reg       gobal register address
          value     addre of value 
          len       addre of len   len max 512
* @retval None    
*/
static void periph_w55xx_setreg(uint32_t reg, uint8_t *value, uint16_t len)
{ 
  uint16_t timeout = 0xfff;
  uint8_t *w55xx_tx_buf = (uint8_t *)malloc(sizeof(uint8_t) *len + 3);
  uint8_t *w55xx_rx_buf = (uint8_t *)malloc(sizeof(uint8_t) *len + 3);
  w55xx_tx_buf[0] = (reg & 0x00FF0000)>>16;
  w55xx_tx_buf[1] = (reg & 0x0000FF00)>>8;
  w55xx_tx_buf[2] = (reg & 0x000000F8)|0x04;
  memcpy(&w55xx_tx_buf[3], &value[0], len);
  Bsp_Spi_Transaction(w55xx_t.client, &w55xx_rx_buf[0], &w55xx_tx_buf[0], len+3);
  do{
    timeout--;
  }while((w55xx_t.spiFlag == 0) && (timeout != 0));
  free(w55xx_tx_buf);
  free(w55xx_rx_buf);
}

/**
* @brief  W5500  get register.  
* @param  reg       gobal register address
          value     addre of value 
          len       addre of len   len max 512 
* @retval None    
*/
static void periph_w55xx_getreg(uint32_t reg, uint8_t *value, uint16_t len)
{ 
  uint16_t timeout = 0xfff;
  uint8_t *w55xx_tx_buf = (uint8_t *)malloc(sizeof(uint8_t) *len + 3);
  uint8_t *w55xx_rx_buf = (uint8_t *)malloc(sizeof(uint8_t) *len + 3);
  w55xx_tx_buf[0] = (reg & 0x00FF0000)>>16;
  w55xx_tx_buf[1] = (reg & 0x0000FF00)>>8;
  w55xx_tx_buf[2] = (reg & 0x000000F8)|0x00;
  Bsp_Spi_Transaction(w55xx_t.client, &w55xx_rx_buf[0], &w55xx_tx_buf[0], len+3);
  do{
    timeout--;
  }while((w55xx_t.spiFlag == 0) && (timeout != 0));
  memcpy(&value[0],&w55xx_rx_buf[3],len);
  free(w55xx_tx_buf);
  free(w55xx_rx_buf);
}

/**
* @brief  W5500    verify write  register.  
* @param  reg       register address
          value     addre of value 
          len       addre of len   len max 512      
* @retval 0-successful  
          -1-failed   
*/
static int8_t periph_w55xx_verify_setreg(uint32_t reg, uint8_t *value, uint16_t len)
{  
  uint8_t dec = 3;
  int8_t ret = 0;
  uint8_t *rx_value = (uint8_t *)malloc(sizeof(uint8_t) * len + 1);
  uint8_t *tmp_value = (uint8_t *)malloc(sizeof(uint8_t) * len + 1);
  memcpy(tmp_value, value, len);
  tmp_value[len] = '\0';
  rx_value[len] = '\0';
  do {
    periph_w55xx_setreg(reg, value, len);
    periph_w55xx_getreg(reg, rx_value, len);
    dec--;
  } while (strcmp((char const *)tmp_value,(char const *)rx_value) != 0 && (dec != 0)); 
  if (dec == 0) {
    ret = -1;
  }
  free(tmp_value);
  free(rx_value);
  return ret;
}

/**
* @brief  W5500     write  mem.  
* @param  sockfd
          value     addre of value 
          len       addre of len   len max 512      
* @retval 0-successful  
          -1-failed   
*/
static int8_t periph_w55xx_write_mem(uint8_t sockfd, uint8_t *value, uint16_t len)
{
  uint16_t send_ptr = 0;
  uint32_t addr = 0;
  uint8_t  temp_buf[2] = {0};
  periph_w55xx_getreg(Sn_TX_WR0(sockfd), temp_buf, 2);
  send_ptr = (temp_buf[0] << 8) | temp_buf[1];
  addr = (uint32_t)(send_ptr<<8) + (sockfd<<5) + 0x10;
  periph_w55xx_setreg(addr, value, len);  
  send_ptr += len;
  temp_buf[0] = (uint8_t)((send_ptr & 0xff00) >> 8);
  temp_buf[1] = (uint8_t)(send_ptr & 0x00ff); 
  periph_w55xx_setreg(Sn_TX_WR0(sockfd), temp_buf, 2);
  return 0;
}

/**
* @brief  W5500    read  mem.  
* @param  sockfd
          value     addre of value 
          len       addre of len   len max 512      
* @retval 0-successful  
          -1-failed   
*/
static int8_t periph_w55xx_read_mem(uint8_t sockfd, uint8_t *value, uint16_t len)
{
  uint16_t read_ptr = 0;
  uint32_t addr = 0;
  uint8_t  temp_buf[2] = {0};
  uint8_t ir_value = 0xff;
  periph_w55xx_getreg(Sn_RX_RD0(sockfd), temp_buf, 2);
  read_ptr = (temp_buf[0] << 8) | temp_buf[1];
  addr = (uint32_t)(read_ptr<<8) + (sockfd<<5) + 0x18;
  periph_w55xx_getreg(addr, value, len);  
  periph_w55xx_setreg(Sn_IR(sockfd), &ir_value, 1);
  read_ptr += len;
  temp_buf[0] = (uint8_t)((read_ptr & 0xff00) >> 8);
  temp_buf[1] = (uint8_t)(read_ptr & 0x00ff); 
  periph_w55xx_setreg(Sn_RX_RD0(sockfd), temp_buf, 2);
  return 0;
}

/********************************END OF FILE***********************************/
