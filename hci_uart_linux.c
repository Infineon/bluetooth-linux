/*
* Copyright 2022, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*/

/*****************************************************************************
 **
 **  Name:          hci_uart_linux.c
 **
 **  Description: hanlde hci uart communication.
 **
 ******************************************************************************/

#include <time.h>
#include <sys/timeb.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "wiced_bt_types.h"
#include "userial.h"
#include <ctype.h>
#include <pthread.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <stdarg.h>

#include "data_types.h"
#include "platform_linux.h"

#define HCI_COMM_PORT_BUFSIZE		4096

#ifdef DEBUG_UART_RX
#define MAX_DEBUG_RX_BUFSIZE (1 << 10)
typedef struct{
    int fd;
    uint8_t buf[MAX_DEBUG_RX_BUFSIZE];
    uint32_t size_in_use;
}debug_rx_t;

debug_rx_t debug_rx;
#endif


// HCI Transport packet types. Each packet sent or
// received must be one of these four types, though if running
// on a host we do not send events, and if running on a controller
// we do not send commands.
#define HCIT_TYPE_COMMAND		1
#define HCIT_TYPE_ACL_DATA		2
#define HCIT_TYPE_SCO_DATA		3
#define HCIT_TYPE_EVENT			4
#define HCIT_TYPE_ISOC_DATA		5
#define HCIT_TYPE_DIAG			7

#define HCIT_TYPE_WICED_HCI     25
typedef enum
{
    WAIT_TYPE, WAIT_OPCODE_1, WAIT_HANDLE_1, WAIT_HANDLE_2, WAIT_LEN_1, WAIT_LEN_2, WAIT_DATA
} eRX_STATE;

typedef struct
{
    uint8_t     data[2000];
    eRX_STATE   state;       // State of the receiver
    uint8_t     type;        // Type of the packet being received
    uint16_t    len;
    uint16_t    hci_len;
}tUART_RX;

static tUART_RX rx_pkt;
static void*  uartRecv (void *p);

static wiced_bool_t ProcessChar (uint8_t in_byte);
static void pktRcvd (void);
static void ClosePort(void);

extern void debug_Printf(const char* format, ... );
extern void debug_PrintError(const char* format, ... );
extern void ProcessEventFromHCI (uint8_t *pData, uint32_t length);
extern void ProcessAclFromHCI (uint8_t *pData, uint32_t length);
extern void ProcessScoFromHCI(uint8_t *pData, uint32_t length);
extern void ProcessDiagFromHCI(uint8_t* pData, uint32_t length);
extern void ProcessIsocFromHCI (uint8_t *pData, uint32_t length);

BOOL32 uart_reconfigure(uint32_t uBaudRate);

typedef struct
{
    /* Name of the opened device (for later usage) */
    char devname[20];
    /* File descriptor of the serial device */
    int fd;
    /* Indicate what kind of device is opened */
    uint8_t devicetype;
    /* Thread to use for serial read wait */
    pthread_t read_thread;
    /* Open configuration */
    tUSERIAL_OPEN_CFG cfg;
    /* Serial device callback when data is received */

} tUSERIAL_CB;

static tUSERIAL_CB userial_cb;

int devid = -1;

void wait_for_uart_recv_thread()
{
    pthread_join(userial_cb.read_thread, NULL);
}
BOOL32 uartInit (char *pPortName, uint32_t uBaudRate)
{
    struct sched_param param;
    int policy;
    int flags;

    memset(&userial_cb, 0, sizeof(userial_cb));
    userial_cb.fd = -1;
    userial_cb.cfg.fmt = USERIAL_DATABITS_8 | USERIAL_PARITY_NONE | USERIAL_STOPBITS_1;
    userial_cb.cfg.fc = 0;
    userial_cb.cfg.buf = USERIAL_BUF_BYTE;


    /* Save the open parameters */
    strncpy(userial_cb.devname, pPortName, sizeof(userial_cb.devname)-1);

    if (strlen(userial_cb.devname) == 0)
    {
        strncpy(userial_cb.devname, "/dev/ttyUSB0", sizeof(userial_cb.devname)-1);
    }

    flags = O_RDWR;

    /* Try to open the device depending on the device type */
    userial_cb.fd = open(userial_cb.devname, flags);
    if (userial_cb.fd < 0)
    {
        printf("ERROR: open(%s) failed(%d)", userial_cb.devname, userial_cb.fd);
        return FALSE;
    }
    devid = userial_cb.fd;

    if(!uart_reconfigure(uBaudRate))
    {
        printf("uart_reconfigure failed");
        return FALSE;
    }

    pthread_attr_t thread_attr;
    pthread_attr_init(&thread_attr);
    pthread_attr_setdetachstate(&thread_attr, PTHREAD_CREATE_DETACHED);

    if (pthread_create(&(userial_cb.read_thread), &thread_attr, uartRecv, NULL) < 0)
    {
        printf("pthread_create failed");
        return FALSE;
    }

    if (pthread_getschedparam(userial_cb.read_thread, &policy, &param) == 0)
    {
        policy = SCHED_FIFO;
        param.sched_priority = sched_get_priority_max(SCHED_FIFO);
        debug_Printf("sched priority %d\n", param.sched_priority);
        pthread_setschedparam(userial_cb.read_thread, policy, &param);
    }
    sleep(1);

#ifdef DEBUG_UART_RX
    {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        char filename[100];

        printf("Seconds since Jan. 1, 1970: %ld\n", tv.tv_sec);

        snprintf(filename, sizeof(filename),"my_debug_rx_%u",tv.tv_sec);

        debug_rx.fd = open(filename, O_WRONLY|O_CREAT);
        printf("debug uart rx %s fd %u \n", filename, debug_rx.fd);
    }
#endif

    return (TRUE);
}


#ifdef DEBUG_UART_RX
void write_to_debug_rx(uint8_t *p_data, int len)
{
    if(debug_rx.fd == -1)
    {
        return;
    }

    while(len)
    {
        int remaining = MAX_DEBUG_RX_BUFSIZE - debug_rx.size_in_use;
        int to_copy = MIN(len, remaining);

        if(to_copy){
            memcpy(debug_rx.buf + debug_rx.size_in_use, p_data, to_copy);
            debug_rx.size_in_use += to_copy;
            len -= to_copy;
        }

        if(debug_rx.size_in_use == MAX_DEBUG_RX_BUFSIZE){
            write(debug_rx.fd, debug_rx.buf, debug_rx.size_in_use);
            debug_rx.size_in_use = 0;
        }
    }
}

void flush_debug_rx(void)
{
    printf("[%s] closing %u",__FUNCTION__, debug_rx.fd);
    if(debug_rx.fd != -1)
    {
        write(debug_rx.fd, debug_rx.buf, debug_rx.size_in_use);
        fclose(debug_rx.fd);
    }
    exit(-1);
}

#endif
void*  uartRecv (void *ppp)
{
    int		dwBytesRead = 0;
    uint16_t		i;
    uint8_t     scratchPad[2000];

    /* loop forever, polling for serial data */
    while (userial_cb.fd != -1)
    {
        // read the data (read up to a quarter of a buffer each time)
        dwBytesRead = read(userial_cb.fd, scratchPad, HCI_COMM_PORT_BUFSIZE / 4);
        if (dwBytesRead < 0)
        {
            debug_Printf("uartRecv: Readfile failed 0x%x\n", errno);
            break;
        }

        if (userial_cb.fd == -1)
            break;

        // send it on up the stack
        if (dwBytesRead)
        {
#ifdef DEBUG_UART_RX
            write_to_debug_rx(scratchPad, dwBytesRead);
#endif
            for (i = 0; i < dwBytesRead; i++)
            {
                if (ProcessChar(scratchPad[i]) == FALSE){
                    break;
                }
            }
        }
    }

    ClosePort();
    return NULL;

}

/*******************************************************************************
 **
 ** Function            ProcessChar
 **
 ** Description         Parse HCI packets from Controller
 **
 ** Returns             TRUE:   Get HCI packets successful 
 **                     FALSE:  Get HCI packets failed, error in HCI packets 
 **
 *******************************************************************************/
static wiced_bool_t ProcessChar (uint8_t in_byte)
{
    switch (rx_pkt.state)
    {
      case WAIT_TYPE:
        rx_pkt.len = rx_pkt.hci_len = 0;
        switch (in_byte)
        {
          case HCIT_TYPE_ACL_DATA:
            rx_pkt.state = WAIT_HANDLE_1;
            rx_pkt.type  = HCIT_TYPE_ACL_DATA;
            break;

          case HCIT_TYPE_SCO_DATA:
            rx_pkt.state = WAIT_HANDLE_1;
            rx_pkt.type  = HCIT_TYPE_SCO_DATA;
            break;

          case HCIT_TYPE_EVENT:
            rx_pkt.state = WAIT_OPCODE_1;
            rx_pkt.type  = HCIT_TYPE_EVENT;
            break;

          case HCIT_TYPE_WICED_HCI:
              rx_pkt.state = WAIT_HANDLE_1;
              rx_pkt.type  = HCIT_TYPE_WICED_HCI;
              break;

          case HCIT_TYPE_DIAG:
              rx_pkt.state = WAIT_DATA;
              rx_pkt.hci_len = 63;
              rx_pkt.type  = HCIT_TYPE_DIAG;
              break;

          case HCIT_TYPE_ISOC_DATA:
              rx_pkt.state = WAIT_HANDLE_1;
              rx_pkt.type  = HCIT_TYPE_ISOC_DATA;
              break;

          default:
#ifdef DEBUG_UART_RX
            flush_debug_rx();
#endif
            debug_PrintError("[linux] ProcessChar: bad HCI H4 type %d\n", in_byte);
            tcflush(userial_cb.fd, TCIFLUSH);
            return FALSE;
        }
        return TRUE;

      case WAIT_OPCODE_1:
        rx_pkt.data[rx_pkt.len++] = in_byte;
        rx_pkt.state = WAIT_LEN_1;
        return TRUE;

      case WAIT_HANDLE_1:
        rx_pkt.data[rx_pkt.len++] = in_byte;
        rx_pkt.state = WAIT_HANDLE_2;
        return TRUE;

      case WAIT_HANDLE_2:
        rx_pkt.data[rx_pkt.len++] = in_byte;
        rx_pkt.state = WAIT_LEN_1;
        return TRUE;

      case WAIT_LEN_1:
        rx_pkt.data[rx_pkt.len++] = in_byte;
        rx_pkt.hci_len = in_byte;

        if ( (rx_pkt.type != HCIT_TYPE_ACL_DATA) && (rx_pkt.type != HCIT_TYPE_WICED_HCI) && (rx_pkt.type != HCIT_TYPE_ISOC_DATA) )
        {
            if (rx_pkt.hci_len == 0)
                pktRcvd();
            else
                rx_pkt.state = WAIT_DATA;
        }
        else
            rx_pkt.state = WAIT_LEN_2;
        break;

      case WAIT_LEN_2:
        rx_pkt.data[rx_pkt.len++] = in_byte;
        rx_pkt.hci_len += (in_byte << 8);

        if (rx_pkt.type == HCIT_TYPE_ISOC_DATA)
        {
            rx_pkt.hci_len &= 0x3FFF; // 14 bit for data length, 2 bit RFU
        }

        if (rx_pkt.hci_len == 0)
            pktRcvd();
        else
            rx_pkt.state = WAIT_DATA;
        break;

      case WAIT_DATA:
        rx_pkt.data[rx_pkt.len++] = in_byte;

        if (--rx_pkt.hci_len == 0)
            pktRcvd();
        break;
    }
    return TRUE;
}

/****************************************************************************
**
**  Function Name:  pktRcvd
*/
static void pktRcvd (void)
{
    // Reset state
    rx_pkt.state = WAIT_TYPE;
    // We only care about event packets
    if (rx_pkt.type == HCIT_TYPE_EVENT)
    {
        ProcessEventFromHCI (rx_pkt.data, rx_pkt.len);
    }
    else if (rx_pkt.type == HCIT_TYPE_ACL_DATA)
    {
        ProcessAclFromHCI (rx_pkt.data, rx_pkt.len);
    }
    else if (rx_pkt.type == HCIT_TYPE_DIAG)
    {
        ProcessDiagFromHCI(rx_pkt.data, rx_pkt.len);
    }
    else if (rx_pkt.type == HCIT_TYPE_SCO_DATA)
    {
        ProcessScoFromHCI(rx_pkt.data, rx_pkt.len);
    }
    else if (rx_pkt.type == HCIT_TYPE_ISOC_DATA)
    {
        ProcessIsocFromHCI(rx_pkt.data, rx_pkt.len);
    }
    else if (rx_pkt.type == HCIT_TYPE_WICED_HCI)
    {
        debug_Printf("!!!!Dropping WICED HCI Packet Length: %u\n", rx_pkt.len);
    }
    else
    {
        debug_Printf("!!!!Unexpected HCI H4 Packet Type: %u  Length: %u\n", rx_pkt.type, rx_pkt.len);
    }
    memset (rx_pkt.data, 0, sizeof (rx_pkt.data));
}

static void ClosePort(void)
{
    int fd = userial_cb.fd;

    printf("ClosePort fd %d", fd);

    /* if not already closed */
    if (fd != -1)
    {
        userial_cb.fd = -1;
        close(fd);
    }

}


void uartSend (uint32_t bIsCommand, uint8_t *pData, uint32_t len)
{
    uint16_t		dwBytesWritten = 0;
    uint16_t		dwBytesToWrite = len + 1;
    ssize_t      result;
    uint8_t     buff[1200];
    uint8_t     *p_buff = buff;

    // Add in H4 packet type
    if (bIsCommand == 0)
        buff[0] = HCIT_TYPE_ACL_DATA;
    else if (bIsCommand == 0xDD)
        buff[0] = HCIT_TYPE_SCO_DATA;
    else if (bIsCommand == 0x7)
        buff[0] = HCIT_TYPE_DIAG;
    else if (bIsCommand == 0x5)
        buff[0] = HCIT_TYPE_ISOC_DATA;
    else
        buff[0] = HCIT_TYPE_COMMAND;

    memcpy (&buff[1], pData, len);

    // Verify there is a valid port to send out on
    if (userial_cb.fd == -1)
    {
        debug_Printf("fd is -1\n");
        return;
    }

    // Send the data out the serial port, and verify it all gets sent
    do
    {
        result = write(userial_cb.fd, p_buff, dwBytesToWrite);

        if (result < 0)
        {
            debug_Printf("uartSend: Writefile failed: 0x%x\n", errno);

            ClosePort();
            return;
        }
        dwBytesWritten = result;

        p_buff         += dwBytesWritten;
        dwBytesToWrite -= dwBytesWritten;

    } while (dwBytesToWrite != 0);

}

#ifdef PRINT_PACKET
void dumpHex (char *p_title, uint8_t *p, uint32_t len)
{
    uint32_t  xx, yy;
    char    buff1[100], buff2[20];

    if (p_title)
    {
        debug_Printf("%s   Len: %u\n", p_title, len);
    }

    memset (buff2, ' ', 16);
    buff2[16] = 0;

    yy = sprintf (buff1, "%04x: ", 0);
    for (xx = 0; xx < len; xx++)
    {
        if ( (xx) && ((xx & 15) == 0) )
        {
            debug_Printf ("    %s  %s\n", buff1, buff2);
            yy = sprintf(buff1, "%04x: ", xx);
            memset (buff2, ' ', 16);
        }
        yy += sprintf (&buff1[yy], "%02x ", *p);

        if ((*p >= ' ') && (*p <= 'z'))
            buff2[xx & 15] = *p;
        else
            buff2[xx & 15] = '.';

        p++;
    }

    /* Pad out the remainder */
    for ( ; ; xx++)
    {
        if ((xx & 15) == 0)
        {
            debug_Printf ("    %s  %s\n", buff1, buff2);
            break;
        }
        yy += sprintf (&buff1[yy], "   ");
    }
}
#endif

/******************************************************************************
 * Function Name: uart_reconfigure(uint32_t)
 *******************************************************************************
 * Summary:
 *   check input baud rate and set to available baud rate first then  
 *   reconfigure uart baud rate
 *
 * Parameters:
 *   uint32_t uBaudRate: BaudRate for reconfigure UART 
 *
 * Return BOOL32:
 *   TRUE: Success 
     FALSE: Error 
 *
 ******************************************************************************/
BOOL32 uart_reconfigure(uint32_t uBaudRate)
{
    uint32_t baud;
    uint8_t data_bits;
    uint16_t parity;
    uint8_t stop_bits;
    struct termios termios;

    if(uBaudRate < 921600)
    {
        userial_cb.cfg.baud = USERIAL_BAUD_115200;
        baud = B115200;
        uBaudRate = 115200;
    }
    else if (uBaudRate < 2000000)
    {
        userial_cb.cfg.baud = USERIAL_BAUD_921600;
        baud = B921600;
        uBaudRate = 921600;
    }
    else if (uBaudRate < 3000000)
    {
        userial_cb.cfg.baud = USERIAL_BAUD_2M;
        baud = B2000000;
        uBaudRate = 2000000;
    }
    else if (uBaudRate < 4000000)
    {
        userial_cb.cfg.baud = USERIAL_BAUD_3M;
        baud = B3000000;
        uBaudRate = 3000000;
    }
    else
    {
        userial_cb.cfg.baud = USERIAL_BAUD_4M;
        baud = B4000000;
        uBaudRate = 4000000;
    }

    if(userial_cb.cfg.fmt & USERIAL_DATABITS_8)
        data_bits = CS8;
    else if(userial_cb.cfg.fmt & USERIAL_DATABITS_7)
        data_bits = CS7;
    else if(userial_cb.cfg.fmt & USERIAL_DATABITS_6)
        data_bits = CS6;
    else if(userial_cb.cfg.fmt & USERIAL_DATABITS_5)
        data_bits = CS5;
    else
    {
        printf("ERROR: serial_configure bad size format:0x%x", userial_cb.cfg.fmt);
        return FALSE;
    }

    if(userial_cb.cfg.fmt & USERIAL_PARITY_NONE)
        parity = 0;
    else if(userial_cb.cfg.fmt & USERIAL_PARITY_EVEN)
        parity = PARENB;
    else if(userial_cb.cfg.fmt & USERIAL_PARITY_ODD)
        parity = (PARENB | PARODD);
    else
    {
        printf("ERROR: serial_configure bad parity format:0x%x", userial_cb.cfg.fmt);
        return FALSE;
    }

    if(userial_cb.cfg.fmt & USERIAL_STOPBITS_1)
        stop_bits = 0;
    else if (userial_cb.cfg.fmt & USERIAL_STOPBITS_2)
    {
        stop_bits = CSTOPB;
    }
    else
    {
        printf("ERROR: serial_configure bad stop format:0x%x", userial_cb.cfg.fmt);
        return FALSE;
    }

    tcflush(userial_cb.fd, TCIOFLUSH);
    tcgetattr(userial_cb.fd, &termios);

    /* Configure in default raw mode */
    cfmakeraw(&termios);

    /* Clear out what can be overriden */
    termios.c_cflag &= ~(CSIZE | PARENB | PARODD | CSTOPB | CRTSCTS);
    termios.c_cflag |= parity | data_bits | stop_bits;

    /* use hw  flow control */
    userial_cb.cfg.fc = USERIAL_FC_HW;
    if (userial_cb.cfg.fc == USERIAL_FC_HW)
    {
        termios.c_cflag |= CRTSCTS;
    }


    termios.c_cflag &= ~CSTOPB; // 1 stop bit
    tcsetattr(userial_cb.fd, TCSANOW, &termios);

    tcflush(userial_cb.fd, TCIOFLUSH);

    tcsetattr(userial_cb.fd, TCSANOW, &termios);

    tcflush(userial_cb.fd, TCIOFLUSH);
    tcflush(userial_cb.fd, TCIOFLUSH);

    cfsetospeed(&termios, baud);
    cfsetispeed(&termios, baud);
    tcsetattr(userial_cb.fd, TCSANOW, &termios);

    debug_Printf("uart_reconfigure: set baudrate:%d\n", uBaudRate);
    return TRUE;
}

/******************************************************************************
 * Function Name: checkAppBaudRate(uint32_t uBaudRate)
 *******************************************************************************
 * Summary:
 *   After firmware download complete, application will reset the uart baud rate if need.
 *   Check user input Application running baud rate, and return the avaliable baud rate.
 *   The App baud rate speed is limited by BT chip.
 *   supported baud rate:
 *      115200, 921600, 2000000, 3000000, 4000000 
 *
 * Parameters:
 *      uint32_t uBaudRate: user input CE running baud rate 
 *
 * Return:
 *      uint32_t: avaliable baud rate
 *
 ******************************************************************************/
uint32_t checkAppBaudRate(uint32_t uBaudRate) 
{
    uint32_t appBaudRate = DEFAULT_APP_BAUD_RATE;    
    debug_Printf("input app Baudrate:%d\n", uBaudRate);
    if(uBaudRate < 921600)
    {
        appBaudRate = 115200;
    }
    else if (uBaudRate < 2000000)
    {
        appBaudRate = 921600;
    }
    else if (uBaudRate < 3000000)
    {
        appBaudRate = 2000000;
    }
    else if (uBaudRate < 4000000)
    {
        appBaudRate = 3000000;
    }
    else
    {
        appBaudRate = 4000000;
    }
    debug_Printf("appBaudrate:%d\n", appBaudRate);
    return appBaudRate;
}

/******************************************************************************
 * Function Name: checkPatchBaudRate(uint32_t uBaudRate)
 *******************************************************************************
 * Summary:
 *   Check user input patch Firmware baud rate, and return avaliable baud rate.  
 *   Patch baud rate for app download firmware only, when app running download  
 *   firmware to BT chip, use patch firmware baud rate to setup uart speed.
 *   The patch baud rate speed is limited by BT chip. 
 *   This function check the input patch baud rate and adjust to the avaliable
 *   speed.
 *   supported baud rate:
 *       115200, 921600, 3000000  
 *
 * Parameters:
 *   uint32_t uBaudRate: user input patch firmware baud rate 
 *
 * Return:
 *      uint32_t: avaliable baud rate 
 *
 ******************************************************************************/
uint32_t checkPatchBaudRate(uint32_t uBaudRate) 
{
    debug_Printf("input patchBaudRate:%d\n", uBaudRate);
    uint32_t patchBaudRate = DEFAULT_PATCH_BAUD_RATE;    
    if(uBaudRate < 921600)
    {
        patchBaudRate = 115200;
    }
    else if(uBaudRate < 3000000)
    {
        patchBaudRate = 921600;
    }
    else if(uBaudRate == 3000000)
    {
        patchBaudRate = 3000000;
    }
    else
    {
        patchBaudRate = DEFAULT_PATCH_BAUD_RATE;
    }

    debug_Printf("patchBaudRate:%d\n", patchBaudRate);
    return patchBaudRate;
}
