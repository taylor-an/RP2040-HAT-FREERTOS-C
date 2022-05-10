#include <stdio.h>
#include "tcp.h"
#include "socket.h"
#include "wizchip_conf.h"

static uint8_t conenction_client = 0;

int32_t tcps(uint8_t sn, uint16_t port)
{
    int32_t ret = 0;
    uint16_t size = 0, sentsize=0;

#if 1
//#ifdef _TCP_DEBUG_
    uint8_t destip[4];
    uint16_t destport;
#endif

    switch(getSn_SR(sn))
    {
        case SOCK_ESTABLISHED :
            
            if(getSn_IR(sn) & Sn_IR_CON)
            {
#if 1
//#ifdef _TCP_DEBUG_
                getSn_DIPR(sn, destip);
                destport = getSn_DPORT(sn);

                printf("%d:Connected - %d.%d.%d.%d : %d\r\n",sn, destip[0], destip[1], destip[2], destip[3], destport);
#endif
                setSn_IR(sn,Sn_IR_CON);
                conenction_client = 1;
            }

            if(conenction_client == 1)
            {
                ret = 1;
            }
            
        break;
        
        case SOCK_CLOSE_WAIT :
            
#ifdef _TCP_DEBUG_
            //printf("%d:CloseWait\r\n",sn);
#endif
            if((ret = disconnect(sn)) != SOCK_OK)
            {
                return -1;
            }
            ret = 0;
#ifdef _TCP_DEBUG_
            printf("%d:Socket Closed\r\n", sn);
#endif
            conenction_client = 0;
            
        break;
        
        case SOCK_INIT :
            
#ifdef _TCP_DEBUG_
            printf("%d:Listen, TCP server, port [%d]\r\n", sn, port);
#endif
            if( (ret = listen(sn)) != SOCK_OK)
            {
                return -1;
            }
            ret = 2;
            
        break;
        
        case SOCK_CLOSED:
#ifdef _TCP_DEBUG_
            //printf("%d:TCP server loopback start\r\n",sn);
#endif
            if((ret = socket(sn, Sn_MR_TCP, port, 0x00)) != sn)
            {
                return -1;
            }
            ret = 0;
#ifdef _TCP_DEBUG_
        //printf("%d:Socket opened\r\n",sn);
#endif
        break;
        
        case SOCK_LISTEN :
#ifdef _TCP_DEBUG_
            printf("SOCK_LISTEN\n");
#endif
            ret = 2;
        break;
        
        default:
            ret = 0;
        break;
    }
    
    return ret;
}

int32_t tcps_send(uint8_t sn, uint8_t* buf, int32_t len)
{
    int32_t ret = 0;
    uint16_t size = 0, sentsize=0;

#ifdef _TCP_DEBUG_
    uint8_t destip[4];
    uint16_t destport;
#endif

    switch(getSn_SR(sn))
    {
        case SOCK_ESTABLISHED :
        
        if(len > 0 && conenction_client == 1)
        {
            while(len != sentsize)
            {
                ret = send(sn, buf+sentsize, len-sentsize);
                if(ret < 0)
                {
                    close(sn);
                    return ret;
                }
                sentsize += ret;
            }
            ret = sentsize;
        }
        else
        {
            if((len = getSn_RX_RSR(sn)) > 0)
            {
                while(len != 0)
                {
                    if(len >= DATA_BUF_SIZE)
                    {
                        ret = recv(sn, buf, DATA_BUF_SIZE);
                        len -= ret;
                    }
                    else
                    {
                        ret = recv(sn, buf, len);
                        len -= ret;
                    }
                }
                ret = 0;
            }
        }
        
        break;
        
        case SOCK_CLOSE_WAIT :
            
#ifdef _TCP_DEBUG_
            //printf("%d:CloseWait\r\n",sn);
#endif
            if((ret = disconnect(sn)) != SOCK_OK)
            {
                return -1;
            }
            ret = -1;
#ifdef _TCP_DEBUG_
            printf("%d:Socket Closed\r\n", sn);
#endif
            conenction_client = 0;
            
        break;
        
        default:
        break;
    }
    
    return ret;
}

