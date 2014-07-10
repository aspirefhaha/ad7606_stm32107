#include <stdio.h>
#include <rtthread.h>
#include <lwip/sys.h>
#include <lwip/api.h>
#include <lwip/sockets.h> 
#include <lwip/netif.h>
#include <netif/ethernetif.h>
#include "netcnf.h"
#include "uploadapp.h"
#include <dfs_posix.h>

#define TCPCOMPORT 8888
static struct rt_mailbox mbnet;
#define MBNETBUFLEN	32
static unsigned char s_mbnetbuffer[MBNETBUFLEN]={0};
void net_send_pkg(com_pkg_t pPkg)
{
 	com_pkg_t pSendPkg = rt_malloc(sizeof(com_pkg));
	RT_ASSERT(pSendPkg);
	rt_memcpy(pSendPkg,pPkg,sizeof(com_pkg));
	rt_mb_send(&mbnet,(rt_uint32_t)pSendPkg);
	return;	
}
extern int lwip_get_error(int s);
void tcp_netcom_entry(void * parameter)
{
	int sockd,peersock;
	int ret = 0;
	struct sockaddr_in taraddr,selfaddr;
	struct netif * netif = (struct netif*)parameter;
	rt_kprintf("ip address: %s\n", inet_ntoa(*((struct in_addr*)&(netif->ip_addr))));
	rt_mb_init(&mbnet,"mbnet",s_mbnetbuffer,MBNETBUFLEN / 4,RT_IPC_FLAG_FIFO);
	sockd = socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
	rt_kprintf("create TCP socket %d\n",sockd);
	selfaddr.sin_family = AF_INET;
	selfaddr.sin_addr.s_addr = htons(INADDR_ANY);
	selfaddr.sin_port = htons(TCPCOMPORT);

	ret = bind(sockd,(struct sockaddr*)&selfaddr,sizeof(selfaddr));
	if(ret != 0){
	 	rt_kprintf("TCP Bind port %d failed!",TCPCOMPORT);
		closesocket(sockd);
		return;
	}
	ret = listen(sockd,2);
	if(ret != 0){
	 	rt_kprintf("TCP listen port %d failed!",TCPCOMPORT);
		closesocket(sockd);
		return;
	}
	while(1){
		int getsize = sizeof(taraddr);
		peersock = lwip_accept(sockd,(struct sockaddr*)&taraddr,(socklen_t *)&getsize);
		if(peersock < 0){
		 	rt_kprintf("ComServer accept failed!\n");
			closesocket(sockd);
			return;
		}
		else{
			unsigned short buf_offset = 0;
			com_pkg_t pNetPkg;
			#define RECVBUFSIZE 512
			char recvbuf[RECVBUFSIZE]={0};	 
			int len=0;	
			rt_kprintf("get one client %s connect\n",inet_ntoa(taraddr.sin_addr));
			while(1){
				len = recv(peersock, recvbuf+buf_offset,RECVBUFSIZE-buf_offset,MSG_DONTWAIT);
				if(len <= 0){
					
					if(lwip_get_error(peersock) == EWOULDBLOCK) {
						if(rt_mb_recv(&mbnet,(rt_uint32_t*)&pNetPkg,RT_TICK_PER_SECOND/10)==RT_EOK){
							//int i = 0 ;
							//char * pSendData = RT_NULL;
						 	send(peersock,(void *)pNetPkg,sizeof(com_pkg),MSG_WAITALL);
							//pSendData = (char*)pNetPkg;
							//rt_kprintf("want to send data:");
							//for(i=0;i<sizeof(com_pkg);i++){
							//	rt_kprintf("%02x ",*(pSendData+i));
							//}
							//rt_kprintf("\n");
							rt_free(pNetPkg);
						}
						continue;
					}
					else
						break;
				}
				buf_offset += len;
				if(buf_offset >= sizeof(com_pkg)){
					int pret = 0;
					//send(peersock,"nihaoma1nihaomanihaoma",20,0);
					pret = process_pkg((com_pkg_t)recvbuf);
					if(pret){
					 	rt_kprintf("process net pkg failed %d\n",pret);
					}
					buf_offset-= sizeof(com_pkg);
					if(buf_offset!=0){
						rt_memcpy(recvbuf,recvbuf+sizeof(com_pkg),buf_offset); 	
					} 	
				}				
			}
			closesocket(peersock);
		}
	}
	return;	
}

int process_pkg(com_pkg_t pPkg)
{
	if(pPkg->precode != PRECODE) {
		rt_kprintf("get precode %x need %x\n",pPkg->precode,PRECODE);
		return -1;
	}
	if(pPkg->tailcode != TAILCODE){
		rt_kprintf("get tailcode %x need %x\n",pPkg->tailcode,TAILCODE);
		return -1;
	}
	
	
	switch(pPkg->op_type ){
		case 01:{	//设置档位操作
			 set_gear(pPkg->op_value);
			 break;
		}
		case 02:{	//获取操作
			 com_pkg sendPkg;
			 sendPkg.precode = PRECODE;
			 sendPkg.tailcode = TAILCODE;
			 sendPkg.op_type = 3;
			 sendPkg.op_value = curgear;
			 net_send_pkg(&sendPkg);
			 break;
		}
	}
	return 0;
	
}

