#include <stdio.h>
#include <rtthread.h>
#include <lwip/sys.h>
#include <lwip/api.h>
#include <lwip/sockets.h> 
#include <lwip/netif.h>
#include <netif/ethernetif.h>
#include "uploadapp.h"
#include <stm32_eth.h>

//static char emptymbpool[4*POOLSIZE];
static char fullmbpool[4*POOLSIZE];

//struct rt_mailbox emptymb;
struct rt_mailbox fullmb;

void udp_send_entry(void * parameter)
{
	int sockd;
	int i = 0;
	struct sockaddr_in taraddr;
	struct netif * netif = (struct netif*)parameter;
	rt_kprintf("ip address: %s\n", inet_ntoa(*((struct in_addr*)&(netif->ip_addr))));
	sockd = socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
	rt_kprintf("create socket %d\n",sockd);
	taraddr.sin_family = AF_INET;
	taraddr.sin_addr.s_addr = netif->ip_addr.addr | 0xff000000;
	taraddr.sin_port = htons(9001);
	
	//rt_mb_init(&emptymb,"nepy",emptymbpool,sizeof(emptymbpool)/4,RT_IPC_FLAG_FIFO);
	rt_mb_init(&fullmb,"nful",fullmbpool,sizeof(fullmbpool)/4,RT_IPC_FLAG_FIFO);
	
	for(i=1;i<POOLSIZE;i++){	  //��1��ʼ����Ϊ�տ�ʼ 0����Ҫ��ȡ���϶�����
		//rt_mb_send(&emptymb,(rt_uint32_t)i);
	}
	ad_pwm();
	do{
		rt_uint32_t fullindex = 0;
		if(rt_mb_recv(&fullmb,&fullindex,RT_WAITING_FOREVER) == RT_EOK){
			i = sendto(sockd,(const void *)ad_dma_buf[fullindex],AD_TIMES * AD_CHS * 2,0,(struct sockaddr *)&taraddr,sizeof(taraddr));
			//i = sendto(sockd,"baga",4,0,(struct sockaddr *)&taraddr,sizeof(taraddr));
			//rt_mb_send(&emptymb,fullindex);	
			//rt_thread_delay(RT_TICK_PER_SECOND);
		}
		else
			i = -1;
	}while(i >= 1);
	
}

#ifdef RT_LWIP_DHCP
static void netifstatus_callback_fn(struct netif *netif)
{
	rt_thread_t shnet_thread = rt_thread_create("upload",udp_send_entry,(void *)netif,2048,11,20);
	if(shnet_thread!=RT_NULL){
		if(RT_EOK!=rt_thread_startup(shnet_thread)){
		 	rt_kprintf("start smart home net thread failed!!!!\n");
		}
	}
	else{
	 	rt_kprintf("create shnet thread failed!!!!\n");
	}	
}

static void netiflink_callback_fn(struct netif *netif)
{
	rt_kprintf("netif link callback\n");
}

#else
static void netiflink_callback_fn(struct netif *netif)
{
	static int hasstarted = 0;
	register rt_uint16_t phy_bsr = ETH_ReadPHYRegister(0X00, PHY_BSR);	
	rt_kprintf("netif link callback\n");	
	if((phy_bsr & PHY_Linked_Status) != 0 && hasstarted != 1) {
		hasstarted = 1;
		{
			rt_thread_t upload_thread = rt_thread_create("upload",udp_send_entry,(void *)netif,2048,11,20);
			if(upload_thread!=RT_NULL){
				if(RT_EOK!=rt_thread_startup(upload_thread)){
				 	rt_kprintf("start upload net thread failed!!!!\n");
				}
			}
			else{
			 	rt_kprintf("create upload thread failed!!!!\n");
			}
		}
		
	}
}
static void netifstatus_callback_fn(struct netif *netif)
{
	rt_kprintf("netifstatus_callback_fn");
}

#endif

void netcheck(void * parameter)
{
	register rt_uint16_t phy_bsr;
	//static rt_uint32_t tcp_tx_timeout = 0;
	rt_uint8_t oldstate = 2;
	struct netif * eth0  = (struct netif * ) parameter;
	struct eth_device *ethif = RT_NULL;
	if(eth0)
		ethif = (struct eth_device*)eth0->state;
	do{
		phy_bsr = ETH_ReadPHYRegister(0X00, PHY_BSR);
		
		if((phy_bsr & PHY_Linked_Status) == 0)                                                                /*        DM9161A����״̬�Ĵ���        */
		{
		   if(oldstate == 0){
		   
		   }
		   else{ 
		   	if(oldstate == 1){
			 	reset();
			}
		   	rt_kprintf("The net link break! Please check network cable!\r\n");
			if(ethif)
		   		eth_device_linkchange(ethif,RT_FALSE);
			oldstate =0;
		   }
		             
		}
		if(((phy_bsr & PHY_Linked_Status) != 0))                                                          /*                ��������������                */
		{
		     if(oldstate == 1){

			 }
			 else{
		   		rt_kprintf("The network cable has been inserted!\r\n");  
				if(ethif)
		   			eth_device_linkchange(ethif,RT_TRUE);      
				oldstate = 1;
			}
		     
		}
		rt_thread_delay(RT_TICK_PER_SECOND);
	}while(1);	
	
}
void netapp_start(void)
{
	struct netif * eth0 = netif_find("e0");
	rt_thread_t netcheckthread = rt_thread_create("netck",netcheck,eth0,1024,20,20);
	if(netcheckthread){
	 	rt_thread_startup(netcheckthread);
	}
#ifdef RT_LWIP_DHCP
 	 if(eth0){
  		netif_set_status_callback(eth0, netifstatus_callback_fn);
		netif_set_link_callback(eth0,netiflink_callback_fn);
	 }
	else{
		rt_kprintf("find netif e0 failed!!!!!\n");
	}
#else
	if(eth0){
		netif_set_link_callback(eth0,netiflink_callback_fn);
		netif_set_status_callback(eth0, netifstatus_callback_fn);
		
	}
	else{
		rt_kprintf("find netif e0 failed!!!!!\n");
	}

#endif

}