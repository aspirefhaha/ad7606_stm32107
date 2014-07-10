/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006 - 2013, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 */

/**
 * @addtogroup STM32
 */
/*@{*/

#include <board.h>
#include <rtthread.h>

#ifdef RT_USING_DFS
#include <dfs_fs.h>
#endif

#ifdef RT_USING_COMPONENTS_INIT
#include <components.h>
#endif /* RT_USING_COMPONENTS_INIT */

#ifdef RT_USING_LWIP
#include <lwip/sys.h>
#include <lwip/api.h>
#include <netif/ethernetif.h>
#include "stm32_eth.h"
#include "uploadapp.h"
#endif
#include "finsh.h"

#ifdef RT_USING_M3AD
#include "../drivers/m3ad.h"	  
#endif

void rt_init_thread_entry(void* parameter)
{
    {
        extern void rt_platform_init(void);
        rt_platform_init();
    }

#ifdef RT_USING_COMPONENTS_INIT
    /* initialization RT-Thread Components */
    rt_components_init();
#endif

    /* Filesystem Initialization */
#if defined(RT_USING_DFS) && defined(RT_USING_DFS_ELMFAT)
    {
        /* mount sd card fat partition 1 as root directory */
        if (dfs_mount("sd0", "/", "elm", 0, 0) == 0)
        {
            rt_kprintf("File System initialized!\n");
        }
        else
        {
            rt_kprintf("File System initialzation failed!\n");
        }
    }
#endif /* RT_USING_DFS && RT_USING_DFS_ELMFAT */
#ifdef RT_USING_LWIP	
	netapp_start();
#endif
#ifdef RT_USING_M3AD
	m3ad_th();
#endif
}

int rt_application_init(void)
{
    rt_thread_t init_thread;

#if (RT_THREAD_PRIORITY_MAX == 32)
    init_thread = rt_thread_create("init",
                                   rt_init_thread_entry, RT_NULL,
                                   2048, 8, 20);
#else
    init_thread = rt_thread_create("init",
                                   rt_init_thread_entry, RT_NULL,
                                   2048, 80, 20);
#endif

    if (init_thread != RT_NULL)
    {
        rt_thread_startup(init_thread);
    }

    return 0;
}

long reset(void)
{
	NVIC_SystemReset();                                                  /* wait until reset */
	return 0;
}

#ifdef RT_USING_FINSH
FINSH_FUNCTION_EXPORT(reset,reboot system);
#endif

/*@}*/
