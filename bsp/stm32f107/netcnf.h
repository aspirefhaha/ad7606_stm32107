
#ifndef __NETAPP_H
#define __NETAPP_H
#include "mpkg.h"
void net_send_pkg(com_pkg_t pPkg);
void tcp_netcom_entry(void * parameter);
extern int curgear;
#endif

