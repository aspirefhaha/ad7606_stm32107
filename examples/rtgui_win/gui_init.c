#include <rtgui/rtgui.h>
#include <rtgui/rtgui_system.h>
#include <rtgui/rtgui_server.h>
#include "touch.h"

void rt_hw_lcd_init(void);
void rt_hw_key_init(void);
void rtgui_panel_init(void);


/* GUI�����ʾ��ڣ�������߳��н��г�ʼ�� */
void rtgui_startup()
{
	/* GUIϵͳ��ʼ�� */
    rtgui_system_server_init();

	/* ������ʼ�� */
	rt_hw_key_init();
	/* LCD������ʼ�� */
	rt_hw_lcd_init();
	rtgui_touch_hw_init();
	rtgui_panel_init();
}
