/*
 * �����嵥��������ʾ
 *
 * ������ӻ��ȴ�����һ����ʾ�õ�view�����������İ�ťʱ�᲻ͬ��ģʽ��������
 */

#include <rtgui/rtgui.h>
#include <rtgui/rtgui_system.h>
#include <rtgui/widgets/window.h>
#include <rtgui/widgets/label.h>
#include <rtgui/widgets/button.h>
#include "demo_view.h"
#include <string.h>

static rtgui_timer_t *timer;
static char label_text[80];
static rt_uint8_t cnt = 5;

/* ��ȡһ�������Ĵ��ڱ��� */
static char* get_win_title(void)
{
	static rt_uint8_t win_no = 0;
	static char win_title[16];

	rt_sprintf(win_title, "win %d", ++win_no);
	return win_title;
}

rt_bool_t auto_window_close(PVOID wdt, rtgui_event_t* event)
{
	if(timer != RT_NULL)
	{
		rtgui_timer_stop(timer);
		rtgui_timer_destory(timer);
	}
	return RT_TRUE;
}

/* �رնԻ���ʱ�Ļص����� */
void diag_close(struct rtgui_timer* timer, void* parameter)
{
	rtgui_label_t *label;

	label = (rtgui_label_t*)parameter;
	cnt --;
	rt_sprintf(label_text, "closed then %d s", cnt);

	/* ���ñ�ǩ�ı������¿ؼ� */
	rtgui_label_set_text(label, label_text);
	rtgui_widget_update(RTGUI_WIDGET(label));

	if (cnt == 0)
	{
		rtgui_win_t *win;
		win = rtgui_win_get_win_by_widget(label);
		if(win == RT_NULL) return;
		/* ��ʱ���رնԻ��� */
		rtgui_win_close(win, RT_NULL);

		/* ֹͣ��ɾ����ʱ�� */
		rtgui_timer_stop(timer);
		rtgui_timer_destory(timer);
	}
}

static rt_uint16_t delta_x = 20;
static rt_uint16_t delta_y = 40;

/* ��������������ʾ */
static void demo_win_onbutton(PVOID wdt, rtgui_event_t* event)
{
	rtgui_win_t *win;
	PVOID parent;
	rtgui_rect_t rect={0, 0, 180, 120};

	parent = rtgui_widget_get_toplevel(wdt);
	rtgui_rect_moveto(&rect, delta_x, delta_y);
	delta_x += 20;
	delta_y += 20;

	/* ����һ������ */
	win = rtgui_win_create(parent, get_win_title(), &rect, RTGUI_WIN_DEFAULT);

	/* ���һ���ı���ǩ */
	rtgui_label_create(win, "����һ����ͨ����", 10, 30, 150, 20);

	/* ��ģ̬��ʾ���� */
	rtgui_win_show(win, RT_FALSE);
}

/* �����Զ�������ʾ */
static void demo_autowin_onbutton(PVOID wdt, rtgui_event_t* event)
{
	PVOID parent;
	rtgui_label_t *label = RT_NULL;
	rtgui_win_t *msgbox;
	struct rtgui_rect rect ={50, 50, 200, 200};

	parent = rtgui_widget_get_toplevel(wdt);
	msgbox = rtgui_win_create(parent, "Information", &rect, RTGUI_WIN_DEFAULT);
	if (msgbox != RT_NULL)
	{
		cnt = 5;
		rt_sprintf(label_text, "closed then %d s", cnt);
		label = rtgui_label_create(msgbox, label_text, 10,30,120,20);
		/* ���ùرմ���ʱ�Ķ��� */
		rtgui_win_set_onclose(msgbox, auto_window_close);

		rtgui_win_show(msgbox, RT_FALSE);
	}

	/* ����һ����ʱ�� */
	timer = rtgui_timer_create(100, RT_TIMER_FLAG_PERIODIC, diag_close, label);
	rtgui_timer_start(timer);
}

/* ����ģ̬������ʾ */
static void demo_modalwin_onbutton(PVOID wdt, rtgui_event_t* event)
{
	rtgui_win_t *win;
	PVOID parent;
	rtgui_rect_t rect = {0, 0, 180, 120};

	parent = rtgui_widget_get_toplevel(wdt);
	rtgui_rect_moveto(&rect, (rtgui_widget_get_width(parent) -180)/2, 
							 (rtgui_widget_get_height(parent)-120)/2);

	/* ����һ������ */
	win = rtgui_win_create(parent,get_win_title(), &rect, RTGUI_WIN_DEFAULT);

	rect.x1 += 20;
	rect.x2 -= 5;
	rect.y1 += 5;
	rect.y2 = rect.y1 + 20;

	rtgui_label_create(win, "����һ��ģʽ����", 20, 30, 150,20);

	/* ģ̬��ʾ���� */
	rtgui_win_show(win, RT_TRUE);
}

/* �����ޱ��ⴰ����ʾ */
static void demo_ntitlewin_onbutton(PVOID wdt, rtgui_event_t* event)
{
	rtgui_win_t *win;
	rtgui_label_t *label;
	rtgui_button_t *button;
	PVOID parent;
	rtgui_rect_t rect = {0, 0, 180, 120};

	parent = rtgui_widget_get_toplevel(wdt);
	rtgui_rect_moveto(&rect, delta_x, delta_y);
	delta_x += 20;
	delta_y += 20;
	/* ����һ�����ڣ����Ϊ�ޱ��⼰�ޱ߿� */
	win = rtgui_win_create(parent,"no title", &rect, RTGUI_WIN_DEFAULT);
	RTGUI_WIDGET_BACKGROUND(win) = white;
	win->level = RTGUI_WIN_LEVEL_EXPERT;
	/* ����һ���ı���ǩ */
	label = rtgui_label_create(win, "�ޱ���������", 10, 30, 100, 20);
	RTGUI_WIDGET_BACKGROUND(label) = white;

	button = rtgui_button_create(win,"�ر�", 65, 85, 60, 25);
	rtgui_button_set_onbutton(button, rtgui_win_close);

	/* ��ģ̬��ʾ���� */
	rtgui_win_show(win, RT_FALSE);
}

rtgui_view_t* demo_gui_window(rtgui_view_t* parent_view)
{
	rtgui_view_t* view;
	rtgui_button_t *button;

	/* ����һ����ʾ�õ���ͼ */
	view = demo_view_create(parent_view, "Window Demo");


	button = rtgui_button_create(view, "Normal Win", 10, 40, 100, 25);
	rtgui_button_set_onbutton(button, demo_win_onbutton);

	button = rtgui_button_create(view, "Auto Win", 10, 70, 100, 25);
	rtgui_button_set_onbutton(button, demo_autowin_onbutton);

	button = rtgui_button_create(view, "Modal Win", 10, 100, 100, 25);
	rtgui_button_set_onbutton(button, demo_modalwin_onbutton);

	button = rtgui_button_create(view, "NoTitle Win", 10, 130, 100, 25);
	rtgui_button_set_onbutton(button, demo_ntitlewin_onbutton);

	return view;
}
