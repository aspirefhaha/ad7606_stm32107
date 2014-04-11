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

static struct rtgui_timer *timer;
static struct rtgui_label* label;
static struct rtgui_win* msgbox = RT_NULL;
static char label_text[80];
static rt_uint8_t cnt = 5;

/* ��ȡһ�������Ĵ��ڱ��� */
static char* get_win_title()
{
	static rt_uint8_t win_no = 0;
	static char win_title[16];

	rt_sprintf(win_title, "���� %d", ++win_no);
	return win_title;
}

/* ���ڹر�ʱ���¼����� */
void window_demo_close(struct rtgui_widget* widget, rtgui_event_t *even)
{
	rtgui_win_t* win;

	/* ������ؼ� */
	win = RTGUI_WIN(rtgui_widget_get_toplevel(widget));

	/* ���ٴ��� */
	rtgui_win_destroy(win);
}

/* �رնԻ���ʱ�Ļص����� */
void diag_close(struct rtgui_timer* timer, void* parameter)
{
	cnt --;
	rt_sprintf(label_text, "closed then %d second!", cnt);

	/* ���ñ�ǩ�ı������¿ؼ� */
	rtgui_label_set_text(label, label_text);
	rtgui_widget_update(RTGUI_WIDGET(label));

	if (cnt == 0)
	{
		/* ��ʱ���رնԻ��� */
		rtgui_win_destroy(msgbox);

		/* ֹͣ��ɾ����ʱ�� */
		rtgui_timer_stop(timer);
		rtgui_timer_destory(timer);
	}
}

/* AUTO���ڹر�ʱ���¼����� */
rt_bool_t auto_window_close(struct rtgui_widget* widget, struct rtgui_event* event)
{
	if (timer != RT_NULL)
	{
		/* ֹͣ��ɾ����ʱ�� */
		rtgui_timer_stop(timer);
		rtgui_timer_destory(timer);

		timer = RT_NULL;
	}

	return RT_TRUE;
}

static rt_uint16_t delta_x = 20;
static rt_uint16_t delta_y = 40;

/* ��������������ʾ */
static void demo_win_onbutton(struct rtgui_widget* widget, rtgui_event_t* event)
{
	rtgui_win_t *win;
	rtgui_label_t *label;
	rtgui_toplevel_t *parent;
	rtgui_rect_t rect = {0, 0, 150, 80};

	parent = RTGUI_TOPLEVEL(rtgui_widget_get_toplevel(widget));
	rtgui_rect_moveto(&rect, delta_x, delta_y);
	delta_x += 20;
	delta_y += 20;

	/* ����һ������ */
	win = rtgui_win_create(parent,
		get_win_title(), &rect, RTGUI_WIN_STYLE_DEFAULT);

	rect.x1 += 20;
	rect.x2 -= 5;
	rect.y1 += 5;
	rect.y2 = rect.y1 + 20;

	/* ���һ���ı���ǩ */
	label = rtgui_label_create("����һ����ͨ����");
	rtgui_widget_set_rect(RTGUI_WIDGET(label), &rect);
	rtgui_container_add_child(RTGUI_CONTAINER(win), RTGUI_WIDGET(label));

	/* ��ģ̬��ʾ���� */
	rtgui_win_show(win, RT_FALSE);
}

/* �����Զ�������ʾ */
static void demo_autowin_onbutton(struct rtgui_widget* widget, rtgui_event_t* event)
{
	rtgui_toplevel_t *parent;
	struct rtgui_rect rect ={50, 50, 200, 200};

	parent = RTGUI_TOPLEVEL(rtgui_widget_get_toplevel(widget));
	msgbox = rtgui_win_create(parent, "Information", &rect, RTGUI_WIN_STYLE_DEFAULT);
	if (msgbox != RT_NULL)
	{
		cnt = 5;
		rt_sprintf(label_text, "closed then %d second!", cnt);
		label = rtgui_label_create(label_text);
		rect.x1 += 5;
		rect.x2 -= 5;
		rect.y1 += 5;
		rect.y2 = rect.y1 + 20;
		rtgui_widget_set_rect(RTGUI_WIDGET(label), &rect);
		rtgui_container_add_child(RTGUI_CONTAINER(msgbox), RTGUI_WIDGET(label));

		/* ���ùرմ���ʱ�Ķ��� */
		rtgui_win_set_onclose(msgbox, auto_window_close);

		rtgui_win_show(msgbox, RT_FALSE);
	}

	/* ����һ����ʱ�� */
	timer = rtgui_timer_create(100, RT_TIMER_FLAG_PERIODIC, diag_close, RT_NULL);
	rtgui_timer_start(timer);
}

/* ����ģ̬������ʾ */
static void demo_modalwin_onbutton(struct rtgui_widget* widget, rtgui_event_t* event)
{
	rtgui_win_t *win;
	rtgui_label_t *label;
	rtgui_toplevel_t *parent;
	rtgui_rect_t rect = {0, 0, 150, 80};

	parent = RTGUI_TOPLEVEL(rtgui_widget_get_toplevel(widget));
	rtgui_rect_moveto(&rect, delta_x, delta_y);
	delta_x += 20;
	delta_y += 20;

	/* ����һ������ */
	win = rtgui_win_create(parent,
		get_win_title(), &rect, RTGUI_WIN_STYLE_DEFAULT);

	rect.x1 += 20;
	rect.x2 -= 5;
	rect.y1 += 5;
	rect.y2 = rect.y1 + 20;

	label = rtgui_label_create("����һ��ģʽ����");
	rtgui_widget_set_rect(RTGUI_WIDGET(label), &rect);
	rtgui_container_add_child(RTGUI_CONTAINER(win), RTGUI_WIDGET(label));

	/* ģ̬��ʾ���� */
	rtgui_win_show(win, RT_TRUE);
	/* ����ģ̬��ʾ���ڣ��ر�ʱ��������ɾ�����ڣ���Ҫ����ɾ������ */
	rtgui_win_destroy(win);
}

/* �����ޱ��ⴰ����ʾ */
static void demo_ntitlewin_onbutton(struct rtgui_widget* widget, rtgui_event_t* event)
{
	rtgui_win_t *win;
	rtgui_label_t *label;
	rtgui_button_t *button;
	rtgui_toplevel_t *parent;
	rtgui_rect_t widget_rect, rect = {0, 0, 150, 80};

	parent = RTGUI_TOPLEVEL(rtgui_widget_get_toplevel(widget));
	rtgui_rect_moveto(&rect, delta_x, delta_y);
	delta_x += 20;
	delta_y += 20;

	/* ����һ�����ڣ����Ϊ�ޱ��⼰�ޱ߿� */
	win = rtgui_win_create(parent,
		"no title", &rect, RTGUI_WIN_STYLE_NO_TITLE | RTGUI_WIN_STYLE_NO_BORDER);
	RTGUI_WIDGET_BACKGROUND(RTGUI_WIDGET(win)) = white;

	/* ����һ���ı���ǩ */
	label = rtgui_label_create("�ޱ߿򴰿�");
	rtgui_font_get_metrics(RTGUI_WIDGET_FONT(RTGUI_WIDGET(label)), "�ޱ߿򴰿�", &widget_rect);
	rtgui_rect_moveto_align(&rect, &widget_rect, RTGUI_ALIGN_CENTER_HORIZONTAL);
	widget_rect.y1 += 20;
	widget_rect.y2 += 20;
	rtgui_widget_set_rect(RTGUI_WIDGET(label), &widget_rect);
	rtgui_container_add_child(RTGUI_CONTAINER(win), RTGUI_WIDGET(label));
	RTGUI_WIDGET_BACKGROUND(RTGUI_WIDGET(label)) = white;

	/* ����һ���رհ�ť */
	widget_rect.x1 = 0;
	widget_rect.y1 = 0;
	widget_rect.x2 = 40;
	widget_rect.y2 = 20;
	rtgui_rect_moveto_align(&rect, &widget_rect, RTGUI_ALIGN_CENTER_HORIZONTAL);
	widget_rect.y1 += 40;
	widget_rect.y2 += 40;
	button = rtgui_button_create("�ر�");
	rtgui_widget_set_rect(RTGUI_WIDGET(button), &widget_rect);
	rtgui_container_add_child(RTGUI_CONTAINER(win), RTGUI_WIDGET(button));
	rtgui_button_set_onbutton(button, window_demo_close);

	/* ��ģ̬��ʾ���� */
	rtgui_win_show(win, RT_FALSE);
}

rtgui_view_t* demo_view_window(rtgui_workbench_t* workbench)
{
	rtgui_rect_t  rect;
	rtgui_view_t* view;
	rtgui_button_t *button;

	/* ����һ����ʾ�õ���ͼ */
	view = demo_view(workbench, "Window Demo");

	demo_view_get_rect(view, &rect);
	rect.x1 += 5;
	rect.x2 = rect.x1 + 100;
	rect.y1 += 5;
	rect.y2 = rect.y1 + 20;
	/* ������ť������ʾ�������� */
	button = rtgui_button_create("Normal Win");
	rtgui_widget_set_rect(RTGUI_WIDGET(button), &rect);
	rtgui_container_add_child(RTGUI_CONTAINER(view), RTGUI_WIDGET(button));
	/* ����onbuttonΪdemo_win_onbutton���� */
	rtgui_button_set_onbutton(button, demo_win_onbutton);

	demo_view_get_rect(view, &rect);
	rect.x1 += 5;
	rect.x2 = rect.x1 + 100;
	rect.y1 += 5 + 25;
	rect.y2 = rect.y1 + 20;
	/* ������ť������ʾһ���Զ��رյĴ��� */
	button = rtgui_button_create("Auto Win");
	rtgui_widget_set_rect(RTGUI_WIDGET(button), &rect);
	rtgui_container_add_child(RTGUI_CONTAINER(view), RTGUI_WIDGET(button));
	/* ����onbuttonΪdemo_autowin_onbutton���� */
	rtgui_button_set_onbutton(button, demo_autowin_onbutton);

	demo_view_get_rect(view, &rect);
	rect.x1 += 5;
	rect.x2 = rect.x1 + 100;
	rect.y1 += 5 + 25 + 25;
	rect.y2 = rect.y1 + 20;
	/* ������ť���ڴ���һ��ģʽ���� */
	button = rtgui_button_create("Modal Win");
	rtgui_widget_set_rect(RTGUI_WIDGET(button), &rect);
	rtgui_container_add_child(RTGUI_CONTAINER(view), RTGUI_WIDGET(button));
	/* ����onbuttonΪdemo_modalwin_onbutton���� */
	rtgui_button_set_onbutton(button, demo_modalwin_onbutton);

	demo_view_get_rect(view, &rect);
	rect.x1 += 5;
	rect.x2 = rect.x1 + 100;
	rect.y1 += 5 + 25 + 25 + 25;
	rect.y2 = rect.y1 + 20;
	/* ������ť���ڴ���һ������������Ĵ��� */
	button = rtgui_button_create("NoTitle Win");
	rtgui_widget_set_rect(RTGUI_WIDGET(button), &rect);
	rtgui_container_add_child(RTGUI_CONTAINER(view), RTGUI_WIDGET(button));
	/* ����onbuttonΪdemo_ntitlewin_onbutton���� */
	rtgui_button_set_onbutton(button, demo_ntitlewin_onbutton);

	return view;
}
