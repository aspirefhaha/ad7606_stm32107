/*
 * ���һ��RTGUI�����ӣ���ʾ����δ���һ��RTGUI����
 * ��rtgui_win�����֧�У�û��toplevel�ؼ���Ĭ��panel
 * ��Ϊtoplevel����ؼ������԰�������һ�������桱
 */
#include <rtthread.h>
#include <rtgui/rtgui.h>
#include <panel.h>
#include <rtgui/event.h>
#include <rtgui/widgets/widget.h>
#include <rtgui/widgets/button.h>
#include <rtgui/widgets/view.h>
#include <rtgui/rtgui_theme.h>

#include "demo_view.h"

/* ���ڴ����ʾ��ͼ�����飬���ɴ���32����ʾ��ͼ */
static rtgui_view_t* demo_list[32];
/* ��ǰ��ʾ��ͼ���� */
static rt_uint16_t demo_current = 0;
/* �ܹ���������ʾ��ͼ��Ŀ */
static rt_uint16_t demo_number = 0;

/* ��ʾǰһ����ʾ��ͼ */
void demo_gui_prev(PVOID wdt, rtgui_event_t *event)
{
	rtgui_panel_t *panel = rtgui_panel_get();

	if (demo_current != 0)
	{
		RTGUI_WIDGET_HIDE(demo_list[demo_current]);
		demo_current --;
		RTGUI_WIDGET_UNHIDE(demo_list[demo_current]);
		rtgui_panel_update_clip(panel);
		rtgui_panel_redraw(&RTGUI_WIDGET_EXTENT(demo_list[demo_current]));
	}
}

/* ��ʾ��һ����ʾ��ͼ */
void demo_gui_next(PVOID wdt, rtgui_event_t *event)
{
	rtgui_panel_t *panel = rtgui_panel_get();

	if (demo_current + 1< demo_number)
	{
		RTGUI_WIDGET_HIDE(demo_list[demo_current]);
		demo_current ++;
		RTGUI_WIDGET_UNHIDE(demo_list[demo_current]);
		rtgui_panel_update_clip(panel);
		rtgui_panel_redraw(&RTGUI_WIDGET_EXTENT(demo_list[demo_current]));
	}
}

rtgui_view_t* demo_view_create(rtgui_view_t* parent_view, const char* title)
{
	rtgui_view_t* view;

	/* ������ͼ������ */
	view = rtgui_view_create(parent_view, title, 0,0,
				rtgui_widget_get_width(parent_view),
				rtgui_widget_get_height(parent_view));
	if (view == RT_NULL) return RT_NULL;
	rtgui_widget_set_style(view, RTGUI_BORDER_SIMPLE);
	RTGUI_WIDGET_HIDE(view);

	/* ���������õı�ǩ */
	rtgui_label_create(view, title, 10, 5, 200, 20);
	/* ����һ��ˮƽ��staticline�� */
	rtgui_staticline_create(view, 10, 30, 2, rtgui_widget_get_width(view)-20, RTGUI_HORIZONTAL);

	/* �����ɹ�����ӵ������� */
	demo_list[demo_number] = view;
	demo_number ++;

	return view;
}
static void rtgui_panel_entry(void* parameter)
{
	const struct rtgui_graphic_driver* gd = rtgui_graphic_driver_get_default();
	struct rt_messagequeue* mq;
	rtgui_panel_t *panel;
	rtgui_view_t *view;
	rtgui_button_t *button;
	
	/* ����GUIӦ����Ҫ����Ϣ���� */
	mq = rt_mq_create("Panel", 256, 32, RT_IPC_FLAG_FIFO);
	/* ע�ᵱǰ�߳�ΪGUI�߳� */
	rtgui_thread_register(rt_thread_self(), mq);
	panel = rtgui_panel_create(0,0,gd->width,gd->height);

	//{{{ TODO: START ADD CODE HERE.

	view = rtgui_view_create(panel, "demo_view", 5, 5, gd->width-10,gd->height-40);

	button = rtgui_button_create(panel, "Prev", 5,gd->height-30,50,25);
	rtgui_button_set_onbutton(button, demo_gui_prev);
	button = rtgui_button_create(panel, "Next", gd->width-55,gd->height-30,50,25);
	rtgui_button_set_onbutton(button, demo_gui_next);


	/* ��ʼ���������ӵ���ͼ */
#if RT_VERSION == 4
	demo_gui_benchmark(view);
#endif

/*	demo_view_dc(view);
#if RT_VERSION == 4
#ifdef RTGUI_USING_TTF
	demo_view_ttf(view);
#endif
#endif */

#ifndef RTGUI_USING_SMALL_SIZE
	demo_gui_dc_buffer(view);
#endif
	demo_gui_animation(view);
#ifndef RTGUI_USING_SMALL_SIZE
	demo_gui_buffer_animation(view);
#endif
	demo_gui_window(view);
	demo_gui_label(view);
	demo_gui_button(view);
	demo_gui_checkbox(view);
	demo_gui_progressbar(view);
	demo_gui_scrollbar(view);
	demo_gui_radiobox(view);
	demo_gui_textbox(view);
	demo_gui_listbox(view);
	demo_gui_menu(view);
	demo_gui_listctrl(view);
	demo_gui_combobox(view);
	demo_gui_slider(view);

#if defined(RTGUI_USING_DFS_FILERW) || defined(RTGUI_USING_STDIO_FILERW)
	demo_gui_image(view);
#endif
#ifdef RT_USING_MODULE	
#if defined(RTGUI_USING_DFS_FILERW) || defined(RTGUI_USING_STDIO_FILERW)
	demo_gui_module(view);
#endif
#endif
	/* demo_gui_listview(view); */
	/* demo_gui_listview_icon(view); */
#if defined(RTGUI_USING_DFS_FILERW) || defined(RTGUI_USING_STDIO_FILERW)
	demo_gui_fnview(view);
#endif
	demo_gui_rttab(view);
	rtgui_view_show(demo_list[demo_current]);

	//}}} END ADD CODE.

	rtgui_panel_show(panel);	

	/* ִ�й���̨�¼�ѭ�� */
	rtgui_panel_event_loop(panel);
	
	/* ȥע��GUI�߳� */
	rtgui_thread_deregister(rt_thread_self());
	rt_mq_delete(mq);
}

void rtgui_panel_init(void)
{
	static rt_bool_t main_inited = RT_FALSE;

	if(main_inited == RT_FALSE) /* �����ظ���ʼ�������ı��� */
	{
		struct rt_thread* tid;

		tid = rt_thread_create("Panel", rtgui_panel_entry, RT_NULL, 4096, 4, 5);

		if(tid != RT_NULL) rt_thread_startup(tid);

		main_inited = RT_TRUE;
	}
}

