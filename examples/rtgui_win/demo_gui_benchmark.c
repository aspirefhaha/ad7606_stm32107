#include <rtgui/dc.h>
#include <rtgui/dc_hw.h>
#include <rtgui/kbddef.h>
#include <rtgui/rtgui_system.h>
#include <rtgui/widgets/view.h>
#include <stdlib.h>
#include "demo_view.h"

#if RT_VERSION == 4
#define RAND(x1, x2) ((rand() % (x2 - x1)) + x1)

static rtgui_view_t* view = RT_NULL;
static int running = 0;

void _onidle(PVOID wdt, rtgui_event_t *event)
{
	rtgui_color_t color;
	rtgui_rect_t rect, draw_rect;
	struct rtgui_dc *dc;

	/* ��ÿؼ�������DC */
	dc = rtgui_dc_begin_drawing(view);
	if (dc == RT_NULL) return ;

	rtgui_widget_get_rect(view, &rect);
	rtgui_rect_inflate(&rect, -5);
	rect.y1 += 35;
	draw_rect.x1 = RAND(rect.x1, rect.x2);
	draw_rect.y1 = RAND(rect.y1, rect.y2);
	draw_rect.x2 = RAND(draw_rect.x1, rect.x2);
	draw_rect.y2 = RAND(draw_rect.y1, rect.y2);

	color = RTGUI_RGB(rand() % 255, rand() % 255, rand() % 255);
	RTGUI_WIDGET_BACKGROUND(view) = color;

	rtgui_dc_fill_rect(dc, &draw_rect);

	/* ��ͼ��� */
	rtgui_dc_end_drawing(dc);
}

void _draw_default(PVOID wdt, rtgui_event_t* event)
{
	struct rtgui_dc* dc;
	rtgui_widget_t* widget = (rtgui_widget_t*)wdt;
	rtgui_rect_t rect;

	/* ��Ϊ�õ���demo view�����汾����һ���ֿؼ��������ڻ�ͼʱ��Ҫ��demo view�Ȼ�ͼ */
	rtgui_view_event_handler(widget, event);

	/* ��ÿؼ�������DC */
	dc = rtgui_dc_begin_drawing(widget);
	if (dc == RT_NULL) /* ��������������DC�����أ�����ؼ��򸸿ؼ�������״̬��DC�ǻ�ȡ���ɹ��ģ� */
		return ;

	/* ���demo view�����ͼ������ */
	rtgui_widget_get_rect(widget, &rect);
	rtgui_rect_inflate(&rect, -5);
	rect.y1 += 35;

	/* �������� */
	RTGUI_WIDGET_BACKGROUND(widget) = default_background;
	rtgui_dc_fill_rect(dc, &rect);

	/* ��ʾ��ʾ */
	rtgui_dc_draw_text(dc, "��������ʼ/ֹͣ����...", &rect);

	/* ��ͼ��� */
	rtgui_dc_end_drawing(dc);
}

rt_bool_t benchmark_event_handler(PVOID wdt, rtgui_event_t *event)
{
	rtgui_widget_t *widget = (rtgui_widget_t*)wdt;

	if (event->type == RTGUI_EVENT_PAINT)
	{
		_draw_default(widget, event);
	}
	else if (event->type == RTGUI_EVENT_MOUSE_BUTTON)
	{
		rtgui_event_mouse_t *emouse = (rtgui_event_mouse_t*)event;

		if (emouse->button & RTGUI_MOUSE_BUTTON_DOWN)
		{
			if (running)
			{
				/* stop */
				rtgui_thread_set_onidle(RT_NULL);
				_draw_default(widget, event);
			}
			else
			{
				/* run */
				rtgui_thread_set_onidle(_onidle);
			}

			running = !running;
		}
		return RT_TRUE;
	}
	else
	{
		/* ����Ĭ�ϵ��¼������� */
		return rtgui_view_event_handler(widget, event);
	}

	return RT_FALSE;
}

rtgui_view_t *demo_gui_benchmark(rtgui_view_t* parent_view)
{
	srand(100);
	view = demo_view_create(parent_view, "��ͼ����");
	rtgui_widget_set_event_handler(view, benchmark_event_handler);

	return view;
}
#endif
