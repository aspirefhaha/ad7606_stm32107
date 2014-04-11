#include <rtgui/dc.h>
#include <rtgui/rtgui_system.h>
#include <rtgui/widgets/widget.h>
#include <rtgui/widgets/view.h>
#include "demo_view.h"

/*
 * ֱ����DC�ϻ�ͼ��ʵ�ֶ���Ч��
 *
 * �����������ڶ�ʱ�������ģ������·�����ʾ����
 * "�����ҷ�"
 */
static rt_int8_t dx = 1, dy = 1;
static rtgui_rect_t text_rect;
static rtgui_timer_t *timer;

void timeout(struct rtgui_timer* timer, void* parameter)
{
	struct rtgui_dc* dc;
	rtgui_rect_t rect;
	rtgui_widget_t *widget;

	/* �ؼ�(view)ͨ��parameter�������ݸ���ʱ�� */
	widget = (rtgui_widget_t*)parameter;

	/* ��ÿؼ�������DC */
	dc = rtgui_dc_begin_drawing(widget);
	if (dc == RT_NULL) /* ��������������DC�����أ�����ؼ��򸸿ؼ�������״̬��DC�ǻ�ȡ���ɹ��ģ� */
		return ;

	/* ���demo view�����ͼ��������Ҫ�����жϱ߽� */
	rtgui_widget_get_rect(widget, &rect);
	rtgui_rect_inflate(&rect, -5);
	rect.y1 += 35;

	/* �ж��Ƿ��ǵ�һ�λ�ͼ */
	if ((text_rect.x1 == 0) && (text_rect.y1 == 0))
	{
		rtgui_rect_moveto(&text_rect, rect.x1, rect.y1);
	}
	else
	{
	    /* �����ϵ����� */
	    rtgui_dc_fill_rect(dc, &text_rect);
	}

    /* ����dx��dy */
	if (text_rect.x2 >= rect.x2) dx = -1;
	if (text_rect.x1 < rect.x1)  dx = 1;
	if (text_rect.y2 >= rect.y2) dy = -1;
	if (text_rect.y1 < rect.y1) dy = 1;

    /* �ƶ��ı����λ�� */
	text_rect.x1 += dx; text_rect.x2 += dx;
	text_rect.y1 += dy; text_rect.y2 += dy;

    /* ��ͼ */
	rtgui_dc_draw_text(dc, "�����ҷ�", &text_rect);

	/* ��ͼ��� */
	rtgui_dc_end_drawing(dc);
}

rt_bool_t animation_event_handler(PVOID wdt, rtgui_event_t *event)
{
	rtgui_widget_t *widget = (rtgui_widget_t*)wdt;
	if (event->type == RTGUI_EVENT_PAINT)
	{
		rtgui_dc_t* dc;
		rtgui_rect_t rect;

		/* ��Ϊ�õ���demo view�����汾����һ���ֿؼ��������ڻ�ͼʱ��Ҫ��demo view�Ȼ�ͼ */
		rtgui_view_event_handler(widget, event);

		/* ��ÿؼ�������DC */
		dc = rtgui_dc_begin_drawing(widget);
		if (dc == RT_NULL) /* ��������������DC�����أ�����ؼ��򸸿ؼ�������״̬��DC�ǻ�ȡ���ɹ��ģ� */
			return RT_FALSE;

        /* ���demo view�����ͼ������ */
        rtgui_widget_get_rect(widget, &rect);
		rtgui_rect_inflate(&rect, -5);
		rect.y1 += 35;

	    /* �������� */
	    rtgui_dc_fill_rect(dc, &rect);

        /* ��ͼ */
        rtgui_dc_draw_text(dc, "�����ҷ�", &text_rect);

		/* ��ͼ��� */
		rtgui_dc_end_drawing(dc);
	}
	else
	{
		/* ����Ĭ�ϵ��¼������� */
		return rtgui_view_event_handler(widget, event);
	}

	return RT_FALSE;
}

rtgui_view_t *demo_gui_animation(rtgui_view_t* parent_view)
{
	rtgui_view_t *view;

	view = demo_view_create(parent_view, "DC ����");
	if (view != RT_NULL)
		rtgui_widget_set_event_handler(view, animation_event_handler);

	rtgui_font_get_metrics(RTGUI_WIDGET_FONT(view), "�����ҷ�", &text_rect);
	rtgui_rect_moveto(&text_rect, 5, 40);
	/* ������ʱ���Դ������� */
	timer = rtgui_timer_create(2, RT_TIMER_FLAG_PERIODIC, timeout, view);
	rtgui_timer_start(timer);

	return view;
}
