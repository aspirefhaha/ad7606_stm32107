#include <rtgui/dc.h>
#include <rtgui/rtgui_system.h>
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
static struct rtgui_dc *dc_buffer;
static void timeout(struct rtgui_timer* timer, void* parameter)
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
	demo_view_get_logic_rect(RTGUI_VIEW(widget), &rect);
	rect.y2 -= 5;

	/* �ж��Ƿ��ǵ�һ�λ�ͼ */
	if ((text_rect.x1 == 0) && (text_rect.y1 == 0))
	{
		rtgui_rect_moveto(&text_rect, rect.x1, rect.y1);
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
	rect = text_rect;
	rect.x2 += 2; rect.y2 += 2;
	rtgui_dc_blit(dc_buffer, NULL, dc, &rect);

	/* ��ͼ��� */
	rtgui_dc_end_drawing(dc);
}

static rt_bool_t animation_event_handler(rtgui_widget_t* widget, rtgui_event_t *event)
{
	if (event->type == RTGUI_EVENT_PAINT)
	{
		struct rtgui_dc* dc;
		rtgui_rect_t rect;

		/* ��Ϊ�õ���demo view�����汾����һ���ֿؼ��������ڻ�ͼʱ��Ҫ��demo view�Ȼ�ͼ */
		rtgui_view_event_handler(widget, event);

		/* ��ÿؼ�������DC */
		dc = rtgui_dc_begin_drawing(widget);
		if (dc == RT_NULL) /* ��������������DC�����أ�����ؼ��򸸿ؼ�������״̬��DC�ǻ�ȡ���ɹ��ģ� */
			return RT_FALSE;

        /* ���demo view�����ͼ������ */
        demo_view_get_logic_rect(RTGUI_VIEW(widget), &rect);

		/* ��ͼ */
		rect = text_rect;
		rtgui_rect_inflate(&rect, +1);
		rtgui_dc_blit(dc_buffer, NULL, dc, &rect);

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

rtgui_view_t *demo_view_buffer_animation(rtgui_workbench_t* workbench)
{
	rtgui_view_t *view;

	view = demo_view(workbench, "DC ����������");
	if (view != RT_NULL)
		rtgui_widget_set_event_handler(RTGUI_WIDGET(view), animation_event_handler);

	rtgui_font_get_metrics(RTGUI_WIDGET_FONT(RTGUI_WIDGET(view)), "���嶯��", &text_rect);
	if (dc_buffer == RT_NULL)
	{
		rtgui_rect_t rect;

		rect.x1 = 0; rect.x2 = rtgui_rect_width(text_rect) + 2;
		rect.y1 = 0; rect.y2 = rtgui_rect_height(text_rect) + 2;

		/* ���� DC Buffer���� 50���� 50 */
		dc_buffer = rtgui_dc_buffer_create(rtgui_rect_width(rect), rtgui_rect_height(rect));
		RTGUI_DC_FC(dc_buffer) = RTGUI_WIDGET_BACKGROUND(RTGUI_WIDGET(view));
		rtgui_dc_fill_rect(dc_buffer, &rect);
		RTGUI_DC_FC(dc_buffer) = black;
		rect.x1 = 1; rect.y1 = 1;
		rtgui_dc_draw_text(dc_buffer, "���嶯��", &rect);
	}

	/* ������ʱ���Դ������� */
	timer = rtgui_timer_create(1, RT_TIMER_FLAG_PERIODIC, timeout, (void*)view);
	rtgui_timer_start(timer);

	return view;
}
