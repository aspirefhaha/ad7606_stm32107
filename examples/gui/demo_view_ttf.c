/*
 * �����嵥��TTF������ʾ��ʾ
 *
 * ������ӻ��ڴ�������view�Ͻ���TTF������ʾ����ʾ
 */

#include "demo_view.h"
#include <rtgui/dc.h>
#include <rtgui/font.h>
#include <rtgui/rtgui_system.h>

#ifdef RTGUI_USING_TTF
static 	rtgui_font_t *font_16, *font_24, *font_36, *font_48;

/*
 * view���¼�������
 */
rt_bool_t ttf_event_handler(rtgui_widget_t* widget, rtgui_event_t *event)
{
	/* ����PAINT�¼����д��� */
	if (event->type == RTGUI_EVENT_PAINT)
	{
		struct rtgui_dc* dc;
		rtgui_rect_t rect;
		rtgui_font_t* saved;

		/*
		 * ��Ϊ�õ���demo view�����汾����һ���ֿؼ��������ڻ�ͼʱ��Ҫ��demo view
		 * �Ȼ�ͼ
		 */
		rtgui_view_event_handler(widget, event);

		/************************************************************************/
		/* �������DC�Ĳ���                                                     */
		/************************************************************************/

		/* ��ÿؼ�������DC */
		dc = rtgui_dc_begin_drawing(widget);
		/* ��������������DC�����أ�����ؼ��򸸿ؼ�������״̬��DC�ǻ�ȡ���ɹ��ģ� */
		if (dc == RT_NULL)
			return RT_FALSE;

		/* ���demo view�����ͼ������ */
		demo_view_get_rect(RTGUI_VIEW(widget), &rect);

		saved = RTGUI_WIDGET_FONT(widget);

		RTGUI_WIDGET_FONT(widget) = font_16;
		rtgui_dc_draw_text(dc, "ABCD����", &rect);
		rect.y1 += 18; 

		RTGUI_WIDGET_FONT(widget) = font_24;
		rtgui_dc_draw_text(dc, "ABCD����", &rect);
		rect.y1 += 26; 

		RTGUI_WIDGET_FONT(widget) = font_36;
		rtgui_dc_draw_text(dc, "ABCD����", &rect);
		rect.y1 += 38; 

		RTGUI_WIDGET_FONT(widget) = font_48;
		rtgui_dc_draw_text(dc, "ABCD����", &rect);

		RTGUI_WIDGET_FONT(widget) = saved;
		/* ��ͼ��� */
		rtgui_dc_end_drawing(dc);
	}
	else
	{
		/* �����¼�������Ĭ�ϵ��¼������� */
		return rtgui_view_event_handler(widget, event);
	}

	return RT_FALSE;
}

/* ��������TTF������ʾ��ʾ�õ���ͼ */
rtgui_view_t *demo_view_ttf(rtgui_workbench_t* workbench)
{
	rtgui_view_t *view;

	font_16 = rtgui_freetype_font_create("d:/simsun.ttf", 0, 0, 16);
	font_24 = rtgui_freetype_font_create("d:/simsun.ttf", 0, 0, 24);
	font_36 = rtgui_freetype_font_create("d:/simsun.ttf", 0, 0, 36);
	font_48 = rtgui_freetype_font_create("d:/simsun.TTF", 0, 0, 72);

	view = demo_view(workbench, "TTF ��ʾ");
	if (view != RT_NULL)
	{
		RTGUI_WIDGET_BACKGROUND(RTGUI_WIDGET(view)) = white;
		/* ���ó��Լ����¼������� */
		rtgui_widget_set_event_handler(RTGUI_WIDGET(view), ttf_event_handler);
	}

	return view;
}
#endif
