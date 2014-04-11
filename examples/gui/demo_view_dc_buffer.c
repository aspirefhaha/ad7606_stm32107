/*
 * �����嵥��DC Buffer��ʾ
 *
 * ������ӻ��ڴ�������view�Ͻ���DC Buffer����ʾ
 */

#include "demo_view.h"
#include <rtgui/rtgui_system.h>
#include <rtgui/widgets/label.h>
#include <rtgui/widgets/slider.h>
#include <rtgui/image.h>

static rtgui_image_t *background;
static struct rtgui_dc *dc_buffer;

/*
 * view���¼�������
 */
static rt_bool_t dc_buffer_event_handler(rtgui_widget_t* widget, rtgui_event_t *event)
{

	/* ����PAINT�¼����д��� */
	if (event->type == RTGUI_EVENT_PAINT)
	{
		struct rtgui_dc* dc;
		rtgui_rect_t rect;

		/*
		 * ��Ϊ�õ���demo view�����汾����һ���ֿؼ��������ڻ�ͼʱ��Ҫ��demo view
		 * �Ȼ�ͼ
		 */
		rtgui_view_event_handler(widget, event);

		/* ��ÿؼ�������DC */
		dc = rtgui_dc_begin_drawing(widget);
		/* ��������������DC�����أ�����ؼ��򸸿ؼ�������״̬��DC�ǻ�ȡ���ɹ��ģ� */
		if (dc == RT_NULL)
			return RT_FALSE;

		/* ���demo view�����ͼ������ */
		demo_view_get_logic_rect(RTGUI_VIEW(widget), &rect);

		rect.x1 += 10;
		rect.y1 += 10;
		rtgui_dc_blit(dc_buffer, NULL, dc, &rect);

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

/* ��������DC Buffer������ʾ�õ���ͼ */
rtgui_view_t *demo_view_dc_buffer(rtgui_workbench_t* workbench)
{
	rtgui_view_t *view;

	if (dc_buffer == RT_NULL)
	{
		rtgui_rect_t rect = {0, 0, 50, 50};

		/* ���� DC Buffer���� 50���� 50 */
		dc_buffer = rtgui_dc_buffer_create(50, 50);
		RTGUI_DC_FC(dc_buffer) = blue;
		rtgui_dc_fill_rect(dc_buffer, &rect);

		RTGUI_DC_FC(dc_buffer) = red;
		rtgui_dc_draw_circle(dc_buffer, 25, 25, 10);
	}

	view = demo_view(workbench, "����DC��ʾ");
	if (view != RT_NULL)
		/* ���ó��Լ����¼������� */
		rtgui_widget_set_event_handler(RTGUI_WIDGET(view), dc_buffer_event_handler);

	return view;
}
