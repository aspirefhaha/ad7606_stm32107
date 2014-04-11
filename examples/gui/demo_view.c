#include <rtgui/rtgui.h>
#include <rtgui/widgets/view.h>
#include <rtgui/widgets/button.h>
#include <rtgui/widgets/workbench.h>
#include <rtgui/widgets/staticline.h>

/* ���ڴ����ʾ��ͼ�����飬���ɴ���32����ʾ��ͼ */
static rtgui_view_t* demo_view_list[32];
/* ��ǰ��ʾ��ͼ���� */
static rt_uint16_t demo_view_current = 0;
/* �ܹ���������ʾ��ͼ��Ŀ */
static rt_uint16_t demo_view_number = 0;

/* ��ʾ��һ����ʾ��ͼ */
void demo_view_next(struct rtgui_widget* widget, rtgui_event_t *event)
{
	if (demo_view_current + 1< demo_view_number)
	{
		demo_view_current ++;
		rtgui_view_show(demo_view_list[demo_view_current], RT_FALSE);
	}
}

/* ��ʾǰһ����ʾ��ͼ */
void demo_view_prev(struct rtgui_widget* widget, rtgui_event_t *event)
{
	if (demo_view_current != 0)
	{
		demo_view_current --;
		rtgui_view_show(demo_view_list[demo_view_current], RT_FALSE);
	}
}

/* ����һ����ʾ��ͼ�����ṩ��workbench����ʾ�õı��� */
rtgui_view_t* demo_view(rtgui_workbench_t* workbench, const char* title)
{
	struct rtgui_view* view;

	/* ������ͼ������ */
	view = rtgui_view_create(title);
	if (view == RT_NULL) return RT_NULL;

	/* �����ɹ�����ӵ������� */
	demo_view_list[demo_view_number] = view;
	demo_view_number ++;

	/* ��ӵ���workbench�� */
	rtgui_workbench_add_view(workbench, view);

	/* �����һ����ͼ��ǰһ����ͼ��ť */
	{
		struct rtgui_rect rect;
		struct rtgui_button *next_btn, *prev_btn;
		struct rtgui_label *label;
		struct rtgui_staticline *line;

		/* �����ͼ��λ����Ϣ(�ڼ��뵽workbench��ʱ��workbench���Զ�������ͼ�Ĵ�С) */
		rtgui_widget_get_rect(RTGUI_WIDGET(view), &rect);
		rtgui_widget_rect_to_device(RTGUI_WIDGET(view), &rect);
		rect.x1 += 5;
		rect.y1 += 5;
		rect.x2 -= 5;
		rect.y2 = rect.y1 + 20;

		/* ���������õı�ǩ */
		label = rtgui_label_create(title);
		/* ���ñ�ǩλ����Ϣ */
		rtgui_widget_set_rect(RTGUI_WIDGET(label), &rect);
		/* ��ӱ�ǩ����ͼ�� */
		rtgui_container_add_child(RTGUI_CONTAINER(view), RTGUI_WIDGET(label));

		rect.y1 += 20;
		rect.y2 += 20;
		/* ����һ��ˮƽ��staticline�� */
		line = rtgui_staticline_create(RTGUI_HORIZONTAL);
		/* ���þ�̬�ߵ�λ����Ϣ */
		rtgui_widget_set_rect(RTGUI_WIDGET(line), &rect);
		/* ��Ӿ�̬�ߵ���ͼ�� */
		rtgui_container_add_child(RTGUI_CONTAINER(view), RTGUI_WIDGET(line));

		/* �����ͼ��λ����Ϣ */
		rtgui_widget_get_rect(RTGUI_WIDGET(view), &rect);
		rtgui_widget_rect_to_device(RTGUI_WIDGET(view), &rect);
		rect.x2 -= 5;
		rect.y2 -= 5;
		rect.x1 = rect.x2 - 100;
		rect.y1 = rect.y2 - 25;

		/* ����"��һ��"��ť */
		next_btn = rtgui_button_create("��һ��");
		/* ����onbutton������demo_view_next���� */
		rtgui_button_set_onbutton(next_btn, demo_view_next);
		/* ���ð�ť��λ����Ϣ */
		rtgui_widget_set_rect(RTGUI_WIDGET(next_btn), &rect);
		/* ��Ӱ�ť����ͼ�� */
		rtgui_container_add_child(RTGUI_CONTAINER(view), RTGUI_WIDGET(next_btn));

		/* �����ͼ��λ����Ϣ */
		rtgui_widget_get_rect(RTGUI_WIDGET(view), &rect);
		rtgui_widget_rect_to_device(RTGUI_WIDGET(view), &rect);
		rect.x1 += 5;
		rect.y2 -= 5;
		rect.x2 = rect.x1 + 100;
		rect.y1 = rect.y2 - 25;

		/* ����"��һ��"��ť */
		prev_btn = rtgui_button_create("��һ��");
		/* ����onbutton������demo_view_prev���� */
		rtgui_button_set_onbutton(prev_btn, demo_view_prev);
		/* ���ð�ť��λ����Ϣ */
		rtgui_widget_set_rect(RTGUI_WIDGET(prev_btn), &rect);
		/* ��Ӱ�ť����ͼ�� */
		rtgui_container_add_child(RTGUI_CONTAINER(view), RTGUI_WIDGET(prev_btn));
	}

	/* ���ش�������ͼ */
	return view;
}

/* ����������ڷ�����ʾ��ͼ�Ķ���������� */
void demo_view_get_rect(rtgui_view_t* view, rtgui_rect_t *rect)
{
	RT_ASSERT(view != RT_NULL);
	RT_ASSERT(rect != RT_NULL);

	rtgui_widget_get_rect(RTGUI_WIDGET(view), rect);
	rtgui_widget_rect_to_device(RTGUI_WIDGET(view), rect);
	/* ȥ����ʾ������·���ť������ */
	rect->y1 += 45;
	rect->y2 -= 35;
}

void demo_view_get_logic_rect(rtgui_view_t* view, rtgui_rect_t *rect)
{
	RT_ASSERT(view != RT_NULL);
	RT_ASSERT(rect != RT_NULL);

	rtgui_widget_get_rect(RTGUI_WIDGET(view), rect);
	/* ȥ����ʾ������·���ť������ */
	rect->y1 += 45;
	rect->y2 -= 35;
}

/* ���Ǳ�׼�汾ʱ������������ڷ����Զ���������box�ؼ� */
#ifndef RTGUI_USING_SMALL_SIZE
rtgui_box_t* demo_view_create_box(rtgui_view_t* view, int orient)
{
	rtgui_rect_t rect;
	rtgui_box_t* box;

	/* �����ͼ��λ����Ϣ */
	rtgui_widget_get_rect(RTGUI_WIDGET(view), &rect);
	rect.y1 += 45;
	rect.y2 -= 25;

	/* ����һ���Զ��������� */
	box = rtgui_box_create(orient, &rect);
	/* ���box�ؼ�����ͼ�� */
	rtgui_container_add_child(RTGUI_CONTAINER(view), RTGUI_WIDGET(box));

	return box;
}
#endif

/* �������������ʾ��ǰ����ͼ */
void demo_view_show()
{
	if (demo_view_number != 0)
	{
		rtgui_view_show(demo_view_list[demo_view_current], RT_FALSE);
	}
}
