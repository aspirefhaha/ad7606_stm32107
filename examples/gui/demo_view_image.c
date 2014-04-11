/*
 * �����嵥��DC����ʾͼ����ʾ
 *
 * ������ӻ��ڴ�������view����ʾͼ��
 */

#include "demo_view.h"
#include <rtgui/widgets/button.h>
#include <rtgui/widgets/filelist_view.h>
#include <string.h>

static rtgui_image_t* image = RT_NULL;
static rtgui_view_t* _view = RT_NULL;

#if defined(RTGUI_USING_DFS_FILERW) || defined(RTGUI_USING_STDIO_FILERW)
/* �򿪰�ť�Ļص����� */
static void open_btn_onbutton(rtgui_widget_t* widget, struct rtgui_event* event)
{
	rtgui_filelist_view_t *view;
	rtgui_workbench_t *workbench;
	rtgui_rect_t rect;

	/* ��ö����workbench */
	workbench = RTGUI_WORKBENCH(rtgui_widget_get_toplevel(widget));
	rtgui_widget_get_rect(RTGUI_WIDGET(workbench), &rect);

	/* WIN32ƽ̨�Ϻ���ʵ�豸�ϵĳ�ʼ·������ */
#ifdef _WIN32
	view = rtgui_filelist_view_create(workbench, "d:\\", "*.*", &rect);
#else
	view = rtgui_filelist_view_create(workbench, "/", "*.*", &rect);
#endif
	/* ģ̬��ʾһ���ļ��б���ͼ�����ṩ���û�ѡ��ͼ���ļ� */
	if (rtgui_view_show(RTGUI_VIEW(view), RT_TRUE) == RTGUI_MODAL_OK)
	{
		char path[32], image_type[8];

		/* �����ļ�·���ı�ǩ */
		rtgui_filelist_view_get_fullpath(view, path, sizeof(path));
		if (image != RT_NULL) 
		{
			rtgui_image_destroy(image);
			image = RT_NULL;
		}

		rt_memset(image_type, 0, sizeof(image_type));

		/* ���ͼ������� */
		if (rt_strstr(path, ".bmp") != RT_NULL ||
			rt_strstr(path, ".BMP") != RT_NULL)
			strcat(image_type, "bmp");
		if (rt_strstr(path, ".png") != RT_NULL ||
			rt_strstr(path, ".PNG") != RT_NULL)
			strcat(image_type, "png");
		if (rt_strstr(path, ".jpg") != RT_NULL ||
			rt_strstr(path, ".JPG") != RT_NULL)
			strcat(image_type, "jpeg");
		if (rt_strstr(path, ".hdc") != RT_NULL ||
			rt_strstr(path, ".HDC") != RT_NULL)
			strcat(image_type, "hdc");

		/* ���ͼ���ļ���Ч��������Ӧ��rtgui_image���� */
		if (image_type[0] != '\0')
			image = rtgui_image_create_from_file(image_type, path, RT_TRUE);
	}

	/* ɾ�� �ļ��б� ��ͼ */
	rtgui_view_destroy(RTGUI_VIEW(view));
	rtgui_view_show(_view, RT_FALSE);
}

/* ��ʾ��ͼ���¼������� */
static rt_bool_t demo_view_event_handler(rtgui_widget_t* widget, rtgui_event_t *event)
{
	rt_bool_t result;

	/* �ȵ���Ĭ�ϵ��¼�������(����ֻ����PAINT�¼�������ʾ��ͼ���б����һЩ�ؼ�) */
	result = rtgui_view_event_handler(widget, event);

	if (event->type == RTGUI_EVENT_PAINT)
	{
		struct rtgui_dc* dc;
		rtgui_rect_t rect;

		/* ��ÿؼ�������DC */
		dc = rtgui_dc_begin_drawing(widget);
		if (dc == RT_NULL)
			/* ��������������DC������(����ؼ��򸸿ؼ�������״̬��DC�ǻ�ȡ���ɹ���) */
			return RT_FALSE;

		/* ���demo view�����ͼ������ */
		demo_view_get_rect(RTGUI_VIEW(widget), &rect);

		/* ���ͼ����ʾ���� */
		rect.x1 += 5; rect.x2 -= 5;
		rect.y1 += 30;

		if (image != RT_NULL)
			rtgui_image_blit(image, dc, &rect);

		/* ��ͼ��� */
		rtgui_dc_end_drawing(dc);
	}

	return result;
}

/* ����������ʾͼ�����ʾ��ͼ */
rtgui_view_t* demo_view_image(rtgui_workbench_t* workbench)
{
	rtgui_rect_t rect;
	rtgui_button_t* open_btn;

	/* �ȴ���һ����ʾ��ͼ */
	_view = demo_view(workbench, "ͼ����ʾ");
	if (_view != RT_NULL)
		/* ����Ĭ�ϵ��¼���������demo_view_event_handler���� */
		rtgui_widget_set_event_handler(RTGUI_WIDGET(_view), demo_view_event_handler);

	/* ���һ����ť */
	demo_view_get_rect(_view, &rect);
	rect.x1 += 5; rect.x2 = rect.x1 + 120;
	rect.y2 = rect.y1 + 20;
	open_btn = rtgui_button_create("��ͼ���ļ�");
	rtgui_container_add_child(RTGUI_CONTAINER(_view), RTGUI_WIDGET(open_btn));
	rtgui_widget_set_rect(RTGUI_WIDGET(open_btn), &rect);
	rtgui_button_set_onbutton(open_btn, open_btn_onbutton);

	return _view;
}
#endif
