/*
 * �����嵥��DC����ʾͼ����ʾ
 *
 * ������ӻ��ڴ�������view����ʾͼ��
 */

#include "demo_view.h"
#include <rtgui/widgets/button.h>
#include <rtgui/widgets/filelist_view.h>
#include <string.h>

#if defined(RTGUI_USING_DFS_FILERW) || defined(RTGUI_USING_STDIO_FILERW)

rtgui_filelist_view_t *demo_fview;
static rtgui_view_t *image_view = RT_NULL;
static rtgui_image_t *demo_image = RT_NULL;

/* ���ļ�������,����ֻ����ͼ���ļ� */
rt_bool_t demo_image_fview_on_item(PVOID wdt, rtgui_event_t *event)
{
	rtgui_filelist_view_t *fview = wdt;
	if(fview == RT_NULL) return RT_FALSE;

	if(fview->pattern != RT_NULL && fview->items != RT_NULL)
	{
		char path[32], image_type[8];

		/* �����ļ�·���ı�ǩ */
		rtgui_filelist_view_get_fullpath(fview, path, sizeof(path));
		if (demo_image != RT_NULL) 
		{
			rtgui_image_destroy(demo_image);
			demo_image = RT_NULL;
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
			demo_image = rtgui_image_create_from_file(image_type, path, RT_TRUE);
	}

	rtgui_widget_update(image_view);
	return RT_TRUE;
}

/* ���ذ�ť�Ļص����� */
static void back_btn_onbutton(PVOID wdt, rtgui_event_t* event)
{
	if(demo_fview != RT_NULL)
	{
		rtgui_filelist_view_goto_topfolder(demo_fview);
	}		
}

/* �򿪰�ť�Ļص����� */
static void open_btn_onbutton(PVOID wdt, rtgui_event_t* event)
{
	if(demo_fview != RT_NULL)
	{
		rtgui_filelist_view_on_enter(demo_fview);
	}		
}

/* ��ʾ��ͼ���¼������� */
static rt_bool_t demo_gui_image_handler(PVOID wdt, rtgui_event_t *event)
{
	rtgui_widget_t* widget = wdt;

	if(event->type == RTGUI_EVENT_PAINT)
	{
		struct rtgui_dc* dc;
		rtgui_rect_t rect;

		/* ��ÿؼ�������DC */
		dc = rtgui_dc_begin_drawing(widget);
		if(dc == RT_NULL) return RT_FALSE;

		/* ���demo view�����ͼ������ */
		rtgui_widget_get_rect(widget, &rect);
		/* ������� */
		rtgui_dc_fill_rect(dc, &rect);
		/* ����ͼƬ */
		if(demo_image != RT_NULL) rtgui_image_blit(demo_image, dc, &rect);

		/* ��ͼ��� */
		rtgui_dc_end_drawing(dc);
	}
	else
		return rtgui_view_event_handler(widget, event);
	
	return RT_TRUE;
}

/* ����������ʾͼ�����ʾ��ͼ */
rtgui_view_t* demo_gui_image(rtgui_view_t* parent_view)
{
	rtgui_button_t* button;
	rtgui_view_t *view;
	
	/* �ȴ���һ����ʾ��ͼ */
	view = demo_view_create(parent_view, "ͼ����ʾ");

	/* ����һ���ļ�����б� */
#ifdef _WIN32
	demo_fview = rtgui_filelist_view_create(view, "d:\\", "*.hdc", 5, 32, 200, 68);
#else
	demo_fview = rtgui_filelist_view_create(view, "/", "*.hdc", 5, 32, 200, 68);
#endif
	demo_fview->on_item = demo_image_fview_on_item;

	/* ���һ�����ذ�ť,����ļ���ʱ,���ڷ�����һ��Ŀ¼ */
	button = rtgui_button_create(view, "back", 5, 102, 40, 24);
	rtgui_button_set_onbutton(button, back_btn_onbutton);
	/* ���һ���򿪰�ť,����ļ�����,���ڽ�����һ��Ŀ¼,���ߴ��ļ� */
	button = rtgui_button_create(view, "open", 5, 130, 40, 24);
	rtgui_button_set_onbutton(button, open_btn_onbutton);
	
	/* ����һ����ͼ,������ʾͼƬ */
	image_view = rtgui_view_create(view, "image_view", 50, 102, 160, 120);
	if(image_view == RT_NULL) return RT_NULL;
	/* ��image_view����һ���¼������� */
	rtgui_widget_set_event_handler(image_view, demo_gui_image_handler);

	return view;
}
#endif
