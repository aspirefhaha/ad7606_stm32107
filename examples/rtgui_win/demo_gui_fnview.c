/*
 * �����嵥���ļ��б���ͼ��ʾ
 *
 * ������ӻ��ȴ�����һ����ʾ�õ�view�����������İ�ťʱ�ᰴ��ģʽ��ʾ����ʽ��ʾ
 * �µ��ļ��б���ͼ��
 */
#include "demo_view.h"
#include <rtgui/widgets/label.h>
#include <rtgui/widgets/button.h>
#include <rtgui/widgets/filelist_view.h>

#if defined(RTGUI_USING_DFS_FILERW) || defined(RTGUI_USING_STDIO_FILERW)

/* �ļ������� */
rt_bool_t demo_fview_on_item(PVOID wdt, struct rtgui_event *event)
{
	rt_kprintf("fview file on item.\n");
	return RT_TRUE;
}

/* ����������ʾ�ļ��б���ͼ����ͼ */
/* �����: �ƶ�
 * �س���: ������һ��Ŀ¼,������ļ�������
 * �˸��: ������һ��Ŀ¼ 
 */
rtgui_view_t* demo_gui_fnview(rtgui_view_t* parent_view)
{
	rtgui_view_t *view;
	rtgui_filelist_view_t *fview;

	/* ������ʾ�õ���ͼ */
	view = demo_view_create(parent_view, "FileList View");

#ifdef _WIN32
	fview = rtgui_filelist_view_create(view, "d:\\", "*.*", 5, 40, 200, 150);
#else
	fview = rtgui_filelist_view_create(view, "/", "*.*", 5, 40, 200, 150);
#endif

	fview->on_item = demo_fview_on_item;

	return view;
}
#endif
