/*
 * �����嵥��texbox�ؼ���ʾ
 *
 * ������ӻ��ڴ�������view����Ӽ�����ͬ���͵�textbox�ؼ�
 */
#include "demo_view.h"
#include <rtgui/widgets/label.h>
#include <rtgui/widgets/textbox.h>

/* ����������ʾtextbox�ؼ�����ͼ */
rtgui_view_t* demo_gui_textbox(rtgui_view_t* parent_view)
{
	rtgui_view_t* view;
	
	/* �ȴ���һ����ʾ�õ���ͼ */
	view = demo_view_create(parent_view, "TextBox View");

	rtgui_label_create(view, "����: ", 5, 40, 50, 20);
	rtgui_textbox_create(view, "bernard",5, 60, 200, 20, RTGUI_TEXTBOX_NONE);

	rtgui_label_create(view, "�ʼ�: ", 5, 80, 50, 20);
	rtgui_textbox_create(view, "bernard.xiong@gmail.com", 5, 100, 200, 20, RTGUI_TEXTBOX_NONE);

	rtgui_label_create(view, "����: ", 5, 120, 50, 20);
	rtgui_textbox_create(view, "rt-thread", 5, 140, 200, 20, RTGUI_TEXTBOX_MASK);

	rtgui_label_create(view, "��ҳ: ", 5, 160, 50, 20);
	rtgui_textbox_create(view, "http://www.rt-thread.org", 5, 180, 200, 20, RTGUI_TEXTBOX_NONE);

	return view;
}
