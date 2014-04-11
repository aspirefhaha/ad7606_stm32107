/*
 * �����嵥��button�ؼ���ʾ
 *
 * ������ӻ��ڴ�������view����Ӽ�����ͬ���͵�button�ؼ�
 */

#include "demo_view.h"
#include <rtgui/widgets/button.h>

/* ����������ʾbutton�ؼ�����ͼ */
rtgui_view_t* demo_gui_button(rtgui_view_t* parent_view)
{
	rtgui_view_t* view;
	rtgui_button_t* button;
	rtgui_font_t* font;

	/* �ȴ���һ����ʾ�õ���ͼ */
	view = demo_view_create(parent_view, "Button View");

	/* ����һ��button�ؼ� */
	button = rtgui_button_create(view, "Red", 5, 40, 100, 25);
	/* ����label�ؼ���ǰ��ɫΪ��ɫ */
	RTGUI_WIDGET_FOREGROUND(button) = red;

	button = rtgui_button_create(view, "Blue", 5, 70, 100, 25);
	RTGUI_WIDGET_FOREGROUND(button) = blue;
	
	button = rtgui_button_create(view, "12 font", 5, 100, 100, 25);
	/* ��������Ϊ12�����asc���� */
	font = rtgui_font_refer("asc", 12);
	RTGUI_WIDGET_FONT(button) = font;
	
	button = rtgui_button_create(view, "16 font", 5, 130, 100, 25);
	/* ��������Ϊ16�����asc���� */
	font = rtgui_font_refer("asc", 16);
	RTGUI_WIDGET_FONT(button) = font;


	return view;
}
