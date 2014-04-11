/*
 * �����嵥��checkbox�ؼ���ʾ
 *
 * ������ӻ��ڴ�������view����Ӽ���checkbox�ؼ�
 */

#include "demo_view.h"
#include <rtgui/widgets/checkbox.h>

/* ����������ʾcheckbox�ؼ�����ͼ */
rtgui_view_t* demo_gui_checkbox(rtgui_view_t* parent_view)
{
	rtgui_view_t* view;
	rtgui_checkbox_t* checkbox;
	rtgui_font_t* font;

	/* �ȴ���һ����ʾ�õ���ͼ */
	view = demo_view_create(parent_view, "CheckBox View");

	/* ����һ��checkbox�ؼ� */
	checkbox = rtgui_checkbox_create(view, "Red", RT_TRUE, 5, 40);
	/* ����ǰ��ɫΪ��ɫ */
	RTGUI_WIDGET_FOREGROUND(checkbox) = red;

	checkbox = rtgui_checkbox_create(view, "Blue", RT_TRUE, 5, 60);
	RTGUI_WIDGET_FOREGROUND(checkbox) = blue;

	checkbox = rtgui_checkbox_create(view, "12 font", RT_TRUE, 5, 80);
	font = rtgui_font_refer("asc", 12);
	RTGUI_WIDGET_FONT(checkbox) = font;

	checkbox = rtgui_checkbox_create(view, "16 font", RT_TRUE, 5, 100);
	font = rtgui_font_refer("asc", 16);
	RTGUI_WIDGET_FONT(checkbox) = font;

	return view;
}
