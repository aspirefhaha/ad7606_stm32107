/*
 * �����嵥��label�ؼ���ʾ
 *
 * ������ӻ��ڴ�������view����Ӽ�����ͬ���͵�label�ؼ�
 */
#include "demo_view.h"
#include <rtgui/widgets/label.h>

/* ����������ʾlabel�ؼ�����ͼ */
rtgui_view_t* demo_gui_label(rtgui_view_t* parent_view)
{
	rtgui_view_t* view;
	rtgui_label_t* label;
	rtgui_font_t* font;

	/* �ȴ���һ����ʾ�õ���ͼ */
	view = demo_view_create(parent_view, "Label View");

	/* ����һ��label�ؼ� */
	label = rtgui_label_create(view, "Red Left", 10, 40, 200, 20);
	/* ����label�ؼ��ϵ��ı����뷽ʽΪ������� */
	RTGUI_WIDGET_TEXTALIGN(label) = RTGUI_ALIGN_LEFT|RTGUI_ALIGN_CENTER_VERTICAL;
	/* ����label�ؼ���ǰ��ɫΪ��ɫ */
	RTGUI_WIDGET_FOREGROUND(label) = red;
	RTGUI_WIDGET_BACKGROUND(label) = white;
	

	label = rtgui_label_create(view, "Blue Right", 10, 65, 200, 20);
	RTGUI_WIDGET_TEXTALIGN(label) = RTGUI_ALIGN_RIGHT|RTGUI_ALIGN_CENTER_VERTICAL;
	RTGUI_WIDGET_FOREGROUND(label) = blue;
	RTGUI_WIDGET_BACKGROUND(label) = white;
	
	label = rtgui_label_create(view, "Green Center", 10, 90, 200, 20);
	RTGUI_WIDGET_TEXTALIGN(label) = RTGUI_ALIGN_CENTER_HORIZONTAL|RTGUI_ALIGN_CENTER_VERTICAL;
	RTGUI_WIDGET_FOREGROUND(label) = green;
	RTGUI_WIDGET_BACKGROUND(label) = white;
	
	label = rtgui_label_create(view, "12 font",10, 115, 200, 20);
	/* ��������Ϊ12�����asc���� */
	font = rtgui_font_refer("asc", 12);
	RTGUI_WIDGET_FONT(label) = font;
	RTGUI_WIDGET_BACKGROUND(label) = white;

	label = rtgui_label_create(view, "16 font", 10, 140, 200, 20);
	font = rtgui_font_refer("asc", 16);
	RTGUI_WIDGET_FONT(RTGUI_WIDGET(label)) = font;
	RTGUI_WIDGET_BACKGROUND(label) = white;

	return view;
}

