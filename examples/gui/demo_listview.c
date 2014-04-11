/*
 * �����嵥���б���ͼ��ʾ
 *
 * ������ӻ��ȴ�����һ����ʾ�õ�view�����������İ�ťʱ�ᰴ��ģʽ��ʾ����ʽ��ʾ
 * �µ��б���ͼ
 */
#include "demo_view.h"
#include <rtgui/widgets/label.h>
#include <rtgui/widgets/button.h>
#include <rtgui/widgets/window.h>
#include <rtgui/widgets/list_view.h>

static rtgui_workbench_t* workbench = RT_NULL;
static rtgui_list_view_t* _view = RT_NULL;
// static rtgui_image_t* return_image = RT_NULL;

/* �б���Ķ������� */
#if RT_VERSION == 4
static void listitem_action(rtgui_widget_t *widget, void* parameter)
#else
static void listitem_action(void* parameter)
#endif
{
	char label_text[32];
	rtgui_win_t *win;
	rtgui_label_t *label;
	rtgui_rect_t rect = {0, 0, 150, 80};
	int no = (int)parameter;

	rtgui_rect_moveto(&rect, 20, 50);

	/* ��ʾ��Ϣ���� */
	win = rtgui_win_create(RTGUI_TOPLEVEL(workbench),
		"����", &rect, RTGUI_WIN_STYLE_DEFAULT);

	rect.x1 += 20;
	rect.x2 -= 5;
	rect.y1 += 5;
	rect.y2 = rect.y1 + 20;

	/* �����Ӧ�ı�ǩ */
	rt_sprintf(label_text, "���� %d", no);
	label = rtgui_label_create(label_text);

	rtgui_widget_set_rect(RTGUI_WIDGET(label), &rect);
	rtgui_container_add_child(RTGUI_CONTAINER(win), RTGUI_WIDGET(label));

	/* ��ģ̬��ʾ���� */
	rtgui_win_show(win, RT_FALSE);
}

/* ���ع��ܵĶ������� */
#if RT_VERSION == 4
static void return_action(rtgui_widget_t* widget, void* parameter)
#else
static void return_action(void* parameter)
#endif
{
	rtgui_view_end_modal(RTGUI_VIEW(_view), RTGUI_MODAL_OK);
}

/* �����б���� */
static struct rtgui_list_item items[] =
{
	{"�б���1", RT_NULL, listitem_action, (void*)1},
	{"�б���2", RT_NULL, listitem_action, (void*)2},
	{"�б���3", RT_NULL, listitem_action, (void*)3},
	{"�б���4", RT_NULL, listitem_action, (void*)4},
	{"�б���5", RT_NULL, listitem_action, (void*)5},
	{"����",    RT_NULL, return_action,    RT_NULL},
};

/* ���б���ͼ�õİ�ť�������� */
static void open_btn_onbutton(rtgui_widget_t* widget, struct rtgui_event* event)
{
	rtgui_rect_t rect;

	/* ��ö����workbench */
	workbench = RTGUI_WORKBENCH(rtgui_widget_get_toplevel(widget));
	rtgui_widget_get_rect(RTGUI_WIDGET(workbench), &rect);

	/* ����һ���б���ͼ�� ��ָ��Ϊitems */
	_view = rtgui_list_view_create(items, sizeof(items)/sizeof(struct rtgui_list_item),
		&rect, RTGUI_LIST_VIEW_LIST);
	/* ��workbench�������Ӧ����ͼ */
	rtgui_workbench_add_view(workbench, RTGUI_VIEW(_view));

	/* ģʽ��ʾ��ͼ */
	rtgui_view_show(RTGUI_VIEW(_view), RT_TRUE);
	rtgui_view_destroy(RTGUI_VIEW(_view));
	_view = RT_NULL;
}

/* ����������ʾ�б���ͼ����ͼ */
rtgui_view_t* demo_listview_view(rtgui_workbench_t* workbench)
{
	rtgui_rect_t rect;
	rtgui_view_t *view;
	rtgui_button_t* open_btn;

	view = demo_view(workbench, "�б���ͼ��ʾ");

	/* ��Ӷ�����ť */
	demo_view_get_rect(view, &rect);
	rect.x1 += 5;
	rect.x2 = rect.x1 + 80;
	rect.y1 += 30;
	rect.y2 = rect.y1 + 20;
	open_btn = rtgui_button_create("���б�");
	rtgui_container_add_child(RTGUI_CONTAINER(view), RTGUI_WIDGET(open_btn));
	rtgui_widget_set_rect(RTGUI_WIDGET(open_btn), &rect);
	rtgui_button_set_onbutton(open_btn, open_btn_onbutton);

	return view;
}
