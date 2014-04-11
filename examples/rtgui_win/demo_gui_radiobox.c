/*
 * �����嵥��radiobox�ؼ���ʾ
 *
 * ������ӻ��ڴ�������view�����������ͬ�����radiobox�ؼ�
 */

#include "demo_view.h"
#include <rtgui/widgets/radiobox.h>

static rt_uint32_t bind_var;

/* ����������ʾradiobox�ؼ�����ͼ */
rtgui_view_t* demo_gui_radiobox(rtgui_view_t* parent_view)
{
	rtgui_view_t* view;
	rtgui_radiobox_t *rbox;
	rtgui_rb_group_t *group,*_group;

	/* �ȴ���һ����ʾ�õ���ͼ */
	view = demo_view_create(parent_view, "RadioBox View");

	/* ʹ�÷���һ */
	/* �����ȴ���һ���������ʹ�� */
	group = rtgui_radiobox_create_group();
	rtgui_radiobox_create(view, "radio1", 5, 40, 100, 20, group);
	rtgui_radiobox_create(view, "radio2", 5, 60, 100, 20, group);
	
	/* ʹ�÷����� */
	rbox = rtgui_radiobox_create(view, "radio-x", 5, 90, 100, 20, RT_NULL);
	/* Ҳ���Դ�radiobox�ؼ��л��һ������� */
	group = rtgui_radiobox_get_group(rbox);

	_group = rtgui_radiobox_create_group();
	rtgui_radiobox_create(view, "radio_m", 20,110, 100, 20, _group);
	rtgui_radiobox_create(view, "radio_n", 20,130, 100, 20, _group); 
	/* �趨һ����ʼֵ */
	rtgui_rb_group_set_sel(_group, 1);
	rtgui_radiobox_create(view, "radio-y", 5, 150, 100, 20, group);

	/* ����Ϊ���һ������,֮�����ʹ�øñ������group�ĵ�ǰ���� */
	rtgui_rb_group_bind(group, &bind_var);

	return view;
}
