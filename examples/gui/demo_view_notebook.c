/*
 * 程序清单：notebook控件演示
 *
 * 这个例子会在创建出的view上演示notebook控件
 */

#include "demo_view.h"
#include <rtgui/widgets/notebook.h>
#include <rtgui/widgets/listbox.h>

const static struct rtgui_listbox_item items[] = 
{
	{"list #0", RT_NULL},
	{"list #1", RT_NULL},
	{"list #2", RT_NULL},
	{"list #3", RT_NULL},
};

const static struct rtgui_listbox_item items2[] = 
{
	{"list #0", RT_NULL},
	{"list #1", RT_NULL},
	{"list #2", RT_NULL},
	{"new list #1", RT_NULL},
	{"new list #2", RT_NULL},
};

/* 创建用于演示notebook控件的视图 */
rtgui_view_t* demo_view_notebook(rtgui_workbench_t* workbench)
{
	rtgui_rect_t rect;
	rtgui_view_t* view;
	rtgui_notebook_t* notebook;
	rtgui_listbox_t* box;

	/* 先创建一个演示用的视图 */
	view = demo_view(workbench, "Notebook View");

	/* 获得视图的位置信息 */
	demo_view_get_rect(view, &rect);

	notebook = rtgui_notebook_create(&rect, RTGUI_NOTEBOOK_BOTTOM);
	/* view是一个container控件，调用add_child方法添加这个notebook控件 */
	rtgui_container_add_child(RTGUI_CONTAINER(view), RTGUI_WIDGET(notebook));

	box = rtgui_listbox_create(items, sizeof(items)/sizeof(struct rtgui_listbox_item), &rect);
	rtgui_notebook_add(notebook, "Tab 1", RTGUI_WIDGET(box));

	box = rtgui_listbox_create(items2, sizeof(items2)/sizeof(struct rtgui_listbox_item), &rect);
	rtgui_notebook_add(notebook, "Tab 2", RTGUI_WIDGET(box));

	return view;
}
