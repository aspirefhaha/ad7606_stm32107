/*
 * A simple workbench
 */
#include <rtthread.h>
#include <rtgui/rtgui_server.h>
#include <rtgui/rtgui_system.h>
#include <rtgui/widgets/label.h>
#include <rtgui/widgets/workbench.h>

static void workbench_entry(void* parameter)
{
	rt_mq_t mq;
	rtgui_view_t* view;
	rtgui_label_t* label;
	struct rtgui_workbench* workbench;
	rtgui_rect_t rect;

	mq = rt_mq_create("wmq", 256, 8, RT_IPC_FLAG_FIFO);
	/* ע�ᵱǰ�߳�ΪGUI�߳� */
	rtgui_thread_register(rt_thread_self(), mq);
	/* ����һ������̨ */
	workbench = rtgui_workbench_create("main", "workbench #1");
	if (workbench == RT_NULL) return;

	view = rtgui_view_create("view");
	if (view == RT_NULL) return;
	/* ָ����ͼ�ı���ɫ */
	RTGUI_WIDGET_BACKGROUND(RTGUI_WIDGET(view)) = blue;

	/* ���һ��label */
	label = rtgui_label_create("Hello��RT-Thread��");
	rect.x1 = 50; rect.y1 = 50;
	rect.x2 = 210; rect.y2 = 50;
	/* ����label��λ�� */
	rtgui_widget_set_rect(RTGUI_WIDGET(label), &rect);
	rtgui_container_add_child(RTGUI_CONTAINER(view), RTGUI_WIDGET(label));

	/* ��ӵ���workbench�� */
	rtgui_workbench_add_view(workbench, view);
	/* ��ģʽ��ʽ��ʾ��ͼ */
	rtgui_view_show(view, RT_FALSE);

	/* ִ�й���̨�¼�ѭ�� */
	rtgui_workbench_event_loop(workbench);

	/* ȥע��GUI�߳� */
	rtgui_thread_deregister(rt_thread_self());

	/* delete message queue */
	rt_mq_delete(mq);
}

/* ��ʼ��workbench */
void wb_init()
{
	rt_thread_t tid;

	tid = rt_thread_create("wb1", workbench_entry, RT_NULL, 2048, 20, 5);
	if (tid != RT_NULL) rt_thread_startup(tid);
}
