/**
 * @file thread_interface.h
 * @brief Thread function declarations and interfaces
 */

#ifndef THREAD_INTERFACE_H_
#define THREAD_INTERFACE_H_

void sys_mgr_thread(void *p1, void *p2, void *p3);
void mavlink_thread(void *p1, void *p2, void *p3);
void health_thread(void *p1, void *p2, void *p3);

#endif /* THREAD_INTERFACE_H_ */