/*
 * main.h
 *
 *  Created on: 2020年5月8日
 *      Author: root
 */

#ifndef MAIN_H_
#define MAIN_H_

static void set_latency_target(void);
static int check_clock_resolution(void);
static inline int64_t calcdiff_ns(struct timespec t1, struct timespec t2);
static inline void check_sleep_resolution(void);

#endif /* MAIN_H_ */
