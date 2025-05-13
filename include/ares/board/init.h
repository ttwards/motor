// init.h
#ifndef INIT_H
#define INIT_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/led.h>

struct led_rgb;

void led_set_rgb(struct led_rgb *color);
void led_serivce_func(void *p1, void *p2, void *p3);
void board_init(void);

#endif /* INIT_H */