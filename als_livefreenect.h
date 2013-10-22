/*
 * als_livefreenect.h
 *
 *  Created on: Aug 21, 2013
 *      Author: Al
 */



#ifndef ALS_LIVEFREENECT_H_
#define ALS_LIVEFREENECT_H_

// 3.1 (fun_kdep) - freenect_sync_get_depth_cv
IplImage *freenect_sync_get_depth_cv(int index);
// 3.2 (fun_krgb) - freenect_sync_get_rgb_cv
IplImage *freenect_sync_get_rgb_cv(int index);
//  3.3 (fun_glcolview) - GlViewColor
IplImage *GlViewColor(IplImage *depth);
// 3.4 (fun_kled_r) - freenect_sync_set_led_red_cv
void freenect_sync_set_led_red_cv(int index);
// 3.5 (fun_kled_g) - freenect_sync_set_led_green_
void freenect_sync_set_led_green_cv(int index);
// 3.6 (fun_kled_bg)- freenect_sync_set_led_blink_green_cv
void freenect_sync_set_led_blink_green_cv(int index);
// 3.7 (fun_kled_y) - freenect_sync_set_led_yellow_cv
void freenect_sync_set_led_yellow_cv(int index);
// 3.8 (fun_kled_off)- freenect_sync_set_led_off_cv
void freenect_sync_set_led_off_cv(int index);
// 3.9 (fun_kstop) - freenect_sync_stop_cv
void freenect_sync_stop_cv (void);

#endif /* ALS_LIVEFREENECT_H_ */
