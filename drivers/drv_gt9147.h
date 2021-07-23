#ifndef __DRV_GT9147_H__
#define __DRV_GT9147_H__

#include "rtdef.h"

#define GT9147_CURRTN_VERSION				0x61//current_version


#define GT9147_DEVICE_ADDR					0X5D

#define GT9147_CMD_ADDR						0X8040
#define GT9147_CFG_REG_ADDR					0X8047
#define GT9147_COORD_REG_ADDR				0X8140
#define GT9147_GESTURE_REG_ADDR				0X8140
#define GT9147_GESTURE_COORD_REG_ADDR		0X9420
#define GT9147_CMD_STATUS_REG_ADDR			0X81A8

struct gt9147_cfg_reg
{
	rt_uint8_t cfg_version;
	rt_uint8_t x_max_lsb;
	rt_uint8_t x_max_msb;
	rt_uint8_t y_max_lsb;
	rt_uint8_t y_max_msb;
	rt_uint8_t touch_numb;
	rt_uint8_t module_sw1;
	rt_uint8_t module_sw2;
	rt_uint8_t shake_count;
	rt_uint8_t filter;
	rt_uint8_t large_touch;
	rt_uint8_t noise_reduction;
	rt_uint8_t screen_touch_level;
	rt_uint8_t screen_leave_level;
	rt_uint8_t low_power_ctrl;
	rt_uint8_t refresh_rate;
	rt_uint8_t x_threshold;
	rt_uint8_t y_threshold;
	rt_uint8_t gesture_sw1;
	rt_uint8_t gesture_sw2;
	rt_uint8_t space_up_down;
	rt_uint8_t space_left_right;
	rt_uint8_t mini_filter;
	rt_uint8_t stretch_r0;
	rt_uint8_t stretch_r1;
	rt_uint8_t stretch_r2;
	rt_uint8_t stretch_rm;
	rt_uint8_t drv_group_a_numb;
	rt_uint8_t drv_group_b_numb;
	rt_uint8_t sensor_numb;
	rt_uint8_t freq_a_factor;
	rt_uint8_t freq_b_factor;
	rt_uint8_t pannel_bit_freq_lsb;
	rt_uint8_t pannel_bit_freq_msb;
	rt_uint8_t module_sw4;
	rt_uint8_t gesture_long_press_time;
	rt_uint8_t pannel_tx_gain;
	rt_uint8_t pannel_rx_gain;
	rt_uint8_t pannel_dump_shift;
	rt_uint8_t drv_frame_ctrl;
	rt_uint8_t s_feedback;
	rt_uint8_t module_sw3;
	rt_uint8_t screen_neg_thres;
	rt_uint8_t shape_ctrl;
	rt_uint8_t edge_complem_thres;
	rt_uint8_t edge_complement_x;
	rt_uint8_t edge_complement_y;
	rt_uint8_t water_frame_time;
	rt_uint8_t water_update_time;
	rt_uint8_t charging_level_up;
	rt_uint8_t gesture_ctrl;
	rt_uint8_t freq_hopping_start;
	rt_uint8_t freq_hopping_end;
	rt_uint8_t noise_detect_time;
	rt_uint8_t hopping_flag;
	rt_uint8_t hopping_threshold;
	rt_uint8_t hot_knot_noise_map;
	rt_uint8_t noise_min_threshold;
	rt_uint8_t reserved1;
	rt_uint8_t hopping_sensor_group;
	rt_uint8_t hopping_seg1_normalize;
	rt_uint8_t hopping_seg1_factor;
	rt_uint8_t main_clock_adjust;
	rt_uint8_t hopping_seg2_normalize;
	rt_uint8_t hopping_seg2_factor;
	rt_uint8_t reserved2;
	rt_uint8_t hopping_seg3_normalize;
	rt_uint8_t hopping_seg3_factor;
	rt_uint8_t reserved3;
	rt_uint8_t hopping_seg4_normalize;
	rt_uint8_t hopping_seg4_factor;
	rt_uint8_t reserved4;
	rt_uint8_t hopping_seg5_normalize;
	rt_uint8_t hopping_seg5_factor;
	rt_uint8_t reserved5;
	rt_uint8_t hopping_seg6_normalize;
	rt_uint8_t key1;
	rt_uint8_t key2;
	rt_uint8_t key3;
	rt_uint8_t key4;
	rt_uint8_t key_area;
	rt_uint8_t key_touch_level;
	rt_uint8_t key_leave_level;
	rt_uint8_t key1_2_sens;
	rt_uint8_t key3_4_sens;
	rt_uint8_t key_restrain;
	rt_uint8_t key_down_edge_filter;
	rt_uint8_t proximity_valid_time;
	rt_uint8_t proximity_press_time1;
	rt_uint8_t proximity_press_time2;
	rt_uint8_t proximity_large_touch;
	rt_uint8_t proximity_drv_select;
	rt_uint8_t proximity_sens_select;
	rt_uint8_t proximity_touch_level;
	rt_uint8_t proximity_leave_level;
	rt_uint8_t proximity_sample_add_time;
	rt_uint8_t proximity_sample_dec_lsb;
	rt_uint8_t proximity_sample_dec_msb;
	rt_uint8_t proximity_leave_shake_count;
	rt_uint8_t data_threshold;
	rt_uint8_t pxy_threshold;
	rt_uint8_t dump_shift;
	rt_uint8_t rx_gain;
	rt_uint8_t freq_gain0;
	rt_uint8_t freq_gain1;
	rt_uint8_t freq_gain2;
	rt_uint8_t freq_gain3;
	rt_uint8_t gesture_refresh_rate;
	rt_uint8_t combine_dis;
	rt_uint8_t split_set;
	rt_uint8_t gesture_touch_level;
	rt_uint8_t new_green_wake_up_level;
	rt_uint8_t sensor_ch[14];
	rt_uint8_t reserved6[16];
	rt_uint8_t driver_ch[26];
	rt_uint8_t reserved7[3];
	rt_uint8_t driver_gain_0_1;
	rt_uint8_t driver_gain_2_3;
	rt_uint8_t driver_gain_4_5;
	rt_uint8_t driver_gain_6_7;
	rt_uint8_t driver_gain_8_9;
	rt_uint8_t driver_gain_10_11;
	rt_uint8_t driver_gain_12_13;
	rt_uint8_t driver_gain_14_15;
	rt_uint8_t driver_gain_16_17;
	rt_uint8_t driver_gain_18_19;
	rt_uint8_t driver_gain_20_21;
	rt_uint8_t driver_gain_22_23;
	rt_uint8_t driver_gain_24_25;
	rt_uint8_t config_chksum;
	rt_uint8_t config_fresh;
};
typedef struct gt9147_cfg_reg *gt9147_cfg_reg_t;

struct gt9147_coord_reg
{
	rt_uint8_t product_id_1;
	rt_uint8_t product_id_2;
	rt_uint8_t product_id_3;
	rt_uint8_t product_id_4;
	rt_uint8_t firm_ver_lsb;
	rt_uint8_t firm_ver_msb;
	rt_uint8_t x_coord_resolution_lsb;
	rt_uint8_t x_coord_resolution_msb;
	rt_uint8_t y_coord_resolution_lsb;
	rt_uint8_t y_coord_resolution_msb;
	rt_uint8_t vendor_id;
	rt_uint8_t reserved0[3];
	rt_uint8_t status;
	rt_uint8_t track_id1;
	rt_uint8_t point1_x_coord_lsb;
	rt_uint8_t point1_x_coord_msb;
	rt_uint8_t point1_y_coord_lsb;
	rt_uint8_t point1_y_coord_msb;
	rt_uint8_t point1_size_lsb;
	rt_uint8_t point1_size_msb;
	rt_uint8_t reserved2;
	rt_uint8_t track_id2;
	rt_uint8_t point2_x_coord_lsb;
	rt_uint8_t point2_x_coord_msb;
	rt_uint8_t point2_y_coord_lsb;
	rt_uint8_t point2_y_coord_msb;
	rt_uint8_t point2_size_lsb;
	rt_uint8_t point2_size_msb;
	rt_uint8_t reserved3;
	rt_uint8_t track_id3;
	rt_uint8_t point3_x_coord_lsb;
	rt_uint8_t point3_x_coord_msb;
	rt_uint8_t point3_y_coord_lsb;
	rt_uint8_t point3_y_coord_msb;
	rt_uint8_t point3_size_lsb;
	rt_uint8_t point3_size_msb;
	rt_uint8_t reserved4;
	rt_uint8_t track_id4;
	rt_uint8_t point4_x_coord_lsb;
	rt_uint8_t point4_x_coord_msb;
	rt_uint8_t point4_y_coord_lsb;
	rt_uint8_t point4_y_coord_msb;
	rt_uint8_t point4_size_lsb;
	rt_uint8_t point4_size_msb;
	rt_uint8_t reserved5;
	rt_uint8_t track_id5;
	rt_uint8_t point5_x_coord_lsb;
	rt_uint8_t point5_x_coord_msb;
	rt_uint8_t point5_y_coord_lsb;
	rt_uint8_t point5_y_coord_msb;
	rt_uint8_t point5_size_lsb;
	rt_uint8_t point5_size_msb;
	rt_uint8_t reserved6[9];
	rt_uint8_t key_value;
};
typedef struct gt9147_coord_reg *gt9147_coord_reg_t;

struct gt9147_gesture_reg
{
	rt_uint8_t gesture_id1;
	rt_uint8_t gesture_id2;
	rt_uint8_t gesture_id3;
	rt_uint8_t gesture_id4;
	rt_uint8_t gesture_firm_ver_lsb;
	rt_uint8_t gesture_firm_ver_msb;
	rt_uint8_t x_coord_resolution_lsb;
	rt_uint8_t x_coord_resolution_msb;
	rt_uint8_t y_coord_resolution_lsb;
	rt_uint8_t y_coord_resolution_msb;
	rt_uint8_t reserved0;
	rt_uint8_t gesture_type;
	rt_uint8_t gesture_touch_numb;
	rt_uint8_t gesture_start_point_x_coord_lsb;
	rt_uint8_t gesture_start_point_x_coord_msb;
	rt_uint8_t gesture_start_point_y_coord_lsb;
	rt_uint8_t gesture_start_point_y_coord_msb;
	rt_uint8_t gesture_end_point_x_coord_lsb;
	rt_uint8_t gesture_end_point_x_coord_msb;
	rt_uint8_t gesture_end_point_y_coord_lsb;
	rt_uint8_t gesture_end_point_y_coord_msb;
	rt_uint8_t gesture_width_lsb;
	rt_uint8_t gesture_width_msb;
	rt_uint8_t gesture_height_lsb;
	rt_uint8_t gesture_height_msb;
	rt_uint8_t gesture_mid_x_coord_lsb;
	rt_uint8_t gesture_mid_x_coord_msb;
	rt_uint8_t gesture_mid_y_coord_lsb;
	rt_uint8_t gesture_mid_y_coord_msb;
	rt_uint8_t gesture_p1_x_coord_lsb;
	rt_uint8_t gesture_p1_x_coord_msb;
	rt_uint8_t gesture_p1_y_coord_lsb;
	rt_uint8_t gesture_p1_y_coord_msb;
	rt_uint8_t gesture_p2_x_coord_lsb;
	rt_uint8_t gesture_p2_x_coord_msb;
	rt_uint8_t gesture_p2_y_coord_lsb;
	rt_uint8_t gesture_p2_y_coord_msb;
	rt_uint8_t gesture_p3_x_coord_lsb;
	rt_uint8_t gesture_p3_x_coord_msb;
	rt_uint8_t gesture_p3_y_coord_lsb;
	rt_uint8_t gesture_p3_y_coord_msb;
	rt_uint8_t gesture_p4_x_coord_lsb;
	rt_uint8_t gesture_p4_x_coord_msb;
	rt_uint8_t gesture_p4_y_coord_lsb;
	rt_uint8_t gesture_p4_y_coord_msb;
};
typedef struct gt9147_gesture_reg *gt9147_gesture_reg_t;

struct gt9147_gesture_coord
{
	rt_uint8_t gesture_point1_x_coord_lsb;
	rt_uint8_t gesture_point1_x_coord_msb;
	rt_uint8_t gesture_point1_y_coord_lsb;
	rt_uint8_t gesture_point1_y_coord_msb;
};

struct gt9147_gesture_coord_reg
{
	struct gt9147_gesture_coord coord[64];
};
typedef struct gt9147_gesture_coord_reg *gt9147_gesture_coord_reg_t;

struct gt9147_cmd_status_reg
{
	rt_uint8_t status;
	rt_uint8_t status_bak;
};
struct gt9147_cmd_status_reg *gt9147_cmd_status_reg_t;

#endif
