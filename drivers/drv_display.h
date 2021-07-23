#ifndef __DRV_DISPLAY_H__
#define __DRV_DISPLAY_H__

#include "rtdef.h"

#define TCON_BASE_ADDR 0x01C0C000

struct tina_tcon
{
	volatile rt_uint32_t ctrl;				  /* 0x00 */
	volatile rt_uint32_t int0;				  /* 0x04 */
	volatile rt_uint32_t int1;				  /* 0x08 */
	volatile rt_uint8_t res0[0x04];			  /* 0x0c */
	volatile rt_uint32_t tcon0_frm_ctrl;	  /* 0x10 */
	volatile rt_uint32_t tcon0_frm_seed[6];	  /* 0x14 */
	volatile rt_uint32_t tcon0_frm_table[4];  /* 0x2c */
	volatile rt_uint8_t res1[4];			  /* 0x3c */
	volatile rt_uint32_t tcon0_ctrl;		  /* 0x40 */
	volatile rt_uint32_t tcon0_dclk;		  /* 0x44 */
	volatile rt_uint32_t tcon0_timing_active; /* 0x48 */
	volatile rt_uint32_t tcon0_timing_h;	  /* 0x4c */
	volatile rt_uint32_t tcon0_timing_v;	  /* 0x50 */
	volatile rt_uint32_t tcon0_timing_sync;	  /* 0x54 */
	volatile rt_uint32_t tcon0_hv_intf;		  /* 0x58 */
	volatile rt_uint8_t res2[0x04];			  /* 0x5c */
	volatile rt_uint32_t tcon0_cpu_intf;	  /* 0x60 */
	volatile rt_uint32_t tcon0_cpu_wr_dat;	  /* 0x64 */
	volatile rt_uint32_t tcon0_cpu_rd_dat0;	  /* 0x68 */
	volatile rt_uint32_t tcon0_cpu_rd_dat1;	  /* 0x6c */
	volatile rt_uint32_t tcon0_ttl_timing0;	  /* 0x70 */
	volatile rt_uint32_t tcon0_ttl_timing1;	  /* 0x74 */
	volatile rt_uint32_t tcon0_ttl_timing2;	  /* 0x78 */
	volatile rt_uint32_t tcon0_ttl_timing3;	  /* 0x7c */
	volatile rt_uint32_t tcon0_ttl_timing4;	  /* 0x80 */
	volatile rt_uint32_t tcon0_lvds_intf;	  /* 0x84 */
	volatile rt_uint32_t tcon0_io_polarity;	  /* 0x88 */
	volatile rt_uint32_t tcon0_io_tristate;	  /* 0x8c */
	volatile rt_uint32_t tcon1_ctrl;		  /* 0x90 */
	volatile rt_uint32_t tcon1_timing_source; /* 0x94 */
	volatile rt_uint32_t tcon1_timing_scale;  /* 0x98 */
	volatile rt_uint32_t tcon1_timing_out;	  /* 0x9c */
	volatile rt_uint32_t tcon1_timing_h;	  /* 0xa0 */
	volatile rt_uint32_t tcon1_timing_v;	  /* 0xa4 */
	volatile rt_uint32_t tcon1_timing_sync;	  /* 0xa8 */
	volatile rt_uint8_t res3[0x44];			  /* 0xac */
	volatile rt_uint32_t tcon1_io_polarity;	  /* 0xf0 */
	volatile rt_uint32_t tcon1_io_tristate;	  /* 0xf4 */
	volatile rt_uint8_t res4[0x108];		  /* 0xf8 */
	volatile rt_uint32_t mux_ctrl;			  /* 0x200 */
	volatile rt_uint8_t res5[0x1c];			  /* 0x204 */
	volatile rt_uint32_t lvds_ana0;			  /* 0x220 */
	volatile rt_uint32_t lvds_ana1;			  /* 0x224 */
};

typedef struct tina_tcon *tina_tcon_t;
#define TCON ((tina_tcon_t)TCON_BASE_ADDR)

#define DEFE_BASE_ADDR 0x01E00000

struct tina_defe
{
	rt_uint32_t enable;					 /* 0x000 */
	rt_uint32_t frame_ctrl;				 /* 0x004 */
	rt_uint32_t bypass;					 /* 0x008 */
	rt_uint32_t algorithm_sel;			 /* 0x00c */
	rt_uint32_t line_int_ctrl;			 /* 0x010 */
	rt_uint8_t res0[0x0c];				 /* 0x014 */
	rt_uint32_t ch0_addr;				 /* 0x020 */
	rt_uint32_t ch1_addr;				 /* 0x024 */
	rt_uint32_t ch2_addr;				 /* 0x028 */
	rt_uint32_t field_sequence;			 /* 0x02c */
	rt_uint32_t ch0_offset;				 /* 0x030 */
	rt_uint32_t ch1_offset;				 /* 0x034 */
	rt_uint32_t ch2_offset;				 /* 0x038 */
	rt_uint8_t res1[0x04];				 /* 0x03c */
	rt_uint32_t ch0_stride;				 /* 0x040 */
	rt_uint32_t ch1_stride;				 /* 0x044 */
	rt_uint32_t ch2_stride;				 /* 0x048 */
	rt_uint32_t input_fmt;				 /* 0x04c */
	rt_uint32_t ch3_addr;				 /* 0x050 */
	rt_uint32_t ch4_addr;				 /* 0x054 */
	rt_uint32_t ch5_addr;				 /* 0x058 */
	rt_uint32_t output_fmt;				 /* 0x05c */
	rt_uint32_t int_enable;				 /* 0x060 */
	rt_uint32_t int_status;				 /* 0x064 */
	rt_uint32_t status;					 /* 0x068 */
	rt_uint8_t res2[0x04];				 /* 0x06c */
	rt_uint32_t csc_coef00;				 /* 0x070 */
	rt_uint32_t csc_coef01;				 /* 0x074 */
	rt_uint32_t csc_coef02;				 /* 0x078 */
	rt_uint32_t csc_coef03;				 /* 0x07c */
	rt_uint32_t csc_coef10;				 /* 0x080 */
	rt_uint32_t csc_coef11;				 /* 0x084 */
	rt_uint32_t csc_coef12;				 /* 0x088 */
	rt_uint32_t csc_coef13;				 /* 0x08c */
	rt_uint32_t csc_coef20;				 /* 0x090 */
	rt_uint32_t csc_coef21;				 /* 0x094 */
	rt_uint32_t csc_coef22;				 /* 0x098 */
	rt_uint32_t csc_coef23;				 /* 0x09c */
	rt_uint32_t deinterlace_ctrl;		 /* 0x0a0 */
	rt_uint32_t deinterlace_diag;		 /* 0x0a4 */
	rt_uint32_t deinterlace_tempdiff;	 /* 0x0a8 */
	rt_uint32_t deinterlace_sawtooth;	 /* 0x0ac */
	rt_uint32_t deinterlace_spatcomp;	 /* 0x0b0 */
	rt_uint32_t deinterlace_burstlen;	 /* 0x0b4 */
	rt_uint32_t deinterlace_preluma;	 /* 0x0b8 */
	rt_uint32_t deinterlace_tile_addr;	 /* 0x0bc */
	rt_uint32_t deinterlace_tile_stride; /* 0x0c0 */
	rt_uint8_t res3[0x0c];				 /* 0x0c4 */
	rt_uint32_t wb_stride_enable;		 /* 0x0d0 */
	rt_uint32_t ch3_stride;				 /* 0x0d4 */
	rt_uint32_t ch4_stride;				 /* 0x0d8 */
	rt_uint32_t ch5_stride;				 /* 0x0dc */
	rt_uint32_t fe_3d_ctrl;				 /* 0x0e0 */
	rt_uint32_t fe_3d_ch0_addr;			 /* 0x0e4 */
	rt_uint32_t fe_3d_ch1_addr;			 /* 0x0e8 */
	rt_uint32_t fe_3d_ch2_addr;			 /* 0x0ec */
	rt_uint32_t fe_3d_ch0_offset;		 /* 0x0f0 */
	rt_uint32_t fe_3d_ch1_offset;		 /* 0x0f4 */
	rt_uint32_t fe_3d_ch2_offset;		 /* 0x0f8 */
	rt_uint8_t res4[0x04];				 /* 0x0fc */
	rt_uint32_t ch0_insize;				 /* 0x100 */
	rt_uint32_t ch0_outsize;			 /* 0x104 */
	rt_uint32_t ch0_horzfact;			 /* 0x108 */
	rt_uint32_t ch0_vertfact;			 /* 0x10c */
	rt_uint32_t ch0_horzphase;			 /* 0x110 */
	rt_uint32_t ch0_vertphase0;			 /* 0x114 */
	rt_uint32_t ch0_vertphase1;			 /* 0x118 */
	rt_uint8_t res5[0x04];				 /* 0x11c */
	rt_uint32_t ch0_horztapoffset0;		 /* 0x120 */
	rt_uint32_t ch0_horztapoffset1;		 /* 0x124 */
	rt_uint32_t ch0_verttapoffset;		 /* 0x128 */
	rt_uint8_t res6[0xd4];				 /* 0x12c */
	rt_uint32_t ch1_insize;				 /* 0x200 */
	rt_uint32_t ch1_outsize;			 /* 0x204 */
	rt_uint32_t ch1_horzfact;			 /* 0x208 */
	rt_uint32_t ch1_vertfact;			 /* 0x20c */
	rt_uint32_t ch1_horzphase;			 /* 0x210 */
	rt_uint32_t ch1_vertphase0;			 /* 0x214 */
	rt_uint32_t ch1_vertphase1;			 /* 0x218 */
	rt_uint8_t res7[0x04];				 /* 0x21c */
	rt_uint32_t ch1_horztapoffset0;		 /* 0x220 */
	rt_uint32_t ch1_horztapoffset1;		 /* 0x224 */
	rt_uint32_t ch1_verttapoffset;		 /* 0x228 */
	rt_uint8_t res8[0x1d4];				 /* 0x22c */
	rt_uint32_t ch0_horzcoef0[32];		 /* 0x400 */
	rt_uint32_t ch0_horzcoef1[32];		 /* 0x480 */
	rt_uint32_t ch0_vertcoef[32];		 /* 0x500 */
	rt_uint8_t res9[0x80];				 /* 0x580 */
	rt_uint32_t ch1_horzcoef0[32];		 /* 0x600 */
	rt_uint32_t ch1_horzcoef1[32];		 /* 0x680 */
	rt_uint32_t ch1_vertcoef[32];		 /* 0x700 */
	rt_uint8_t res10[0x280];			 /* 0x780 */
	rt_uint32_t vpp_enable;				 /* 0xa00 */
	rt_uint32_t vpp_dcti;				 /* 0xa04 */
	rt_uint32_t vpp_lp1;				 /* 0xa08 */
	rt_uint32_t vpp_lp2;				 /* 0xa0c */
	rt_uint32_t vpp_wle;				 /* 0xa10 */
	rt_uint32_t vpp_ble;				 /* 0xa14 */
};

typedef struct tina_defe *tina_defe_t;
#define DEFE ((tina_defe_t)DEFE_BASE_ADDR)

#define DEBE_BASE_ADDR 0x01E60000

struct tina_debe
{
	volatile rt_uint8_t res0[0x800];			/* 0x000 */
	volatile rt_uint32_t mode;					/* 0x800 */
	volatile rt_uint32_t backcolor;				/* 0x804 */
	volatile rt_uint32_t disp_size;				/* 0x808 */
	volatile rt_uint8_t res1[0x4];				/* 0x80c */
	volatile rt_uint32_t layer0_size;			/* 0x810 */
	volatile rt_uint32_t layer1_size;			/* 0x814 */
	volatile rt_uint32_t layer2_size;			/* 0x818 */
	volatile rt_uint32_t layer3_size;			/* 0x81c */
	volatile rt_uint32_t layer0_pos;			/* 0x820 */
	volatile rt_uint32_t layer1_pos;			/* 0x824 */
	volatile rt_uint32_t layer2_pos;			/* 0x828 */
	volatile rt_uint32_t layer3_pos;			/* 0x82c */
	volatile rt_uint8_t res2[0x10];				/* 0x830 */
	volatile rt_uint32_t layer0_stride;			/* 0x840 */
	volatile rt_uint32_t layer1_stride;			/* 0x844 */
	volatile rt_uint32_t layer2_stride;			/* 0x848 */
	volatile rt_uint32_t layer3_stride;			/* 0x84c */
	volatile rt_uint32_t layer0_addr_low32b;	/* 0x850 */
	volatile rt_uint32_t layer1_addr_low32b;	/* 0x854 */
	volatile rt_uint32_t layer2_addr_low32b;	/* 0x858 */
	volatile rt_uint32_t layer3_addr_low32b;	/* 0x85c */
	volatile rt_uint32_t layer0_addr_high4b;	/* 0x860 */
	volatile rt_uint32_t layer1_addr_high4b;	/* 0x864 */
	volatile rt_uint32_t layer2_addr_high4b;	/* 0x868 */
	volatile rt_uint32_t layer3_addr_high4b;	/* 0x86c */
	volatile rt_uint32_t reg_ctrl;				/* 0x870 */
	volatile rt_uint8_t res3[0xc];				/* 0x874 */
	volatile rt_uint32_t color_key_max;			/* 0x880 */
	volatile rt_uint32_t color_key_min;			/* 0x884 */
	volatile rt_uint32_t color_key_config;		/* 0x888 */
	volatile rt_uint8_t res4[0x4];				/* 0x88c */
	volatile rt_uint32_t layer0_attr0_ctrl;		/* 0x890 */
	volatile rt_uint32_t layer1_attr0_ctrl;		/* 0x894 */
	volatile rt_uint32_t layer2_attr0_ctrl;		/* 0x898 */
	volatile rt_uint32_t layer3_attr0_ctrl;		/* 0x89c */
	volatile rt_uint32_t layer0_attr1_ctrl;		/* 0x8a0 */
	volatile rt_uint32_t layer1_attr1_ctrl;		/* 0x8a4 */
	volatile rt_uint32_t layer2_attr1_ctrl;		/* 0x8a8 */
	volatile rt_uint32_t layer3_attr1_ctrl;		/* 0x8ac */
	volatile rt_uint8_t res5[0x110];			/* 0x8b0 */
	volatile rt_uint32_t output_color_ctrl;		/* 0x9c0 */
	volatile rt_uint8_t res6[0xc];				/* 0x9c4 */
	volatile rt_uint32_t output_color_coef[12]; /* 0x9d0 */
};

typedef struct tina_debe *tina_debe_t;
#define DEBE ((tina_debe_t)DEBE_BASE_ADDR)

/**
 * @brief LCD bits per pixel
 */
enum tcon_depth
{
	TCON_16_DEPTH = 0,
	TCON_18_DEPTH,
};

/**
 * @brief DEFE module channel parameters
 */
struct display_defe_ch_param
{
	rt_uint16_t in_width;	//The width of the input image
	rt_uint16_t in_height;	//The height of the input image
	rt_uint16_t out_width;	//The width of the output image
	rt_uint16_t out_height; //The height of the output image

	rt_uint32_t *framebuffer; //The address of the framebuffer
};

/**
 * @brief DEBE module layer parameters
 */
struct display_debe_layer_param
{
	rt_uint8_t layer_index; //4 moveable and size-adjustable layer

	rt_bool_t enable; //Whether the layer is enabled or not

	rt_uint16_t width;	//The width of the layer
	rt_uint16_t height; //The height of the layer
	rt_uint16_t x;		//The x coordinate of the layer
	rt_uint16_t y;		//The y coordinate of the layer

	rt_uint32_t *framebuffer; //The address of the framebuffer

	rt_uint8_t priority; //The priority of the layer,a higher value indicates a higher priority
};

/**
 * @brief Output port polarity control
 */
struct display_io_pol
{
	rt_uint32_t data_pol : 24;
	rt_uint32_t clk_pol : 1;
	rt_uint32_t de_pol : 1;
	rt_uint32_t hsync : 1;
	rt_uint32_t vsync : 1;
};

/**
 * @brief Display parameters
 */
struct display_param
{
	rt_uint32_t width; //The width of the display
	rt_uint32_t hspw;  //Horizontal sync pulse width
	rt_uint32_t hbp;   //Horizontal back porch
	rt_uint32_t hfp;   //Horizontal front porch

	rt_uint32_t height; //The height of the display
	rt_uint32_t vspw;	//Vertical sync pulse width
	rt_uint32_t vbp;	//Vertical back porch
	rt_uint32_t vfp;	//Vertical front porch

	rt_uint32_t clk;	   //LCD clock frequency
	enum tcon_depth depth; //Bits per pixel
	rt_bool_t auto_reload; //Whether auto reload mode is enabled or not

	struct display_io_pol io_pol;
};

void drv_lcd_cfg_pinmux(enum tcon_depth depth);
rt_err_t drv_lcd_set_clk(int clk);
rt_err_t drv_debe_set_clk(int clk);
rt_err_t drv_defe_set_clk(int clk);
void drv_tcon0_init(const struct display_param *param);
void drv_lcd_reset(void);
void drv_defe_init(void);
void drv_defe_reset(void);
rt_err_t drv_defe_set_ch_param(struct display_defe_ch_param *param);
void drv_debe_init(const struct display_param *param);
void drv_debe_reset(void);
void drv_debe_reload_buffer(void);
rt_err_t drv_debe_set_layer_param(struct display_debe_layer_param *param);
rt_err_t drv_lcd_init(struct display_param *lcd_param, struct display_debe_layer_param *debe_param);

#endif // ! __DRV_LCD_H
