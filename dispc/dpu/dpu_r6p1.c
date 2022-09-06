// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Unisoc Inc.
 */

#include <drm/drm_vblank.h>
#include <linux/backlight.h>
#include <linux/dma-buf.h>
#include <linux/delay.h>
#include <linux/of_address.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/dma-mapping.h>
#include <linux/trusty/smcall.h>
#include <asm/cacheflush.h>
#include "sprd_bl.h"
#include "sprd_dpu.h"
#include "dpu_enhance_param.h"
#include "sprd_dsi.h"
#include "sprd_crtc.h"
#include "sprd_plane.h"
#include "sprd_dsi_panel.h"
#include <../drivers/trusty/trusty.h>

#define XFBC8888_HEADER_SIZE(w, h) (ALIGN((ALIGN((w), 16)) * \
				(ALIGN((h), 16)) / 16, 128))
#define XFBC8888_PAYLOAD_SIZE(w, h) (ALIGN((w), 16) * ALIGN((h), 16) * 4)
#define XFBC8888_BUFFER_SIZE(w, h) (XFBC8888_HEADER_SIZE(w, h) \
				+ XFBC8888_PAYLOAD_SIZE(w, h))

#define SLP_BRIGHTNESS_THRESHOLD 0x20

/* DPU registers size, 4 Bytes(32 Bits) */
#define DPU_REG_SIZE					0x04
/* Layer registers offset */
#define DPU_LAY_REG_OFFSET				0x10

#define DPU_MAX_REG_OFFSET				0x19AC

#define DSC_REG_OFFSET					0x1A00
#define DSC1_REG_OFFSET					0x1B00

#define DPU_REG_RD(reg) readl_relaxed(reg)

#define DPU_REG_WR(reg, mask) writel_relaxed(mask, reg)

#define DPU_REG_SET(reg, mask) \
		writel_relaxed(readl_relaxed(reg) | mask, reg)

#define DPU_REG_CLR(reg, mask) \
		writel_relaxed(readl_relaxed(reg) & ~mask, reg)

#define DPU_LAY_REG(reg, index) \
		(reg + index * DPU_LAY_REG_OFFSET * DPU_REG_SIZE)

#define DPU_LAY_PLANE_ADDR(reg, index, plane) \
		(reg + index * DPU_LAY_REG_OFFSET * DPU_REG_SIZE + plane * DPU_REG_SIZE)

#define DSC_REG(reg) (reg + DSC_REG_OFFSET)

#define DSC1_REG(reg) (reg + DSC1_REG_OFFSET)

/* DSC_PicW_PicH_SliceW_SliceH  */
#define DSC_1440_2560_720_2560	0
#define DSC_1080_2408_540_8	1
#define DSC_720_2560_720_8	2
#define DSC_1080_2400_540_2400	3

/*Global control registers */
#define REG_DPU_CTRL					0x08
#define REG_DPU_CFG0					0x0C
#define REG_DPU_CFG1					0x10
#define REG_PANEL_SIZE					0x18
#define REG_BLEND_SIZE					0x1C
#define REG_SCL_EN						0x20
#define REG_BG_COLOR					0x24

/* Layer enable */
#define REG_LAYER_ENABLE				0x2c

/* Layer0 control registers */
#define REG_LAY_BASE_ADDR				0x30
#define REG_LAY_CTRL					0x40
#define REG_LAY_DES_SIZE				0x44
#define REG_LAY_SRC_SIZE				0x48
#define REG_LAY_PITCH					0x4C
#define REG_LAY_POS						0x50
#define REG_LAY_ALPHA					0x54
#define REG_LAY_CK						0x58
#define REG_LAY_PALLETE					0x5C
#define REG_LAY_CROP_START				0x60

/* Write back config registers */
#define REG_WB_BASE_ADDR				0x230
#define REG_WB_CTRL						0x234
#define REG_WB_CFG						0x238
#define REG_WB_PITCH					0x23C

/* Interrupt control registers */
#define REG_DPU_INT_EN					0x250
#define REG_DPU_INT_CLR					0x254
#define REG_DPU_INT_STS					0x258
#define REG_DPU_INT_RAW					0x25C

/* DPI control registers */
#define REG_DPI_CTRL					0x260
#define REG_DPI_H_TIMING				0x264
#define REG_DPI_V_TIMING				0x268
#define REG_DPI_VFP						0x26C
#define REG_DPI_S_VFP					0x270

/* DPU STS */
#define REG_DPU_STS_21					0x754
#define REG_DPU_STS_22					0x758

#define REG_DPU_MMU0_UPDATE				0x1808
#define REG_DPU_MODE					0x04

/* DPU SCL */
#define REG_DPU_SCL_EN					0x20

/* DSC REG */
#define REG_DSC_CTRL					0x00
#define REG_DSC_PIC_SIZE				0x04
#define REG_DSC_GRP_SIZE				0x08
#define REG_DSC_SLICE_SIZE				0x0c
#define REG_DSC_H_TIMING				0x10
#define REG_DSC_V_TIMING				0x14
#define REG_DSC_CFG0					0x18
#define REG_DSC_CFG1					0x1c
#define REG_DSC_CFG2					0x20
#define REG_DSC_CFG3					0x24
#define REG_DSC_CFG4					0x28
#define REG_DSC_CFG5					0x2c
#define REG_DSC_CFG6					0x30
#define REG_DSC_CFG7					0x34
#define REG_DSC_CFG8					0x38
#define REG_DSC_CFG9					0x3c
#define REG_DSC_CFG10					0x40
#define REG_DSC_CFG11					0x44
#define REG_DSC_CFG12					0x48
#define REG_DSC_CFG13					0x4c
#define REG_DSC_CFG14					0x50
#define REG_DSC_CFG15					0x54
#define REG_DSC_CFG16					0x58
#define REG_DSC_STS0					0x5c
#define REG_DSC_STS1					0x60
#define REG_DSC_VERSION					0x64

/* PQ Enhance config registers */
#define REG_DPU_ENHANCE_CFG				0x500
#define REG_ENHANCE_UPDATE				0x504
#define REG_SLP_LUT_BASE_ADDR				0x510
#define REG_THREED_LUT_BASE_ADDR			0x514
#define REG_HSV_LUT_BASE_ADDR				0x518
#define REG_GAMMA_LUT_BASE_ADDR				0x51C
#define REG_EPF_EPSILON					0x520
#define REG_EPF_GAIN0_3					0x524
#define REG_EPF_GAIN4_7					0x528
#define REG_EPF_DIFF					0x52C
#define REG_CM_COEF01_00				0x530
#define REG_CM_COEF03_02				0x534
#define REG_CM_COEF11_10				0x538
#define REG_CM_COEF13_12				0x53C
#define REG_CM_COEF21_20				0x540
#define REG_CM_COEF23_22				0x544
#define REG_SLP_CFG0					0x550
#define REG_SLP_CFG1					0x554
#define REG_SLP_CFG2					0x558
#define REG_SLP_CFG3					0x55C
#define REG_SLP_CFG4					0x560
#define REG_SLP_CFG5					0x564
#define REG_SLP_CFG6					0x568
#define REG_SLP_CFG7					0x56C
#define REG_SLP_CFG8					0x570
#define REG_SLP_CFG9					0x574
#define REG_SLP_CFG10					0x578
#define REG_HSV_CFG					0x580
#define REG_CABC_CFG0					0x590
#define REG_CABC_CFG1					0x594
#define REG_CABC_CFG2					0x598
#define REG_CABC_CFG3					0x59C
#define REG_CABC_CFG4					0x5A0
#define REG_CABC_CFG5					0x5A4
#define REG_UD_CFG0					0x5B0
#define REG_UD_CFG1					0x5B4
#define REG_CABC_HIST0					0x600
#define REG_GAMMA_LUT_ADDR				0x780
#define REG_GAMMA_LUT_RDATA				0x784
#define REG_SLP_LUT_ADDR				0x798
#define REG_SLP_LUT_RDATA				0x79C
#define REG_HSV_LUT0_ADDR				0x7A0
#define REG_HSV_LUT0_RDATA				0x7A4
#define REG_THREED_LUT0_ADDR				0x7C0
#define REG_THREED_LUT0_RDATA				0x7C4
#define REG_DPU_MMU_EN					0x1804
#define REG_DPU_MMU_INV_ADDR_RD				0x185C
#define REG_DPU_MMU_INV_ADDR_WR				0x1860
#define REG_DPU_MMU_UNS_ADDR_RD				0x1864
#define REG_DPU_MMU_UNS_ADDR_WR				0x1868
#define REG_DPU_MMU_INT_EN				0x18A0
#define REG_DPU_MMU_INT_CLR				0x18A4
#define REG_DPU_MMU_INT_STS				0x18A8
#define REG_DPU_MMU_INT_RAW				0x18AC

/* Global control bits */
#define BIT_DPU_RUN					BIT(0)
#define BIT_DPU_STOP					BIT(1)
#define BIT_DPU_ALL_UPDATE				BIT(2)
#define BIT_DPU_REG_UPDATE				BIT(3)
#define BIT_LAY_REG_UPDATE				BIT(4)
#define BIT_DPU_IF_EDPI					BIT(0)

/* scaling config bits */
#define BIT_DPU_SCALING_EN				BIT(0)

/* Layer control bits */
// #define BIT_DPU_LAY_EN				BIT(0)
#define BIT_DPU_LAY_LAYER_ALPHA				(0x01 << 2)
#define BIT_DPU_LAY_COMBO_ALPHA				(0x01 << 3)
#define BIT_DPU_LAY_FORMAT_YUV422_2PLANE		(0x00 << 4)
#define BIT_DPU_LAY_FORMAT_YUV420_2PLANE		(0x01 << 4)
#define BIT_DPU_LAY_FORMAT_YUV420_3PLANE		(0x02 << 4)
#define BIT_DPU_LAY_FORMAT_ARGB8888			(0x03 << 4)
#define BIT_DPU_LAY_FORMAT_RGB565			(0x04 << 4)
#define BIT_DPU_LAY_FORMAT_XFBC_ARGB8888		(0x08 << 4)
#define BIT_DPU_LAY_FORMAT_XFBC_RGB565			(0x09 << 4)
#define BIT_DPU_LAY_FORMAT_XFBC_YUV420			(0x0A << 4)
#define BIT_DPU_LAY_DATA_ENDIAN_B0B1B2B3		(0x00 << 8)
#define BIT_DPU_LAY_DATA_ENDIAN_B3B2B1B0		(0x01 << 8)
#define BIT_DPU_LAY_DATA_ENDIAN_B2B3B0B1		(0x02 << 8)
#define BIT_DPU_LAY_DATA_ENDIAN_B1B0B3B2		(0x03 << 8)
#define BIT_DPU_LAY_NO_SWITCH				(0x00 << 10)
#define BIT_DPU_LAY_RGB888_RB_SWITCH			(0x01 << 10)
#define BIT_DPU_LAY_RGB565_RB_SWITCH			(0x01 << 12)
#define BIT_DPU_LAY_PALLETE_EN				(0x01 << 13)
#define BIT_DPU_LAY_MODE_BLEND_NORMAL			(0x01 << 16)
#define BIT_DPU_LAY_MODE_BLEND_PREMULT			(0x01 << 16)

/*Interrupt control & status bits */
#define BIT_DPU_INT_DONE				BIT(0)
#define BIT_DPU_INT_TE					BIT(1)
#define BIT_DPU_INT_ERR					BIT(2)
#define BIT_DPU_INT_VSYNC				BIT(4)
#define BIT_DPU_INT_WB_DONE_EN				BIT(5)
#define BIT_DPU_INT_WB_ERR_EN				BIT(6)
#define BIT_DPU_INT_FBC_PLD_ERR				BIT(7)
#define BIT_DPU_INT_FBC_HDR_ERR				BIT(8)
#define BIT_DPU_INT_DPU_ALL_UPDATE_DONE			BIT(16)
#define BIT_DPU_INT_DPU_REG_UPDATE_DONE			BIT(17)
#define BIT_DPU_INT_LAY_REG_UPDATE_DONE			BIT(18)
#define BIT_DPU_INT_PQ_REG_UPDATE_DONE			BIT(19)
#define BIT_DPU_INT_PQ_LUT_UPDATE_DONE			BIT(20)

/* DPI control bits */
#define BIT_DPU_EDPI_TE_EN				BIT(8)
#define BIT_DPU_EDPI_FROM_EXTERNAL_PAD			BIT(10)
#define BIT_DPU_DPI_HALT_EN				BIT(16)
#define BIT_DPU_STS_RCH_DPU_BUSY			BIT(15)

/* MMU Interrupt bits */
#define BIT_DPU_INT_MMU_VAOR_RD_MASK			BIT(0)
#define BIT_DPU_INT_MMU_VAOR_WR_MASK			BIT(1)
#define BIT_DPU_INT_MMU_INV_RD_MASK			BIT(2)
#define BIT_DPU_INT_MMU_INV_WR_MASK			BIT(3)
#define BIT_DPU_INT_MMU_UNS_RD_MASK			BIT(4)
#define BIT_DPU_INT_MMU_UNS_WR_MASK			BIT(5)
#define BIT_DPU_INT_MMU_PAOR_RD_MASK			BIT(6)
#define BIT_DPU_INT_MMU_PAOR_WR_MASK			BIT(7)


struct layer_info {
	u16 dst_x;
	u16 dst_y;
	u16 dst_w;
	u16 dst_h;
};

enum sprd_fw_attr {
	FW_ATTR_NON_SECURE = 0,
	FW_ATTR_SECURE,
	FW_ATTR_PROTECTED,
};

struct wb_region {
	u32 index;
	u16 pos_x;
	u16 pos_y;
	u16 size_w;
	u16 size_h;
};

struct layer_reg {
	u32 addr[4];
	u32 ctrl;
	u32 dst_size;
	u32 src_size;
	u32 pitch;
	u32 pos;
	u32 alpha;
	u32 ck;
	u32 pallete;
	u32 crop_start;
	u32 reserved[3];
};

struct enhance_module {
	u32 epf_en: 1;
	u32 hsv_en: 1;
	u32 cm_en: 1;
	u32 gamma_en: 1;
	u32 lut3d_en: 1;
	u32 dither_en: 1;
	u32 slp_en: 1;
	u32 ltm_en: 1;
	u32 slp_mask_en: 1;
	u32 cabc_en: 1;
	u32 ud_en: 1;
	u32 ud_local_en: 1;
	u32 ud_mask_en: 1;
	u32 scl_en: 1;
};

struct luts_typeindex {
	u16 type;
	u16 index;
};

struct scale_cfg {
	u32 in_w;
	u32 in_h;
};

struct rgb_integrate_arr {
	uint32_t rgb_value[729];
};


//static void dpu_sr_config(struct dpu_context *ctx);


static void dpu_clean_all(struct dpu_context *ctx);
static void dpu_layer(struct dpu_context *ctx,
		    struct sprd_layer_state *hwlayer);
//static void dpu_enhance_reload(struct dpu_context *ctx);

static void dpu_version(struct dpu_context *ctx)
{
	ctx->version = "dpu-r6p1";
}

static void dpu_dump(struct dpu_context *ctx)
{
	u32 *reg = (u32 *)ctx->base;
	int i;

	pr_info("      0          4          8          C\n");
	for (i = 0; i < 256; i += 4) {
		pr_info("%04x: 0x%08x 0x%08x 0x%08x 0x%08x\n",
			i * 4, reg[i], reg[i + 1], reg[i + 2], reg[i + 3]);
	}
}

static u32 check_mmu_isr(struct dpu_context *ctx, u32 reg_val)
{
	u32 mmu_mask = BIT_DPU_INT_MMU_VAOR_RD_MASK |
			BIT_DPU_INT_MMU_VAOR_WR_MASK |
			BIT_DPU_INT_MMU_INV_RD_MASK |
			BIT_DPU_INT_MMU_INV_WR_MASK |
			BIT_DPU_INT_MMU_UNS_RD_MASK |
			BIT_DPU_INT_MMU_UNS_WR_MASK |
			BIT_DPU_INT_MMU_PAOR_RD_MASK |
			BIT_DPU_INT_MMU_PAOR_WR_MASK;
	u32 val = reg_val & mmu_mask;

	if (val) {
		pr_err("--- iommu interrupt err: 0x%04x ---\n", val);

		pr_err("iommu invalid read error, addr: 0x%08x\n",
			DPU_REG_RD(ctx->base + REG_DPU_MMU_INV_ADDR_RD));
		pr_err("iommu invalid write error, addr: 0x%08x\n",
			DPU_REG_RD(ctx->base + REG_DPU_MMU_INV_ADDR_WR));
		pr_err("iommu unsecurity read error, addr: 0x%08x\n",
			DPU_REG_RD(ctx->base + REG_DPU_MMU_UNS_ADDR_RD));
		pr_err("iommu unsecurity  write error, addr: 0x%08x\n",
			DPU_REG_RD(ctx->base + REG_DPU_MMU_UNS_ADDR_WR));

		pr_err("BUG: iommu failure at %s:%d/%s()!\n",
			__FILE__, __LINE__, __func__);

		dpu_dump(ctx);

		/* panic("iommu panic\n"); */
	}

	return val;
}

static u32 dpu_isr(struct dpu_context *ctx)
{
	struct sprd_dpu *dpu =
		(struct sprd_dpu *)container_of(ctx, struct sprd_dpu, ctx);
	u32 reg_val, int_mask = 0;
	u32 mmu_reg_val, mmu_int_mask = 0;

	reg_val = DPU_REG_RD(ctx->base + REG_DPU_INT_STS);
	mmu_reg_val = DPU_REG_RD(ctx->base + REG_DPU_MMU_INT_STS);

	/* disable err interrupt */
	if (reg_val & BIT_DPU_INT_ERR)
		int_mask |= BIT_DPU_INT_ERR;

	/* dpu vsync isr */
	if (reg_val & BIT_DPU_INT_VSYNC) {
		drm_crtc_handle_vblank(&dpu->crtc->base);

		/* write back feature */
		if ((ctx->vsync_count == ctx->max_vsync_count) && ctx->wb_en)
			schedule_work(&ctx->wb_work);

		ctx->vsync_count++;
	}

	/* dpu update done isr */
	if (reg_val & BIT_DPU_INT_LAY_REG_UPDATE_DONE) {
		/* dpu dvfs feature */
	//	tasklet_schedule(&ctx->dvfs_task);

		ctx->evt_update = true;
		wake_up_interruptible_all(&ctx->wait_queue);
	}

	if (reg_val & BIT_DPU_INT_DPU_REG_UPDATE_DONE) {
		ctx->evt_all_regs_update = true;
		wake_up_interruptible_all(&ctx->wait_queue);
	}

	if (reg_val & BIT_DPU_INT_DPU_ALL_UPDATE_DONE) {
		ctx->evt_all_update = true;
		wake_up_interruptible_all(&ctx->wait_queue);
	}

	/* dpu stop done isr */
	if (reg_val & BIT_DPU_INT_DONE) {
		ctx->evt_stop = true;
		wake_up_interruptible_all(&ctx->wait_queue);
	}

	/* dpu write back done isr */
	if (reg_val & BIT_DPU_INT_WB_DONE_EN) {
		/*
		 * The write back is a time-consuming operation. If there is a
		 * flip occurs before write back done, the write back buffer is
		 * no need to display. Otherwise the new frame will be covered
		 * by the write back buffer, which is not what we wanted.
		 */
		if (ctx->wb_en && (ctx->vsync_count > ctx->max_vsync_count)) {
			ctx->wb_en = false;
			schedule_work(&ctx->wb_work);
			/*reg_val |= DPU_INT_FENCE_SIGNAL_REQUEST;*/
		}

		pr_debug("wb done\n");
	}

	/* dpu write back error isr */
	if (reg_val & BIT_DPU_INT_WB_ERR_EN) {
		pr_err("dpu write back fail\n");
		/*give a new chance to write back*/
		// if (ctx->max_vsync_count > 0) {
		ctx->wb_en = true;
		ctx->vsync_count = 0;
		// }
	}

	/* dpu afbc payload error isr */
	if (reg_val & BIT_DPU_INT_FBC_PLD_ERR) {
		int_mask |= BIT_DPU_INT_FBC_PLD_ERR;
		pr_err("dpu afbc payload error\n");
	}

	/* dpu afbc header error isr */
	if (reg_val & BIT_DPU_INT_FBC_HDR_ERR) {
		int_mask |= BIT_DPU_INT_FBC_HDR_ERR;
		pr_err("dpu afbc header error\n");
	}

	DPU_REG_WR(ctx->base + REG_DPU_INT_CLR, reg_val);
	DPU_REG_CLR(ctx->base + REG_DPU_INT_EN, int_mask);

	mmu_int_mask |= check_mmu_isr(ctx, mmu_reg_val);

	DPU_REG_WR(ctx->base + REG_DPU_MMU_INT_CLR, mmu_reg_val);
	DPU_REG_CLR(ctx->base + REG_DPU_MMU_INT_EN, mmu_int_mask);

	return reg_val;
}

static int dpu_wait_stop_done(struct dpu_context *ctx)
{
	int rc, i;
	u32 dpu_sts_21, dpu_sts_22;
	struct sprd_dpu *dpu =
		(struct sprd_dpu *)container_of(ctx, struct sprd_dpu, ctx);

	if (ctx->stopped)
		return 0;

	/* wait for stop done interrupt */
	rc = wait_event_interruptible_timeout(ctx->wait_queue, ctx->evt_stop,
					       msecs_to_jiffies(500));
	ctx->evt_stop = false;

	ctx->stopped = true;

	if (!rc)
		/* time out */
		pr_err("dpu wait for stop done time out!\n");

	for (i = 1; i <= 3000; i++) {
		dpu_sts_21 = DPU_REG_RD(ctx->base + REG_DPU_STS_21);
		dpu_sts_22 = DPU_REG_RD(ctx->base + REG_DPU_STS_22);
		if ((dpu_sts_21 & BIT(15)) ||
		  (dpu_sts_22 & BIT(15)))
			mdelay(1);
		else {
			pr_info("dpu is idle now\n");
			break;
		}

		if (i == 3000) {
			pr_err("wait for dpu read idle 3s timeout need to reset dpu\n");
			dpu->glb->reset(ctx);
			break;
		}
	}

	return 0;
}

static int dpu_wait_update_done(struct dpu_context *ctx)
{
	int rc;

	/* clear the event flag before wait */
	if (!ctx->stopped)
		ctx->evt_update = false;

	/* wait for reg update done interrupt */
	rc = wait_event_interruptible_timeout(ctx->wait_queue, ctx->evt_update,
					       msecs_to_jiffies(500));

	if (!rc) {
		/* time out */
		pr_err("dpu wait for reg update done time out!\n");
		return -1;
	}

	return 0;
}

//static int dpu_wait_all_regs_update_done(struct dpu_context *ctx)
//{
//	int rc;

	/* clear the event flag before wait */
//	ctx->evt_all_regs_update = false;

	/* wait for reg update done interrupt */
//	rc = wait_event_interruptible_timeout(ctx->wait_queue,
//			ctx->evt_all_regs_update, msecs_to_jiffies(500));

//	if (!rc) {
		/* time out */
//		pr_err("dpu wait for all regs update done time out!\n");
//		return -1;
//	}

//	return 0;
//}


//static int dpu_wait_all_update_done(struct dpu_context *ctx)
//{
//	int rc;

	/* clear the event flag before wait */
//	ctx->evt_all_update = false;

	/* wait for reg update done interrupt */
//	rc = wait_event_interruptible_timeout(ctx->wait_queue, ctx->evt_all_update,
//					       msecs_to_jiffies(500));

//	if (!rc) {
		/* time out */
//		pr_err("dpu wait for reg update done time out!\n");
//		return -1;
//	}

//	return 0;
//}

static void dpu_stop(struct dpu_context *ctx)
{
	if (ctx->if_type == SPRD_DPU_IF_DPI)
		DPU_REG_SET(ctx->base + REG_DPU_CTRL, BIT_DPU_STOP);

	dpu_wait_stop_done(ctx);
	ctx->evt_update = false;
	pr_info("dpu stop\n");
}

static void dpu_run(struct dpu_context *ctx)
{
	DPU_REG_SET(ctx->base + REG_DPU_CTRL, BIT(4) | BIT(0));
	ctx->stopped = false;

	pr_info("dpu run\n");

	if (ctx->if_type == SPRD_DPU_IF_EDPI) {
		/*
		 * If the panel read GRAM speed faster than
		 * DSI write GRAM speed, it will display some
		 * mass on screen when backlight on. So wait
		 * a TE period after flush the GRAM.
		 */
		if (!ctx->panel_ready) {
			dpu_wait_stop_done(ctx);
			/* wait for TE again */
			mdelay(20);
			ctx->panel_ready = true;
		}
	}
}


static void dpu_wb_trigger(struct dpu_context *ctx, u8 count, bool debug)
{
	int mode_width  = DPU_REG_RD(ctx->base + REG_BLEND_SIZE) & 0xFFFF;
	int mode_height = DPU_REG_RD(ctx->base + REG_BLEND_SIZE) >> 16;

	ctx->wb_layer.dst_w = mode_width;
	ctx->wb_layer.dst_h = mode_height;
	ctx->wb_layer.src_w = mode_width;
	ctx->wb_layer.src_h = mode_height;
	ctx->wb_layer.pitch[0] = ALIGN(mode_width, 16) * 4;
	ctx->wb_layer.fbc_hsize_r = XFBC8888_HEADER_SIZE(mode_width,
						mode_height) / 128;
	DPU_REG_WR(ctx->base + REG_WB_PITCH, ALIGN((mode_width), 16));

	ctx->wb_layer.xfbc = ctx->wb_xfbc_en;

	if (ctx->wb_xfbc_en) {
		DPU_REG_WR(ctx->base + REG_WB_CFG, (ctx->wb_layer.fbc_hsize_r << 16) | BIT(0));
		DPU_REG_WR(ctx->base + REG_WB_BASE_ADDR, ctx->wb_layer.addr[0] +
				ctx->wb_layer.fbc_hsize_r);
		}
	else {
		DPU_REG_WR(ctx->base + REG_WB_CFG, 0);
		DPU_REG_WR(ctx->base + REG_WB_BASE_ADDR, ctx->wb_layer.addr[0]);
		}

	DPU_REG_WR(ctx->base + REG_WB_PITCH, ctx->vm.hactive);

	if (debug)
		/* writeback debug trigger */
		DPU_REG_WR(ctx->base + REG_WB_CTRL, BIT(1));
	else
		DPU_REG_SET(ctx->base + REG_WB_CTRL, BIT(0));

	/* update trigger */
	DPU_REG_SET(ctx->base + REG_DPU_CTRL, BIT(4));
	dpu_wait_update_done(ctx);

	pr_debug("write back trigger\n");
}

static void dpu_wb_flip(struct dpu_context *ctx)
{
	dpu_clean_all(ctx);
	dpu_layer(ctx, &ctx->wb_layer);

	DPU_REG_SET(ctx->base + REG_DPU_CTRL, BIT(4));
	dpu_wait_update_done(ctx);
	pr_debug("write back flip\n");
}

static void dpu_wb_work_func(struct work_struct *data)
{
	struct dpu_context *ctx =
		container_of(data, struct dpu_context, wb_work);

	down(&ctx->lock);

	if (!ctx->enabled) {
		up(&ctx->lock);
		pr_err("dpu is not initialized\n");
		return;
	}

	if (ctx->flip_pending) {
		up(&ctx->lock);
		pr_warn("dpu flip is disabled\n");
		return;
	}

	if (ctx->wb_en && (ctx->vsync_count > ctx->max_vsync_count))
		dpu_wb_trigger(ctx, 1, false);
	else if (!ctx->wb_en)
		dpu_wb_flip(ctx);

	up(&ctx->lock);
}

static int dpu_write_back_config(struct dpu_context *ctx)
{
	static int need_config = 0;
	size_t wb_buf_size;
	struct sprd_dpu *dpu =
		(struct sprd_dpu *)container_of(ctx, struct sprd_dpu, ctx);
	struct drm_device *drm = dpu->crtc->base.dev;
	int mode_width  = DPU_REG_RD(ctx->base + REG_BLEND_SIZE) & 0xFFFF;

	if (!need_config) {
		pr_debug("no need to open wb function\n");
		return 0;
	}

	ctx->wb_configed = true;
	if (ctx->wb_configed) {
		DPU_REG_WR(ctx->base + REG_WB_BASE_ADDR, ctx->wb_addr_p);
		DPU_REG_WR(ctx->base + REG_WB_PITCH, ALIGN((mode_width), 16));
		if (ctx->wb_xfbc_en)
			DPU_REG_WR(ctx->base + REG_WB_CFG, ((ctx->wb_layer.fbc_hsize_r << 16) | BIT(0)));
		else
			DPU_REG_WR(ctx->base + REG_WB_CFG, 0);
		pr_debug("write back has configed\n");
		return 0;
	}

	wb_buf_size = XFBC8888_BUFFER_SIZE(dpu->mode->hdisplay,
						dpu->mode->vdisplay);
	pr_info("use wb_reserved memory for writeback, size:0x%zx\n", wb_buf_size);
	ctx->wb_addr_v = dma_alloc_wc(drm->dev, wb_buf_size, &ctx->wb_addr_p, GFP_KERNEL);
	if (!ctx->wb_addr_p) {
		ctx->max_vsync_count = 0;
		return -ENOMEM;
	}

	// ctx->wb_xfbc_en = 1;
	ctx->wb_layer.index = 7;
	ctx->wb_layer.planes = 1;
	ctx->wb_layer.alpha = 0xff;
	ctx->wb_layer.format = DRM_FORMAT_ABGR8888;
	ctx->wb_layer.addr[0] = ctx->wb_addr_p;
	DPU_REG_WR(ctx->base + REG_WB_BASE_ADDR, ctx->wb_addr_p);
	DPU_REG_WR(ctx->base + REG_WB_PITCH, ALIGN((mode_width), 16));
	if (ctx->wb_xfbc_en) {
		ctx->wb_layer.xfbc = ctx->wb_xfbc_en;
		DPU_REG_WR(ctx->base + REG_WB_CFG, ((ctx->wb_layer.fbc_hsize_r << 16) | BIT(0)));
	}

	ctx->max_vsync_count = 0;
	need_config = 0;
	ctx->wb_configed = true;

	INIT_WORK(&ctx->wb_work, dpu_wb_work_func);

	return 0;
}
/*
static int dpu_config_dsc_param(struct dpu_context *ctx)
{
	u32 reg_val;
	struct sprd_dpu *dpu =
		(struct sprd_dpu *)container_of(ctx, struct sprd_dpu, ctx);
	struct sprd_panel *panel =
		(struct sprd_panel *)container_of(dpu->dsi->panel, struct sprd_panel, base);

	if (panel->info.dual_dsi_en) {
		reg_val = (ctx->vm.vactive << 16) |
			((ctx->vm.hactive >> 1)  << 0);
		DPU_REG_WR(ctx->base + DSC_REG(REG_DSC_PIC_SIZE), reg_val);
	} else {
		reg_val = (ctx->vm.vactive << 16) |
			(ctx->vm.hactive << 0);
		DPU_REG_WR(ctx->base + DSC_REG(REG_DSC_PIC_SIZE), reg_val);
	}
	if (panel->info.dual_dsi_en) {
		reg_val = ((ctx->vm.hsync_len >> 1) << 0) |
			((ctx->vm.hback_porch  >> 1) << 8) |
			((ctx->vm.hfront_porch >> 1) << 20);
		DPU_REG_WR(ctx->base + DSC_REG(REG_DSC_H_TIMING), reg_val);
	} else {
		reg_val = (ctx->vm.hsync_len << 0) |
			(ctx->vm.hback_porch  << 8) |
			(ctx->vm.hfront_porch << 20);
		DPU_REG_WR(ctx->base + DSC_REG(REG_DSC_H_TIMING), reg_val);
	}
	reg_val = (ctx->vm.vsync_len << 0) |
			(ctx->vm.vback_porch  << 8) |
			(ctx->vm.vfront_porch << 20);
	DPU_REG_WR(ctx->base + DSC_REG(REG_DSC_V_TIMING), reg_val);

	DPU_REG_WR(ctx->base + DSC_REG(REG_DSC_GRP_SIZE), ctx->dsc_cfg.reg.dsc_grp_size);
	DPU_REG_WR(ctx->base + DSC_REG(REG_DSC_SLICE_SIZE), ctx->dsc_cfg.reg.dsc_slice_size);

	DPU_REG_WR(ctx->base + DSC_REG(REG_DSC_CFG0), ctx->dsc_cfg.reg.dsc_cfg0);
	DPU_REG_WR(ctx->base + DSC_REG(REG_DSC_CFG1), ctx->dsc_cfg.reg.dsc_cfg1);
	DPU_REG_WR(ctx->base + DSC_REG(REG_DSC_CFG2), ctx->dsc_cfg.reg.dsc_cfg2);
	DPU_REG_WR(ctx->base + DSC_REG(REG_DSC_CFG3), ctx->dsc_cfg.reg.dsc_cfg3);
	DPU_REG_WR(ctx->base + DSC_REG(REG_DSC_CFG4), ctx->dsc_cfg.reg.dsc_cfg4);
	DPU_REG_WR(ctx->base + DSC_REG(REG_DSC_CFG5), ctx->dsc_cfg.reg.dsc_cfg5);
	DPU_REG_WR(ctx->base + DSC_REG(REG_DSC_CFG6), ctx->dsc_cfg.reg.dsc_cfg6);
	DPU_REG_WR(ctx->base + DSC_REG(REG_DSC_CFG7), ctx->dsc_cfg.reg.dsc_cfg7);
	DPU_REG_WR(ctx->base + DSC_REG(REG_DSC_CFG8), ctx->dsc_cfg.reg.dsc_cfg8);
	DPU_REG_WR(ctx->base + DSC_REG(REG_DSC_CFG9), ctx->dsc_cfg.reg.dsc_cfg9);
	DPU_REG_WR(ctx->base + DSC_REG(REG_DSC_CFG10), ctx->dsc_cfg.reg.dsc_cfg10);
	DPU_REG_WR(ctx->base + DSC_REG(REG_DSC_CFG11), ctx->dsc_cfg.reg.dsc_cfg11);
	DPU_REG_WR(ctx->base + DSC_REG(REG_DSC_CFG12), ctx->dsc_cfg.reg.dsc_cfg12);
	DPU_REG_WR(ctx->base + DSC_REG(REG_DSC_CFG13), ctx->dsc_cfg.reg.dsc_cfg13);
	DPU_REG_WR(ctx->base + DSC_REG(REG_DSC_CFG14), ctx->dsc_cfg.reg.dsc_cfg14);
	DPU_REG_WR(ctx->base + DSC_REG(REG_DSC_CFG15), ctx->dsc_cfg.reg.dsc_cfg15);
	DPU_REG_WR(ctx->base + DSC_REG(REG_DSC_CFG16), ctx->dsc_cfg.reg.dsc_cfg16);

	if (panel->info.dual_dsi_en) {
		reg_val = (ctx->vm.vactive << 16) |
			((ctx->vm.hactive >> 1) << 0);
		DPU_REG_WR(ctx->base + DSC1_REG(REG_DSC_PIC_SIZE), reg_val);
		reg_val = ((ctx->vm.hsync_len >> 1) << 0) |
			((ctx->vm.hback_porch  >> 1) << 8) |
			((ctx->vm.hfront_porch >> 1) << 20);
		DPU_REG_WR(ctx->base + DSC1_REG(REG_DSC_H_TIMING), reg_val);
		reg_val = (ctx->vm.vsync_len << 0) |
			(ctx->vm.vback_porch  << 8) |
			(ctx->vm.vfront_porch << 20);
		DPU_REG_WR(ctx->base + DSC1_REG(REG_DSC_V_TIMING), reg_val);
		DPU_REG_WR(ctx->base + DSC1_REG(REG_DSC_GRP_SIZE), ctx->dsc_cfg.reg.dsc_grp_size);
		DPU_REG_WR(ctx->base + DSC1_REG(REG_DSC_SLICE_SIZE),
							ctx->dsc_cfg.reg.dsc_slice_size);
		DPU_REG_WR(ctx->base + DSC1_REG(REG_DSC_CFG0), ctx->dsc_cfg.reg.dsc_cfg0);
		DPU_REG_WR(ctx->base + DSC1_REG(REG_DSC_CFG1), ctx->dsc_cfg.reg.dsc_cfg1);
		DPU_REG_WR(ctx->base + DSC1_REG(REG_DSC_CFG2), ctx->dsc_cfg.reg.dsc_cfg2);
		DPU_REG_WR(ctx->base + DSC1_REG(REG_DSC_CFG3), ctx->dsc_cfg.reg.dsc_cfg3);
		DPU_REG_WR(ctx->base + DSC1_REG(REG_DSC_CFG4), ctx->dsc_cfg.reg.dsc_cfg4);
		DPU_REG_WR(ctx->base + DSC1_REG(REG_DSC_CFG5), ctx->dsc_cfg.reg.dsc_cfg5);
		DPU_REG_WR(ctx->base + DSC1_REG(REG_DSC_CFG6), ctx->dsc_cfg.reg.dsc_cfg6);
		DPU_REG_WR(ctx->base + DSC1_REG(REG_DSC_CFG7), ctx->dsc_cfg.reg.dsc_cfg7);
		DPU_REG_WR(ctx->base + DSC1_REG(REG_DSC_CFG8), ctx->dsc_cfg.reg.dsc_cfg8);
		DPU_REG_WR(ctx->base + DSC1_REG(REG_DSC_CFG9), ctx->dsc_cfg.reg.dsc_cfg9);
		DPU_REG_WR(ctx->base + DSC1_REG(REG_DSC_CFG10), ctx->dsc_cfg.reg.dsc_cfg10);
		DPU_REG_WR(ctx->base + DSC1_REG(REG_DSC_CFG11), ctx->dsc_cfg.reg.dsc_cfg11);
		DPU_REG_WR(ctx->base + DSC1_REG(REG_DSC_CFG12), ctx->dsc_cfg.reg.dsc_cfg12);
		DPU_REG_WR(ctx->base + DSC1_REG(REG_DSC_CFG13), ctx->dsc_cfg.reg.dsc_cfg13);
		DPU_REG_WR(ctx->base + DSC1_REG(REG_DSC_CFG14), ctx->dsc_cfg.reg.dsc_cfg14);
		DPU_REG_WR(ctx->base + DSC1_REG(REG_DSC_CFG15), ctx->dsc_cfg.reg.dsc_cfg15);
		DPU_REG_WR(ctx->base + DSC1_REG(REG_DSC_CFG16), ctx->dsc_cfg.reg.dsc_cfg16);
		if (dpu->dsi->ctx.work_mode == DSI_MODE_CMD)
			DPU_REG_WR(ctx->base + DSC1_REG(REG_DSC_CTRL), 0x2000010b);
		else
			DPU_REG_WR(ctx->base + DSC1_REG(REG_DSC_CTRL), 0x2000000b);
	}

	if (dpu->dsi->ctx.work_mode == DSI_MODE_CMD)
		DPU_REG_WR(ctx->base + DSC_REG(REG_DSC_CTRL), 0x2000010b);
	else
		DPU_REG_WR(ctx->base + DSC_REG(REG_DSC_CTRL), 0x2000000b);

	return 0;
}
*/
static int dpu_init(struct dpu_context *ctx)
{
	u32 reg_val, size;
	struct sprd_dpu *dpu = (struct sprd_dpu *)container_of(ctx, struct sprd_dpu, ctx);
//	struct sprd_panel *panel =
//		(struct sprd_panel *)container_of(dpu->dsi->panel, struct sprd_panel, base);

	//calc_dsc_params(&ctx->dsc_init);

//	if (panel->info.dual_dsi_en)
//		DPU_REG_WR(ctx->base + REG_DPU_MODE, BIT(0));

	//if (panel->info.dsc_en)
	//	dpu_config_dsc_param(ctx);

	/* set bg color */
	DPU_REG_WR(ctx->base + REG_BG_COLOR, 0x00);

	/* set dpu output size */
	size = (ctx->vm.vactive << 16) | ctx->vm.hactive;
	DPU_REG_WR(ctx->base + REG_PANEL_SIZE, size);
	DPU_REG_WR(ctx->base + REG_BLEND_SIZE, size);

	DPU_REG_WR(ctx->base + REG_DPU_CFG0, 0x00);
	if (dpu->dsi->ctx.work_mode == DSI_MODE_CMD ) {
		DPU_REG_SET(ctx->base + REG_DPU_CFG0, BIT(1));
		ctx->is_single_run = true;
	}

	reg_val = (ctx->qos_cfg.awqos_high << 12) |
		(ctx->qos_cfg.awqos_low << 8) |
		(ctx->qos_cfg.arqos_high << 4) |
		(ctx->qos_cfg.arqos_low) | BIT(18) | BIT(22) | BIT(23);
	DPU_REG_WR(ctx->base + REG_DPU_CFG1, reg_val);;
	if (ctx->stopped)
		dpu_clean_all(ctx);

	DPU_REG_WR(ctx->base + REG_DPU_INT_CLR, 0xffff);


	dpu_write_back_config(ctx);

	return 0;
}

static void dpu_fini(struct dpu_context *ctx)
{

	DPU_REG_WR(ctx->base + REG_DPU_INT_EN, 0x00);
	DPU_REG_WR(ctx->base + REG_DPU_INT_CLR, 0xff);

	ctx->panel_ready = false;
}

enum {
	DPU_LAYER_FORMAT_YUV422_2PLANE,
	DPU_LAYER_FORMAT_YUV420_2PLANE,
	DPU_LAYER_FORMAT_YUV420_3PLANE,
	DPU_LAYER_FORMAT_ARGB8888,
	DPU_LAYER_FORMAT_RGB565,
	DPU_LAYER_FORMAT_XFBC_ARGB8888 = 8,
	DPU_LAYER_FORMAT_XFBC_RGB565,
	DPU_LAYER_FORMAT_XFBC_YUV420,
	DPU_LAYER_FORMAT_MAX_TYPES,
};

enum {
	DPU_LAYER_ROTATION_0,
	DPU_LAYER_ROTATION_90,
	DPU_LAYER_ROTATION_180,
	DPU_LAYER_ROTATION_270,
	DPU_LAYER_ROTATION_0_M,
	DPU_LAYER_ROTATION_90_M,
	DPU_LAYER_ROTATION_180_M,
	DPU_LAYER_ROTATION_270_M,
};

static u32 to_dpu_rotation(u32 angle)
{
	u32 rot = DPU_LAYER_ROTATION_0;

	switch (angle) {
	case 0:
	case DRM_MODE_ROTATE_0:
		rot = DPU_LAYER_ROTATION_0;
		break;
	case DRM_MODE_ROTATE_90:
		rot = DPU_LAYER_ROTATION_90;
		break;
	case DRM_MODE_ROTATE_180:
		rot = DPU_LAYER_ROTATION_180;
		break;
	case DRM_MODE_ROTATE_270:
		rot = DPU_LAYER_ROTATION_270;
		break;
	case DRM_MODE_REFLECT_Y:
		rot = DPU_LAYER_ROTATION_180_M;
		break;
	case (DRM_MODE_REFLECT_Y | DRM_MODE_ROTATE_90):
		rot = DPU_LAYER_ROTATION_90_M;
		break;
	case DRM_MODE_REFLECT_X:
		rot = DPU_LAYER_ROTATION_0_M;
		break;
	case (DRM_MODE_REFLECT_X | DRM_MODE_ROTATE_90):
		rot = DPU_LAYER_ROTATION_270_M;
		break;
	default:
		pr_err("rotation convert unsupport angle (drm)= 0x%x\n", angle);
		break;
	}

	return rot;
}

static u32 dpu_img_ctrl(u32 format, u32 blending, u32 compression, u32 y2r_coef,
		u32 rotation)
{
	int reg_val = 0;
	/* layer enable */
	// reg_val |= BIT_DPU_LAY_EN;

	switch (format) {
	case DRM_FORMAT_BGRA8888:
		/* BGRA8888 -> ARGB8888 */
		reg_val |= BIT_DPU_LAY_DATA_ENDIAN_B3B2B1B0;
		if (compression)
			/* XFBC-ARGB8888 */
			reg_val |= (BIT_DPU_LAY_FORMAT_XFBC_ARGB8888);
		else
			reg_val |= (BIT_DPU_LAY_FORMAT_ARGB8888);
		break;
	case DRM_FORMAT_RGBX8888:
	case DRM_FORMAT_RGBA8888:
		/* RGBA8888 -> ABGR8888 */
		reg_val |= BIT_DPU_LAY_DATA_ENDIAN_B3B2B1B0;
		/* FALLTHRU */
	case DRM_FORMAT_ABGR8888:
		/* rb switch */
		reg_val |= BIT_DPU_LAY_RGB888_RB_SWITCH;
		/* FALLTHRU */
	case DRM_FORMAT_ARGB8888:
		if (compression)
			/* XFBC-ARGB8888 */
			reg_val |= (BIT_DPU_LAY_FORMAT_XFBC_ARGB8888);
		else
			reg_val |= (BIT_DPU_LAY_FORMAT_ARGB8888);
		break;
	case DRM_FORMAT_XBGR8888:
		/* rb switch */
		reg_val |= BIT_DPU_LAY_RGB888_RB_SWITCH;
		/* FALLTHRU */
	case DRM_FORMAT_XRGB8888:
		if (compression)
			/* XFBC-ARGB8888 */
			reg_val |= (BIT_DPU_LAY_FORMAT_XFBC_ARGB8888);
		else
			reg_val |= (BIT_DPU_LAY_FORMAT_ARGB8888);
		break;
	case DRM_FORMAT_BGR565:
		/* rb switch */
		reg_val |= BIT_DPU_LAY_RGB565_RB_SWITCH;
		/* FALLTHRU */
	case DRM_FORMAT_RGB565:
		if (compression)
			/* XFBC-RGB565 */
			reg_val |= (BIT_DPU_LAY_FORMAT_XFBC_RGB565);
		else
			reg_val |= (BIT_DPU_LAY_FORMAT_RGB565);
		break;
	case DRM_FORMAT_NV12:
		if (compression)
			/*2-Lane: Yuv420 */
			reg_val |= BIT_DPU_LAY_FORMAT_XFBC_YUV420;
		else
			reg_val |= BIT_DPU_LAY_FORMAT_YUV420_2PLANE;
		/*Y endian */
		reg_val |= BIT_DPU_LAY_DATA_ENDIAN_B0B1B2B3;
		/*UV endian */
		reg_val |= BIT_DPU_LAY_DATA_ENDIAN_B0B1B2B3;
		break;
	case DRM_FORMAT_NV21:
		if (compression)
			/*2-Lane: Yuv420 */
			reg_val |= BIT_DPU_LAY_FORMAT_XFBC_YUV420;
		else
			reg_val |= BIT_DPU_LAY_FORMAT_YUV420_2PLANE;
		/*Y endian */
		reg_val |= BIT_DPU_LAY_DATA_ENDIAN_B0B1B2B3;
		/*UV endian */
		reg_val |= BIT_DPU_LAY_DATA_ENDIAN_B3B2B1B0 << 2;
		break;
	case DRM_FORMAT_NV16:
		/*2-Lane: Yuv422 */
		reg_val |= BIT_DPU_LAY_FORMAT_YUV422_2PLANE;
		/*Y endian */
		reg_val |= BIT_DPU_LAY_DATA_ENDIAN_B3B2B1B0;
		/*UV endian */
		reg_val |= BIT_DPU_LAY_DATA_ENDIAN_B3B2B1B0 << 2;
		break;
	case DRM_FORMAT_NV61:
		/*2-Lane: Yuv422 */
		reg_val |= BIT_DPU_LAY_FORMAT_YUV422_2PLANE;
		/*Y endian */
		reg_val |= BIT_DPU_LAY_DATA_ENDIAN_B0B1B2B3;
		/*UV endian */
		reg_val |= BIT_DPU_LAY_DATA_ENDIAN_B0B1B2B3;
		break;
	case DRM_FORMAT_YUV420:
		reg_val |= BIT_DPU_LAY_FORMAT_YUV420_3PLANE;
		/*Y endian */
		reg_val |= BIT_DPU_LAY_DATA_ENDIAN_B0B1B2B3;
		/*UV endian */
		reg_val |= BIT_DPU_LAY_DATA_ENDIAN_B0B1B2B3;
		break;
	default:
		pr_err("error: invalid format %c%c%c%c\n", format,
						format >> 8,
						format >> 16,
						format >> 24);
		break;
	}

	switch (blending) {
	case DRM_MODE_BLEND_PIXEL_NONE:
		/* don't do blending, maybe RGBX */
		/* alpha mode select - layer alpha */
		reg_val |= BIT_DPU_LAY_LAYER_ALPHA;
		break;
	case DRM_MODE_BLEND_COVERAGE:
		/* alpha mode select - combo alpha */
		reg_val |= BIT_DPU_LAY_COMBO_ALPHA;
		/* blending mode select - normal mode */
		reg_val &= (~BIT_DPU_LAY_MODE_BLEND_NORMAL);
		break;
	case DRM_MODE_BLEND_PREMULTI:
		/* alpha mode select - combo alpha */
		reg_val |= BIT_DPU_LAY_COMBO_ALPHA;
		/* blending mode select - pre-mult mode */
		reg_val |= BIT_DPU_LAY_MODE_BLEND_PREMULT;
		break;
	default:
		/* alpha mode select - layer alpha */
		reg_val |= BIT_DPU_LAY_LAYER_ALPHA;
		break;
	}

	reg_val |= y2r_coef << 28;
	rotation = to_dpu_rotation(rotation);
	reg_val |= (rotation & 0x7) << 20;

	return reg_val;
}

static void dpu_clean_all(struct dpu_context *ctx)
{
	int i;

	for (i = 0; i < 8; i++)
		DPU_REG_WR(ctx->base + DPU_LAY_REG(REG_LAY_CTRL, i), 0x00);

	DPU_REG_WR(ctx->base + REG_LAYER_ENABLE, 0);
}

static void dpu_bgcolor(struct dpu_context *ctx, u32 color)
{

	if (ctx->if_type == SPRD_DPU_IF_EDPI)
		dpu_wait_stop_done(ctx);

	DPU_REG_WR(ctx->base + REG_BG_COLOR, color);

	dpu_clean_all(ctx);

	if (ctx->is_single_run) {
		DPU_REG_SET(ctx->base + REG_DPU_CTRL, BIT(4));
		DPU_REG_SET(ctx->base + REG_DPU_CTRL, BIT(0));
	} else if (ctx->if_type == SPRD_DPU_IF_EDPI) {
		DPU_REG_SET(ctx->base + REG_DPU_CTRL, BIT_DPU_RUN);
		ctx->stopped = false;
	} else if ((ctx->if_type == SPRD_DPU_IF_DPI) && !ctx->stopped) {
		DPU_REG_SET(ctx->base + REG_DPU_CTRL, BIT_LAY_REG_UPDATE);
		dpu_wait_update_done(ctx);
	}
}


static void dpu_layer(struct dpu_context *ctx,
		    struct sprd_layer_state *hwlayer)
{
	const struct drm_format_info *info;
	struct layer_reg tmp = {};
	u32 dst_size, src_size, offset, wd, rot;
	int i;

	/* for secure displaying, just use layer 7 as secure layer */
	if (hwlayer->secure_en || ctx->secure_debug)
		hwlayer->index = 7;

	offset = (hwlayer->dst_x & 0xffff) | ((hwlayer->dst_y) << 16);
	src_size = (hwlayer->src_w & 0xffff) | ((hwlayer->src_h) << 16);
	dst_size = (hwlayer->dst_w & 0xffff) | ((hwlayer->dst_h) << 16);

	if (hwlayer->pallete_en) {
		tmp.pos = offset;
		tmp.src_size = src_size;
		tmp.dst_size = dst_size;
		tmp.alpha = hwlayer->alpha;
		tmp.pallete = hwlayer->pallete_color;

		/* pallete layer enable */
		tmp.ctrl = 0x2004;
		pr_debug("dst_x = %d, dst_y = %d, dst_w = %d, dst_h = %d, pallete:%d\n",
				hwlayer->dst_x, hwlayer->dst_y,
				hwlayer->dst_w, hwlayer->dst_h, tmp.pallete);
	} else {
		if (src_size != dst_size) {
			rot = to_dpu_rotation(hwlayer->rotation);
			if ((rot == DPU_LAYER_ROTATION_90) || (rot == DPU_LAYER_ROTATION_270) ||
				(rot == DPU_LAYER_ROTATION_90_M) || (rot == DPU_LAYER_ROTATION_270_M))
				dst_size = (hwlayer->dst_h & 0xffff) | ((hwlayer->dst_w) << 16);
		}

				tmp.ctrl = BIT(24);



		for (i = 0; i < hwlayer->planes; i++) {
			if (hwlayer->addr[i] % 16)
				pr_err("layer addr[%d] is not 16 bytes align, it's 0x%08x\n",
						i, hwlayer->addr[i]);
			tmp.addr[i] = hwlayer->addr[i];
		}

		tmp.pos = offset;
		tmp.src_size = src_size;
		tmp.dst_size = dst_size;
		tmp.crop_start = (hwlayer->src_y << 16) | hwlayer->src_x;
		tmp.alpha = hwlayer->alpha;

		info = drm_format_info(hwlayer->format);
		wd = info->cpp[0];
		if (wd == 0) {
			pr_err("layer[%d] bytes per pixel is invalid\n", hwlayer->index);
			return;
		}

		if (hwlayer->planes == 3)
			/* UV pitch is 1/2 of Y pitch*/
			tmp.pitch = (hwlayer->pitch[0] / wd) |
				(hwlayer->pitch[0] / wd << 15);
		else
			tmp.pitch = hwlayer->pitch[0] / wd;

		tmp.ctrl |= dpu_img_ctrl(hwlayer->format, hwlayer->blending,
				hwlayer->xfbc, hwlayer->y2r_coef, hwlayer->rotation);
	}

	for (i = 0; i < hwlayer->planes; i++)
		DPU_REG_WR(ctx->base + DPU_LAY_PLANE_ADDR(REG_LAY_BASE_ADDR,
					hwlayer->index, i), tmp.addr[i]);

	DPU_REG_WR(ctx->base + DPU_LAY_REG(REG_LAY_POS,
			hwlayer->index), tmp.pos);
	DPU_REG_WR(ctx->base + DPU_LAY_REG(REG_LAY_SRC_SIZE,
			hwlayer->index), tmp.src_size);
	DPU_REG_WR(ctx->base + DPU_LAY_REG(REG_LAY_DES_SIZE,
			hwlayer->index), tmp.dst_size);
	DPU_REG_WR(ctx->base + DPU_LAY_REG(REG_LAY_CROP_START,
			hwlayer->index), tmp.crop_start);
	DPU_REG_WR(ctx->base + DPU_LAY_REG(REG_LAY_ALPHA,
			hwlayer->index), tmp.alpha);
	DPU_REG_WR(ctx->base + DPU_LAY_REG(REG_LAY_PITCH,
			hwlayer->index), tmp.pitch);
	DPU_REG_WR(ctx->base + DPU_LAY_REG(REG_LAY_CTRL,
			hwlayer->index), tmp.ctrl);
	DPU_REG_SET(ctx->base + REG_LAYER_ENABLE,
			(1 << hwlayer->index));
	// DPU_REG_WR(ctx->base + DPU_LAY_REG(REG_LAY_PALLETE,
				// hwlayer->index), tmp.pallete);

	pr_debug("dst_x = %d, dst_y = %d, dst_w = %d, dst_h = %d\n",
				hwlayer->dst_x, hwlayer->dst_y,
				hwlayer->dst_w, hwlayer->dst_h);
	pr_debug("start_x = %d, start_y = %d, start_w = %d, start_h = %d\n",
				hwlayer->src_x, hwlayer->src_y,
				hwlayer->src_w, hwlayer->src_h);
}


static void dpu_flip(struct dpu_context *ctx,
		     struct sprd_plane planes[], u8 count)
{
	int i;
	u32 reg_val;
	struct sprd_plane_state *state;
//	struct sprd_dpu *dpu = container_of(ctx, struct sprd_dpu, ctx);
//	struct dpu_enhance *enhance = ctx->enhance;

	ctx->vsync_count = 0;
	if (ctx->max_vsync_count > 0 && count > 1)
		ctx->wb_en = true;
	/*
	 * Make sure the dpu is in stop status. DPU_r6p0 has no shadow
	 * registers in EDPI mode. So the config registers can only be
	 * updated in the rising edge of DPU_RUN bit.
	 */
	if (ctx->if_type == SPRD_DPU_IF_EDPI)
		dpu_wait_stop_done(ctx);

	/* reset the bgcolor to black */
	DPU_REG_WR(ctx->base + REG_BG_COLOR, 0x00);

	 /* to check if dpu need change the frame rate */
//	if (dpu->crtc->fps_mode_changed)
//		dpu_vrr(ctx);

	/* disable all the layers */
	dpu_clean_all(ctx);

	/* to check if dpu need scaling the frame for SR */
//	if (!dpu->dsi->ctx.surface_mode)
//		dpu_scaling(ctx, planes, count);

	/* start configure dpu layers */
	for (i = 0; i < count; i++) {
		state = to_sprd_plane_state(planes[i].base.state);
		dpu_layer(ctx, &state->layer);
	}
	/* update trigger and wait */
	if (ctx->is_single_run) {
		DPU_REG_SET(ctx->base + REG_DPU_CTRL, BIT(4));
		DPU_REG_SET(ctx->base + REG_DPU_CTRL, BIT(0));
	} else if (ctx->if_type == SPRD_DPU_IF_EDPI) {
		DPU_REG_SET(ctx->base + REG_DPU_CTRL, BIT_DPU_RUN);
		ctx->stopped = false;
	} else if (ctx->if_type == SPRD_DPU_IF_DPI) {
		if (!ctx->stopped) {

				DPU_REG_SET(ctx->base + REG_DPU_CTRL, BIT_LAY_REG_UPDATE);
				dpu_wait_update_done(ctx);

		}

		DPU_REG_SET(ctx->base + REG_DPU_INT_EN, BIT_DPU_INT_ERR);
	}

	/*
	 * If the following interrupt was disabled in isr,
	 * re-enable it.
	 */
	reg_val = BIT_DPU_INT_FBC_PLD_ERR |
	BIT_DPU_INT_FBC_HDR_ERR;
	DPU_REG_SET(ctx->base + REG_DPU_INT_EN, reg_val);
}


static void dpu_dpi_init(struct dpu_context *ctx)
{
	//struct sprd_dpu *dpu = container_of(ctx, struct sprd_dpu, ctx);
	//struct sprd_dsi *dsi = dpu->dsi;
	//struct sprd_panel *panel = container_of(dsi->panel, struct sprd_panel, base);
	u32 int_mask = 0;
	u32 reg_val;

	if (ctx->if_type == SPRD_DPU_IF_DPI) {
		/* use dpi as interface */
		DPU_REG_CLR(ctx->base + REG_DPU_CFG0, BIT_DPU_IF_EDPI);

		/* enable Halt function for SPRD DSI */
		DPU_REG_SET(ctx->base + REG_DPI_CTRL, BIT_DPU_DPI_HALT_EN);

		if (ctx->is_single_run)
			DPU_REG_SET(ctx->base + REG_DPU_CTRL, BIT(0));

		/* set dpi timing */
		reg_val = ctx->vm.hsync_len << 0 |
			  ctx->vm.hback_porch << 8 |
			  ctx->vm.hfront_porch << 20;
		DPU_REG_WR(ctx->base + REG_DPI_H_TIMING, reg_val);

		reg_val = ctx->vm.vsync_len << 0 |
			  ctx->vm.vback_porch << 8;
		DPU_REG_WR(ctx->base + REG_DPI_V_TIMING, reg_val);

		reg_val = ctx->vm.vfront_porch;
		DPU_REG_WR(ctx->base + REG_DPI_VFP, reg_val);

		if (ctx->vm.vsync_len + ctx->vm.vback_porch < 32)
			pr_warn("Warning: (vsync + vbp) < 32, "
				"underflow risk!\n");

		/* enable dpu update done INT */
		int_mask |= BIT_DPU_INT_DPU_ALL_UPDATE_DONE;
		int_mask |= BIT_DPU_INT_DPU_REG_UPDATE_DONE;
		int_mask |= BIT_DPU_INT_PQ_LUT_UPDATE_DONE;
		int_mask |= BIT_DPU_INT_LAY_REG_UPDATE_DONE;
		int_mask |= BIT_DPU_INT_PQ_REG_UPDATE_DONE;
		/* enable dpu DONE  INT */
		int_mask |= BIT_DPU_INT_DONE;
		/* enable dpu dpi vsync */
		int_mask |= BIT_DPU_INT_VSYNC;
		/* enable dpu TE INT */
		int_mask |= BIT_DPU_INT_TE;
		/* enable underflow err INT */
		int_mask |= BIT_DPU_INT_ERR;
		/* enable write back done INT */
		int_mask |= BIT_DPU_INT_WB_DONE_EN;
		/* enable write back fail INT */
		int_mask |= BIT_DPU_INT_WB_ERR_EN;

	} else if (ctx->if_type == SPRD_DPU_IF_EDPI) {
		/* use edpi as interface */
		DPU_REG_SET(ctx->base + REG_DPU_CFG0, BIT_DPU_IF_EDPI);

		/* use external te */
		DPU_REG_SET(ctx->base + REG_DPI_CTRL, BIT_DPU_EDPI_FROM_EXTERNAL_PAD);;

		/* enable te */
		DPU_REG_SET(ctx->base + REG_DPI_CTRL, BIT_DPU_EDPI_TE_EN);

		/* enable stop DONE INT */
		int_mask |= BIT_DPU_INT_DONE;
		/* enable TE INT */
		int_mask |= BIT_DPU_INT_TE;
	}

	/* enable ifbc payload error INT */
	int_mask |= BIT_DPU_INT_FBC_PLD_ERR;
	/* enable ifbc header error INT */
	int_mask |= BIT_DPU_INT_FBC_HDR_ERR;

	DPU_REG_WR(ctx->base + REG_DPU_INT_EN, int_mask);
}

static void enable_vsync(struct dpu_context *ctx)
{
	DPU_REG_SET(ctx->base + REG_DPU_INT_EN, BIT_DPU_INT_VSYNC);
}

static void disable_vsync(struct dpu_context *ctx)
{
	//DPU_REG_CLR(ctx->base + REG_DPU_INT_EN, BIT_DPU_INT_VSYNC);
}

static int dpu_context_init(struct dpu_context *ctx, struct device_node *np)
{
	struct device_node *qos_np;
	int ret = 0;

	qos_np = of_parse_phandle(np, "sprd,qos", 0);
	if (!qos_np)
		pr_warn("can't find dpu qos cfg node\n");

	ret = of_property_read_u8(qos_np, "arqos-low",
					&ctx->qos_cfg.arqos_low);
	if (ret) {
		pr_warn("read arqos-low failed, use default\n");
		ctx->qos_cfg.arqos_low = 0x0a;
	}

	ret = of_property_read_u8(qos_np, "arqos-high",
					&ctx->qos_cfg.arqos_high);
	if (ret) {
		pr_warn("read arqos-high failed, use default\n");
		ctx->qos_cfg.arqos_high = 0x0c;
	}

	ret = of_property_read_u8(qos_np, "awqos-low",
					&ctx->qos_cfg.awqos_low);
	if (ret) {
		pr_warn("read awqos_low failed, use default\n");
		ctx->qos_cfg.awqos_low = 0x0a;
	}

	ret = of_property_read_u8(qos_np, "awqos-high",
					&ctx->qos_cfg.awqos_high);
	if (ret) {
		pr_warn("read awqos-high failed, use default\n");
		ctx->qos_cfg.awqos_high = 0x0c;
	}

	return 0;
}



static int dpu_modeset(struct dpu_context *ctx,
		struct drm_display_mode *mode)
{
	struct scale_config_param *scale_cfg = &ctx->scale_cfg;

	scale_cfg->in_w = mode->hdisplay;
	scale_cfg->in_h = mode->vdisplay;

		if ((mode->hdisplay != ctx->vm.hactive) || (mode->vdisplay != ctx->vm.vactive))
			scale_cfg->need_scale = true;
		else
			scale_cfg->need_scale = false;
	pr_info("begin switch to %u x %u\n", mode->hdisplay, mode->vdisplay);

	return 0;
}

static const u32 primary_fmts[] = {
	DRM_FORMAT_XRGB8888, DRM_FORMAT_XBGR8888,
	DRM_FORMAT_ARGB8888, DRM_FORMAT_ABGR8888,
	DRM_FORMAT_RGBA8888, DRM_FORMAT_BGRA8888,
	DRM_FORMAT_RGBX8888, DRM_FORMAT_BGRX8888,
	DRM_FORMAT_RGB565, DRM_FORMAT_BGR565,
	DRM_FORMAT_NV12, DRM_FORMAT_NV21,
	DRM_FORMAT_NV16, DRM_FORMAT_NV61,
	DRM_FORMAT_YUV420,
};

static void dpu_capability(struct dpu_context *ctx,
			struct sprd_crtc_capability *cap)
{
	cap->max_layers = 6;
	cap->fmts_ptr = primary_fmts;
	cap->fmts_cnt = ARRAY_SIZE(primary_fmts);
}

const struct dpu_core_ops dpu_r6p1_core_ops = {
	.version = dpu_version,
	.init = dpu_init,
	.fini = dpu_fini,
	.run = dpu_run,
	.stop = dpu_stop,
	.isr = dpu_isr,
	.ifconfig = dpu_dpi_init,
	.capability = dpu_capability,
	.bg_color = dpu_bgcolor,
	.flip = dpu_flip,
	.enable_vsync = enable_vsync,
	.disable_vsync = disable_vsync,
	.context_init = dpu_context_init,
	.modeset = dpu_modeset,
	.write_back = dpu_wb_trigger,
};
