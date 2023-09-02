// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Unisoc Inc.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>

#include "../dsi/core/dsi_ctrl_r1p1.h"
#include "../sprd_dsi.h"
#include "../sysfs/sysfs_display.h"

#include "umb9230s.h"


/* dsi rx registers */
#define REG_DSI_RX_BASE                     0x8000
#define REG_DSI_RX_PHY_PD_N                 (REG_DSI_RX_BASE + 0x04)
#define REG_DSI_RX_RST_DPHY_N               (REG_DSI_RX_BASE + 0x08)
#define REG_DSI_RX_PHY_STATE                (REG_DSI_RX_BASE + 0x0C)
#define REG_DSI_RX_PHY_TEST_CTRL0           (REG_DSI_RX_BASE + 0x2C)
#define REG_DSI_RX_PHY_TEST_CTRL1           (REG_DSI_RX_BASE + 0x30)
#define REG_DSI_RX_PHY_RX_TRIGGERS          (REG_DSI_RX_BASE + 0x34)

/* dsi tx registers */
#define REG_DSI_TX_BASE                         0x9400
#define REG_DSI_TX_PHY_CLK_LANE_LP_CTRL         (REG_DSI_TX_BASE + 0x74)
#define REG_DSI_TX_PHY_INTERFACE_CTRL           (REG_DSI_TX_BASE + 0x78)
#define REG_DSI_TX_PHY_TX_TRIGGERS              (REG_DSI_TX_BASE + 0x7C)
#define REG_DSI_TX_PHY_STATUS                   (REG_DSI_TX_BASE + 0x9C)
#define REG_DSI_TX_PHY_MIN_STOP_TIME            (REG_DSI_TX_BASE + 0xA0)
#define REG_DSI_TX_PHY_LANE_NUM_CONFIG          (REG_DSI_TX_BASE + 0xA4)
#define REG_DSI_TX_PHY_CLKLANE_TIME_CONFIG      (REG_DSI_TX_BASE + 0xA8)
#define REG_DSI_TX_PHY_DATALANE_TIME_CONFIG     (REG_DSI_TX_BASE + 0xAC)
#define REG_DSI_TX_PHY_TST_CTRL0                (REG_DSI_TX_BASE + 0xF0)
#define REG_DSI_TX_PHY_TST_CTRL1                (REG_DSI_TX_BASE + 0xF4)
#define REG_DSI_TX_INT_PLL_STS                  (REG_DSI_TX_BASE + 0x200)
#define REG_DSI_TX_INT_PLL_MSK                  (REG_DSI_TX_BASE + 0x204)
#define REG_DSI_TX_INT_PLL_CLR                  (REG_DSI_TX_BASE + 0x208)

/* dphy tx/rx registers */
#define REG_PHY_TEST_CTRL           0xB000
#define REG_PHY_TX_BASE             (REG_PHY_TEST_CTRL + 0x400)
#define REG_PHY_RX_BASE             REG_PHY_TEST_CTRL

void umb9230s_phy_rx_init(struct umb9230s_device *umb9230s)
{
    struct dsi_rx_reg reg = {};
    u32 buf[2];

    pr_info("umb9230s lanes : %d\n", umb9230s->phy_ctx.lanes);

    /* rstz */
    iic2cmd_read(umb9230s->i2c_addr, REG_DSI_RX_RST_DPHY_N, &reg.RST_DPHY_N.val, 1);
    reg.RST_DPHY_N.bits.rst_dphy_n = 0;
    buf[0] = REG_DSI_RX_RST_DPHY_N;
    buf[1] = reg.RST_DPHY_N.val;
    iic2cmd_write(umb9230s->i2c_addr, buf, 2);

    /* shutdownz */
    iic2cmd_read(umb9230s->i2c_addr, REG_DSI_RX_PHY_PD_N, &reg.PHY_PD_N.val, 1);
    reg.PHY_PD_N.bits.phy_pd_n = 0;
    buf[0] = REG_DSI_RX_PHY_PD_N;
    buf[1] = reg.PHY_PD_N.val;
    iic2cmd_write(umb9230s->i2c_addr, buf, 2);

    /* test_clr */
    iic2cmd_read(umb9230s->i2c_addr, REG_DSI_RX_PHY_TEST_CTRL0, &reg.PHY_TEST_CTRL0.val, 1);
    reg.PHY_TEST_CTRL0.bits.phy_testclr = 0;
    buf[0] = REG_DSI_RX_PHY_TEST_CTRL0;
    buf[1] = reg.PHY_TEST_CTRL0.val;
    iic2cmd_write(umb9230s->i2c_addr, buf, 2);

    /* test_clr */
    reg.PHY_TEST_CTRL0.bits.phy_testclr = 1;
    buf[0] = REG_DSI_RX_PHY_TEST_CTRL0;
    buf[1] = reg.PHY_TEST_CTRL0.val;
    iic2cmd_write(umb9230s->i2c_addr, buf, 2);

    /* test_clr */
    reg.PHY_TEST_CTRL0.bits.phy_testclr = 0;
    buf[0] = REG_DSI_RX_PHY_TEST_CTRL0;
    buf[1] = reg.PHY_TEST_CTRL0.val;
    iic2cmd_write(umb9230s->i2c_addr, buf, 2);

    /* shutdownz */
    reg.PHY_PD_N.bits.phy_pd_n = 1;
    buf[0] = REG_DSI_RX_PHY_PD_N;
    buf[1] = reg.PHY_PD_N.val;
    iic2cmd_write(umb9230s->i2c_addr, buf, 2);

    /* rstz */
    reg.RST_DPHY_N.bits.rst_dphy_n = 1;
    buf[0] = REG_DSI_RX_RST_DPHY_N;
    buf[1] = reg.RST_DPHY_N.val;
    iic2cmd_write(umb9230s->i2c_addr, buf, 2);
}

static int umb9230s_phy_tx_wait_pll_locked(struct umb9230s_device *umb9230s)
{
    u32 i = 0;
    union _0x9C phy_status;

    pr_info("phy tx wait pll locked\n");

    for (i = 0; i < 50000; i++) {
        iic2cmd_read(umb9230s->i2c_addr, REG_DSI_TX_PHY_STATUS, &phy_status.val, 1);
        if (phy_status.bits.phy_lock)
            return 0;
        udelay(3);
    }

    pr_err("error: umb9230s dphy pll can not be locked\n");
    return -ETIMEDOUT;
}

static int umb9230s_phy_tx_wait_datalane_stop_state(struct umb9230s_device *umb9230s, u8 mask)
{
    u32 i = 0;
    union _0x9C phy_status;
    u8 state = 0;

    for (i = 0; i < 5000; i++) {
        /* is_stop_state_datalane */
        iic2cmd_read(umb9230s->i2c_addr, REG_DSI_TX_PHY_STATUS, &phy_status.val, 1);

        if (phy_status.bits.phy_stopstate0lane)
            state |= BIT(0);
        if (phy_status.bits.phy_stopstate1lane)
            state |= BIT(1);
        if (phy_status.bits.phy_stopstate2lane)
            state |= BIT(2);
        if (phy_status.bits.phy_stopstate3lane)
            state |= BIT(3);

        if (state == mask)
            return 0;
        udelay(10);
    }

    pr_err("wait umb9230s datalane stop-state time out\n");
    return -ETIMEDOUT;
}

static int umb9230s_phy_tx_wait_datalane_ulps_active(struct umb9230s_device *umb9230s, u8 mask)
{
    u32 i = 0;
    union _0x9C phy_status;
    u8 state = 0;

    for (i = 0; i < 5000; i++) {
        /* is_ulps_active_datalane */
        iic2cmd_read(umb9230s->i2c_addr, REG_DSI_TX_PHY_STATUS, &phy_status.val, 1);

        if (!phy_status.bits.phy_ulpsactivenot0lane)
            state |= BIT(0);
        if (!phy_status.bits.phy_ulpsactivenot1lane)
            state |= BIT(1);
        if (!phy_status.bits.phy_ulpsactivenot2lane)
            state |= BIT(2);
        if (!phy_status.bits.phy_ulpsactivenot3lane)
            state |= BIT(3);

        if (state == mask)
            return 0;
        udelay(10);
    }

    pr_err("wait umb9230s datalane ulps-active time out\n");
    return -ETIMEDOUT;
}

static int umb9230s_phy_tx_wait_clklane_stop_state(struct umb9230s_device *umb9230s)
{
    u32 i = 0;
    union _0x9C phy_status;

    for (i = 0; i < 5000; i++) {
        /* is_stop_state_clklane */
        iic2cmd_read(umb9230s->i2c_addr, REG_DSI_TX_PHY_STATUS, &phy_status.val, 1);

        if (phy_status.bits.phy_stopstateclklane)
            return 0;
        udelay(10);
    }

    pr_err("wait umb9230s clklane stop-state time out\n");
    return -ETIMEDOUT;
}

static int umb9230s_phy_tx_wait_clklane_ulps_active(struct umb9230s_device *umb9230s)
{
    u32 i = 0;
    union _0x9C phy_status;

    for (i = 0; i < 5000; i++) {
        /* is_ulps_active_clklane */
        iic2cmd_read(umb9230s->i2c_addr, REG_DSI_TX_PHY_STATUS, &phy_status.val, 1);

        if (!phy_status.bits.phy_ulpsactivenotclk)
            return 0;
        udelay(10);
    }

    pr_err("wait umb9230s clklane ulps-active time out\n");
    return -ETIMEDOUT;
}

int umb9230s_phy_tx_enable(struct umb9230s_device *umb9230s)
{
    struct dsi_reg reg = {};
    int ret;
    u32 buf[2];
    const struct dphy_tx_pll_ops *pll = umb9230s->pll;

    pr_info("phy tx enable\n");

    /* rstz */
    iic2cmd_read(umb9230s->i2c_addr, REG_DSI_TX_PHY_INTERFACE_CTRL, &reg.PHY_INTERFACE_CTRL.val, 1);
    reg.PHY_INTERFACE_CTRL.bits.rf_phy_reset_n = 0;
    buf[0] = REG_DSI_TX_PHY_INTERFACE_CTRL;
    buf[1] = reg.PHY_INTERFACE_CTRL.val;
    iic2cmd_write(umb9230s->i2c_addr, buf, 2);

    /* shutdownz */
    reg.PHY_INTERFACE_CTRL.bits.rf_phy_shutdown = 0;
    buf[0] = REG_DSI_TX_PHY_INTERFACE_CTRL;
    buf[1] = reg.PHY_INTERFACE_CTRL.val;
    iic2cmd_write(umb9230s->i2c_addr, buf, 2);

    /* clklane_en */
    reg.PHY_INTERFACE_CTRL.bits.rf_phy_clk_en = 0;
    buf[0] = REG_DSI_TX_PHY_INTERFACE_CTRL;
    buf[1] = reg.PHY_INTERFACE_CTRL.val;
    iic2cmd_write(umb9230s->i2c_addr, buf, 2);

    /* test_clr */
    iic2cmd_read(umb9230s->i2c_addr, REG_DSI_TX_PHY_TST_CTRL0, &reg.PHY_TST_CTRL0.val, 1);
    reg.PHY_TST_CTRL0.bits.phy_testclr = 0;
    buf[0] = REG_DSI_TX_PHY_TST_CTRL0;
    buf[1] = reg.PHY_TST_CTRL0.val;
    iic2cmd_write(umb9230s->i2c_addr, buf, 2);

    /* test_clr */
    reg.PHY_TST_CTRL0.bits.phy_testclr = 1;
    buf[0] = REG_DSI_TX_PHY_TST_CTRL0;
    buf[1] = reg.PHY_TST_CTRL0.val;
    iic2cmd_write(umb9230s->i2c_addr, buf, 2);

    /* test_clr */
    reg.PHY_TST_CTRL0.bits.phy_testclr = 0;
    buf[0] = REG_DSI_TX_PHY_TST_CTRL0;
    buf[1] = reg.PHY_TST_CTRL0.val;
    iic2cmd_write(umb9230s->i2c_addr, buf, 2);

    pll->pll_config(&umb9230s->phy_ctx);
    pll->timing_config(&umb9230s->phy_ctx);

    /* shutdownz */
    reg.PHY_INTERFACE_CTRL.bits.rf_phy_shutdown = 1;
    buf[0] = REG_DSI_TX_PHY_INTERFACE_CTRL;
    buf[1] = reg.PHY_INTERFACE_CTRL.val;
    iic2cmd_write(umb9230s->i2c_addr, buf, 2);

    /* rstz */
    reg.PHY_INTERFACE_CTRL.bits.rf_phy_reset_n = 1;
    buf[1] = reg.PHY_INTERFACE_CTRL.val;
    iic2cmd_write(umb9230s->i2c_addr, buf, 2);

    /* stop_wait_time */
    buf[0] = REG_DSI_TX_PHY_MIN_STOP_TIME;
    buf[1] = 0x1C;
    iic2cmd_write(umb9230s->i2c_addr, buf, 2);

    /* clklane_en */
    reg.PHY_INTERFACE_CTRL.bits.rf_phy_clk_en = 1;
    buf[0] = REG_DSI_TX_PHY_INTERFACE_CTRL;
    buf[1] = reg.PHY_INTERFACE_CTRL.val;
    iic2cmd_write(umb9230s->i2c_addr, buf, 2);

    /* datalane_en */
    buf[0] = REG_DSI_TX_PHY_LANE_NUM_CONFIG;
    buf[1] = umb9230s->phy_ctx.lanes - 1;;
    iic2cmd_write(umb9230s->i2c_addr, buf, 2);

    /* wait_pll_locked */
    ret = umb9230s_phy_tx_wait_pll_locked(umb9230s);
    if (ret)
        return ret;

    return 0;
}

static void umb9230s_phy_tx_data_ulps_enter(struct umb9230s_device *umb9230s)
{
    union _0x78 phy_interface_ctrl;
    u8 lane_mask = (1 << umb9230s->phy_ctx.lanes) - 1;
    u32 buf[2];

    /* datalane_ulps_rqst */
    iic2cmd_read(umb9230s->i2c_addr, REG_DSI_TX_PHY_INTERFACE_CTRL, &phy_interface_ctrl.val, 1);
    phy_interface_ctrl.bits.rf_phy_data_txrequlps = 1;
    buf[0] = REG_DSI_TX_PHY_INTERFACE_CTRL;
    buf[1] = phy_interface_ctrl.val;
    iic2cmd_write(umb9230s->i2c_addr, buf, 2);

    /* wait_datalane_ulps_active */
    umb9230s_phy_tx_wait_datalane_ulps_active(umb9230s, lane_mask);

    /* dphy_hal_datalane_ulps_rqst */
    phy_interface_ctrl.bits.rf_phy_data_txrequlps = 0;
    buf[0] = REG_DSI_TX_PHY_INTERFACE_CTRL;
    buf[1] = phy_interface_ctrl.val;
    iic2cmd_write(umb9230s->i2c_addr, buf, 2);
}

static void umb9230s_phy_tx_data_ulps_exit(struct umb9230s_device *umb9230s)
{
    u8 lane_mask = (1 << umb9230s->phy_ctx.lanes) - 1;
    union _0x78 phy_interface_ctrl;
    u32 buf[2];

    /* datalane_ulps_exit(1)*/
    iic2cmd_read(umb9230s->i2c_addr, REG_DSI_TX_PHY_INTERFACE_CTRL, &phy_interface_ctrl.val, 1);
    phy_interface_ctrl.bits.rf_phy_data_txexitulps = 1;
    buf[0] = REG_DSI_TX_PHY_INTERFACE_CTRL;
    buf[1] = phy_interface_ctrl.val;
    iic2cmd_write(umb9230s->i2c_addr, buf, 2);

    /* wait_datalane_stop_state */
    umb9230s_phy_tx_wait_datalane_stop_state(umb9230s, lane_mask);

    /* datalane_ulps_exit(0)*/
    phy_interface_ctrl.bits.rf_phy_data_txexitulps = 0;
    buf[0] = REG_DSI_TX_PHY_INTERFACE_CTRL;
    buf[1] = phy_interface_ctrl.val;
    iic2cmd_write(umb9230s->i2c_addr, buf, 2);
}

static void umb9230s_phy_tx_clk_ulps_enter(struct umb9230s_device *umb9230s)
{
    union _0x78 phy_interface_ctrl;
    u32 buf[2];

    /* clklane_ulps_rqst(1) */
    iic2cmd_read(umb9230s->i2c_addr, REG_DSI_TX_PHY_INTERFACE_CTRL, &phy_interface_ctrl.val, 1);
    phy_interface_ctrl.bits.rf_phy_clk_txrequlps = 1;
    buf[0] = REG_DSI_TX_PHY_INTERFACE_CTRL;
    buf[1] = phy_interface_ctrl.val;
    iic2cmd_write(umb9230s->i2c_addr, buf, 2);

    /* wait_clklane_ulps_active */
    umb9230s_phy_tx_wait_clklane_ulps_active(umb9230s);

    /* clklane_ulps_rqst(0) */
    phy_interface_ctrl.bits.rf_phy_clk_txrequlps = 0;
    buf[0] = REG_DSI_TX_PHY_INTERFACE_CTRL;
    buf[1] = phy_interface_ctrl.val;
    iic2cmd_write(umb9230s->i2c_addr, buf, 2);
}

static void umb9230s_phy_tx_clk_ulps_exit(struct umb9230s_device *umb9230s)
{
    union _0x78 phy_interface_ctrl;
    u32 buf[2];

    /* clklane_ulps_exit(1) */
    iic2cmd_read(umb9230s->i2c_addr, REG_DSI_TX_PHY_INTERFACE_CTRL, &phy_interface_ctrl.val, 1);
    phy_interface_ctrl.bits.rf_phy_clk_txexitulps = 1;
    buf[0] = REG_DSI_TX_PHY_INTERFACE_CTRL;
    buf[1] = phy_interface_ctrl.val;
    iic2cmd_write(umb9230s->i2c_addr, buf, 2);

    /* wait_clklane_stop_state */
    umb9230s_phy_tx_wait_clklane_stop_state(umb9230s);

    /* clklane_ulps_exit */
    phy_interface_ctrl.bits.rf_phy_clk_txexitulps = 0;
    buf[0] = REG_DSI_TX_PHY_INTERFACE_CTRL;
    buf[1] = phy_interface_ctrl.val;
    iic2cmd_write(umb9230s->i2c_addr, buf, 2);
}

void umb9230s_phy_tx_hs_clk_en(struct umb9230s_device *umb9230s, bool enable)
{
    union _0x74 phy_clk_lane_lp_ctrl;
    u32 buf[2];

    phy_clk_lane_lp_ctrl.bits.auto_clklane_ctrl_en = 0;
    phy_clk_lane_lp_ctrl.bits.phy_clklane_tx_req_hs = enable;

    buf[0] = REG_DSI_TX_PHY_CLK_LANE_LP_CTRL;
    buf[1] = phy_clk_lane_lp_ctrl.val;
    iic2cmd_write(umb9230s->i2c_addr, buf, 2);

    /* wait_pll_locked */
    umb9230s_phy_tx_wait_pll_locked(umb9230s);
}

void umb9230s_phy_tx_ulps_enter(struct umb9230s_device *umb9230s)
{
    if (!umb9230s || !umb9230s->phy_ctx.ulps_enable)
        return;

    /* hs_clk_en(false) */
    umb9230s_phy_tx_hs_clk_en(umb9230s, false);

    umb9230s_phy_tx_data_ulps_enter(umb9230s);
    umb9230s_phy_tx_clk_ulps_enter(umb9230s);
}

void umb9230s_phy_tx_ulps_exit(struct umb9230s_device *umb9230s)
{
    union _0x78 phy_interface_ctrl;
    u32 buf[2];

    /* force_pll(true) */
    iic2cmd_read(umb9230s->i2c_addr, REG_DSI_TX_PHY_INTERFACE_CTRL, &phy_interface_ctrl.val, 1);
    phy_interface_ctrl.bits.rf_phy_force_pll = true;
    buf[0] = REG_DSI_TX_PHY_INTERFACE_CTRL;
    buf[1] = phy_interface_ctrl.val;
    iic2cmd_write(umb9230s->i2c_addr, buf, 2);

    umb9230s_phy_tx_clk_ulps_exit(umb9230s);
    umb9230s_phy_tx_data_ulps_exit(umb9230s);

    /* force_pll(false) */
    phy_interface_ctrl.bits.rf_phy_force_pll = false;
    buf[0] = REG_DSI_TX_PHY_INTERFACE_CTRL;
    buf[1] = phy_interface_ctrl.val;
    iic2cmd_write(umb9230s->i2c_addr, buf, 2);
}