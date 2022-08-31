/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2022 Suguru Saito <sg.sgch07@gmail.com> 
 *
 * Based on mx6sabresd.c:
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
 */

#include <init.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/global_data.h>
#include <env.h>
#include <asm/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/mach-imx/video.h>
#include <mmc.h>
#include <fsl_esdhc_imx.h>
#include <asm/arch/crm_regs.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <i2c.h>
#include <input.h>
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define I2C_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define I2C_PMIC	1

#define I2C_PAD MUX_PAD_CTRL(I2C_PAD_CTRL)

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();
	return 0;
}

static iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_CSI0_DAT10__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_CSI0_DAT11__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static struct i2c_pads_info mx6dl_i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX6_PAD_KEY_COL3__I2C2_SCL | I2C_PAD,
		.gpio_mode = MX6_PAD_KEY_COL3__GPIO4_IO12 | I2C_PAD,
		.gp = IMX_GPIO_NR(4, 12)
	},
	.sda = {
		.i2c_mode = MX6_PAD_KEY_ROW3__I2C2_SDA | I2C_PAD,
		.gpio_mode = MX6_PAD_KEY_ROW3__GPIO4_IO13 | I2C_PAD,
		.gp = IMX_GPIO_NR(4, 13)
	}
};

static void setup_iomux_uart(void)
{
	SETUP_IOMUX_PADS(uart1_pads);
}

#ifdef CONFIG_FSL_ESDHC_IMX
int board_mmc_get_env_dev(int devno)
{
	return devno - 1;
}
#endif

#if defined(CONFIG_VIDEO_IPUV3)
struct display_info_t const displays[] = {{
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= NULL,
	.enable	= NULL,
	.di = 1,
	.mode	= {
		.name           = "XGA",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 768,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
	}
}};
size_t display_count = ARRAY_SIZE(displays);

static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	struct anatop_regs *anatop = (struct anatop_regs *)ANATOP_BASE_ADDR;
	int reg;

	/* Disable ipu1_clk/ipu1_di_clk_1/ldb_di1_clk. */
	disable_ipu_clock();
	reg = readl(&mxc_ccm->CCGR3);
	reg &= ~MXC_CCM_CCGR3_IPU1_IPU_DI1_MASK;
	reg &= ~MXC_CCM_CCGR3_LDB_DI1_MASK;
	writel(reg, &mxc_ccm->CCGR3);

	/*
	 * IPU1 HSP clock tree:
	 * osc_clk(24M)->pll3_usb_otg_main_clk(480M)->
	 * pll3_pfd_540M(540M)->ipu1_clk(270M)
	 */
	/* pll3_usb_otg_main_clk */
	/* divider: Fout=Fref*20(0) */
	writel(BM_ANADIG_USB1_PLL_480_CTRL_DIV_SELECT,
			&anatop->usb1_pll_480_ctrl_clr);

	/* pll3_pfd_540M */
	/* divider: 16 */
	writel(BM_ANADIG_PFD_480_PFD1_FRAC, &anatop->pfd_480_clr);
	writel(BF_ANADIG_PFD_480_PFD1_FRAC(16), &anatop->pfd_480_set);
	/* enable */
	writel(BM_ANADIG_PFD_480_PFD1_CLKGATE, &anatop->pfd_480_clr);

	/* ipu1_clk */
	reg = readl(&mxc_ccm->cscdr3);
	/* source: PLL3 PFD1(3) */
	reg |= (3 << MXC_CCM_CSCDR3_IPU1_HSP_CLK_SEL_OFFSET);
	/* divider: divide by 2(1) */
	reg &= ~MXC_CCM_CSCDR3_IPU1_HSP_PODF_MASK;
	reg |= (1 << MXC_CCM_CSCDR3_IPU1_HSP_PODF_OFFSET);
	writel(reg, &mxc_ccm->cscdr3);

	/*
	 * ipu1_pixel_clk_x clock tree:
	 * osc_clk(24M)->pll2_528_bus_main_clk(528M)->
	 * pll2_pfd_352M(452.57M)->ldb_dix_clk(64.65M)->
	 * ipu1_di_clk_x(64.65M)->ipu1_pixel_clk_x(64.65M)
	 */
	/* pll2_528_bus_main_clk */
	/* divider: Fout=Fref*22(1) */
	writel(BF_ANADIG_PLL_SYS_DIV_SELECT(1), &anatop->pll_528_set);

	/* pll2_pfd_352M */
	/* disable */
	writel(BM_ANADIG_PFD_528_PFD0_CLKGATE, &anatop->pfd_528_set);
	/* divider: 21 */
	writel(BM_ANADIG_PFD_528_PFD0_FRAC, &anatop->pfd_528_clr);
	writel(BF_ANADIG_PFD_528_PFD0_FRAC(21), &anatop->pfd_528_set);

	/* ldb_dix_clk */
	reg = readl(&mxc_ccm->cs2cdr);
	/* source
	 * ldb_di0: PLL2 PFD0(1)
	 * ldb_di1: PLL2 PFD0(1)
	 */
	reg &= ~MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK;
	reg &= ~MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK;
	reg |= (1 << MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET);
	reg |= (1 << MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->cs2cdr);
	/* divider */
	reg = readl(&mxc_ccm->cscmr2);
	reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV;
	reg |= MXC_CCM_CSCMR2_LDB_DI1_IPU_DIV;
	writel(reg, &mxc_ccm->cscmr2);

	/* pll2_pfd_352M */
	/* enable after ldb_dix_clk source is set */
	writel(BM_ANADIG_PFD_528_PFD0_CLKGATE, &anatop->pfd_528_clr);

	/* ipu1_di_clk_x */
	reg = readl(&mxc_ccm->chsccdr);
	/* source
	 * ipu1_di0_clk_sel: ldb_di0_clk(3)
	 * ipu1_di1_clk_sel: ldb_di1_clk(4)
	 */
	reg &= ~MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_MASK;
	reg &= ~MXC_CCM_CHSCCDR_IPU1_DI1_CLK_SEL_MASK;
	reg |= (3 << MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
	reg |= (4 << MXC_CCM_CHSCCDR_IPU1_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);

	/* Enable ipu1/ipu1_di1/ldb_di1 clocks. */
	enable_ipu_clock();
	reg = readl(&mxc_ccm->CCGR3);
	reg |= MXC_CCM_CCGR3_IPU1_IPU_DI1_MASK;
	reg |= MXC_CCM_CCGR3_LDB_DI1_MASK;
	writel(reg, &mxc_ccm->CCGR3);

	/*
	 * LVDS mux control
	 * LVDS0: IPU1 DI0
	 * LVDS1: IPU1 DI1
	 */
	reg = readl(&iomux->gpr[3]);
	reg &= ~IOMUXC_GPR3_LVDS0_MUX_CTL_MASK;
	reg &= ~IOMUXC_GPR3_LVDS1_MUX_CTL_MASK;
	reg |= (IOMUXC_GPR3_MUX_SRC_IPU1_DI0 << IOMUXC_GPR3_LVDS0_MUX_CTL_OFFSET);
	reg |= (IOMUXC_GPR3_MUX_SRC_IPU1_DI1 << IOMUXC_GPR3_LVDS1_MUX_CTL_OFFSET);
	writel(reg, &iomux->gpr[3]);

	/* LDB Control Register */
	reg  = IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_LOW;
	reg |= IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_HIGH;
	reg |= IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG;
	reg |= IOMUXC_GPR2_DATA_WIDTH_CH1_24BIT;
	reg |= IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG;
	reg |= IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT;
	reg |= IOMUXC_GPR2_LVDS_CH1_MODE_ENABLED_DI1;
	reg |= IOMUXC_GPR2_LVDS_CH0_MODE_DISABLED;
	writel(reg, &iomux->gpr[2]);
}
#endif /* CONFIG_VIDEO_IPUV3 */

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

int board_early_init_f(void)
{
	setup_iomux_uart();

	return 0;
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &mx6dl_i2c_pad_info1);
#if defined(CONFIG_VIDEO_IPUV3)
	setup_display();
#endif

	return 0;
}

int power_init_board(void)
{
	struct pmic *p;
	unsigned int reg, dev_id, rev_id;
	int ret;

	ret = power_pfuze100_init(I2C_PMIC);
	if (ret == -ENODEV)
		return ret;

	p = pmic_get("PFUZE100");
	ret = pmic_probe(p);
	if (ret)
		return ret;

	pmic_reg_read(p, PFUZE100_DEVICEID, &dev_id);
	pmic_reg_read(p, PFUZE100_REVID, &rev_id);
	printf("PMIC: PFUZE100! DEV_ID=0x%x REV_ID=0x%x\n", dev_id, rev_id);

	/* Increase VGEN4 from 1.8 to 2.0V */
	pmic_reg_read(p, PFUZE100_VGEN4VOL, &reg);
	reg &= ~LDO_VOL_MASK;
	reg |= LDOB_2_00V;
	pmic_reg_write(p, PFUZE100_VGEN4VOL, reg);

	return 0;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"sd3",	 MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
	/* 8 bit bus width */
	{"emmc", MAKE_CFGVAL(0x60, 0x58, 0x00, 0x00)},
	{NULL,	 0},
};
#endif

int board_late_init(void)
{
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

	return 0;
}
