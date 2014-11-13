/*
 * intel_pmic_crc.c - Intel CrystalCove PMIC operation region Driver
 *
 * Copyright (C) 2014 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/acpi.h>
#include <linux/mfd/intel_soc_pmic.h>
#include <linux/regmap.h>
#include <linux/platform_device.h>
#include "intel_pmic.h"

#define PWR_SOURCE_SELECT	BIT(1)

#define PMIC_A0LOCK_REG		0xc5

static struct pmic_pwr_table pwr_table[] = {
	{
		.address = 0x24,
		.pwr_reg = {
			.reg = 0x66,
			.bit = 0x00,
		},
	},	/* X285 -> V2P85SX, camara */
	{
		.address = 0x48,
		.pwr_reg = {
			.reg = 0x5d,
			.bit = 0x00,
		},
	},	/* V18X -> V1P8SX, eMMC/camara/audio */
};

static struct pmic_dptf_table dptf_table[] = {
	{
		.address = 0x00,
		.reg = 0x75
	},	/* TMP0 -> SYS0_THRM_RSLT_L */
	{
		.address = 0x04,
		.reg = 0x95
	},	/* AX00 -> SYS0_THRMALRT0_L */
	{
		.address = 0x08,
		.reg = 0x97
	},	/* AX01 -> SYS0_THRMALRT1_L */
	{
		.address = 0x0c,
		.reg = 0x77
	},	/* TMP1 -> SYS1_THRM_RSLT_L */
	{
		.address = 0x10,
		.reg = 0x9a
	},	/* AX10 -> SYS1_THRMALRT0_L */
	{
		.address = 0x14,
		.reg = 0x9c
	},	/* AX11 -> SYS1_THRMALRT1_L */
	{
		.address = 0x18,
		.reg = 0x79
	},	/* TMP2 -> SYS2_THRM_RSLT_L */
	{
		.address = 0x1c,
		.reg = 0x9f
	},	/* AX20 -> SYS2_THRMALRT0_L */
	{
		.address = 0x20,
		.reg = 0xa1
	},	/* AX21 -> SYS2_THRMALRT1_L */
	{
		.address = 0x48,
		.reg = 0x94
	},	/* PEN0 -> SYS0_THRMALRT0_H */
	{
		.address = 0x4c,
		.reg = 0x99
	},	/* PEN1 -> SYS1_THRMALRT1_H */
	{
		.address = 0x50,
		.reg = 0x9e
	},	/* PEN2 -> SYS2_THRMALRT2_H */
};

static int intel_crc_pmic_get_power(struct regmap *regmap,
				    struct pmic_pwr_reg *preg, u64 *value)
{
	int data;

	if (regmap_read(regmap, preg->reg, &data))
		return -EIO;

	*value = (data & PWR_SOURCE_SELECT) && (data & BIT(preg->bit)) ? 1 : 0;
	return 0;
}

static int intel_crc_pmic_update_power(struct regmap *regmap,
				       struct pmic_pwr_reg *preg, bool on)
{
	int data;

	if (regmap_read(regmap, preg->reg, &data))
		return -EIO;

	if (on) {
		data |= PWR_SOURCE_SELECT | BIT(preg->bit);
	} else {
		data &= ~BIT(preg->bit);
		data |= PWR_SOURCE_SELECT;
	}

	if (regmap_write(regmap, preg->reg, data))
		return -EIO;
	return 0;
}

/* Raw temperature value is 10bits: 8bits in reg and 2bits in reg-1 bit0,1 */
static int intel_crc_pmic_get_raw_temp(struct regmap *regmap, int reg)
{
	int temp_l, temp_h;

	if (regmap_read(regmap, reg, &temp_l) ||
	    regmap_read(regmap, reg - 1, &temp_h))
		return -EIO;

	return (temp_l | ((temp_h & 0x3) << 8));
}

static int
intel_crc_pmic_update_aux(struct regmap *regmap, int reg, int raw)
{
	if (regmap_write(regmap, reg, raw) ||
	    regmap_update_bits(regmap, reg - 1, 0x3, raw >> 8))
		return -EIO;

	return 0;
}

static int
intel_crc_pmic_get_policy(struct regmap *regmap, int reg, u64 *value)
{
	int pen;

	if (regmap_read(regmap, reg, &pen))
		return -EIO;
	*value = pen >> 7;
	return 0;
}

static int intel_crc_pmic_update_policy(struct regmap *regmap,
					int reg, int enable)
{
	int alert0;

	/* Update to policy enable bit requires unlocking a0lock */
	if (regmap_read(regmap, PMIC_A0LOCK_REG, &alert0))
		return -EIO;
	if (regmap_update_bits(regmap, PMIC_A0LOCK_REG, 0x01, 0))
		return -EIO;

	if (regmap_update_bits(regmap, reg, 0x80, enable << 7))
		return -EIO;

	/* restore alert0 */
	if (regmap_write(regmap, PMIC_A0LOCK_REG, alert0))
		return -EIO;

	return 0;
}

static struct intel_soc_pmic_opregion_data intel_crc_pmic_opregion_data = {
	.get_power	= intel_crc_pmic_get_power,
	.update_power	= intel_crc_pmic_update_power,
	.get_raw_temp	= intel_crc_pmic_get_raw_temp,
	.update_aux	= intel_crc_pmic_update_aux,
	.get_policy	= intel_crc_pmic_get_policy,
	.update_policy	= intel_crc_pmic_update_policy,
	.pwr_table	= pwr_table,
	.pwr_table_count= ARRAY_SIZE(pwr_table),
	.dptf_table	= dptf_table,
	.dptf_table_count = ARRAY_SIZE(dptf_table),
};

static int intel_crc_pmic_opregion_probe(struct platform_device *pdev)
{
	struct intel_soc_pmic *pmic = dev_get_drvdata(pdev->dev.parent);
	return intel_soc_pmic_install_opregion_handler(&pdev->dev,
			ACPI_HANDLE(pdev->dev.parent), pmic->regmap,
			&intel_crc_pmic_opregion_data);
}

static struct platform_driver intel_crc_pmic_opregion_driver = {
	.probe = intel_crc_pmic_opregion_probe,
	.driver = {
		.name = "crystal_cove_region",
	},
};

static int __init intel_crc_pmic_opregion_driver_init(void)
{
	return platform_driver_register(&intel_crc_pmic_opregion_driver);
}
module_init(intel_crc_pmic_opregion_driver_init);

MODULE_DESCRIPTION("CrystalCove ACPI opration region driver");
MODULE_LICENSE("GPL");
