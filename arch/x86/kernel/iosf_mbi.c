/*
 * IOSF-SB MailBox Interface Driver
 * Copyright (c) 2013, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 *
 * The IOSF-SB is a fabric bus available on Atom based SOC's that uses a
 * mailbox interface (MBI) to communicate with mutiple devices. This
 * driver implements access to this interface for those platforms that can
 * enumerate the device using PCI.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/pci.h>
#include <linux/debugfs.h>
#include <linux/capability.h>
#ifdef CONFIG_ACPI
#include <linux/acpi.h>
#include <linux/platform_device.h>
#endif

#include <asm/iosf_mbi.h>

#define PCI_DEVICE_ID_BAYTRAIL		0x0F00
#define PCI_DEVICE_ID_BRASWELL		0x2280
#define PCI_DEVICE_ID_QUARK_X1000	0x0958

#define ACPI_DEVICE_ID_BAYTRAIL		"INT33BD"
#define ACPI_OPREGION_ID		0x87

static DEFINE_SPINLOCK(iosf_mbi_lock);

static inline u32 iosf_mbi_form_mcr(u8 op, u8 port, u8 offset)
{
	return (op << 24) | (port << 16) | (offset << 8) | MBI_ENABLE;
}

static struct pci_dev *mbi_pdev;	/* one mbi device */

static int iosf_mbi_pci_read_mdr(u32 mcrx, u32 mcr, u32 *mdr)
{
	int result;

	if (!mbi_pdev)
		return -ENODEV;

	if (mcrx) {
		result = pci_write_config_dword(mbi_pdev, MBI_MCRX_OFFSET,
						mcrx);
		if (result < 0)
			goto fail_read;
	}

	result = pci_write_config_dword(mbi_pdev, MBI_MCR_OFFSET, mcr);
	if (result < 0)
		goto fail_read;

	result = pci_read_config_dword(mbi_pdev, MBI_MDR_OFFSET, mdr);
	if (result < 0)
		goto fail_read;

	return 0;

fail_read:
	dev_err(&mbi_pdev->dev, "PCI config access failed with %d\n", result);
	return result;
}

static int iosf_mbi_pci_write_mdr(u32 mcrx, u32 mcr, u32 mdr)
{
	int result;

	if (!mbi_pdev)
		return -ENODEV;

	result = pci_write_config_dword(mbi_pdev, MBI_MDR_OFFSET, mdr);
	if (result < 0)
		goto fail_write;

	if (mcrx) {
		result = pci_write_config_dword(mbi_pdev, MBI_MCRX_OFFSET,
						mcrx);
		if (result < 0)
			goto fail_write;
	}

	result = pci_write_config_dword(mbi_pdev, MBI_MCR_OFFSET, mcr);
	if (result < 0)
		goto fail_write;

	return 0;

fail_write:
	dev_err(&mbi_pdev->dev, "PCI config access failed with %d\n", result);
	return result;
}

int iosf_mbi_read(u8 port, u8 opcode, u32 offset, u32 *mdr)
{
	u32 mcr, mcrx;
	unsigned long flags;
	int ret;

	/*Access to the GFX unit is handled by GPU code */
	if (port == BT_MBI_UNIT_GFX) {
		WARN_ON(1);
		return -EPERM;
	}

	mcr = iosf_mbi_form_mcr(opcode, port, offset & MBI_MASK_LO);
	mcrx = offset & MBI_MASK_HI;

	spin_lock_irqsave(&iosf_mbi_lock, flags);
	ret = iosf_mbi_pci_read_mdr(mcrx, mcr, mdr);
	spin_unlock_irqrestore(&iosf_mbi_lock, flags);

	return ret;
}
EXPORT_SYMBOL(iosf_mbi_read);

int iosf_mbi_write(u8 port, u8 opcode, u32 offset, u32 mdr)
{
	u32 mcr, mcrx;
	unsigned long flags;
	int ret;

	/*Access to the GFX unit is handled by GPU code */
	if (port == BT_MBI_UNIT_GFX) {
		WARN_ON(1);
		return -EPERM;
	}

	mcr = iosf_mbi_form_mcr(opcode, port, offset & MBI_MASK_LO);
	mcrx = offset & MBI_MASK_HI;

	spin_lock_irqsave(&iosf_mbi_lock, flags);
	ret = iosf_mbi_pci_write_mdr(mcrx, mcr, mdr);
	spin_unlock_irqrestore(&iosf_mbi_lock, flags);

	return ret;
}
EXPORT_SYMBOL(iosf_mbi_write);

int iosf_mbi_modify(u8 port, u8 opcode, u32 offset, u32 mdr, u32 mask)
{
	u32 mcr, mcrx;
	u32 value;
	unsigned long flags;
	int ret;

	/*Access to the GFX unit is handled by GPU code */
	if (port == BT_MBI_UNIT_GFX) {
		WARN_ON(1);
		return -EPERM;
	}

	mcr = iosf_mbi_form_mcr(opcode, port, offset & MBI_MASK_LO);
	mcrx = offset & MBI_MASK_HI;

	spin_lock_irqsave(&iosf_mbi_lock, flags);

	/* Read current mdr value */
	ret = iosf_mbi_pci_read_mdr(mcrx, mcr & MBI_RD_MASK, &value);
	if (ret < 0) {
		spin_unlock_irqrestore(&iosf_mbi_lock, flags);
		return ret;
	}

	/* Apply mask */
	value &= ~mask;
	mdr &= mask;
	value |= mdr;

	/* Write back */
	ret = iosf_mbi_pci_write_mdr(mcrx, mcr | MBI_WR_MASK, value);

	spin_unlock_irqrestore(&iosf_mbi_lock, flags);

	return ret;
}
EXPORT_SYMBOL(iosf_mbi_modify);

bool iosf_mbi_available(void)
{
	/* Mbi isn't hot-pluggable. No remove routine is provided */
	return mbi_pdev;
}
EXPORT_SYMBOL(iosf_mbi_available);

#ifdef CONFIG_IOSF_MBI_DEBUG
static u32	dbg_mdr;
static u32	dbg_mcr;
static u32	dbg_mcrx;

static int mcr_get(void *data, u64 *val)
{
	*val = *(u32 *)data;
	return 0;
}

static int mcr_set(void *data, u64 val)
{
	u8 command = ((u32)val & 0xFF000000) >> 24,
	   port	   = ((u32)val & 0x00FF0000) >> 16,
	   offset  = ((u32)val & 0x0000FF00) >> 8;
	int err;

	*(u32 *)data = val;

	if (!capable(CAP_SYS_RAWIO))
		return -EACCES;

	if (command & 1u)
		err = iosf_mbi_write(port,
			       command,
			       dbg_mcrx | offset,
			       dbg_mdr);
	else
		err = iosf_mbi_read(port,
			      command,
			      dbg_mcrx | offset,
			      &dbg_mdr);

	return err;
}
DEFINE_SIMPLE_ATTRIBUTE(iosf_mcr_fops, mcr_get, mcr_set , "%llx\n");

static struct dentry *iosf_dbg;

static void iosf_sideband_debug_init(void)
{
	struct dentry *d;

	iosf_dbg = debugfs_create_dir("iosf_sb", NULL);
	if (IS_ERR_OR_NULL(iosf_dbg))
		return;

	/* mdr */
	d = debugfs_create_x32("mdr", 0660, iosf_dbg, &dbg_mdr);
	if (IS_ERR_OR_NULL(d))
		goto cleanup;

	/* mcrx */
	debugfs_create_x32("mcrx", 0660, iosf_dbg, &dbg_mcrx);
	if (IS_ERR_OR_NULL(d))
		goto cleanup;

	/* mcr - initiates mailbox tranaction */
	debugfs_create_file("mcr", 0660, iosf_dbg, &dbg_mcr, &iosf_mcr_fops);
	if (IS_ERR_OR_NULL(d))
		goto cleanup;

	return;

cleanup:
	debugfs_remove_recursive(d);
}

static void iosf_debugfs_init(void)
{
	iosf_sideband_debug_init();
}

static void iosf_debugfs_remove(void)
{
	debugfs_remove_recursive(iosf_dbg);
}
#else
static inline void iosf_debugfs_init(void) { }
static inline void iosf_debugfs_remove(void) { }
#endif /* CONFIG_IOSF_MBI_DEBUG */

static int iosf_mbi_probe(struct pci_dev *pdev,
			  const struct pci_device_id *unused)
{	int ret;

	ret = pci_enable_device(pdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "error: could not enable device\n");
		return ret;
	}

	mbi_pdev = pci_dev_get(pdev);
	return 0;
}

static const struct pci_device_id iosf_mbi_pci_ids[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_BAYTRAIL) },
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_BRASWELL) },
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_QUARK_X1000) },
	{ 0, },
};
MODULE_DEVICE_TABLE(pci, iosf_mbi_pci_ids);

static struct pci_driver iosf_mbi_pci_driver = {
	.name		= "iosf_mbi_pci",
	.probe		= iosf_mbi_probe,
	.id_table	= iosf_mbi_pci_ids,
};

#ifdef CONFIG_ACPI

struct mutex iosf_mbi_op_lock;
static union {
	char bytes [0x30];
	struct {
		u32 port;
		u32 reg;
		u32 data;
		u32 mask;
		u32 be;
		u32 op;
	} regs;
} iosf_mbi_op;

static acpi_status iosf_mbi_acpi_opregion_handler(u32 function, acpi_physical_address address,
				u32 bits, u64 *value64,
				void *handler_context, void *region_context) {
	int result=0;

	if (bits != 32 || !value64)
		return AE_BAD_PARAMETER;

	if (address<0 || address>0x30)
		return AE_BAD_PARAMETER;

	mutex_lock(&iosf_mbi_op_lock);

	if (function == ACPI_READ) {
		if (address==8 && iosf_mbi_op.regs.op==0) {
			result=iosf_mbi_read(iosf_mbi_op.regs.port,iosf_mbi_op.regs.op,iosf_mbi_op.regs.reg,&iosf_mbi_op.regs.data);
			iosf_mbi_op.regs.op=0xffffffff;
		}
		*value64 = *((u32*)&(iosf_mbi_op.bytes[address]));
	}
	else {
		*((u32*)&(iosf_mbi_op.bytes[address]))=*value64;
		if (address==8 && iosf_mbi_op.regs.op==1) {
			result=iosf_mbi_write(iosf_mbi_op.regs.port,iosf_mbi_op.regs.op,iosf_mbi_op.regs.reg,iosf_mbi_op.regs.data);
			iosf_mbi_op.regs.op=0xffffffff;
		}
		if (address==12 && iosf_mbi_op.regs.op==2) {
			result=iosf_mbi_modify(iosf_mbi_op.regs.port,iosf_mbi_op.regs.op,iosf_mbi_op.regs.reg,iosf_mbi_op.regs.data,iosf_mbi_op.regs.mask);
			iosf_mbi_op.regs.op=0xffffffff;
		}
	}
	mutex_unlock(&iosf_mbi_op_lock);

	return result?AE_ERROR:AE_OK;
}

static int iosf_mbi_acpi_probe(struct platform_device *pdev)
{
	acpi_handle *handle;
	acpi_status status;

	if (!(handle=ACPI_HANDLE(&pdev->dev)))
	{
		dev_err(&pdev->dev,"Device as no acpi handle");
		return -ENODEV;
	}

	if (ACPI_COMPANION(&pdev->dev)->dep_unmet)
		return -EPROBE_DEFER;

	if (!iosf_mbi_available())
		return -EPROBE_DEFER;

	mutex_init(&iosf_mbi_op_lock);

	iosf_mbi_op.regs.op=0xffffffff;

	status = acpi_install_address_space_handler(handle,
			ACPI_OPREGION_ID,
			iosf_mbi_acpi_opregion_handler,
			NULL, NULL);

	if (status != AE_OK)
		return -ENODEV;

	acpi_walk_dep_device_list(handle);

	return 0;
}

static const struct acpi_device_id iosf_mbi_acpi_ids[] = {
	{ ACPI_DEVICE_ID_BAYTRAIL },
	{ },
};
MODULE_DEVICE_TABLE(acpi,iosf_mbi_acpi_ids);

static struct platform_driver iosf_mbi_acpi_driver = {
	.probe		= iosf_mbi_acpi_probe,
	.driver		= {
		.name	= "iosf_mbi_acpi",
		.acpi_match_table = ACPI_PTR( iosf_mbi_acpi_ids ),
	},
};
#endif

static int __init iosf_mbi_init(void)
{
	iosf_debugfs_init();

#ifdef CONFIG_ACPI
	platform_driver_register(&iosf_mbi_acpi_driver);
#endif
	return pci_register_driver(&iosf_mbi_pci_driver);
}

static void __exit iosf_mbi_exit(void)
{
	iosf_debugfs_remove();

	pci_unregister_driver(&iosf_mbi_pci_driver);
	if (mbi_pdev) {
		pci_dev_put(mbi_pdev);
		mbi_pdev = NULL;
	}
}

module_init(iosf_mbi_init);
module_exit(iosf_mbi_exit);

MODULE_AUTHOR("David E. Box <david.e.box@linux.intel.com>");
MODULE_DESCRIPTION("IOSF Mailbox Interface accessor");
MODULE_LICENSE("GPL v2");
