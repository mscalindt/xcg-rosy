/*
 * isp1301_omap - ISP 1301 USB transceiver, talking to OMAP OTG controller
 *
 * Copyright (C) 2004 Texas Instruments
 * Copyright (C) 2004 David Brownell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb.h>
#include <linux/usb/otg.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>

#include <asm/irq.h>
#include <asm/mach-types.h>

#include <mach/mux.h>

#include <mach/usb.h>

#undef VERBOSE


#define	DRIVER_VERSION	"24 August 2004"
#define	DRIVER_NAME	(isp1301_driver.driver.name)

MODULE_DESCRIPTION("ISP1301 USB OTG Transceiver Driver");
MODULE_LICENSE("GPL");

struct isp1301 {
	struct usb_phy		phy;
	struct i2c_client	*client;
	void			(*i2c_release)(struct device *dev);

	int			irq_type;

	u32			last_otg_ctrl;
	unsigned		working:1;

	struct timer_list	timer;

	/* use keventd context to change the state for us */
	struct work_struct	work;

	unsigned long		todo;
#		define WORK_UPDATE_ISP	0	/* update ISP from OTG */
#		define WORK_UPDATE_OTG	1	/* update OTG from ISP */
#		define WORK_HOST_RESUME	4	/* resume host */
#		define WORK_TIMER	6	/* timer fired */
#		define WORK_STOP	7	/* don't resubmit */
};


/* bits in OTG_CTRL */

#define	OTG_XCEIV_OUTPUTS \
	(OTG_ASESSVLD|OTG_BSESSEND|OTG_BSESSVLD|OTG_VBUSVLD|OTG_ID)
#define	OTG_XCEIV_INPUTS \
	(OTG_PULLDOWN|OTG_PULLUP|OTG_DRV_VBUS|OTG_PD_VBUS|OTG_PU_VBUS|OTG_PU_ID)
#define	OTG_CTRL_BITS \
	(OTG_A_BUSREQ|OTG_A_SETB_HNPEN|OTG_B_BUSREQ|OTG_B_HNPEN|OTG_BUSDROP)
	/* and OTG_PULLUP is sometimes written */

#define	OTG_CTRL_MASK	(OTG_DRIVER_SEL| \
	OTG_XCEIV_OUTPUTS|OTG_XCEIV_INPUTS| \
	OTG_CTRL_BITS)


/*-------------------------------------------------------------------------*/

/* board-specific PM hooks */

#if defined(CONFIG_MACH_OMAP_H2) || defined(CONFIG_MACH_OMAP_H3)

#if	defined(CONFIG_TPS65010) || defined(CONFIG_TPS65010_MODULE)

#include <linux/i2c/tps65010.h>

#else

static inline int tps65010_set_vbus_draw(unsigned mA)
{
	pr_debug("tps65010: draw %d mA (STUB)\n", mA);
	return 0;
}

#endif

static void enable_vbus_draw(struct isp1301 *isp, unsigned mA)
{
	int status = tps65010_set_vbus_draw(mA);
	if (status < 0)
		pr_debug("  VBUS %d mA error %d\n", mA, status);
}

#else

static void enable_vbus_draw(struct isp1301 *isp, unsigned mA)
{
	/* H4 controls this by DIP switch S2.4; no soft control.
	 * ON means the charger is always enabled.  Leave it OFF
	 * unless the OTG port is used only in B-peripheral mode.
	 */
}

#endif

static void enable_vbus_source(struct isp1301 *isp)
{
	/* this board won't supply more than 8mA vbus power.
	 * some boards can switch a 100ma "unit load" (or more).
	 */
}


/* products will deliver OTG messages with LEDs, GUI, etc */
static inline void notresponding(struct isp1301 *isp)
{
	printk(KERN_NOTICE "OTG device not responding.\n");
}


/*-------------------------------------------------------------------------*/

static struct i2c_driver isp1301_driver;

/* smbus apis are used for portability */

static inline u8
isp1301_get_u8(struct isp1301 *isp, u8 reg)
{
	return i2c_smbus_read_byte_data(isp->client, reg + 0);
}

static inline int
isp1301_get_u16(struct isp1301 *isp, u8 reg)
{
	return i2c_smbus_read_word_data(isp->client, reg);
}

static inline int
isp1301_set_bits(struct isp1301 *isp, u8 reg, u8 bits)
{
	return i2c_smbus_write_byte_data(isp->client, reg + 0, bits);
}

static inline int
isp1301_clear_bits(struct isp1301 *isp, u8 reg, u8 bits)
{
	return i2c_smbus_write_byte_data(isp->client, reg + 1, bits);
}

/*-------------------------------------------------------------------------*/

/* identification */
#define	ISP1301_VENDOR_ID		0x00	/* u16 read */
#define	ISP1301_PRODUCT_ID		0x02	/* u16 read */
#define	ISP1301_BCD_DEVICE		0x14	/* u16 read */

#define	I2C_VENDOR_ID_PHILIPS		0x04cc
#define	I2C_PRODUCT_ID_PHILIPS_1301	0x1301

/* operational registers */
#define	ISP1301_MODE_CONTROL_1		0x04	/* u8 read, set, +1 clear */
#	define	MC1_SPEED		(1 << 0)
#	define	MC1_SUSPEND		(1 << 1)
#	define	MC1_DAT_SE0		(1 << 2)
#	define	MC1_TRANSPARENT		(1 << 3)
#	define	MC1_BDIS_ACON_EN	(1 << 4)
#	define	MC1_OE_INT_EN		(1 << 5)
#	define	MC1_UART_EN		(1 << 6)
#	define	MC1_MASK		0x7f
#define	ISP1301_MODE_CONTROL_2		0x12	/* u8 read, set, +1 clear */
#	define	MC2_GLOBAL_PWR_DN	(1 << 0)
#	define	MC2_SPD_SUSP_CTRL	(1 << 1)
#	define	MC2_BI_DI		(1 << 2)
#	define	MC2_TRANSP_BDIR0	(1 << 3)
#	define	MC2_TRANSP_BDIR1	(1 << 4)
#	define	MC2_AUDIO_EN		(1 << 5)
#	define	MC2_PSW_EN		(1 << 6)
#	define	MC2_EN2V7		(1 << 7)
#define	ISP1301_OTG_CONTROL_1		0x06	/* u8 read, set, +1 clear */
#	define	OTG1_DP_PULLUP		(1 << 0)
#	define	OTG1_DM_PULLUP		(1 << 1)
#	define	OTG1_DP_PULLDOWN	(1 << 2)
#	define	OTG1_DM_PULLDOWN	(1 << 3)
#	define	OTG1_ID_PULLDOWN	(1 << 4)
#	define	OTG1_VBUS_DRV		(1 << 5)
#	define	OTG1_VBUS_DISCHRG	(1 << 6)
#	define	OTG1_VBUS_CHRG		(1 << 7)
#define	ISP1301_OTG_STATUS		0x10	/* u8 readonly */
#	define	OTG_B_SESS_END		(1 << 6)
#	define	OTG_B_SESS_VLD		(1 << 7)

#define	ISP1301_INTERRUPT_SOURCE	0x08	/* u8 read */
#define	ISP1301_INTERRUPT_LATCH		0x0A	/* u8 read, set, +1 clear */

#define	ISP1301_INTERRUPT_FALLING	0x0C	/* u8 read, set, +1 clear */
#define	ISP1301_INTERRUPT_RISING	0x0E	/* u8 read, set, +1 clear */

/* same bitfields in all interrupt registers */
#	define	INTR_VBUS_VLD		(1 << 0)
#	define	INTR_SESS_VLD		(1 << 1)
#	define	INTR_DP_HI		(1 << 2)
#	define	INTR_ID_GND		(1 << 3)
#	define	INTR_DM_HI		(1 << 4)
#	define	INTR_ID_FLOAT		(1 << 5)
#	define	INTR_BDIS_ACON		(1 << 6)
#	define	INTR_CR_INT		(1 << 7)

/*-------------------------------------------------------------------------*/

static inline const char *state_name(struct isp1301 *isp)
{
	return usb_otg_state_string(isp->phy.state);
}

/*-------------------------------------------------------------------------*/

/* NOTE:  some of this ISP1301 setup is specific to H2 boards;
 * not everything is guarded by board-specific checks, or even using
 * omap_usb_config data to deduce MC1_DAT_SE0 and MC2_BI_DI.
 *
 * ALSO:  this currently doesn't use ISP1301 low-power modes
 * while OTG is running.
 */

static void power_down(struct isp1301 *isp)
{
	isp->phy.state = OTG_STATE_UNDEFINED;

	// isp1301_set_bits(isp, ISP1301_MODE_CONTROL_2, MC2_GLOBAL_PWR_DN);
	isp1301_set_bits(isp, ISP1301_MODE_CONTROL_1, MC1_SUSPEND);

	isp1301_clear_bits(isp, ISP1301_OTG_CONTROL_1, OTG1_ID_PULLDOWN);
	isp1301_clear_bits(isp, ISP1301_MODE_CONTROL_1, MC1_DAT_SE0);
}

static void power_up(struct isp1301 *isp)
{
	// isp1301_clear_bits(isp, ISP1301_MODE_CONTROL_2, MC2_GLOBAL_PWR_DN);
	isp1301_clear_bits(isp, ISP1301_MODE_CONTROL_1, MC1_SUSPEND);

	/* do this only when cpu is driving transceiver,
	 * so host won't see a low speed device...
	 */
	isp1301_set_bits(isp, ISP1301_MODE_CONTROL_1, MC1_DAT_SE0);
}

#define	NO_HOST_SUSPEND

static int host_suspend(struct isp1301 *isp)
{
#ifdef	NO_HOST_SUSPEND
	return 0;
#else
	struct device	*dev;

	if (!isp->phy.otg->host)
		return -ENODEV;

	/* Currently ASSUMES only the OTG port matters;
	 * other ports could be active...
	 */
	dev = isp->phy.otg->host->controller;
	return dev->driver->suspend(dev, 3, 0);
#endif
}

static int host_resume(struct isp1301 *isp)
{
#ifdef	NO_HOST_SUSPEND
	return 0;
#else
	struct device	*dev;

	if (!isp->phy.otg->host)
		return -ENODEV;

	dev = isp->phy.otg->host->controller;
	return dev->driver->resume(dev, 0);
#endif
}

static int gadget_suspend(struct isp1301 *isp)
{
	isp->phy.otg->gadget->b_hnp_enable = 0;
	isp->phy.otg->gadget->a_hnp_support = 0;
	isp->phy.otg->gadget->a_alt_hnp_support = 0;
	return usb_gadget_vbus_disconnect(isp->phy.otg->gadget);
}

/*-------------------------------------------------------------------------*/

#define	TIMER_MINUTES	10
#define	TIMER_JIFFIES	(TIMER_MINUTES * 60 * HZ)

/* Almost all our I2C messaging comes from a work queue's task context.
 * NOTE: guaranteeing certain response times might mean we shouldn't
 * share keventd's work queue; a realtime task might be safest.
 */
static void isp1301_defer_work(struct isp1301 *isp, int work)
{
	int status;

	if (isp && !test_and_set_bit(work, &isp->todo)) {
		(void) get_device(&isp->client->dev);
		status = schedule_work(&isp->work);
		if (!status && !isp->working)
			dev_vdbg(&isp->client->dev,
				"work item %d may be lost\n", work);
	}
}

/* called from irq handlers */
static void a_idle(struct isp1301 *isp, const char *tag)
{
	u32 l;

	if (isp->phy.state == OTG_STATE_A_IDLE)
		return;

	isp->phy.otg->default_a = 1;
	if (isp->phy.otg->host) {
		isp->phy.otg->host->is_b_host = 0;
		host_suspend(isp);
	}
	if (isp->phy.otg->gadget) {
		isp->phy.otg->gadget->is_a_peripheral = 1;
		gadget_suspend(isp);
	}
	isp->phy.state = OTG_STATE_A_IDLE;
	l = omap_readl(OTG_CTRL) & OTG_XCEIV_OUTPUTS;
	omap_writel(l, OTG_CTRL);
	isp->last_otg_ctrl = l;
	pr_debug("  --> %s/%s\n", state_name(isp), tag);
}

/* called from irq handlers */
static void b_idle(struct isp1301 *isp, const char *tag)
{
	u32 l;

	if (isp->phy.state == OTG_STATE_B_IDLE)
		return;

	isp->phy.otg->default_a = 0;
	if (isp->phy.otg->host) {
		isp->phy.otg->host->is_b_host = 1;
		host_suspend(isp);
	}
	if (isp->phy.otg->gadget) {
		isp->phy.otg->gadget->is_a_peripheral = 0;
		gadget_suspend(isp);
	}
	isp->phy.state = OTG_STATE_B_IDLE;
	l = omap_readl(OTG_CTRL) & OTG_XCEIV_OUTPUTS;
	omap_writel(l, OTG_CTRL);
	isp->last_otg_ctrl = l;
	pr_debug("  --> %s/%s\n", state_name(isp), tag);
}

static void
dump_regs(struct isp1301 *isp, const char *label)
{
	u8	ctrl = isp1301_get_u8(isp, ISP1301_OTG_CONTROL_1);
	u8	status = isp1301_get_u8(isp, ISP1301_OTG_STATUS);
	u8	src = isp1301_get_u8(isp, ISP1301_INTERRUPT_SOURCE);

	pr_debug("otg: %06x, %s %s, otg/%02x stat/%02x.%02x\n",
		omap_readl(OTG_CTRL), label, state_name(isp),
		ctrl, status, src);
	/* mode control and irq enables don't change much */
}

/*-------------------------------------------------------------------------*/

static void b_peripheral(struct isp1301 *isp)
{
	u32 l;

	l = omap_readl(OTG_CTRL) & OTG_XCEIV_OUTPUTS;
	omap_writel(l, OTG_CTRL);

	usb_gadget_vbus_connect(isp->phy.otg->gadget);

	enable_vbus_draw(isp, 100);
	/* UDC driver just set OTG_BSESSVLD */
	isp1301_set_bits(isp, ISP1301_OTG_CONTROL_1, OTG1_DP_PULLUP);
	isp1301_clear_bits(isp, ISP1301_OTG_CONTROL_1, OTG1_DP_PULLDOWN);
	isp->phy.state = OTG_STATE_B_PERIPHERAL;
	pr_debug("  --> b_peripheral\n");
	dump_regs(isp, "2periph");
}

static void isp_update_otg(struct isp1301 *isp, u8 stat)
{
	struct usb_otg		*otg = isp->phy.otg;
	u8			isp_stat, isp_bstat;
	enum usb_otg_state	state = isp->phy.state;

	if (stat & INTR_BDIS_ACON)
		pr_debug("OTG:  BDIS_ACON, %s\n", state_name(isp));

	/* start certain state transitions right away */
	isp_stat = isp1301_get_u8(isp, ISP1301_INTERRUPT_SOURCE);
	if (isp_stat & INTR_ID_GND) {
		if (otg->default_a) {
			switch (state) {
			case OTG_STATE_B_IDLE:
				a_idle(isp, "idle");
				/* FALLTHROUGH */
			case OTG_STATE_A_IDLE:
				enable_vbus_source(isp);
				/* FALLTHROUGH */
			case OTG_STATE_A_WAIT_VRISE:
				/* we skip over OTG_STATE_A_WAIT_BCON, since
				 * the HC will transition to A_HOST (or
				 * A_SUSPEND!) without our noticing except
				 * when HNP is used.
				 */
				if (isp_stat & INTR_VBUS_VLD)
					isp->phy.state = OTG_STATE_A_HOST;
				break;
			case OTG_STATE_A_WAIT_VFALL:
				if (!(isp_stat & INTR_SESS_VLD))
					a_idle(isp, "vfell");
				break;
			default:
				if (!(isp_stat & INTR_VBUS_VLD))
					isp->phy.state = OTG_STATE_A_VBUS_ERR;
				break;
			}
			isp_bstat = isp1301_get_u8(isp, ISP1301_OTG_STATUS);
		} else {
			switch (state) {
			case OTG_STATE_B_PERIPHERAL:
			case OTG_STATE_B_HOST:
			case OTG_STATE_B_WAIT_ACON:
				usb_gadget_vbus_disconnect(otg->gadget);
				break;
			default:
				break;
			}
			if (state != OTG_STATE_A_IDLE)
				a_idle(isp, "id");
			if (otg->host && state == OTG_STATE_A_IDLE)
				isp1301_defer_work(isp, WORK_HOST_RESUME);
			isp_bstat = 0;
		}
	} else {
		u32 l;

		/* if user unplugged mini-A end of cable,
		 * don't bypass A_WAIT_VFALL.
		 */
		if (otg->default_a) {
			switch (state) {
			default:
				isp->phy.state = OTG_STATE_A_WAIT_VFALL;
				break;
			case OTG_STATE_A_WAIT_VFALL:
				state = OTG_STATE_A_IDLE;
				/* hub_wq may take a while to notice and
				 * handle this disconnect, so don't go
				 * to B_IDLE quite yet.
				 */
				break;
			case OTG_STATE_A_IDLE:
				host_suspend(isp);
				isp1301_clear_bits(isp, ISP1301_MODE_CONTROL_1,
						MC1_BDIS_ACON_EN);
				isp->phy.state = OTG_STATE_B_IDLE;
				l = omap_readl(OTG_CTRL) & OTG_CTRL_MASK;
				l &= ~OTG_CTRL_BITS;
				omap_writel(l, OTG_CTRL);
				break;
			case OTG_STATE_B_IDLE:
				break;
			}
		}
		isp_bstat = isp1301_get_u8(isp, ISP1301_OTG_STATUS);

		switch (isp->phy.state) {
		case OTG_STATE_B_PERIPHERAL:
		case OTG_STATE_B_WAIT_ACON:
		case OTG_STATE_B_HOST:
			if (likely(isp_bstat & OTG_B_SESS_VLD))
				break;
			enable_vbus_draw(isp, 0);
			/* UDC driver will clear OTG_BSESSVLD */
			isp1301_set_bits(isp, ISP1301_OTG_CONTROL_1,
						OTG1_DP_PULLDOWN);
			isp1301_clear_bits(isp, ISP1301_OTG_CONTROL_1,
						OTG1_DP_PULLUP);
			dump_regs(isp, __func__);
			/* FALLTHROUGH */
		case OTG_STATE_B_SRP_INIT:
			b_idle(isp, __func__);
			l = omap_readl(OTG_CTRL) & OTG_XCEIV_OUTPUTS;
			omap_writel(l, OTG_CTRL);
			/* FALLTHROUGH */
		case OTG_STATE_B_IDLE:
			if (otg->gadget && (isp_bstat & OTG_B_SESS_VLD)) {
				b_peripheral(isp);
			} else if (!(isp_stat & (INTR_VBUS_VLD|INTR_SESS_VLD)))
				isp_bstat |= OTG_B_SESS_END;
			break;
		case OTG_STATE_A_WAIT_VFALL:
			break;
		default:
			pr_debug("otg: unsupported b-device %s\n",
				state_name(isp));
			break;
		}
	}

	if (state != isp->phy.state)
		pr_debug("  isp, %s -> %s\n",
				usb_otg_state_string(state), state_name(isp));

	dump_regs(isp, "isp1301->otg");
}

/*-------------------------------------------------------------------------*/

static u8 isp1301_clear_latch(struct isp1301 *isp)
{
	u8 latch = isp1301_get_u8(isp, ISP1301_INTERRUPT_LATCH);
	isp1301_clear_bits(isp, ISP1301_INTERRUPT_LATCH, latch);
	return latch;
}

static void
isp1301_work(struct work_struct *work)
{
	struct isp1301	*isp = container_of(work, struct isp1301, work);
	int		stop;

	/* implicit lock:  we're the only task using this device */
	isp->working = 1;
	do {
		stop = test_bit(WORK_STOP, &isp->todo);

		/* transfer state from isp1301 to otg engine */
		if (test_and_clear_bit(WORK_UPDATE_OTG, &isp->todo)) {
			u8		stat = isp1301_clear_latch(isp);

			isp_update_otg(isp, stat);
			put_device(&isp->client->dev);
		}

		if (test_and_clear_bit(WORK_HOST_RESUME, &isp->todo)) {
			u32	otg_ctrl;

			/*
			 * skip A_WAIT_VRISE; hc transitions invisibly
			 * skip A_WAIT_BCON; same.
			 */
			switch (isp->phy.state) {
			case OTG_STATE_A_WAIT_BCON:
			case OTG_STATE_A_WAIT_VRISE:
				isp->phy.state = OTG_STATE_A_HOST;
				pr_debug("  --> a_host\n");
				otg_ctrl = omap_readl(OTG_CTRL);
				otg_ctrl |= OTG_A_BUSREQ;
				otg_ctrl &= ~(OTG_BUSDROP|OTG_B_BUSREQ)
						& OTG_CTRL_MASK;
				omap_writel(otg_ctrl, OTG_CTRL);
				break;
			case OTG_STATE_B_WAIT_ACON:
				isp->phy.state = OTG_STATE_B_HOST;
				pr_debug("  --> b_host (acon)\n");
				break;
			case OTG_STATE_B_HOST:
			case OTG_STATE_B_IDLE:
			case OTG_STATE_A_IDLE:
				break;
			default:
				pr_debug("  host resume in %s\n",
						state_name(isp));
			}
			host_resume(isp);
			// mdelay(10);
			put_device(&isp->client->dev);
		}

		if (test_and_clear_bit(WORK_TIMER, &isp->todo)) {
#ifdef	VERBOSE
			dump_regs(isp, "timer");
			if (!stop)
				mod_timer(&isp->timer, jiffies + TIMER_JIFFIES);
#endif
			put_device(&isp->client->dev);
		}

		if (isp->todo)
			dev_vdbg(&isp->client->dev,
				"work done, todo = 0x%lx\n",
				isp->todo);
		if (stop) {
			dev_dbg(&isp->client->dev, "stop\n");
			break;
		}
	} while (isp->todo);
	isp->working = 0;
}

static irqreturn_t isp1301_irq(int irq, void *isp)
{
	isp1301_defer_work(isp, WORK_UPDATE_OTG);
	return IRQ_HANDLED;
}

static void isp1301_timer(unsigned long _isp)
{
	isp1301_defer_work((void *)_isp, WORK_TIMER);
}

/*-------------------------------------------------------------------------*/

static void isp1301_release(struct device *dev)
{
	struct isp1301	*isp;

	isp = dev_get_drvdata(dev);

	/* FIXME -- not with a "new style" driver, it doesn't!! */

	/* ugly -- i2c hijacks our memory hook to wait_for_completion() */
	if (isp->i2c_release)
		isp->i2c_release(dev);
	kfree(isp->phy.otg);
	kfree (isp);
}

static struct isp1301 *the_transceiver;

static int isp1301_remove(struct i2c_client *i2c)
{
	struct isp1301	*isp;

	isp = i2c_get_clientdata(i2c);

	isp1301_clear_bits(isp, ISP1301_INTERRUPT_FALLING, ~0);
	isp1301_clear_bits(isp, ISP1301_INTERRUPT_RISING, ~0);
	free_irq(i2c->irq, isp);
	if (machine_is_omap_h2())
		gpio_free(2);

	isp->timer.data = 0;
	set_bit(WORK_STOP, &isp->todo);
	del_timer_sync(&isp->timer);
	flush_work(&isp->work);

	put_device(&i2c->dev);
	the_transceiver = NULL;

	return 0;
}

/* add or disable the host device+driver */
static int
isp1301_set_host(struct usb_otg *otg, struct usb_bus *host)
{
	struct isp1301	*isp = container_of(otg->phy, struct isp1301, phy);

	if (isp != the_transceiver)
		return -ENODEV;

	if (!host) {
		omap_writew(0, OTG_IRQ_EN);
		power_down(isp);
		otg->host = NULL;
		return 0;
	}

#if !IS_ENABLED(CONFIG_USB_OMAP)
	// FIXME update its refcount
	otg->host = host;

	power_up(isp);

	if (machine_is_omap_h2())
		isp1301_set_bits(isp, ISP1301_MODE_CONTROL_1, MC1_DAT_SE0);

	dev_info(&isp->client->dev, "A-Host sessions ok\n");
	isp1301_set_bits(isp, ISP1301_INTERRUPT_RISING,
		INTR_ID_GND);
	isp1301_set_bits(isp, ISP1301_INTERRUPT_FALLING,
		INTR_ID_GND);

	/* If this has a Mini-AB connector, this mode is highly
	 * nonstandard ... but can be handy for testing, especially with
	 * the Mini-A end of an OTG cable.  (Or something nonstandard
	 * like MiniB-to-StandardB, maybe built with a gender mender.)
	 */
	isp1301_set_bits(isp, ISP1301_OTG_CONTROL_1, OTG1_VBUS_DRV);

	dump_regs(isp, __func__);

	return 0;

#else
	dev_dbg(&isp->client->dev, "host sessions not allowed\n");
	return -EINVAL;
#endif

}

static int
isp1301_set_peripheral(struct usb_otg *otg, struct usb_gadget *gadget)
{
	struct isp1301	*isp = container_of(otg->phy, struct isp1301, phy);

	if (isp != the_transceiver)
		return -ENODEV;

	if (!gadget) {
		omap_writew(0, OTG_IRQ_EN);
		if (!otg->default_a)
			enable_vbus_draw(isp, 0);
		usb_gadget_vbus_disconnect(otg->gadget);
		otg->gadget = NULL;
		power_down(isp);
		return 0;
	}

#if	!defined(CONFIG_USB_OHCI_HCD) && !defined(CONFIG_USB_OHCI_HCD_MODULE)
	otg->gadget = gadget;
	// FIXME update its refcount

	{
		u32 l;

		l = omap_readl(OTG_CTRL) & OTG_CTRL_MASK;
		l &= ~(OTG_XCEIV_OUTPUTS|OTG_CTRL_BITS);
		l |= OTG_ID;
		omap_writel(l, OTG_CTRL);
	}

	power_up(isp);
	isp->phy.state = OTG_STATE_B_IDLE;

	if (machine_is_omap_h2() || machine_is_omap_h3())
		isp1301_set_bits(isp, ISP1301_MODE_CONTROL_1, MC1_DAT_SE0);

	isp1301_set_bits(isp, ISP1301_INTERRUPT_RISING,
		INTR_SESS_VLD);
	isp1301_set_bits(isp, ISP1301_INTERRUPT_FALLING,
		INTR_VBUS_VLD);
	dev_info(&isp->client->dev, "B-Peripheral sessions ok\n");
	dump_regs(isp, __func__);

	/* If this has a Mini-AB connector, this mode is highly
	 * nonstandard ... but can be handy for testing, so long
	 * as you don't plug a Mini-A cable into the jack.
	 */
	if (isp1301_get_u8(isp, ISP1301_INTERRUPT_SOURCE) & INTR_VBUS_VLD)
		b_peripheral(isp);

	return 0;

#else
	dev_dbg(&isp->client->dev, "peripheral sessions not allowed\n");
	return -EINVAL;
#endif
}


/*-------------------------------------------------------------------------*/

static int
isp1301_set_power(struct usb_phy *dev, unsigned mA)
{
	if (!the_transceiver)
		return -ENODEV;
	if (dev->state == OTG_STATE_B_PERIPHERAL)
		enable_vbus_draw(the_transceiver, mA);
	return 0;
}

static int
isp1301_start_srp(struct usb_otg *otg)
{
	struct isp1301	*isp = container_of(otg->phy, struct isp1301, phy);
	u32		otg_ctrl;

	if (isp != the_transceiver || isp->phy.state != OTG_STATE_B_IDLE)
		return -ENODEV;

	otg_ctrl = omap_readl(OTG_CTRL);
	if (!(otg_ctrl & OTG_BSESSEND))
		return -EINVAL;

	otg_ctrl |= OTG_B_BUSREQ;
	otg_ctrl &= ~OTG_A_BUSREQ & OTG_CTRL_MASK;
	omap_writel(otg_ctrl, OTG_CTRL);
	isp->phy.state = OTG_STATE_B_SRP_INIT;

	pr_debug("otg: SRP, %s ... %06x\n", state_name(isp),
			omap_readl(OTG_CTRL));
	return 0;
}

static int
isp1301_start_hnp(struct usb_otg *otg)
{
	/* srp-only */
	return -EINVAL;
}

/*-------------------------------------------------------------------------*/

static int
isp1301_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	int			status;
	struct isp1301		*isp;

	if (the_transceiver)
		return 0;

	isp = kzalloc(sizeof *isp, GFP_KERNEL);
	if (!isp)
		return 0;

	isp->phy.otg = kzalloc(sizeof *isp->phy.otg, GFP_KERNEL);
	if (!isp->phy.otg) {
		kfree(isp);
		return 0;
	}

	INIT_WORK(&isp->work, isp1301_work);
	init_timer(&isp->timer);
	isp->timer.function = isp1301_timer;
	isp->timer.data = (unsigned long) isp;

	i2c_set_clientdata(i2c, isp);
	isp->client = i2c;

	/* verify the chip (shouldn't be necessary) */
	status = isp1301_get_u16(isp, ISP1301_VENDOR_ID);
	if (status != I2C_VENDOR_ID_PHILIPS) {
		dev_dbg(&i2c->dev, "not philips id: %d\n", status);
		goto fail;
	}
	status = isp1301_get_u16(isp, ISP1301_PRODUCT_ID);
	if (status != I2C_PRODUCT_ID_PHILIPS_1301) {
		dev_dbg(&i2c->dev, "not isp1301, %d\n", status);
		goto fail;
	}
	isp->i2c_release = i2c->dev.release;
	i2c->dev.release = isp1301_release;

	/* initial development used chiprev 2.00 */
	status = i2c_smbus_read_word_data(i2c, ISP1301_BCD_DEVICE);
	dev_info(&i2c->dev, "chiprev %x.%02x, driver " DRIVER_VERSION "\n",
		status >> 8, status & 0xff);

	/* make like power-on reset */
	isp1301_clear_bits(isp, ISP1301_MODE_CONTROL_1, MC1_MASK);

	isp1301_set_bits(isp, ISP1301_MODE_CONTROL_2, MC2_BI_DI);
	isp1301_clear_bits(isp, ISP1301_MODE_CONTROL_2, ~MC2_BI_DI);

	isp1301_set_bits(isp, ISP1301_OTG_CONTROL_1,
				OTG1_DM_PULLDOWN | OTG1_DP_PULLDOWN);
	isp1301_clear_bits(isp, ISP1301_OTG_CONTROL_1,
				~(OTG1_DM_PULLDOWN | OTG1_DP_PULLDOWN));

	isp1301_clear_bits(isp, ISP1301_INTERRUPT_LATCH, ~0);
	isp1301_clear_bits(isp, ISP1301_INTERRUPT_FALLING, ~0);
	isp1301_clear_bits(isp, ISP1301_INTERRUPT_RISING, ~0);

	if (machine_is_omap_h2()) {
		/* full speed signaling by default */
		isp1301_set_bits(isp, ISP1301_MODE_CONTROL_1,
			MC1_SPEED);
		isp1301_set_bits(isp, ISP1301_MODE_CONTROL_2,
			MC2_SPD_SUSP_CTRL);

		/* IRQ wired at M14 */
		omap_cfg_reg(M14_1510_GPIO2);
		if (gpio_request(2, "isp1301") == 0)
			gpio_direction_input(2);
		isp->irq_type = IRQF_TRIGGER_FALLING;
	}

	status = request_irq(i2c->irq, isp1301_irq,
			isp->irq_type, DRIVER_NAME, isp);
	if (status < 0) {
		dev_dbg(&i2c->dev, "can't get IRQ %d, err %d\n",
				i2c->irq, status);
		goto fail;
	}

	isp->phy.dev = &i2c->dev;
	isp->phy.label = DRIVER_NAME;
	isp->phy.set_power = isp1301_set_power,

	isp->phy.otg->phy = &isp->phy;
	isp->phy.otg->set_host = isp1301_set_host,
	isp->phy.otg->set_peripheral = isp1301_set_peripheral,
	isp->phy.otg->start_srp = isp1301_start_srp,
	isp->phy.otg->start_hnp = isp1301_start_hnp,

	enable_vbus_draw(isp, 0);
	power_down(isp);
	the_transceiver = isp;

	dump_regs(isp, __func__);

#ifdef	VERBOSE
	mod_timer(&isp->timer, jiffies + TIMER_JIFFIES);
	dev_dbg(&i2c->dev, "scheduled timer, %d min\n", TIMER_MINUTES);
#endif

	status = usb_add_phy(&isp->phy, USB_PHY_TYPE_USB2);
	if (status < 0)
		dev_err(&i2c->dev, "can't register transceiver, %d\n",
			status);

	return 0;

fail:
	kfree(isp->phy.otg);
	kfree(isp);
	return -ENODEV;
}

static const struct i2c_device_id isp1301_id[] = {
	{ "isp1301_omap", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, isp1301_id);

static struct i2c_driver isp1301_driver = {
	.driver = {
		.name	= "isp1301_omap",
	},
	.probe		= isp1301_probe,
	.remove		= isp1301_remove,
	.id_table	= isp1301_id,
};

/*-------------------------------------------------------------------------*/

static int __init isp_init(void)
{
	return i2c_add_driver(&isp1301_driver);
}
subsys_initcall(isp_init);

static void __exit isp_exit(void)
{
	if (the_transceiver)
		usb_remove_phy(&the_transceiver->phy);
	i2c_del_driver(&isp1301_driver);
}
module_exit(isp_exit);

