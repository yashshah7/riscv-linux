// SPDX-License-Identifier: GPL-2.0+
/*
 * Synopsys DesignWare APB RTC driver
 *
 * Copyright (C) 2020 SiFive, Inc.
 */

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>

#define RTC_CCVR		0x0
#define RTC_CMR			0x4
#define RTC_CLR			0x8
#define RTC_CCR			0xC
#define    RTC_PSCLR_EN		BIT(4)
#define    RTC_IEN		BIT(0)
#define    RTC_IDIS		0
#define RTC_STAT		0x10
#define    RTC_INT_STAT		BIT(0)
#define RTC_RSTAT		0x14
#define RTC_EOI			0x18
#define RTC_COMP_VERSION	0x1C
#define RTC_CPSR		0x20

#define RTC_VERSION_ID		0x3230372A

struct dwapb_rtc {
	struct rtc_device *rtc;
	void __iomem *reg_base;
	int irq;
};

static int dwapb_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
        unsigned long read_time;
        struct dwapb_rtc *dwapb_rtc = dev_get_drvdata(dev);

	read_time = readl(dwapb_rtc->reg_base + RTC_CCVR);
	rtc_time64_to_tm(read_time, tm);

        return 0;
}

static int dwapb_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	unsigned long set_time;
	struct dwapb_rtc *dwapb_rtc = dev_get_drvdata(dev);

	set_time = rtc_tm_to_time64(tm);
	writel(set_time, (dwapb_rtc->reg_base + RTC_CLR));

        return 0;
}

static int dwapb_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct dwapb_rtc *dwapb_rtc = dev_get_drvdata(dev);

	rtc_time64_to_tm(readl(dwapb_rtc->reg_base + RTC_CMR), &alrm->time);
	alrm->enabled = readl(dwapb_rtc->reg_base + RTC_CCR) & RTC_IEN;

	return 0;
}

static int dwapb_rtc_alarm_irq_enable(struct device *dev, u32 enabled)
{
	struct dwapb_rtc *dwapb_rtc = dev_get_drvdata(dev);

	if (enabled)
		writel(RTC_IEN, dwapb_rtc->reg_base + RTC_CCR);
	else
		writel(RTC_IDIS, dwapb_rtc->reg_base + RTC_CCR);

	return 0;
}

static int dwapb_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
        struct dwapb_rtc *dwapb_rtc = dev_get_drvdata(dev);
        unsigned long alarm_time;

        alarm_time = rtc_tm_to_time64(&alrm->time);

        writel((u32)alarm_time, (dwapb_rtc->reg_base + RTC_CMR));

        dwapb_rtc_alarm_irq_enable(dev, alrm->enabled);

        return 0;
}

static irqreturn_t dwapb_rtc_interrupt(int irq, void *id)
{
        struct dwapb_rtc *dwapb_rtc = (struct dwapb_rtc *)id;
        unsigned int status;

        status = readl(dwapb_rtc->reg_base + RTC_STAT);

        if (!(status & RTC_INT_STAT))
                return IRQ_NONE;

	/* reading EOI register clears the interrupt */
        readl(dwapb_rtc->reg_base + RTC_EOI);

        return IRQ_HANDLED;
}

static int dwapb_rtc_set_prescaler_counter(struct device *dev, unsigned long val)
{
	struct dwapb_rtc *dwapb_rtc = dev_get_drvdata(dev);

	writel(RTC_PSCLR_EN, (dwapb_rtc->reg_base + RTC_CCR));
	writel(val, (dwapb_rtc->reg_base + RTC_CPSR));

	return 0;
}

static const struct rtc_class_ops dwapb_rtc_ops = {
        .set_time         = dwapb_rtc_set_time,
        .read_time        = dwapb_rtc_read_time,
        .read_alarm       = dwapb_rtc_read_alarm,
        .set_alarm        = dwapb_rtc_set_alarm,
        .alarm_irq_enable = dwapb_rtc_alarm_irq_enable,
};


static int dwapb_rtc_probe(struct platform_device *pdev)
{
	struct dwapb_rtc *dwapb_rtc;
	struct resource *res;
	unsigned id;
	int ret;

	dwapb_rtc = devm_kzalloc(&pdev->dev, sizeof(*dwapb_rtc), GFP_KERNEL);
	if (!dwapb_rtc)
		return -ENOMEM;

        dwapb_rtc->irq = platform_get_irq(pdev, 0);
        if (dwapb_rtc->irq < 0) {
                dev_warn(&pdev->dev, "can't get interrupt resource\n");
                return dwapb_rtc->irq;
        }

	platform_set_drvdata(pdev, dwapb_rtc);

	dwapb_rtc->rtc = devm_rtc_allocate_device(&pdev->dev);
        if (IS_ERR(dwapb_rtc->rtc))
	                return PTR_ERR(dwapb_rtc->rtc);

	dwapb_rtc->rtc->ops = &dwapb_rtc_ops;
	dwapb_rtc->rtc->range_max = U32_MAX;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dwapb_rtc->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dwapb_rtc->reg_base))
		ret = PTR_ERR(dwapb_rtc->reg_base);

	ret = devm_request_irq(&pdev->dev, dwapb_rtc->irq, dwapb_rtc_interrupt,
			       0, dev_name(&pdev->dev), dwapb_rtc);
	if (ret) {
		dev_err(&pdev->dev, "request irq failed\n");
		return ret;
	};

	id = readl(dwapb_rtc->reg_base + RTC_COMP_VERSION);

	if(id != RTC_VERSION_ID) {
		dev_err(&pdev->dev, "ID Value got: %x\n", id);
		return -ENODEV;
	}

	/* load RTC with 32768(RTC clock XO) to get 1Hz counter */
	ret = dwapb_rtc_set_prescaler_counter(&pdev->dev, 32768);
	if(ret)
		return ret;

	device_init_wakeup(&pdev->dev, 1);

	dwapb_rtc->rtc = devm_rtc_device_register(&pdev->dev, "rtc-dwapb",
						  &dwapb_rtc_ops, THIS_MODULE);
        if (IS_ERR(dwapb_rtc->rtc)) {
                ret = PTR_ERR(dwapb_rtc->rtc);
                dev_err(&pdev->dev, "Failed to register RTC device -> %d\n", ret);
                return ret;
        }

	return 0;
}

static const struct of_device_id dwapb_rtc_of_match[] = {
	{.compatible = "snps,dw-apb-rtc" },
	{ }
};
MODULE_DEVICE_TABLE(of, dwapb_rtc_of_match);

static struct platform_driver dwapb_rtc_driver = {
	.probe = dwapb_rtc_probe,
	.driver = {
		.name = "rtc-dwapb",
		.of_match_table = dwapb_rtc_of_match,
	},
};
module_platform_driver(dwapb_rtc_driver);

MODULE_AUTHOR("himanshu.jha@sifive.com");
MODULE_DESCRIPTION("Synopsys DesignWare APB RTC Driver");
MODULE_LICENSE("GPL");
