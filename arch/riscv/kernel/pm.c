#include <linux/init.h>
#include <linux/suspend.h>
#include <linux/sched/hotplug.h>
#include <linux/cpu.h>

static int sifive_pm_enter(suspend_state_t state)
{
        pr_info("%s:state:%d\n", __func__, state);

	/* Do not disable external interrupt to wake up cpu after suspend */
	csr_clear(CSR_IE, IE_TIE);
	wait_for_software_interrupt();
	csr_set(CSR_IE, IE_TIE);

	return 0;
}

static const struct platform_suspend_ops sifive_pm_ops = {
        .valid = suspend_valid_only_mem,
        .enter = sifive_pm_enter,
};

static int __init sifive_pm_init(void)
{
        pr_info("Enter %s\n", __func__);
        suspend_set_ops(&sifive_pm_ops);
        return 0;
}
late_initcall(sifive_pm_init);
