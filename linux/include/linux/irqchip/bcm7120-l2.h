#ifndef __LINUX_IRQCHIP_BCM7120_L2_H_
#define __LINUX_IRQCHUP_BCM7120_L2_H_

struct device_node;

#ifdef CONFIG_BCM7120_L2_IRQ
int bcm7120_l2_intc_of_init(struct device_node *np,
				struct device_node *parent);
#else
static inline int bcm7120_l2_intc_of_init(struct device_node *np,
					  struct device_node *parent)
{
	return 0;
}
#endif /* CONFIG_BCM7120_L2_IRQ */

#endif /* __LINUX_IRQCHUP_BCM7120_L2_H_ */
