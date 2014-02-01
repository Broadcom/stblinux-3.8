#ifndef __LINUX_IRQCHIP_BRCMSTB_L2_H_
#define __LINUX_IRQCHIP_BRCMSTB_L2_H_

struct device_node;

int brcmstb_l2_intc_of_init(struct device_node *np,
				struct device_node *parent);

#endif /* _LINUX_IRQCHIP_BRCMSTB_L2_H_ */
