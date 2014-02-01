SECTIONS {
	.text :
	{
		/* Need to copy this code to SRAM */
		_brcmstb_pm_s2_start = .;
		*s2.o
		_brcmstb_pm_s2_end = .;
	}
};
