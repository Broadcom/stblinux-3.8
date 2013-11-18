# Link ethsw before genet for the ethsw PHY driver to be
# probe before BCMGENET
obj-$(CONFIG_BCMGENET) += ethsw.o
obj-$(CONFIG_BCMGENET) += genet.o
genet-objs := bcmgenet.o bcmmii.o
