# generic gpio support: platform drivers, dedicated expander chips, etc
obj-$(CONFIG_HIMS_BRLCELL)		+= brlcell.o
obj-$(CONFIG_HIMS_BRLKEY)		+= brlkey.o
obj-$(CONFIG_HIMS_H532B_LCD) += h532b_lcd.o
obj-$(CONFIG_HIMS_BUZZER) += h532b_buzzer.o
obj-$(CONFIG_HIMS_VIBRATOR) += h532b_vibrator.o
obj-$(CONFIG_HIMS_DEVICE_ENABLE_CONTROLLER) += h532b_gpio_en.o
obj-$(CONFIG_HIMS_SECONDARY_RTC) += h532b_ds3232.o
obj-$(CONFIG_HIMS_IRDA) += h532b_irda.o