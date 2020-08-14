IMUSRC =    $(COMMONDIR)/imu/BMI160_driver/bmi160.c \
            $(COMMONDIR)/imu/bmi160_wrapper.c \
            $(COMMONDIR)/imu/ahrs.c

IMUINC =    $(COMMONDIR)/imu \
            $(COMMONDIR)/imu/BMI160_driver
			
ALLCSRC += $(IMUSRC)
ALLINC += $(IMUINC)
