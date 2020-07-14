IMUSRC =    imu/BMI160_driver/bmi160.c \
            imu/bmi160_wrapper.c \
            imu/ahrs.c

IMUINC =    imu \
            imu/BMI160_driver
			
ALLCSRC += $(IMUSRC)
ALLINC += $(IMUINC)
