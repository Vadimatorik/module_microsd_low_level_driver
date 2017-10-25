#**********************************************************************
# Драйвер SD карты (micro_sd_driver_by_vadimatorik).
#**********************************************************************
MICRO_SD_DRIVER_H_FILE		:= $(shell find micro_sd_driver_by_vadimatorik/ -maxdepth 3 -type f -name "*.h" )
MICRO_SD_DRIVER_CPP_FILE	:= $(shell find micro_sd_driver_by_vadimatorik/ -maxdepth 3 -type f -name "*.cpp" )
MICRO_SD_DRIVER_DIR		:= $(shell find micro_sd_driver_by_vadimatorik/ -maxdepth 3 -type d -name "*" )
MICRO_SD_DRIVER_PATH		:= $(addprefix -I, $(MICRO_SD_DRIVER_DIR))
MICRO_SD_DRIVER_OBJ_FILE	:= $(addprefix build/obj/, $(MICRO_SD_DRIVER_CPP_FILE))
MICRO_SD_DRIVER_OBJ_FILE	:= $(patsubst %.cpp, %.o, $(MICRO_SD_DRIVER_OBJ_FILE))

build/obj/micro_sd_driver_by_vadimatorik/%.o:	micro_sd_driver_by_vadimatorik/%.cpp
	@echo [CPP] $<
	@mkdir -p $(dir $@)
	@$(CPP) $(CPP_FLAGS) $(MK_INTER_PATH) $(MICRO_SD_DRIVER_PATH) $(USER_CFG_PATH) $(STM32_F2_API_PATH) $(FREE_RTOS_PATH)  $(MICRO_SD_DRIVER_OPTIMIZATION) -c $< -o $@
	
# Добавляем к общим переменным проекта.
PROJECT_PATH			+= $(MICRO_SD_DRIVER_PATH)
PROJECT_OBJ_FILE		+= $(MICRO_SD_DRIVER_OBJ_FILE)