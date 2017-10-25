#**********************************************************************
# Драйвер SD карты (module_microsd_low_level_driver).
#**********************************************************************
MICRO_SD_DRIVER_H_FILE		:= $(shell find module_microsd_low_level_driver/ -maxdepth 3 -type f -name "*.h" )
MICRO_SD_DRIVER_CPP_FILE	:= $(shell find module_microsd_low_level_driver/ -maxdepth 3 -type f -name "*.cpp" )
MICRO_SD_DRIVER_DIR			:= $(shell find module_microsd_low_level_driver/ -maxdepth 3 -type d -name "*" )
MICRO_SD_DRIVER_PATH		:= $(addprefix -I, $(MICRO_SD_DRIVER_DIR))
MICRO_SD_DRIVER_OBJ_FILE	:= $(addprefix build/obj/, $(MICRO_SD_DRIVER_CPP_FILE))
MICRO_SD_DRIVER_OBJ_FILE	:= $(patsubst %.cpp, %.o, $(MICRO_SD_DRIVER_OBJ_FILE))

build/obj/module_microsd_low_level_driver/%.o:	module_microsd_low_level_driver/%.cpp
	@echo [CPP] $<
	@mkdir -p $(dir $@)
	@$(CPP) $(CPP_FLAGS) $(MK_INTER_PATH) $(MICRO_SD_DRIVER_PATH) $(USER_CFG_PATH) $(STM32_F2_API_PATH) $(FREE_RTOS_PATH)  $(MICRO_SD_DRIVER_OPTIMIZATION) -c $< -o $@
	
# Добавляем к общим переменным проекта.
PROJECT_PATH			+= $(MICRO_SD_DRIVER_PATH)
PROJECT_OBJ_FILE		+= $(MICRO_SD_DRIVER_OBJ_FILE)