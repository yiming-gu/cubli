
# Consider dependencies only in project.
set(CMAKE_DEPENDS_IN_PROJECT_ONLY OFF)

# The set of languages for which implicit dependencies are needed:
set(CMAKE_DEPENDS_LANGUAGES
  "ASM"
  )
# The set of files for implicit dependencies of each language:
set(CMAKE_DEPENDS_CHECK_ASM
  "D:/cubli/cubli/code/startup/startup_stm32f103xb.s" "D:/cubli/cubli/code/cmake-build-debug/CMakeFiles/code.elf.dir/startup/startup_stm32f103xb.s.obj"
  )
set(CMAKE_ASM_COMPILER_ID "GNU")

# Preprocessor definitions for this target.
set(CMAKE_TARGET_DEFINITIONS_ASM
  "STM32F103xB"
  "USE_HAL_DRIVER"
  )

# The include file search paths:
set(CMAKE_ASM_TARGET_INCLUDE_PATH
  "../Core/Inc"
  "../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy"
  "../Drivers/STM32F1xx_HAL_Driver/Inc"
  "../Middlewares/Third_Party/FreeRTOS/Source/include"
  "../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS"
  "../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3"
  "../Drivers/CMSIS/Device/ST/STM32F1xx/Include"
  "../Drivers/CMSIS/Include"
  "../Code"
  )

# The set of dependency files which are needed:
set(CMAKE_DEPENDS_DEPENDENCY_FILES
  "D:/cubli/cubli/code/Code/FreeRTOSTasks.c" "CMakeFiles/code.elf.dir/Code/FreeRTOSTasks.c.obj" "gcc" "CMakeFiles/code.elf.dir/Code/FreeRTOSTasks.c.obj.d"
  "D:/cubli/cubli/code/Code/angle.c" "CMakeFiles/code.elf.dir/Code/angle.c.obj" "gcc" "CMakeFiles/code.elf.dir/Code/angle.c.obj.d"
  "D:/cubli/cubli/code/Code/control.c" "CMakeFiles/code.elf.dir/Code/control.c.obj" "gcc" "CMakeFiles/code.elf.dir/Code/control.c.obj.d"
  "D:/cubli/cubli/code/Code/mpu6050.c" "CMakeFiles/code.elf.dir/Code/mpu6050.c.obj" "gcc" "CMakeFiles/code.elf.dir/Code/mpu6050.c.obj.d"
  "D:/cubli/cubli/code/Code/mpuiic.c" "CMakeFiles/code.elf.dir/Code/mpuiic.c.obj" "gcc" "CMakeFiles/code.elf.dir/Code/mpuiic.c.obj.d"
  "D:/cubli/cubli/code/Code/oled.c" "CMakeFiles/code.elf.dir/Code/oled.c.obj" "gcc" "CMakeFiles/code.elf.dir/Code/oled.c.obj.d"
  "D:/cubli/cubli/code/Code/retarget.c" "CMakeFiles/code.elf.dir/Code/retarget.c.obj" "gcc" "CMakeFiles/code.elf.dir/Code/retarget.c.obj.d"
  "D:/cubli/cubli/code/Core/Src/delay.c" "CMakeFiles/code.elf.dir/Core/Src/delay.c.obj" "gcc" "CMakeFiles/code.elf.dir/Core/Src/delay.c.obj.d"
  "D:/cubli/cubli/code/Core/Src/freertos.c" "CMakeFiles/code.elf.dir/Core/Src/freertos.c.obj" "gcc" "CMakeFiles/code.elf.dir/Core/Src/freertos.c.obj.d"
  "D:/cubli/cubli/code/Core/Src/main.c" "CMakeFiles/code.elf.dir/Core/Src/main.c.obj" "gcc" "CMakeFiles/code.elf.dir/Core/Src/main.c.obj.d"
  "D:/cubli/cubli/code/Core/Src/stm32f1xx_hal_msp.c" "CMakeFiles/code.elf.dir/Core/Src/stm32f1xx_hal_msp.c.obj" "gcc" "CMakeFiles/code.elf.dir/Core/Src/stm32f1xx_hal_msp.c.obj.d"
  "D:/cubli/cubli/code/Core/Src/stm32f1xx_it.c" "CMakeFiles/code.elf.dir/Core/Src/stm32f1xx_it.c.obj" "gcc" "CMakeFiles/code.elf.dir/Core/Src/stm32f1xx_it.c.obj.d"
  "D:/cubli/cubli/code/Core/Src/sys.c" "CMakeFiles/code.elf.dir/Core/Src/sys.c.obj" "gcc" "CMakeFiles/code.elf.dir/Core/Src/sys.c.obj.d"
  "D:/cubli/cubli/code/Core/Src/syscalls.c" "CMakeFiles/code.elf.dir/Core/Src/syscalls.c.obj" "gcc" "CMakeFiles/code.elf.dir/Core/Src/syscalls.c.obj.d"
  "D:/cubli/cubli/code/Core/Src/system_stm32f1xx.c" "CMakeFiles/code.elf.dir/Core/Src/system_stm32f1xx.c.obj" "gcc" "CMakeFiles/code.elf.dir/Core/Src/system_stm32f1xx.c.obj.d"
  "D:/cubli/cubli/code/Core/Src/usart.c" "CMakeFiles/code.elf.dir/Core/Src/usart.c.obj" "gcc" "CMakeFiles/code.elf.dir/Core/Src/usart.c.obj.d"
  "D:/cubli/cubli/code/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c" "CMakeFiles/code.elf.dir/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c.obj" "gcc" "CMakeFiles/code.elf.dir/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c.obj.d"
  "D:/cubli/cubli/code/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c" "CMakeFiles/code.elf.dir/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c.obj" "gcc" "CMakeFiles/code.elf.dir/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c.obj.d"
  "D:/cubli/cubli/code/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c" "CMakeFiles/code.elf.dir/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c.obj" "gcc" "CMakeFiles/code.elf.dir/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c.obj.d"
  "D:/cubli/cubli/code/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c" "CMakeFiles/code.elf.dir/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c.obj" "gcc" "CMakeFiles/code.elf.dir/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c.obj.d"
  "D:/cubli/cubli/code/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c" "CMakeFiles/code.elf.dir/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c.obj" "gcc" "CMakeFiles/code.elf.dir/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c.obj.d"
  "D:/cubli/cubli/code/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c" "CMakeFiles/code.elf.dir/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c.obj" "gcc" "CMakeFiles/code.elf.dir/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c.obj.d"
  "D:/cubli/cubli/code/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c" "CMakeFiles/code.elf.dir/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c.obj" "gcc" "CMakeFiles/code.elf.dir/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c.obj.d"
  "D:/cubli/cubli/code/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c" "CMakeFiles/code.elf.dir/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c.obj" "gcc" "CMakeFiles/code.elf.dir/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c.obj.d"
  "D:/cubli/cubli/code/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c" "CMakeFiles/code.elf.dir/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c.obj" "gcc" "CMakeFiles/code.elf.dir/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c.obj.d"
  "D:/cubli/cubli/code/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c" "CMakeFiles/code.elf.dir/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c.obj" "gcc" "CMakeFiles/code.elf.dir/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c.obj.d"
  "D:/cubli/cubli/code/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c" "CMakeFiles/code.elf.dir/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c.obj" "gcc" "CMakeFiles/code.elf.dir/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c.obj.d"
  "D:/cubli/cubli/code/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c" "CMakeFiles/code.elf.dir/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c.obj" "gcc" "CMakeFiles/code.elf.dir/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c.obj.d"
  "D:/cubli/cubli/code/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c" "CMakeFiles/code.elf.dir/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c.obj" "gcc" "CMakeFiles/code.elf.dir/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c.obj.d"
  "D:/cubli/cubli/code/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c" "CMakeFiles/code.elf.dir/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c.obj" "gcc" "CMakeFiles/code.elf.dir/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c.obj.d"
  "D:/cubli/cubli/code/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c" "CMakeFiles/code.elf.dir/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c.obj" "gcc" "CMakeFiles/code.elf.dir/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c.obj.d"
  "D:/cubli/cubli/code/Middlewares/Third_Party/FreeRTOS/Source/croutine.c" "CMakeFiles/code.elf.dir/Middlewares/Third_Party/FreeRTOS/Source/croutine.c.obj" "gcc" "CMakeFiles/code.elf.dir/Middlewares/Third_Party/FreeRTOS/Source/croutine.c.obj.d"
  "D:/cubli/cubli/code/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c" "CMakeFiles/code.elf.dir/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c.obj" "gcc" "CMakeFiles/code.elf.dir/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c.obj.d"
  "D:/cubli/cubli/code/Middlewares/Third_Party/FreeRTOS/Source/list.c" "CMakeFiles/code.elf.dir/Middlewares/Third_Party/FreeRTOS/Source/list.c.obj" "gcc" "CMakeFiles/code.elf.dir/Middlewares/Third_Party/FreeRTOS/Source/list.c.obj.d"
  "D:/cubli/cubli/code/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3/port.c" "CMakeFiles/code.elf.dir/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3/port.c.obj" "gcc" "CMakeFiles/code.elf.dir/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3/port.c.obj.d"
  "D:/cubli/cubli/code/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c" "CMakeFiles/code.elf.dir/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c.obj" "gcc" "CMakeFiles/code.elf.dir/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c.obj.d"
  "D:/cubli/cubli/code/Middlewares/Third_Party/FreeRTOS/Source/queue.c" "CMakeFiles/code.elf.dir/Middlewares/Third_Party/FreeRTOS/Source/queue.c.obj" "gcc" "CMakeFiles/code.elf.dir/Middlewares/Third_Party/FreeRTOS/Source/queue.c.obj.d"
  "D:/cubli/cubli/code/Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c" "CMakeFiles/code.elf.dir/Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c.obj" "gcc" "CMakeFiles/code.elf.dir/Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c.obj.d"
  "D:/cubli/cubli/code/Middlewares/Third_Party/FreeRTOS/Source/tasks.c" "CMakeFiles/code.elf.dir/Middlewares/Third_Party/FreeRTOS/Source/tasks.c.obj" "gcc" "CMakeFiles/code.elf.dir/Middlewares/Third_Party/FreeRTOS/Source/tasks.c.obj.d"
  "D:/cubli/cubli/code/Middlewares/Third_Party/FreeRTOS/Source/timers.c" "CMakeFiles/code.elf.dir/Middlewares/Third_Party/FreeRTOS/Source/timers.c.obj" "gcc" "CMakeFiles/code.elf.dir/Middlewares/Third_Party/FreeRTOS/Source/timers.c.obj.d"
  )

# Targets to which this target links.
set(CMAKE_TARGET_LINKED_INFO_FILES
  )

# Fortran module output directory.
set(CMAKE_Fortran_TARGET_MODULE_DIR "")
