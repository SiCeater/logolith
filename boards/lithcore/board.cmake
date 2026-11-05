# LithCore : STM32F405RGT6, Cortex-M4F, quartz 8 MHz.

add_compile_options(
    -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
    -ffunction-sections -fdata-sections
)
add_link_options(
    -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
    --specs=nano.specs --specs=nosys.specs
    -nostartfiles # le démarrage est fait par startup_stm32f405xx.s, pas par le crt0 de newlib
    -Wl,--gc-sections -Wl,--print-memory-usage -Wl,--no-warn-rwx-segments
)
add_compile_definitions(STM32F405xx HSE_VALUE=8000000U HSI_VALUE=16000000U)
