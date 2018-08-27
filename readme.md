
#STM32F107 Project Template

## Переменные среды 

* ARM_TOOLCHAIN_PATH - путь к установленному тулчейну 


## Использование

    git clone --depth 1 git@github.com:bevice/stm32_hal_template.git new_project
    cd new_project 
    git submodule init
    git submodule update
    git remote remove origin
    
    
Или скриптом:
    
    #!/bin/bash
    git clone --depth 1 git@github.com:bevice/stm32_hal_template.git $1
    cd $1
    git remote remove origin
    sed -i '' -e   "s/stm32f7_hal_template/$1/g" CMakeLists.txt
    git submodule init
    git submodule update
    



## Flashing

### st-link tools
     st-flash --reset write firmware.bin 0x8000000


## OpenOCD

	openocd -f /usr/local/share/openocd/scripts/interface/stlink-v2.cfg -f /usr/local/share/openocd/scripts/target/stm32f7x.cfg 


## GDB script (.gdbinit) для OpenOCD:

	target extended localhost:3333
	monitor reset halt
	load
	monitor reset init

