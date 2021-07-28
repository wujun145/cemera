@REM This batch file has been generated by the IAR Embedded Workbench
@REM C-SPY Debugger, as an aid to preparing a command line for running
@REM the cspybat command line utility using the appropriate settings.
@REM
@REM Note that this file is generated every time a new debug session
@REM is initialized, so you may want to move or rename the file before
@REM making changes.
@REM
@REM You can launch cspybat by typing the name of this batch file followed
@REM by the name of the debug file (usually an ELF/DWARF or UBROF file).
@REM
@REM Read about available command line parameters in the C-SPY Debugging
@REM Guide. Hints about additional command line parameters that may be
@REM useful in specific cases:
@REM   --download_only   Downloads a code image without starting a debug
@REM                     session afterwards.
@REM   --silent          Omits the sign-on message.
@REM   --timeout         Limits the maximum allowed execution time.
@REM 


@echo off 

if not "%1" == "" goto debugFile 

@echo on 

"E:\IAR for arm\common\bin\cspybat" -f "G:\00 WorkSpace\04IVD_SW_Development\Modules\Camera\01-SrcCode\STM32_HOST_UVC_Camera-master\HAL_source\stm32F4_discovery_project\EWARM\settings\host_testing.host_testing.general.xcl" --backend -f "G:\00 WorkSpace\04IVD_SW_Development\Modules\Camera\01-SrcCode\STM32_HOST_UVC_Camera-master\HAL_source\stm32F4_discovery_project\EWARM\settings\host_testing.host_testing.driver.xcl" 

@echo off 
goto end 

:debugFile 

@echo on 

"E:\IAR for arm\common\bin\cspybat" -f "G:\00 WorkSpace\04IVD_SW_Development\Modules\Camera\01-SrcCode\STM32_HOST_UVC_Camera-master\HAL_source\stm32F4_discovery_project\EWARM\settings\host_testing.host_testing.general.xcl" "--debug_file=%1" --backend -f "G:\00 WorkSpace\04IVD_SW_Development\Modules\Camera\01-SrcCode\STM32_HOST_UVC_Camera-master\HAL_source\stm32F4_discovery_project\EWARM\settings\host_testing.host_testing.driver.xcl" 

@echo off 
:end