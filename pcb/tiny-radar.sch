EESchema Schematic File Version 2
LIBS:tiny-radar-rescue
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:wbraun_ic_lib
LIBS:tiny-radar-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 4
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Sheet
S 1700 1900 1200 1200
U 58AA0025
F0 "Control" 60
F1 "Control.sch" 60
F2 "RAMP_ADC" I R 2900 2750 60 
F3 "IF_ADC_P" I R 2900 2400 60 
F4 "RAMP_DAC" I R 2900 2950 60 
F5 "RF_ENABLE" I R 2900 2050 60 
F6 "IF_ADC_N" I R 2900 2300 60 
F7 "AGC_DAC" I R 2900 2850 60 
$EndSheet
$Sheet
S 1700 3400 1200 1000
U 58AA00CB
F0 "Analog" 60
F1 "Analog.sch" 60
F2 "RAMP_DAC" I R 2900 3500 60 
F3 "VCO_RAMP" I R 2900 4250 60 
F4 "ADC_RAMP" I R 2900 3700 60 
F5 "GAIN_CNTRL_DAC" I R 2900 3600 60 
F6 "IF_ADC_P" I R 2900 3800 60 
F7 "IF_ADC_N" I R 2900 3900 60 
F8 "IF+" I R 2900 4050 60 
F9 "IF-" I R 2900 4150 60 
$EndSheet
$Sheet
S 4300 1900 1200 1200
U 58AA00E0
F0 "RF" 60
F1 "RF.sch" 60
F2 "IF+" I L 4300 2000 60 
F3 "IF-" I L 4300 2100 60 
F4 "VCO_TUNE" I L 4300 2300 60 
F5 "VCO_EN" I L 4300 2400 60 
$EndSheet
Wire Wire Line
	2900 2950 3000 2950
Wire Wire Line
	3000 2950 3000 3500
Wire Wire Line
	3000 3500 2900 3500
Wire Wire Line
	2900 2850 3100 2850
Wire Wire Line
	3100 2850 3100 3600
Wire Wire Line
	3100 3600 2900 3600
Wire Wire Line
	2900 2750 3200 2750
Wire Wire Line
	3200 2750 3200 3700
Wire Wire Line
	3200 3700 2900 3700
Wire Wire Line
	2900 2400 3300 2400
Wire Wire Line
	3300 2400 3300 3800
Wire Wire Line
	3300 3800 2900 3800
Wire Wire Line
	2900 2300 3400 2300
Wire Wire Line
	3400 2300 3400 3900
Wire Wire Line
	3400 3900 2900 3900
Wire Wire Line
	2900 4050 3800 4050
Wire Wire Line
	3800 4050 3800 2000
Wire Wire Line
	3800 2000 4300 2000
Wire Wire Line
	2900 4150 3950 4150
Wire Wire Line
	3950 4150 3950 2100
Wire Wire Line
	3950 2100 4300 2100
Wire Wire Line
	2900 4250 4050 4250
Wire Wire Line
	4050 4250 4050 2300
Wire Wire Line
	4050 2300 4300 2300
Wire Wire Line
	4300 2400 3550 2400
Wire Wire Line
	3550 2400 3550 2050
Wire Wire Line
	3550 2050 2900 2050
$EndSCHEMATC
