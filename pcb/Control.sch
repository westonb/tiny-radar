EESchema Schematic File Version 2
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
Sheet 2 4
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L STM32F072CBU U202
U 1 1 58AACEC6
P 4300 3600
F 0 "U202" H 3650 5050 60  0000 C CNN
F 1 "STM32F072CBU" H 3950 2150 60  0000 C CNN
F 2 "Housings_DFN_QFN:QFN-48-1EP_7x7mm_Pitch0.5mm" H 4300 3600 60  0001 C CNN
F 3 "" H 4300 3600 60  0000 C CNN
	1    4300 3600
	1    0    0    -1  
$EndComp
$Comp
L USB_OTG P201
U 1 1 58AAD76F
P 850 1350
F 0 "P201" H 1175 1225 50  0000 C CNN
F 1 "USB_OTG" H 850 1550 50  0000 C CNN
F 2 "Connect:USB_Micro-B_10103594-0001LF" V 800 1250 50  0001 C CNN
F 3 "" V 800 1250 50  0000 C CNN
	1    850  1350
	0    -1   1    0   
$EndComp
Text Label 5300 2800 0    60   ~ 0
USB_N
Text Label 5300 2900 0    60   ~ 0
USB_P
$Comp
L R R201
U 1 1 58AAD9BA
P 1500 1450
F 0 "R201" V 1580 1450 50  0000 C CNN
F 1 "100k" V 1500 1450 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 1430 1450 50  0001 C CNN
F 3 "" H 1500 1450 50  0000 C CNN
	1    1500 1450
	0    1    1    0   
$EndComp
$Comp
L FILTER FB202
U 1 1 58AAE829
P 1600 950
F 0 "FB202" H 1600 1100 50  0000 C CNN
F 1 "FILTER" H 1600 850 50  0000 C CNN
F 2 "Capacitors_SMD:C_0805" H 1600 950 50  0001 C CNN
F 3 "" H 1600 950 50  0000 C CNN
	1    1600 950 
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR01
U 1 1 58AAF10E
P 1700 1500
F 0 "#PWR01" H 1700 1250 50  0001 C CNN
F 1 "GND" H 1700 1350 50  0000 C CNN
F 2 "" H 1700 1500 50  0000 C CNN
F 3 "" H 1700 1500 50  0000 C CNN
	1    1700 1500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR02
U 1 1 58AAF136
P 1250 1700
F 0 "#PWR02" H 1250 1450 50  0001 C CNN
F 1 "GND" H 1250 1550 50  0000 C CNN
F 2 "" H 1250 1700 50  0000 C CNN
F 3 "" H 1250 1700 50  0000 C CNN
	1    1250 1700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR03
U 1 1 58AAF152
P 750 1800
F 0 "#PWR03" H 750 1550 50  0001 C CNN
F 1 "GND" H 750 1650 50  0000 C CNN
F 2 "" H 750 1800 50  0000 C CNN
F 3 "" H 750 1800 50  0000 C CNN
	1    750  1800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR04
U 1 1 58AAF17E
P 3300 5000
F 0 "#PWR04" H 3300 4750 50  0001 C CNN
F 1 "GND" H 3300 4850 50  0000 C CNN
F 2 "" H 3300 5000 50  0000 C CNN
F 3 "" H 3300 5000 50  0000 C CNN
	1    3300 5000
	1    0    0    -1  
$EndComp
Text Label 1250 1350 0    60   ~ 0
USB_P
Text Label 1250 1250 0    60   ~ 0
USB_N
$Comp
L C C209
U 1 1 58AAF2A1
P 3150 3800
F 0 "C209" H 3050 3900 50  0000 L CNN
F 1 "0.1u" V 3100 3550 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 3188 3650 50  0001 C CNN
F 3 "" H 3150 3800 50  0000 C CNN
	1    3150 3800
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR05
U 1 1 58AAF334
P 2900 3850
F 0 "#PWR05" H 2900 3600 50  0001 C CNN
F 1 "GND" H 2900 3700 50  0000 C CNN
F 2 "" H 2900 3850 50  0000 C CNN
F 3 "" H 2900 3850 50  0000 C CNN
	1    2900 3850
	1    0    0    -1  
$EndComp
$Comp
L C C207
U 1 1 58AAF4C2
P 2200 1100
F 0 "C207" H 2225 1200 50  0000 L CNN
F 1 "4.7u" H 2225 1000 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 2238 950 50  0001 C CNN
F 3 "" H 2200 1100 50  0000 C CNN
	1    2200 1100
	1    0    0    -1  
$EndComp
Text Label 3300 2700 2    60   ~ 0
DAC
Text Label 3300 2400 2    60   ~ 0
ADC_1
Text Label 3300 2500 2    60   ~ 0
ADC_2
$Comp
L GND #PWR06
U 1 1 58AAF650
P 2200 1350
F 0 "#PWR06" H 2200 1100 50  0001 C CNN
F 1 "GND" H 2200 1200 50  0000 C CNN
F 2 "" H 2200 1350 50  0000 C CNN
F 3 "" H 2200 1350 50  0000 C CNN
	1    2200 1350
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR07
U 1 1 58AAF670
P 2050 750
F 0 "#PWR07" H 2050 600 50  0001 C CNN
F 1 "+5V" H 2050 890 50  0000 C CNN
F 2 "" H 2050 750 50  0000 C CNN
F 3 "" H 2050 750 50  0000 C CNN
	1    2050 750 
	1    0    0    -1  
$EndComp
$Comp
L AP2114H U201
U 1 1 58AAF711
P 1450 2700
F 0 "U201" H 1250 2950 60  0000 C CNN
F 1 "AP2114H" H 1400 2450 60  0000 C CNN
F 2 "wbraun_smd:SOT-223" H 1450 2700 60  0001 C CNN
F 3 "" H 1450 2700 60  0000 C CNN
	1    1450 2700
	1    0    0    -1  
$EndComp
$Comp
L C C201
U 1 1 58AAF839
P 750 2750
F 0 "C201" H 775 2850 50  0000 L CNN
F 1 "0.47u" H 775 2650 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 788 2600 50  0001 C CNN
F 3 "" H 750 2750 50  0000 C CNN
	1    750  2750
	1    0    0    -1  
$EndComp
$Comp
L C C206
U 1 1 58AAF8AE
P 2150 2750
F 0 "C206" H 2175 2850 50  0000 L CNN
F 1 "0.47u" H 2175 2650 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 2188 2600 50  0001 C CNN
F 3 "" H 2150 2750 50  0000 C CNN
	1    2150 2750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR08
U 1 1 58AAFA33
P 750 3000
F 0 "#PWR08" H 750 2750 50  0001 C CNN
F 1 "GND" H 750 2850 50  0000 C CNN
F 2 "" H 750 3000 50  0000 C CNN
F 3 "" H 750 3000 50  0000 C CNN
	1    750  3000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR09
U 1 1 58AAFA59
P 2150 3000
F 0 "#PWR09" H 2150 2750 50  0001 C CNN
F 1 "GND" H 2150 2850 50  0000 C CNN
F 2 "" H 2150 3000 50  0000 C CNN
F 3 "" H 2150 3000 50  0000 C CNN
	1    2150 3000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR010
U 1 1 58AAFAE1
P 950 3000
F 0 "#PWR010" H 950 2750 50  0001 C CNN
F 1 "GND" H 950 2850 50  0000 C CNN
F 2 "" H 950 3000 50  0000 C CNN
F 3 "" H 950 3000 50  0000 C CNN
	1    950  3000
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR011
U 1 1 58AAFBA5
P 750 2500
F 0 "#PWR011" H 750 2350 50  0001 C CNN
F 1 "+5V" H 750 2640 50  0000 C CNN
F 2 "" H 750 2500 50  0000 C CNN
F 3 "" H 750 2500 50  0000 C CNN
	1    750  2500
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR012
U 1 1 58AAFBCB
P 2150 2500
F 0 "#PWR012" H 2150 2350 50  0001 C CNN
F 1 "+3V3" H 2150 2640 50  0000 C CNN
F 2 "" H 2150 2500 50  0000 C CNN
F 3 "" H 2150 2500 50  0000 C CNN
	1    2150 2500
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR013
U 1 1 58AAFC5E
P 3350 4000
F 0 "#PWR013" H 3350 3850 50  0001 C CNN
F 1 "+3V3" V 3400 4000 50  0000 C CNN
F 2 "" H 3350 4000 50  0000 C CNN
F 3 "" H 3350 4000 50  0000 C CNN
	1    3350 4000
	0    -1   -1   0   
$EndComp
$Comp
L +3V3 #PWR014
U 1 1 58AAFC84
P 3350 4200
F 0 "#PWR014" H 3350 4050 50  0001 C CNN
F 1 "+3V3" V 3250 4300 50  0000 C CNN
F 2 "" H 3350 4200 50  0000 C CNN
F 3 "" H 3350 4200 50  0000 C CNN
	1    3350 4200
	0    -1   -1   0   
$EndComp
Text Label 3350 4100 2    60   ~ 0
VDDA2
$Comp
L FILTER FB201
U 1 1 58AAFE47
P 1550 3800
F 0 "FB201" H 1550 3950 50  0000 C CNN
F 1 "FILTER" H 1550 3700 50  0000 C CNN
F 2 "Capacitors_SMD:C_0805" H 1550 3800 50  0001 C CNN
F 3 "" H 1550 3800 50  0000 C CNN
	1    1550 3800
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR015
U 1 1 58AAFFD3
P 1150 3700
F 0 "#PWR015" H 1150 3550 50  0001 C CNN
F 1 "+3V3" H 1150 3840 50  0000 C CNN
F 2 "" H 1150 3700 50  0000 C CNN
F 3 "" H 1150 3700 50  0000 C CNN
	1    1150 3700
	1    0    0    -1  
$EndComp
$Comp
L C C205
U 1 1 58AAFFFB
P 2000 4000
F 0 "C205" H 2025 4100 50  0000 L CNN
F 1 "4.7u" H 2025 3900 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 2038 3850 50  0001 C CNN
F 3 "" H 2000 4000 50  0000 C CNN
	1    2000 4000
	1    0    0    -1  
$EndComp
$Comp
L C C208
U 1 1 58AB0058
P 2250 4000
F 0 "C208" H 2275 4100 50  0000 L CNN
F 1 "0.47u" H 2275 3900 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 2288 3850 50  0001 C CNN
F 3 "" H 2250 4000 50  0000 C CNN
	1    2250 4000
	1    0    0    -1  
$EndComp
$Comp
L C C202
U 1 1 58AB0095
P 1300 4900
F 0 "C202" H 1325 5000 50  0000 L CNN
F 1 "0.47u" H 1325 4800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 1338 4750 50  0001 C CNN
F 3 "" H 1300 4900 50  0000 C CNN
	1    1300 4900
	1    0    0    -1  
$EndComp
$Comp
L C C203
U 1 1 58AB014C
P 1600 4900
F 0 "C203" H 1625 5000 50  0000 L CNN
F 1 "0.47u" H 1625 4800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 1638 4750 50  0001 C CNN
F 3 "" H 1600 4900 50  0000 C CNN
	1    1600 4900
	1    0    0    -1  
$EndComp
$Comp
L C C204
U 1 1 58AB0189
P 1900 4900
F 0 "C204" H 1925 5000 50  0000 L CNN
F 1 "0.47u" H 1925 4800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 1938 4750 50  0001 C CNN
F 3 "" H 1900 4900 50  0000 C CNN
	1    1900 4900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR016
U 1 1 58AB01F9
P 1300 5150
F 0 "#PWR016" H 1300 4900 50  0001 C CNN
F 1 "GND" H 1300 5000 50  0000 C CNN
F 2 "" H 1300 5150 50  0000 C CNN
F 3 "" H 1300 5150 50  0000 C CNN
	1    1300 5150
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR017
U 1 1 58AB022B
P 1300 4650
F 0 "#PWR017" H 1300 4500 50  0001 C CNN
F 1 "+3V3" H 1300 4790 50  0000 C CNN
F 2 "" H 1300 4650 50  0000 C CNN
F 3 "" H 1300 4650 50  0000 C CNN
	1    1300 4650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR018
U 1 1 58AB025D
P 2000 4250
F 0 "#PWR018" H 2000 4000 50  0001 C CNN
F 1 "GND" H 2000 4100 50  0000 C CNN
F 2 "" H 2000 4250 50  0000 C CNN
F 3 "" H 2000 4250 50  0000 C CNN
	1    2000 4250
	1    0    0    -1  
$EndComp
Text Label 2300 3750 0    60   ~ 0
VDDA2
Text Notes 1450 5250 0    60   ~ 0
VDD Decoupling
Text Label 3300 2300 2    60   ~ 0
RF_ENABLE
Text Label 6900 3400 2    60   ~ 0
ADC_1
Text Label 6900 3500 2    60   ~ 0
ADC_2
Text Label 6900 3600 2    60   ~ 0
DAC
Text Label 6900 3700 2    60   ~ 0
RF_ENABLE
Text HLabel 7300 3400 2    60   Input ~ 0
RAMP_ADC
Text HLabel 7300 3500 2    60   Input ~ 0
IF_ADC
Text HLabel 7300 3600 2    60   Input ~ 0
RAMP_DAC
Text HLabel 7300 3700 2    60   Input ~ 0
RF_ENABLE
$Comp
L LED D201
U 1 1 58AB0D2F
P 6800 5150
F 0 "D201" H 6800 5250 50  0000 C CNN
F 1 "LED" H 6800 5050 50  0000 C CNN
F 2 "LEDs:LED_0603" H 6800 5150 50  0001 C CNN
F 3 "" H 6800 5150 50  0000 C CNN
	1    6800 5150
	0    -1   -1   0   
$EndComp
$Comp
L LED D202
U 1 1 58AB0DB2
P 7200 5150
F 0 "D202" H 7200 5250 50  0000 C CNN
F 1 "LED" H 7200 5050 50  0000 C CNN
F 2 "LEDs:LED_0603" H 7200 5150 50  0001 C CNN
F 3 "" H 7200 5150 50  0000 C CNN
	1    7200 5150
	0    -1   -1   0   
$EndComp
$Comp
L LED D203
U 1 1 58AB0E5A
P 7600 5150
F 0 "D203" H 7600 5250 50  0000 C CNN
F 1 "LED" H 7600 5050 50  0000 C CNN
F 2 "LEDs:LED_0603" H 7600 5150 50  0001 C CNN
F 3 "" H 7600 5150 50  0000 C CNN
	1    7600 5150
	0    -1   -1   0   
$EndComp
$Comp
L R R202
U 1 1 58AB0F5D
P 6800 4750
F 0 "R202" V 6880 4750 50  0000 C CNN
F 1 "1k" V 6800 4750 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 6730 4750 50  0001 C CNN
F 3 "" H 6800 4750 50  0000 C CNN
	1    6800 4750
	1    0    0    -1  
$EndComp
$Comp
L R R203
U 1 1 58AB101C
P 7200 4750
F 0 "R203" V 7280 4750 50  0000 C CNN
F 1 "1k" V 7200 4750 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 7130 4750 50  0001 C CNN
F 3 "" H 7200 4750 50  0000 C CNN
	1    7200 4750
	1    0    0    -1  
$EndComp
$Comp
L R R204
U 1 1 58AB107B
P 7600 4750
F 0 "R204" V 7680 4750 50  0000 C CNN
F 1 "1k" V 7600 4750 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 7530 4750 50  0001 C CNN
F 3 "" H 7600 4750 50  0000 C CNN
	1    7600 4750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR019
U 1 1 58AB10CA
P 6800 5400
F 0 "#PWR019" H 6800 5150 50  0001 C CNN
F 1 "GND" H 6800 5250 50  0000 C CNN
F 2 "" H 6800 5400 50  0000 C CNN
F 3 "" H 6800 5400 50  0000 C CNN
	1    6800 5400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR020
U 1 1 58AB1108
P 7200 5400
F 0 "#PWR020" H 7200 5150 50  0001 C CNN
F 1 "GND" H 7200 5250 50  0000 C CNN
F 2 "" H 7200 5400 50  0000 C CNN
F 3 "" H 7200 5400 50  0000 C CNN
	1    7200 5400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR021
U 1 1 58AB1146
P 7600 5400
F 0 "#PWR021" H 7600 5150 50  0001 C CNN
F 1 "GND" H 7600 5250 50  0000 C CNN
F 2 "" H 7600 5400 50  0000 C CNN
F 3 "" H 7600 5400 50  0000 C CNN
	1    7600 5400
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR022
U 1 1 58AB1677
P 6700 4500
F 0 "#PWR022" H 6700 4350 50  0001 C CNN
F 1 "+3V3" H 6700 4640 50  0000 C CNN
F 2 "" H 6700 4500 50  0000 C CNN
F 3 "" H 6700 4500 50  0000 C CNN
	1    6700 4500
	0    -1   -1   0   
$EndComp
Text Label 7100 4300 2    60   ~ 0
STATUS_1
Text Label 7500 4100 2    60   ~ 0
STATUS_2
Text Label 5400 4000 0    60   ~ 0
SCL
Text Label 5400 4100 0    60   ~ 0
SDA
$Comp
L CONN_01X04 P202
U 1 1 58AB1909
P 5000 5850
F 0 "P202" H 5000 6100 50  0000 C CNN
F 1 "CONN_01X04" V 5100 5850 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04_Pitch2.54mm" H 5000 5850 50  0001 C CNN
F 3 "" H 5000 5850 50  0000 C CNN
	1    5000 5850
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR023
U 1 1 58AB19C8
P 4700 5600
F 0 "#PWR023" H 4700 5450 50  0001 C CNN
F 1 "+3V3" H 4700 5740 50  0000 C CNN
F 2 "" H 4700 5600 50  0000 C CNN
F 3 "" H 4700 5600 50  0000 C CNN
	1    4700 5600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR024
U 1 1 58AB1A08
P 4400 5900
F 0 "#PWR024" H 4400 5650 50  0001 C CNN
F 1 "GND" H 4400 5750 50  0000 C CNN
F 2 "" H 4400 5900 50  0000 C CNN
F 3 "" H 4400 5900 50  0000 C CNN
	1    4400 5900
	0    1    1    0   
$EndComp
Text Label 4700 6000 2    60   ~ 0
SWDIO
Text Label 4700 5800 2    60   ~ 0
SWCLK
Wire Wire Line
	5200 2800 5300 2800
Wire Wire Line
	5200 2900 5300 2900
Wire Wire Line
	1250 1150 1150 1150
Wire Wire Line
	1250 950  1250 1150
Wire Wire Line
	1150 1250 1250 1250
Wire Wire Line
	1150 1350 1250 1350
Wire Wire Line
	1150 1550 1250 1550
Wire Wire Line
	1250 1550 1250 1700
Wire Wire Line
	1150 1450 1350 1450
Wire Wire Line
	1950 950  2200 950 
Wire Wire Line
	2050 750  2050 950 
Wire Wire Line
	3400 4600 3300 4600
Wire Wire Line
	3300 4600 3300 5000
Wire Wire Line
	3400 4700 3300 4700
Connection ~ 3300 4700
Wire Wire Line
	3400 4800 3300 4800
Connection ~ 3300 4800
Wire Wire Line
	3400 4900 3300 4900
Connection ~ 3300 4900
Wire Wire Line
	1650 1450 1700 1450
Wire Wire Line
	1700 1450 1700 1500
Wire Wire Line
	750  1750 750  1800
Wire Wire Line
	2900 3700 2900 3850
Wire Wire Line
	2900 3800 3000 3800
Wire Wire Line
	3400 3800 3300 3800
Wire Wire Line
	3400 3700 2900 3700
Connection ~ 2900 3800
Wire Wire Line
	3400 2700 3300 2700
Wire Wire Line
	3400 2400 3300 2400
Wire Wire Line
	3400 2500 3300 2500
Connection ~ 2050 950 
Wire Wire Line
	2200 1250 2200 1350
Wire Wire Line
	750  2600 1000 2600
Wire Wire Line
	1900 2600 2150 2600
Wire Wire Line
	1900 2800 1950 2800
Wire Wire Line
	1950 2800 1950 2600
Connection ~ 1950 2600
Wire Wire Line
	2150 2900 2150 3000
Wire Wire Line
	750  2900 750  3000
Wire Wire Line
	1000 2800 950  2800
Wire Wire Line
	950  2800 950  3000
Wire Wire Line
	750  2600 750  2500
Wire Wire Line
	2150 2600 2150 2500
Wire Wire Line
	3400 4100 3350 4100
Wire Wire Line
	3350 4200 3400 4200
Wire Wire Line
	3400 4200 3400 4400
Connection ~ 3400 4300
Wire Wire Line
	3400 4000 3350 4000
Wire Wire Line
	1150 3700 1150 3800
Wire Wire Line
	1150 3800 1200 3800
Wire Wire Line
	1900 3800 2250 3800
Wire Wire Line
	2000 3800 2000 3850
Wire Wire Line
	2250 3750 2250 3850
Connection ~ 2000 3800
Wire Wire Line
	2250 4150 2250 4200
Wire Wire Line
	2250 4200 2000 4200
Wire Wire Line
	2000 4150 2000 4250
Connection ~ 2000 4200
Wire Wire Line
	1300 4650 1300 4750
Wire Wire Line
	1300 4750 1900 4750
Connection ~ 1600 4750
Wire Wire Line
	1300 5050 1900 5050
Connection ~ 1600 5050
Wire Wire Line
	1300 5050 1300 5150
Wire Wire Line
	2250 3750 2300 3750
Connection ~ 2250 3800
Wire Wire Line
	3400 2300 3300 2300
Wire Wire Line
	6900 3400 7300 3400
Wire Wire Line
	6900 3500 7300 3500
Wire Wire Line
	6900 3600 7300 3600
Wire Wire Line
	6900 3700 7300 3700
Wire Wire Line
	7600 5300 7600 5400
Wire Wire Line
	7200 5300 7200 5400
Wire Wire Line
	6800 5300 6800 5400
Wire Wire Line
	6800 4900 6800 5000
Wire Wire Line
	7200 4900 7200 5000
Wire Wire Line
	7600 4900 7600 5000
Wire Wire Line
	6800 4600 6800 4500
Wire Wire Line
	6800 4500 6700 4500
Wire Wire Line
	7200 4600 7200 4300
Wire Wire Line
	7200 4300 7100 4300
Wire Wire Line
	7600 4600 7600 4100
Wire Wire Line
	7600 4100 7500 4100
Wire Wire Line
	5200 3000 5300 3000
Wire Wire Line
	5200 3100 5300 3100
Wire Wire Line
	5200 4000 5400 4000
Wire Wire Line
	5200 4100 5400 4100
Wire Wire Line
	4800 6000 4700 6000
Wire Wire Line
	4800 5800 4700 5800
Wire Wire Line
	4700 5600 4700 5700
Wire Wire Line
	4700 5700 4800 5700
Wire Wire Line
	4800 5900 4400 5900
Text Label 5300 3000 0    60   ~ 0
SWDIO
Text Label 5300 3100 0    60   ~ 0
SWCLK
$Comp
L CONN_01X04 P203
U 1 1 58AB1D4D
P 5000 7000
F 0 "P203" H 5000 7250 50  0000 C CNN
F 1 "CONN_01X04" V 5100 7000 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04_Pitch2.54mm" H 5000 7000 50  0001 C CNN
F 3 "" H 5000 7000 50  0000 C CNN
	1    5000 7000
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR025
U 1 1 58AB1E4C
P 4700 6750
F 0 "#PWR025" H 4700 6600 50  0001 C CNN
F 1 "+5V" H 4700 6890 50  0000 C CNN
F 2 "" H 4700 6750 50  0000 C CNN
F 3 "" H 4700 6750 50  0000 C CNN
	1    4700 6750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR026
U 1 1 58AB1E8E
P 4700 7250
F 0 "#PWR026" H 4700 7000 50  0001 C CNN
F 1 "GND" H 4700 7100 50  0000 C CNN
F 2 "" H 4700 7250 50  0000 C CNN
F 3 "" H 4700 7250 50  0000 C CNN
	1    4700 7250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 6950 4700 6950
Wire Wire Line
	4800 7050 4700 7050
Wire Wire Line
	4800 7150 4700 7150
Wire Wire Line
	4700 7150 4700 7250
Wire Wire Line
	4800 6850 4700 6850
Wire Wire Line
	4700 6850 4700 6750
Text Label 4700 6950 2    60   ~ 0
SCL
Text Label 4700 7050 2    60   ~ 0
SDA
Text Notes 4400 7550 0    60   ~ 0
Expansion Connector
Text Notes 4450 6250 0    60   ~ 0
Programming
$Comp
L R R205
U 1 1 58AC294A
P 8500 4650
F 0 "R205" V 8580 4650 50  0000 C CNN
F 1 "4.7k" V 8500 4650 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 8430 4650 50  0001 C CNN
F 3 "" H 8500 4650 50  0000 C CNN
	1    8500 4650
	1    0    0    -1  
$EndComp
$Comp
L R R206
U 1 1 58AC2A0C
P 8700 4650
F 0 "R206" V 8780 4650 50  0000 C CNN
F 1 "4.7k" V 8700 4650 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 8630 4650 50  0001 C CNN
F 3 "" H 8700 4650 50  0000 C CNN
	1    8700 4650
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR027
U 1 1 58AC2AE3
P 8500 4300
F 0 "#PWR027" H 8500 4150 50  0001 C CNN
F 1 "+3V3" H 8500 4440 50  0000 C CNN
F 2 "" H 8500 4300 50  0000 C CNN
F 3 "" H 8500 4300 50  0000 C CNN
	1    8500 4300
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR028
U 1 1 58AC2B29
P 8700 4300
F 0 "#PWR028" H 8700 4150 50  0001 C CNN
F 1 "+3V3" H 8700 4440 50  0000 C CNN
F 2 "" H 8700 4300 50  0000 C CNN
F 3 "" H 8700 4300 50  0000 C CNN
	1    8700 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8500 4300 8500 4500
Wire Wire Line
	8700 4300 8700 4500
Wire Wire Line
	8500 4800 8500 4900
Wire Wire Line
	8500 4900 8400 4900
Wire Wire Line
	8700 4800 8700 5100
Wire Wire Line
	8700 5100 8600 5100
Text Label 8400 4900 2    60   ~ 0
SDA
Text Label 8600 5100 2    60   ~ 0
SCL
$Comp
L R R207
U 1 1 58AC2E89
P 9300 4650
F 0 "R207" V 9380 4650 50  0000 C CNN
F 1 "4.7k" V 9300 4650 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 9230 4650 50  0001 C CNN
F 3 "" H 9300 4650 50  0000 C CNN
	1    9300 4650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR029
U 1 1 58AC2EFD
P 9300 4900
F 0 "#PWR029" H 9300 4650 50  0001 C CNN
F 1 "GND" H 9300 4750 50  0000 C CNN
F 2 "" H 9300 4900 50  0000 C CNN
F 3 "" H 9300 4900 50  0000 C CNN
	1    9300 4900
	1    0    0    -1  
$EndComp
Text Label 9300 4300 0    60   ~ 0
RF_ENABLE
Wire Wire Line
	9300 4300 9300 4500
Wire Wire Line
	9300 4800 9300 4900
Wire Wire Line
	3400 2600 3300 2600
Wire Wire Line
	3400 2800 3300 2800
Text Label 3300 2600 2    60   ~ 0
STATUS_2
Text Label 3300 2800 2    60   ~ 0
STATUS_1
$EndSCHEMATC
