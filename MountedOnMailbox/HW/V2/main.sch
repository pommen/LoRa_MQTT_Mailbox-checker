EESchema Schematic File Version 4
LIBS:main-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
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
L Peters:ATMEGA328P-AU-Arduino U1
U 1 1 5A10DDB9
P 8200 2150
F 0 "U1" H 8250 3517 50  0000 C CNN
F 1 "ATMEGA328P-AU-Arduino" H 8250 3426 50  0000 C CNN
F 2 "Package_QFP:TQFP-32_7x7mm_P0.8mm" H 8350 3650 50  0001 C CIN
F 3 "" H 8200 2150 50  0001 C CNN
	1    8200 2150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR09
U 1 1 5A10DDBE
P 6700 2050
F 0 "#PWR09" H 6700 1800 50  0001 C CNN
F 1 "GND" H 6705 1877 50  0000 C CNN
F 2 "" H 6700 2050 50  0001 C CNN
F 3 "" H 6700 2050 50  0001 C CNN
	1    6700 2050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR013
U 1 1 5A10DDBF
P 7250 3500
F 0 "#PWR013" H 7250 3250 50  0001 C CNN
F 1 "GND" H 7255 3327 50  0000 C CNN
F 2 "" H 7250 3500 50  0001 C CNN
F 3 "" H 7250 3500 50  0001 C CNN
	1    7250 3500
	1    0    0    -1  
$EndComp
Text Notes 2400 2850 0    60   ~ 0
FTDI + ISP
Text GLabel 2400 1450 2    60   Input ~ 0
RST
$Comp
L power:VCC #PWR03
U 1 1 5A10DDC3
P 1650 1000
F 0 "#PWR03" H 1650 850 50  0001 C CNN
F 1 "VCC" H 1667 1173 50  0000 C CNN
F 2 "" H 1650 1000 50  0001 C CNN
F 3 "" H 1650 1000 50  0001 C CNN
	1    1650 1000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 5A10DDC4
P 1650 2300
F 0 "#PWR04" H 1650 2050 50  0001 C CNN
F 1 "GND" H 1900 2250 50  0000 C CNN
F 2 "" H 1650 2300 50  0001 C CNN
F 3 "" H 1650 2300 50  0001 C CNN
	1    1650 2300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR01
U 1 1 5A10DDC5
P 1300 3150
F 0 "#PWR01" H 1300 2900 50  0001 C CNN
F 1 "GND" H 1250 3300 50  0000 C CNN
F 2 "" H 1300 3150 50  0001 C CNN
F 3 "" H 1300 3150 50  0001 C CNN
	1    1300 3150
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR02
U 1 1 5A10DDC6
P 1400 3300
F 0 "#PWR02" H 1400 3150 50  0001 C CNN
F 1 "VCC" H 1417 3473 50  0000 C CNN
F 2 "" H 1400 3300 50  0001 C CNN
F 3 "" H 1400 3300 50  0001 C CNN
	1    1400 3300
	1    0    0    -1  
$EndComp
Text GLabel 950  3350 2    60   Input ~ 0
RXI
Text GLabel 950  3450 2    60   Input ~ 0
TX0
Text GLabel 9200 2650 2    60   Input ~ 0
RXI
Text GLabel 9200 2750 2    60   Input ~ 0
TX0
$Comp
L power:VCC #PWR012
U 1 1 5A10DDCA
P 7200 850
F 0 "#PWR012" H 7200 700 50  0001 C CNN
F 1 "VCC" H 7217 1023 50  0000 C CNN
F 2 "" H 7200 850 50  0001 C CNN
F 3 "" H 7200 850 50  0001 C CNN
	1    7200 850 
	1    0    0    -1  
$EndComp
Text GLabel 9200 1350 2    60   Input ~ 0
MOSI
Text GLabel 9200 1450 2    60   Input ~ 0
MISO
Text GLabel 9200 1550 2    60   Input ~ 0
SCK
Text GLabel 2550 3900 2    60   Input ~ 0
MOSI
Text GLabel 2550 3800 2    60   Input ~ 0
MISO
Text GLabel 2550 4000 2    60   Input ~ 0
SCK
$Comp
L power:GND #PWR015
U 1 1 5A10DDCE
P 10350 2050
F 0 "#PWR015" H 10350 1800 50  0001 C CNN
F 1 "GND" H 10350 1900 50  0000 C CNN
F 2 "" H 10350 2050 50  0001 C CNN
F 3 "" H 10350 2050 50  0001 C CNN
	1    10350 2050
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR05
U 1 1 5A10DDCF
P 2050 3500
F 0 "#PWR05" H 2050 3350 50  0001 C CNN
F 1 "VCC" H 2067 3673 50  0000 C CNN
F 2 "" H 2050 3500 50  0001 C CNN
F 3 "" H 2050 3500 50  0001 C CNN
	1    2050 3500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR06
U 1 1 5A10DDD0
P 2050 4400
F 0 "#PWR06" H 2050 4150 50  0001 C CNN
F 1 "GND" H 2000 4550 50  0000 C CNN
F 2 "" H 2050 4400 50  0001 C CNN
F 3 "" H 2050 4400 50  0001 C CNN
	1    2050 4400
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR07
U 1 1 5A10DDD2
P 6350 900
F 0 "#PWR07" H 6350 750 50  0001 C CNN
F 1 "VCC" H 6367 1073 50  0000 C CNN
F 2 "" H 6350 900 50  0001 C CNN
F 3 "" H 6350 900 50  0001 C CNN
	1    6350 900 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR08
U 1 1 5A10DDD3
P 6350 1200
F 0 "#PWR08" H 6350 950 50  0001 C CNN
F 1 "GND" H 6355 1027 50  0000 C CNN
F 2 "" H 6350 1200 50  0001 C CNN
F 3 "" H 6350 1200 50  0001 C CNN
	1    6350 1200
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR010
U 1 1 5A10DDD5
P 6800 900
F 0 "#PWR010" H 6800 750 50  0001 C CNN
F 1 "VCC" H 6817 1073 50  0000 C CNN
F 2 "" H 6800 900 50  0001 C CNN
F 3 "" H 6800 900 50  0001 C CNN
	1    6800 900 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR011
U 1 1 5A10DDD6
P 6800 1200
F 0 "#PWR011" H 6800 950 50  0001 C CNN
F 1 "GND" H 6805 1027 50  0000 C CNN
F 2 "" H 6800 1200 50  0001 C CNN
F 3 "" H 6800 1200 50  0001 C CNN
	1    6800 1200
	1    0    0    -1  
$EndComp
Text GLabel 1200 1450 0    60   Input ~ 0
reset_in
Text GLabel 9200 2500 2    60   Input ~ 0
reset_in
Text GLabel 9200 2400 2    60   Input ~ 0
scl
Text GLabel 9200 2300 2    60   Input ~ 0
sda
Text Notes 2500 700  0    60   ~ 0
Reset\n
Wire Wire Line
	9200 1650 9850 1650
Wire Wire Line
	9850 1650 9850 1600
Wire Wire Line
	9850 1600 10050 1600
Wire Wire Line
	9200 1750 9850 1750
Wire Wire Line
	9850 1750 9850 1900
Wire Wire Line
	9850 1900 10050 1900
Wire Wire Line
	7300 1650 6700 1650
Wire Wire Line
	6700 1650 6700 1700
Wire Wire Line
	6700 2050 6700 2000
Wire Wire Line
	7200 1350 7300 1350
Wire Wire Line
	7200 850  7200 1050
Wire Wire Line
	7300 1150 7200 1150
Connection ~ 7200 1150
Wire Wire Line
	7300 1050 7200 1050
Connection ~ 7200 1050
Wire Wire Line
	7300 3150 7250 3150
Wire Wire Line
	7250 3150 7250 3250
Wire Wire Line
	7300 3250 7250 3250
Connection ~ 7250 3250
Wire Wire Line
	7300 3350 7250 3350
Connection ~ 7250 3350
Wire Wire Line
	1650 1350 1650 1450
Connection ~ 1650 1450
Wire Wire Line
	1650 1050 1650 1000
Wire Wire Line
	950  3150 1300 3150
Wire Wire Line
	950  3050 1300 3050
Wire Wire Line
	1300 3050 1300 3150
Wire Wire Line
	950  3250 1200 3250
Wire Wire Line
	1200 3250 1200 3300
Wire Wire Line
	1200 3300 1400 3300
Wire Notes Line
	500  2700 2950 2700
Wire Notes Line
	2950 2700 2950 4600
Wire Notes Line
	2950 4600 500  4600
Wire Notes Line
	500  4600 500  2700
Wire Notes Line
	600  550  2800 550 
Wire Notes Line
	2800 550  2800 2450
Wire Notes Line
	2800 2450 600  2450
Wire Notes Line
	600  2450 600  550 
Wire Wire Line
	950  3550 1550 3550
Text GLabel 1550 3550 2    60   Input ~ 0
RST
Text GLabel 2550 4100 2    60   Input ~ 0
reset_in
Wire Wire Line
	7200 1150 7200 1350
Wire Wire Line
	7200 1050 7200 1150
Wire Wire Line
	7250 3250 7250 3350
Wire Wire Line
	7250 3350 7250 3500
Wire Wire Line
	1650 1450 1650 1650
$Comp
L Device:C C2
U 1 1 5B622F49
P 6350 1050
F 0 "C2" H 6465 1096 50  0000 L CNN
F 1 "100n" H 6465 1005 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6388 900 50  0001 C CNN
F 3 "~" H 6350 1050 50  0001 C CNN
	1    6350 1050
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 5B622FD5
P 6800 1050
F 0 "C4" H 6915 1096 50  0000 L CNN
F 1 "100n" H 6915 1005 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6838 900 50  0001 C CNN
F 3 "~" H 6800 1050 50  0001 C CNN
	1    6800 1050
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 5B62300B
P 6700 1850
F 0 "C3" H 6815 1896 50  0000 L CNN
F 1 "C" H 6815 1805 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6738 1700 50  0001 C CNN
F 3 "~" H 6700 1850 50  0001 C CNN
	1    6700 1850
	1    0    0    -1  
$EndComp
$Comp
L Device:Crystal Y1
U 1 1 5B6230C9
P 10050 1750
F 0 "Y1" V 10050 1600 50  0000 R CNN
F 1 "8MHz" V 10050 1400 50  0000 R CNN
F 2 "Crystal:Crystal_SMD_ECS_CSM3X-2Pin_7.6x4.1mm" H 10050 1750 50  0001 C CNN
F 3 "~" H 10050 1750 50  0001 C CNN
	1    10050 1750
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C1
U 1 1 5B63086C
P 1450 1450
F 0 "C1" V 1198 1450 50  0000 C CNN
F 1 "C" V 1289 1450 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 1488 1300 50  0001 C CNN
F 3 "~" H 1450 1450 50  0001 C CNN
	1    1450 1450
	0    1    1    0   
$EndComp
$Comp
L Device:R R1
U 1 1 5B630A62
P 1650 1200
F 0 "R1" H 1720 1246 50  0000 L CNN
F 1 "10K" H 1720 1155 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 1580 1200 50  0001 C CNN
F 3 "~" H 1650 1200 50  0001 C CNN
	1    1650 1200
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C5
U 1 1 5B631105
P 10150 1600
F 0 "C5" V 9921 1600 50  0000 C CNN
F 1 "22p" V 10012 1600 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 10150 1600 50  0001 C CNN
F 3 "~" H 10150 1600 50  0001 C CNN
	1    10150 1600
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C6
U 1 1 5B631171
P 10150 1900
F 0 "C6" V 10300 1900 50  0000 C CNN
F 1 "22p" V 10400 1900 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 10150 1900 50  0001 C CNN
F 3 "~" H 10150 1900 50  0001 C CNN
	1    10150 1900
	0    1    1    0   
$EndComp
Connection ~ 10050 1900
Connection ~ 10050 1600
Wire Wire Line
	10350 1600 10250 1600
Wire Wire Line
	10350 1600 10350 1900
Wire Wire Line
	10250 1900 10350 1900
Connection ~ 10350 1900
Wire Wire Line
	10350 1900 10350 2050
$Comp
L RF_AM_FM:RFM95W-868S2 U2
U 1 1 5B6323F9
P 4450 5250
F 0 "U2" H 4450 5928 50  0000 C CNN
F 1 "RFM95W-868S2" H 3950 5750 50  0000 C CNN
F 2 "RF_Module:HOPERF_RFM9XW_SMD" H 1150 6900 50  0001 C CNN
F 3 "http://www.hoperf.com/upload/rf/RFM95_96_97_98W.pdf" H 1150 6900 50  0001 C CNN
	1    4450 5250
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C7
U 1 1 5B6324D3
P 4200 6600
F 0 "C7" H 4318 6646 50  0000 L CNN
F 1 "100-220u" V 4050 6450 50  0000 L CNN
F 2 "Diode_SMD:D_SMB" H 4238 6450 50  0001 C CNN
F 3 "~" H 4200 6600 50  0001 C CNN
	1    4200 6600
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C9
U 1 1 5B6325B0
P 4800 6650
F 0 "C9" H 4892 6696 50  0000 L CNN
F 1 "100n" H 4892 6605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 4800 6650 50  0001 C CNN
F 3 "~" H 4800 6650 50  0001 C CNN
	1    4800 6650
	1    0    0    -1  
$EndComp
Connection ~ 1300 3150
Wire Wire Line
	4800 6550 4800 6350
Wire Wire Line
	4800 6350 4450 6350
Connection ~ 4450 6350
Wire Wire Line
	4800 6750 4800 6800
Wire Wire Line
	4800 6800 4450 6800
Connection ~ 4450 6800
$Comp
L power:GND #PWR021
U 1 1 5B638309
P 4450 6800
F 0 "#PWR021" H 4450 6550 50  0001 C CNN
F 1 "GND" H 4455 6627 50  0000 C CNN
F 2 "" H 4450 6800 50  0001 C CNN
F 3 "" H 4450 6800 50  0001 C CNN
	1    4450 6800
	1    0    0    -1  
$EndComp
$Comp
L dk_Coaxial-Connectors-RF:CONSMA001-SMD-G J2
U 1 1 5B63867A
P 5500 5150
F 0 "J2" H 5412 5020 60  0000 R CNN
F 1 "CONSMA001-SMD-G" H 5412 5126 60  0000 R CNN
F 2 "Connector_Coaxial:SMA_Molex_73251-1153_EdgeMount_Horizontal" H 5700 5350 60  0001 L CNN
F 3 "https://linxtechnologies.com/wp/wp-content/uploads/consma001-smd-g.pdf" H 5700 5450 60  0001 L CNN
F 4 "CONSMA001-SMD-G-ND" H 5700 5550 60  0001 L CNN "Digi-Key_PN"
F 5 "CONSMA001-SMD-G" H 5700 5650 60  0001 L CNN "MPN"
F 6 "Connectors, Interconnects" H 5700 5750 60  0001 L CNN "Category"
F 7 "Coaxial Connectors (RF)" H 5700 5850 60  0001 L CNN "Family"
F 8 "https://linxtechnologies.com/wp/wp-content/uploads/consma001-smd-g.pdf" H 5700 5950 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/linx-technologies-inc/CONSMA001-SMD-G/CONSMA001-SMD-G-ND/4496569" H 5700 6050 60  0001 L CNN "DK_Detail_Page"
F 10 "CONN SMA RCPT STR 50 OHM SMD" H 5700 6150 60  0001 L CNN "Description"
F 11 "Linx Technologies Inc." H 5700 6250 60  0001 L CNN "Manufacturer"
F 12 "Active" H 5700 6350 60  0001 L CNN "Status"
	1    5500 5150
	-1   0    0    1   
$EndComp
Wire Wire Line
	4950 4950 5500 4950
$Comp
L power:GND #PWR022
U 1 1 5B639BAE
P 5300 5150
F 0 "#PWR022" H 5300 4900 50  0001 C CNN
F 1 "GND" H 5305 4977 50  0000 C CNN
F 2 "" H 5300 5150 50  0001 C CNN
F 3 "" H 5300 5150 50  0001 C CNN
	1    5300 5150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR019
U 1 1 5B639BDB
P 4450 5900
F 0 "#PWR019" H 4450 5650 50  0001 C CNN
F 1 "GND" H 4455 5727 50  0000 C CNN
F 2 "" H 4450 5900 50  0001 C CNN
F 3 "" H 4450 5900 50  0001 C CNN
	1    4450 5900
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 5850 4450 5850
Wire Wire Line
	4550 5850 4450 5850
Connection ~ 4450 5850
Wire Wire Line
	4450 5900 4450 5850
Text GLabel 4950 5650 2    50   Input ~ 0
LoRa_INT
Text GLabel 9200 2850 2    50   Input ~ 0
LoRa_INT
Text GLabel 3950 5050 0    60   Input ~ 0
MOSI
Text GLabel 3950 5150 0    60   Input ~ 0
MISO
Text GLabel 3950 4950 0    60   Input ~ 0
SCK
Text GLabel 3950 5250 0    50   Input ~ 0
LoRaSS
Text GLabel 9200 1150 2    50   Input ~ 0
LoRaSS
Text GLabel 3950 5450 0    50   Input ~ 0
LoRaReset
Text GLabel 9200 1900 2    50   Input ~ 0
LoRaReset
$Comp
L power:+3V3 #PWR020
U 1 1 5B644EE5
P 4450 6350
F 0 "#PWR020" H 4450 6200 50  0001 C CNN
F 1 "+3V3" H 4465 6523 50  0000 C CNN
F 2 "" H 4450 6350 50  0001 C CNN
F 3 "" H 4450 6350 50  0001 C CNN
	1    4450 6350
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR018
U 1 1 5B644F19
P 4450 4700
F 0 "#PWR018" H 4450 4550 50  0001 C CNN
F 1 "+3V3" H 4465 4873 50  0000 C CNN
F 2 "" H 4450 4700 50  0001 C CNN
F 3 "" H 4450 4700 50  0001 C CNN
	1    4450 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4450 4700 4450 4750
$Comp
L Connector:Screw_Terminal_01x03 J1
U 1 1 5B645B39
P 10450 3100
F 0 "J1" H 10529 3142 50  0000 L CNN
F 1 "BoxLidSensor" H 10529 3051 50  0000 L CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_PT-1,5-3-3.5-H_1x03_P3.50mm_Horizontal" H 10450 3100 50  0001 C CNN
F 3 "~" H 10450 3100 50  0001 C CNN
	1    10450 3100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR017
U 1 1 5B645C7B
P 10250 3250
F 0 "#PWR017" H 10250 3000 50  0001 C CNN
F 1 "GND" H 10255 3077 50  0000 C CNN
F 2 "" H 10250 3250 50  0001 C CNN
F 3 "" H 10250 3250 50  0001 C CNN
	1    10250 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	10250 3200 10250 3250
$Comp
L power:VCC #PWR016
U 1 1 5B6467BE
P 10250 2950
F 0 "#PWR016" H 10250 2800 50  0001 C CNN
F 1 "VCC" H 10267 3123 50  0000 C CNN
F 2 "" H 10250 2950 50  0001 C CNN
F 3 "" H 10250 2950 50  0001 C CNN
	1    10250 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	10250 3000 10250 2950
Wire Wire Line
	9200 2950 9850 2950
Wire Wire Line
	10050 2950 10050 3100
Wire Wire Line
	10050 3100 10250 3100
$Comp
L Device:R R2
U 1 1 5B648D77
P 9850 2800
F 0 "R2" H 9920 2846 50  0000 L CNN
F 1 "100K" H 9920 2755 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 9780 2800 50  0001 C CNN
F 3 "~" H 9850 2800 50  0001 C CNN
	1    9850 2800
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR014
U 1 1 5B648E14
P 9850 2600
F 0 "#PWR014" H 9850 2450 50  0001 C CNN
F 1 "VCC" H 9867 2773 50  0000 C CNN
F 2 "" H 9850 2600 50  0001 C CNN
F 3 "" H 9850 2600 50  0001 C CNN
	1    9850 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	9850 2600 9850 2650
$Comp
L Peters:DW01-P U?
U 1 1 5B6338E5
P 1900 5500
AR Path="/5B61F98E/5B6338E5" Ref="U?"  Part="1" 
AR Path="/5B6338E5" Ref="U3"  Part="1" 
F 0 "U3" H 1925 5887 60  0000 C CNN
F 1 "DW01-P" H 1925 5781 60  0000 C CNN
F 2 "Package_TO_SOT_SMD:TSOT-23-6_HandSoldering" H 1650 5950 60  0001 C CNN
F 3 "One Cell Lithium-ion/Polymer Battery Protection IC" H 1700 6100 60  0001 C CNN
	1    1900 5500
	1    0    0    -1  
$EndComp
$Comp
L Peters:IRF7341 Q?
U 2 1 5B6338EC
P 2150 6400
AR Path="/5B61F98E/5B6338EC" Ref="Q?"  Part="2" 
AR Path="/5B6338EC" Ref="Q1"  Part="2" 
F 0 "Q1" V 2394 6400 60  0000 C CNN
F 1 "IRF7341" V 2500 6400 60  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 2200 7050 60  0001 C CNN
F 3 "https://www.infineon.com/dgdl/irf7341.pdf?fileId=5546d462533600a4015355f636411b5d" H 1950 7350 60  0001 C CNN
	2    2150 6400
	0    -1   1    0   
$EndComp
$Comp
L Peters:IRF7341 Q?
U 1 1 5B6338F3
P 1700 6400
AR Path="/5B61F98E/5B6338F3" Ref="Q?"  Part="1" 
AR Path="/5B6338F3" Ref="Q1"  Part="1" 
F 0 "Q1" V 1944 6400 60  0000 C CNN
F 1 "IRF7341" V 2050 6400 60  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 1750 7050 60  0001 C CNN
F 3 "https://www.infineon.com/dgdl/irf7341.pdf?fileId=5546d462533600a4015355f636411b5d" H 1500 7350 60  0001 C CNN
	1    1700 6400
	0    1    1    0   
$EndComp
Wire Wire Line
	1200 5400 1300 5400
Wire Wire Line
	1500 5400 1550 5400
Wire Wire Line
	1550 5550 1300 5550
Wire Wire Line
	1300 5550 1300 5600
Wire Wire Line
	1300 5600 1200 5600
Wire Wire Line
	1200 5400 1200 5200
Wire Wire Line
	2350 5700 2400 5700
Wire Wire Line
	2400 5700 2400 5750
Wire Wire Line
	1500 6500 1200 6500
Wire Wire Line
	1200 5600 1200 5750
Wire Wire Line
	1950 6500 1900 6500
Wire Wire Line
	1700 6300 1700 6100
Wire Wire Line
	1700 6100 1850 6100
Wire Wire Line
	2150 6300 2150 6100
Wire Wire Line
	2150 6100 2050 6100
Wire Wire Line
	2350 6500 2400 6500
Wire Wire Line
	2400 6050 2400 6500
$Comp
L Peters:18650_Holder B?
U 1 1 5B63390B
P 850 5200
AR Path="/5B61F98E/5B63390B" Ref="B?"  Part="1" 
AR Path="/5B63390B" Ref="B1"  Part="1" 
F 0 "B1" H 878 4971 50  0000 L CNN
F 1 "18650_Holder" V 600 4700 50  0000 L CNN
F 2 "Libs:18650-PC2_holder" V 450 4650 50  0001 C CNN
F 3 "" V 550 4750 50  0001 C CNN
	1    850  5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	800  5750 1200 5750
Connection ~ 1200 5750
Wire Wire Line
	1200 5750 1200 6500
Wire Wire Line
	800  5200 1200 5200
$Comp
L Connector:Test_Point TP?
U 1 1 5B633917
P 1300 5850
AR Path="/5B61F98E/5B633917" Ref="TP?"  Part="1" 
AR Path="/5B633917" Ref="TP1"  Part="1" 
F 0 "TP1" H 1242 5877 50  0000 R CNN
F 1 "Test_Point" H 1242 5968 50  0000 R CNN
F 2 "TestPoint:TestPoint_Pad_D1.0mm" H 1500 5850 50  0001 C CNN
F 3 "~" H 1500 5850 50  0001 C CNN
	1    1300 5850
	-1   0    0    1   
$EndComp
Wire Wire Line
	1550 5700 1300 5700
Wire Wire Line
	1300 5700 1300 5850
$Comp
L power:GND #PWR?
U 1 1 5B633920
P 2400 6500
AR Path="/5B61F98E/5B633920" Ref="#PWR?"  Part="1" 
AR Path="/5B633920" Ref="#PWR0103"  Part="1" 
F 0 "#PWR0103" H 2400 6250 50  0001 C CNN
F 1 "GND" H 2405 6327 50  0000 C CNN
F 2 "" H 2400 6500 50  0001 C CNN
F 3 "" H 2400 6500 50  0001 C CNN
	1    2400 6500
	1    0    0    -1  
$EndComp
Connection ~ 2400 6500
$Comp
L power:+BATT #PWR?
U 1 1 5B633929
P 1200 5150
AR Path="/5B61F98E/5B633929" Ref="#PWR?"  Part="1" 
AR Path="/5B633929" Ref="#PWR0104"  Part="1" 
F 0 "#PWR0104" H 1200 5000 50  0001 C CNN
F 1 "+BATT" H 1215 5323 50  0000 C CNN
F 2 "" H 1200 5150 50  0001 C CNN
F 3 "" H 1200 5150 50  0001 C CNN
	1    1200 5150
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 5B633930
P 2400 5900
AR Path="/5B61F98E/5B633930" Ref="R?"  Part="1" 
AR Path="/5B633930" Ref="R4"  Part="1" 
F 0 "R4" H 2470 5946 50  0000 L CNN
F 1 "1k" H 2470 5855 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 2330 5900 50  0001 C CNN
F 3 "~" H 2400 5900 50  0001 C CNN
	1    2400 5900
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R?
U 1 1 5B633937
P 1400 5400
AR Path="/5B61F98E/5B633937" Ref="R?"  Part="1" 
AR Path="/5B633937" Ref="R3"  Part="1" 
F 0 "R3" V 1204 5400 50  0000 C CNN
F 1 "100" V 1295 5400 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 1400 5400 50  0001 C CNN
F 3 "~" H 1400 5400 50  0001 C CNN
	1    1400 5400
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5B63393E
P 1200 5500
AR Path="/5B61F98E/5B63393E" Ref="C?"  Part="1" 
AR Path="/5B63393E" Ref="C10"  Part="1" 
F 0 "C10" H 1292 5546 50  0000 L CNN
F 1 "100n" H 1292 5455 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 1200 5500 50  0001 C CNN
F 3 "~" H 1200 5500 50  0001 C CNN
	1    1200 5500
	1    0    0    -1  
$EndComp
Connection ~ 1200 5400
Connection ~ 1200 5600
$Comp
L Device:C C?
U 1 1 5B63394E
P 7150 5750
AR Path="/5B61F98E/5B63394E" Ref="C?"  Part="1" 
AR Path="/5B63394E" Ref="C11"  Part="1" 
F 0 "C11" H 7265 5796 50  0000 L CNN
F 1 "1u" H 7265 5705 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 7188 5600 50  0001 C CNN
F 3 "~" H 7150 5750 50  0001 C CNN
	1    7150 5750
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5B633955
P 8900 5700
AR Path="/5B61F98E/5B633955" Ref="C?"  Part="1" 
AR Path="/5B633955" Ref="C12"  Part="1" 
F 0 "C12" H 9015 5746 50  0000 L CNN
F 1 "2.2u" H 9015 5655 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 8938 5550 50  0001 C CNN
F 3 "~" H 8900 5700 50  0001 C CNN
	1    8900 5700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5B63395C
P 7150 6000
AR Path="/5B61F98E/5B63395C" Ref="#PWR?"  Part="1" 
AR Path="/5B63395C" Ref="#PWR0105"  Part="1" 
F 0 "#PWR0105" H 7150 5750 50  0001 C CNN
F 1 "GND" H 7155 5827 50  0000 C CNN
F 2 "" H 7150 6000 50  0001 C CNN
F 3 "" H 7150 6000 50  0001 C CNN
	1    7150 6000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5B633962
P 8900 5950
AR Path="/5B61F98E/5B633962" Ref="#PWR?"  Part="1" 
AR Path="/5B633962" Ref="#PWR0106"  Part="1" 
F 0 "#PWR0106" H 8900 5700 50  0001 C CNN
F 1 "GND" H 8905 5777 50  0000 C CNN
F 2 "" H 8900 5950 50  0001 C CNN
F 3 "" H 8900 5950 50  0001 C CNN
	1    8900 5950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5B633968
P 7900 6000
AR Path="/5B61F98E/5B633968" Ref="#PWR?"  Part="1" 
AR Path="/5B633968" Ref="#PWR0107"  Part="1" 
F 0 "#PWR0107" H 7900 5750 50  0001 C CNN
F 1 "GND" H 7905 5827 50  0000 C CNN
F 2 "" H 7900 6000 50  0001 C CNN
F 3 "" H 7900 6000 50  0001 C CNN
	1    7900 6000
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 5450 7150 5600
Wire Wire Line
	7150 6000 7150 5900
Wire Wire Line
	7900 6000 7900 5850
Wire Wire Line
	8900 5950 8900 5850
Wire Wire Line
	8900 5550 8900 5450
$Comp
L power:+BATT #PWR?
U 1 1 5B633975
P 7150 5400
AR Path="/5B61F98E/5B633975" Ref="#PWR?"  Part="1" 
AR Path="/5B633975" Ref="#PWR0108"  Part="1" 
F 0 "#PWR0108" H 7150 5250 50  0001 C CNN
F 1 "+BATT" H 7165 5573 50  0000 C CNN
F 2 "" H 7150 5400 50  0001 C CNN
F 3 "" H 7150 5400 50  0001 C CNN
	1    7150 5400
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 5450 7150 5400
Connection ~ 7150 5450
$Comp
L power:+3V3 #PWR?
U 1 1 5B63397D
P 8900 5200
AR Path="/5B61F98E/5B63397D" Ref="#PWR?"  Part="1" 
AR Path="/5B63397D" Ref="#PWR0109"  Part="1" 
F 0 "#PWR0109" H 8900 5050 50  0001 C CNN
F 1 "+3V3" H 8915 5373 50  0000 C CNN
F 2 "" H 8900 5200 50  0001 C CNN
F 3 "" H 8900 5200 50  0001 C CNN
	1    8900 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	8900 5200 8900 5450
Connection ~ 8900 5450
Wire Wire Line
	1200 5200 1200 5150
Connection ~ 1200 5200
Wire Notes Line
	550  4850 550  7000
Wire Notes Line
	550  7000 2750 7000
Wire Notes Line
	2750 7000 2750 4850
Wire Notes Line
	2750 4850 550  4850
Text Notes 2250 5000 0    50   ~ 0
Batt MGMT
Wire Wire Line
	4200 6450 4200 6350
Wire Wire Line
	4200 6350 4450 6350
Wire Wire Line
	4200 6750 4200 6800
Wire Wire Line
	4200 6800 4450 6800
Wire Notes Line
	6700 4350 6700 7650
Wire Notes Line
	6700 7650 3400 7650
Wire Notes Line
	3400 7650 3400 4350
Wire Notes Line
	3400 4350 6700 4350
Text Notes 3550 4500 0    50   ~ 0
LoRa
Wire Wire Line
	10050 2950 9850 2950
Connection ~ 9850 2950
Wire Notes Line
	11100 550  11100 3800
Wire Notes Line
	11100 3800 6100 3800
Wire Notes Line
	6100 3800 6100 550 
Wire Notes Line
	6100 550  11100 550 
$Comp
L Connector:Conn_01x06_Male J3
U 1 1 5B672CF3
P 750 3250
F 0 "J3" H 856 3628 50  0000 C CNN
F 1 "FTDI" H 856 3537 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x06_P2.54mm_Vertical" H 750 3250 50  0001 C CNN
F 3 "~" H 750 3250 50  0001 C CNN
	1    750  3250
	1    0    0    -1  
$EndComp
$Comp
L Connector:AVR-ISP-6 J4
U 1 1 5B672EDA
P 2150 4000
F 0 "J4" H 1870 4096 50  0000 R CNN
F 1 "AVR-ISP-6" H 1870 4005 50  0000 R CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_2x03_P2.54mm_Vertical" V 1900 4050 50  0001 C CNN
F 3 " ~" H 875 3450 50  0001 C CNN
	1    2150 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 1450 1200 1450
Wire Wire Line
	1600 1450 1650 1450
Wire Wire Line
	1650 1450 2400 1450
$Comp
L Device:LED D1
U 1 1 5B6A7B23
P 9700 950
F 0 "D1" H 9692 695 50  0000 C CNN
F 1 "LED" H 9692 786 50  0000 C CNN
F 2 "LED_SMD:LED_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9700 950 50  0001 C CNN
F 3 "~" H 9700 950 50  0001 C CNN
	1    9700 950 
	-1   0    0    1   
$EndComp
$Comp
L Device:R R6
U 1 1 5B6A7D7F
P 10150 950
F 0 "R6" V 9943 950 50  0000 C CNN
F 1 "1K2" V 10034 950 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 10080 950 50  0001 C CNN
F 3 "~" H 10150 950 50  0001 C CNN
	1    10150 950 
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0110
U 1 1 5B6A7E1A
P 10400 1000
F 0 "#PWR0110" H 10400 750 50  0001 C CNN
F 1 "GND" H 10405 827 50  0000 C CNN
F 2 "" H 10400 1000 50  0001 C CNN
F 3 "" H 10400 1000 50  0001 C CNN
	1    10400 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	9200 1050 9200 950 
Wire Wire Line
	9200 950  9550 950 
Wire Wire Line
	9850 950  10000 950 
Wire Wire Line
	10400 1000 10400 950 
Wire Wire Line
	10400 950  10300 950 
$Comp
L Device:Jumper_NC_Dual JP1
U 1 1 5B6B0099
P 10350 4800
F 0 "JP1" H 10350 5039 50  0000 C CNN
F 1 "Boot_opt" H 10350 4948 50  0000 C CNN
F 2 "Jumper:SolderJumper-3_P1.3mm_Open_RoundedPad1.0x1.5mm" H 10350 4800 50  0001 C CNN
F 3 "~" H 10350 4800 50  0001 C CNN
	1    10350 4800
	1    0    0    -1  
$EndComp
$Comp
L Device:R R7
U 1 1 5B6B02EB
P 10650 4550
F 0 "R7" H 10720 4596 50  0000 L CNN
F 1 "100K" H 10720 4505 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 10580 4550 50  0001 C CNN
F 3 "~" H 10650 4550 50  0001 C CNN
	1    10650 4550
	1    0    0    -1  
$EndComp
Text GLabel 10350 4950 3    50   Input ~ 0
Jumper_selsction
Wire Wire Line
	10350 4950 10350 4900
Wire Wire Line
	10600 4800 10650 4800
Wire Wire Line
	10650 4800 10650 4700
$Comp
L power:VCC #PWR0111
U 1 1 5B6BBAE3
P 10650 4300
F 0 "#PWR0111" H 10650 4150 50  0001 C CNN
F 1 "VCC" H 10667 4473 50  0000 C CNN
F 2 "" H 10650 4300 50  0001 C CNN
F 3 "" H 10650 4300 50  0001 C CNN
	1    10650 4300
	1    0    0    -1  
$EndComp
$Comp
L Device:R R5
U 1 1 5B6BBB32
P 10050 5000
F 0 "R5" H 10120 5046 50  0000 L CNN
F 1 "100K" H 10120 4955 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 9980 5000 50  0001 C CNN
F 3 "~" H 10050 5000 50  0001 C CNN
	1    10050 5000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0112
U 1 1 5B6BBBC8
P 10050 5200
F 0 "#PWR0112" H 10050 4950 50  0001 C CNN
F 1 "GND" H 10055 5027 50  0000 C CNN
F 2 "" H 10050 5200 50  0001 C CNN
F 3 "" H 10050 5200 50  0001 C CNN
	1    10050 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	10050 5150 10050 5200
Wire Wire Line
	10050 4800 10100 4800
Wire Wire Line
	10050 4800 10050 4850
Wire Wire Line
	10650 4400 10650 4300
Text GLabel 9200 3350 2    50   Input ~ 0
Jumper_selsction
$Comp
L power:VCC #PWR0113
U 1 1 5B6D6716
P 9500 6300
F 0 "#PWR0113" H 9500 6150 50  0001 C CNN
F 1 "VCC" H 9517 6473 50  0000 C CNN
F 2 "" H 9500 6300 50  0001 C CNN
F 3 "" H 9500 6300 50  0001 C CNN
	1    9500 6300
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR?
U 1 1 5B6D67FA
P 9700 6300
AR Path="/5B61F98E/5B6D67FA" Ref="#PWR?"  Part="1" 
AR Path="/5B6D67FA" Ref="#PWR0114"  Part="1" 
F 0 "#PWR0114" H 9700 6150 50  0001 C CNN
F 1 "+3V3" H 9715 6473 50  0000 C CNN
F 2 "" H 9700 6300 50  0001 C CNN
F 3 "" H 9700 6300 50  0001 C CNN
	1    9700 6300
	1    0    0    -1  
$EndComp
Wire Wire Line
	9700 6300 9500 6300
$Comp
L Switch:SW_Push SW1
U 1 1 5B6DAA26
P 1650 1850
F 0 "SW1" V 1604 1998 50  0000 L CNN
F 1 "Reset" V 1695 1998 50  0000 L CNN
F 2 "Libs:PushBTN KSC241G" H 1650 2050 50  0001 C CNN
F 3 "" H 1650 2050 50  0001 C CNN
	1    1650 1850
	0    1    1    0   
$EndComp
Wire Wire Line
	1650 2050 1650 2300
$Comp
L Peters:AP2210-3.3V U4
U 1 1 5B6F4B85
P 7900 5250
F 0 "U4" H 7925 5275 50  0000 C CNN
F 1 "AP2210-3.3V" H 7925 5184 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5_HandSoldering" H 8100 4850 50  0001 C CNN
F 3 "http://192.168.0.101/api/part_attachments/632/getFile" H 8000 4850 50  0001 C CNN
	1    7900 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 5450 7350 5450
Wire Wire Line
	7450 5600 7350 5600
Wire Wire Line
	7350 5600 7350 5450
Connection ~ 7350 5450
Wire Wire Line
	7350 5450 7450 5450
Wire Wire Line
	8400 5450 8900 5450
$Comp
L Device:C C?
U 1 1 5B71C914
P 8450 5850
AR Path="/5B61F98E/5B71C914" Ref="C?"  Part="1" 
AR Path="/5B71C914" Ref="C8"  Part="1" 
F 0 "C8" H 8565 5896 50  0000 L CNN
F 1 "100p" H 8565 5805 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 8488 5700 50  0001 C CNN
F 3 "~" H 8450 5850 50  0001 C CNN
	1    8450 5850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5B71C9A4
P 8450 6050
AR Path="/5B61F98E/5B71C9A4" Ref="#PWR?"  Part="1" 
AR Path="/5B71C9A4" Ref="#PWR0115"  Part="1" 
F 0 "#PWR0115" H 8450 5800 50  0001 C CNN
F 1 "GND" H 8455 5877 50  0000 C CNN
F 2 "" H 8450 6050 50  0001 C CNN
F 3 "" H 8450 6050 50  0001 C CNN
	1    8450 6050
	1    0    0    -1  
$EndComp
Wire Wire Line
	8450 6050 8450 6000
Wire Wire Line
	8450 5600 8400 5600
Wire Wire Line
	8450 5600 8450 5700
Entry Wire Line
	12450 2350 12550 2450
$EndSCHEMATC
