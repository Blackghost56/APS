**APS - Automatic Protection System**

---

The module controls the current and voltage, supports the management and transmission of telemetry data on the CAN bus. 

---

Special features:
* STM32 is used as a control controller.
* Configuring peripherals and generating a project in STM32CubeMX.
* Project for System Workbench for STM32.

---

Parameter  					| Note										| Planned add
------------- 				| -------------								| ---
Voltage limit in   			| 9 - 18 V	(hard set in code)				| Add remote setting by CAN bus
Voltage limit out   		| 9 - 18 V	(hard set in code)				| Add remote setting by CAN bus
Current limit	  			| Set by switch S1 (when turned on)			| Add remote setting by CAN bus
Voltage measurement	in		| Sending by CAN bus						| -
Voltage measurement	out		| Sending by CAN bus						| -
Current measurement			| Sending by CAN bus						| -
Temp measurement			| Sending by CAN bus						| -
Commutation (Load On/Off)	| By CAN. By pressing the external button 	| -

---

Project structure:
* Scheme:								[\data\scheme.png](https://github.com/Blackghost56/APS/blob/master/data/scheme.png)
* Task:									[\data\task.docx](https://github.com/Blackghost56/APS/blob/master/data/task.docx)
* Check list:							[\data\check_list.docx](https://github.com/Blackghost56/APS/blob/master/data/check_list.docx)
* Short description						[\data\short_description.docx](https://github.com/Blackghost56/APS/blob/master/data/short_description.docx)
* Project for STM32F103C8 (ver.0.0.1)	[\STM32F103C8](https://github.com/Blackghost56/APS/blob/master/STM32F103C8/)
* Project for STM32F407VG (ver.0.0.1)	[\STM32F407VG](https://github.com/Blackghost56/APS/blob/master/STM32F103C8/)

