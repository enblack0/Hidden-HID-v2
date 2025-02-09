# Hidden-HID-v2
Hidden HID v2 is a tiny USB rubber ducky that can be concealed entirely inside the USB port. I have implemented various hardware improvements compared to v1.

These include: 
1. Removal of SWD programming pins to rely only on STs internal USB bootloader, which avoids the need to painstakingly solder the PCB to the STLink debugger every time I want to change the program. Now it is only necessary to bridge the BOOT0 test pad (middle) with 3V3 to select bootloader, or with GND to select application (see image below). After connecting BOOT0 to 3V3, the STM32 can then be programmed by inserting into the USB port and using the STCubeProgrammer tool to download the hex file. 
2. Layout changes to enable easier assembly of the spacer elements
3. Switching the spacer elements from LEDs with no electrical function to IR phototransistors which also provide a simple on-off remote control. This allows the device to be remotely armed and disarmed once inserted.
   

![rev2-bot](https://github.com/user-attachments/assets/c173be7f-866d-4739-8500-147a9e69233f) 

connect BOOT0 to GND to run application or with 3V3 to reprogram


KiCad files can be found under /KiCad, gerber and BOM separately under /Fabrication.
Under /STM32 I have uploaded a basic test firmware. This firmware waits for an IR signal, then opens a terminal and echoes hello world. You may need to tweak the paths a little to get it running on your system. 

If you are interested in purchasing this PCB, I have uploaded it to the PCBWay community. Here you can order either the bare PCB or the assembled PCB directly: https://www.pcbway.com/project/shareproject/Hidden_HID_an_invisible_USB_Rubber_Ducky_3a8cd27f.html

Also check out the project write-up on hackaday: https://hackaday.io/project/202218-hidden-hid-v2-worlds-smallest-rubber-ducky

![cover-cropped](https://github.com/user-attachments/assets/ce369c1e-d2b1-4981-887d-d5b81b368dbf)


![IMG_20250118_132744_294](https://github.com/user-attachments/assets/21be55e5-5d79-4647-a3af-5c7904761213)
