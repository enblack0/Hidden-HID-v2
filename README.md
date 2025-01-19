# Hidden-HID-v2
Hidden HID v2 is a tiny USB rubber ducky that can be concealed entirely inside the USB port. I have implemented various hardware improvements compared to v1.

These include: 
1. Removal of SWD programming pins to rely only on STs internal USB bootloader, which avoids the need to painstakingly solder the PCB to the STLink debugger every time I want to change the program. Now it is only necessary to bridge the BOOT0 test pad (middle) with 3V3 or GND to select between bootloader and application. The PCB can then be programmed by inserting into the USB port and using the STCubeProgrammer tool.
2. Layout changes to enable easier assembly of the spacer elements
3. Switching the spacer elements from LEDs with no electrical function to IR phototransistors which also provide a simple on-off remote control. This allows the device to be remotely armed and disarmed once inserted.

KiCad files can be found under /KiCad, gerber and BOM separately under /Fabrication.
Under /STM32 I have uploaded a basic test firmware. This firmware waits for an IR signal, then opens a terminal and echoes hello world. You may need to tweak the paths a little to get it running on your system. 

Check out the project post on hackaday: https://hackaday.io/project/202218-hidden-hid-v2-worlds-smallest-rubber-ducky

![cover-cropped](https://github.com/user-attachments/assets/ce369c1e-d2b1-4981-887d-d5b81b368dbf)


![IMG_20250118_132744_294](https://github.com/user-attachments/assets/21be55e5-5d79-4647-a3af-5c7904761213)
