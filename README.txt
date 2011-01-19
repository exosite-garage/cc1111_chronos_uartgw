========================================
About
========================================
CC1111_chronos_uartgw is C code that runs on a Texas Instruments CC1111
wireless System-on-Chip so that data coming in over the wireless link will
be transmitted out the UART.  The code is based on the RF Access Point source 
code found in the eZ430-Chronos Software Package (Rev. A) download from TI's 
website at http://focus.ti.com/docs/toolsw/folders/print/ez430-chronos.html.

The whole reason for this project was to create an UART wireless link that
could be deployed using off-the-shelf components in TI's low-cost EZ430 Chronos
watch development kit.  The off-the-shelf CC1111 based access point uses USB
as its primary communication.  Embedded systems often do not have USB host, so
this project was created to send the data out over UART instead of USB.

License for Exosite modifications, where applicable, is BSD

Built/tested with IAR evaluation version downloadable from 
http://supp.iar.com/Download/SW/?item=EW8051-EVAL

========================================
Quick Start
========================================
****************************************
1) install IAR (either eval version or full)
****************************************
http://supp.iar.com/Download/SW/?item=EW8051-EVAL

****************************************
2) open project in IAR and build
****************************************
--) download source files and copy to a project folder on your PC 
--) double click on the .eww file
--) in IAR, click "Project-> Edit Configurations" and change to the
    915 MHz - Unrestricted IAR Workbench
--) in IAR, click "Project-> Rebuild All"
--) you should see a file called "program_file.hex" generated in the binary
    build output folder

****************************************
3) test it out
****************************************
--) use a CCDebugger in conjunction with TI's Flash Programmer tool
--) first, read the existing code out of the CC1111 and save it off as a backup
--) then, program and verify the new program_file.hex file

****************************************
4) tweak it
****************************************
--) play around, use it, extend it!


Copyright 2010, Exosite LLC
