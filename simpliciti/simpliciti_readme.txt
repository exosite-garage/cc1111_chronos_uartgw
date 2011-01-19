Some notes about the SimpliciTI configuration used in this project

- The source code is based on the SimpliciTI 1.1.1 release.

- A full SimpliciTI installation contains configurations for many targets and device types. To avoid confusion,
  only the configuration (Access Point) and target files (RFUSB) required for the eZ430-Chronos have been used.

- All source code files have been copied into the project physically. Symbolic links have been replaced with
  the real source code file. 
  
- Due to the indirect inclusion scheme of hardware-dependent source code, some source code files have been
  excluded from build. However, they will be included through higher level source code.  

- Some modifications where required to the original source code. All these changes have been marked with [BM].

	mrfi_f1f2.c   		Changed channel assignment (mrfiLogicalChanTable) for three ISM bands
				Changed power output settings (mrfiRFPowerTable) for three ISM bands

	mrfi_radio.c/MRFI_Init(void)    Added frequency offset correction to use calibrated frequency offset
					when starting RF communication
											
	mrfi_radio.c/MRFI_RfIsr(void)   Changed radio ISR to normal function, since we have a shared radio ISR

	nwk_api.c			Made variable sInit_done globally available to allow SimpliciTI to shutdown 
					and restart multiple times


- If you (for whatever reason) want to upgrade to a newer version of SimpliciTI, please bear in mind that

	a) the access point SimpliciTI version is 1.1.1 (and cannot be updated)

	b) the workarounds used here to enable SimpliciTI to shutdown and restart multiple times might not necessarily
	   work when used with later revisions
	   	   
	   
	