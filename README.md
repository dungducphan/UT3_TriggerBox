# UT3 TriggerBox

## Trigger Hardware for UT3 lab

The timing of the laser systems in UT3 is controlled by the THALES’s Master Clock Module. Due to the limited number of output control channels on the Master Clock, we are left with 2 channels to control the experimental devices (shutters, gas jets, diagnostic cameras, etc.). 
The TAU’s timing unit is a cost-effective solution for two problems here:
- Increase the number of timing control channels to serve all the devices in our experimental setup.
- Allow maximum flexibility in setting the delays between different experimental devices and the laser system.

![image3](https://github.com/dungducphan/UT3_TriggerBox/assets/27539190/0337f61f-049f-4db5-801e-7e4c9027c482)
Figure 1: Timing Diagram from the TAU’s timing unit.

The TAU’s timing unit takes two input signals:
- E1 input: 10 Hz signal from the Master Clock. 10 Hz is the rate at which the laser system operates. This is seen as a YELLOW LINE in Figure 1.
- E2 input: Shot Trigger signal, at a lower rate. This indicates the rate at which we want to extract the laser, perform final compression and shoot the target. For demonstration purposes, this signal is currently driven by a signal generator at 1 Hz. This is the BLUE LINE in Figure 1. The HIGH state of E2 signal must have a duration equal to or larger than the period of E1 signal (>= 100 milliseconds in this case).

The timing unit will generate up to 12 trigger pulses (8 channels on PortB B[0-7] and 4 channels on PortD D[0-3]) upon the coincidence between E1 and E2. The pulses will be delayed off the E1 signal and then sent to trigger different devices with different timing requirements.
The idea here is to use one of the delayed trigger outputs to open the shutter for a subsequent (incoming) laser pulse. The rest of the outputs will arm the experimental devices (gas jet, cameras, WFSs, etc) and get them ready for the laser pulse that will pass through the opened shutter.
There are up to 6 different custom delay lines for users. With the current version, the trigger signals on ports B and D are strictly tightened to the delay line 0 and delay line 1, respectively. Users will not be able to set these channels to a different delay line. However, the values of delay lines 0 and 1 can be changed to the user's needs. Notice that for UT3, the delay values must be between 0 and 100 milliseconds.

## Build and Flash Instructions

### Hardware requirements

This firmware works with the Tiva C Series Development Boards from TI:
- EK-TM4C123GXL
- EK-TM4C1294XL
Both boards use ARM® Cortex®-M4F-Based MCU.

To make it easier to interface with experimental devices via BNC cables, a daughter board is needed:
![IMG-0691](https://github.com/dungducphan/UT3_TriggerBox/assets/27539190/7ca3904e-9640-4dde-89bb-053aef4dde50)
![IMG-0690](https://github.com/dungducphan/UT3_TriggerBox/assets/27539190/50cb3c55-1b35-48a3-b0d1-8afc04f3d29e)

### Build
You need the following tools to build this firmware.
IDE: TI's [CCS](https://www.ti.com/tool/CCSTUDIO?utm_source=google&utm_medium=cpc&utm_campaign=epd-der-null-44700045336317962_prodfolderdynamic-cpc-pf-google-wwe_int&utm_content=prodfolddynamic&ds_k=DYNAMIC+SEARCH+ADS&DCM=yes&gclid=Cj0KCQjwuZGnBhD1ARIsACxbAViQL1JEQmgTLINwu4_Q61RwNN7uroVFYOTFyJSGNfrJtYOh3A35QaAaAt5hEALw_wcB&gclsrc=aw.ds#overview)
SDK: [Tivaware](https://www.ti.com/tool/SW-TM4C)

Steps:
- Open CCSTUDIO, File > Import. Choose import from Git > Projects from Git > Clone URI. Fill in the information about this repo and click Finish.
- Once the project is imported, right click onto the project, choose Properties. In tab Project, set compiler to TI v20.2.5 LTS. In tab Products, click Add, browse to the location of the Tivaware SDKs.
- These step setups the build system. Once done, choose menu Project > Build all.

### Flash
- Connect the board to the computer using a USB cable. On Linux, no driver is needed. On Windows, install the flasher driver that comes with Tivaware.
- Make sure correct flasher is choosen: menu Project > Properties. On Device section, the Connection should be Stellaris In-Circuit Debug Interface.
- Make sure you flash the correct build: Debug vs. Release. To change this, Project > Properties > Manage Configurations.
- To flash, menu Run > Load > Select Program to Load. In the dialog, choose the correct path to the build you want: debug vs. release.

