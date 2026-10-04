# Modules
‘PX4 side’ means the PX4/NuttX system running inside Pixhawk flight control hardware, not a normal Linux terminal on Jetson.

## UXRCE_DDS_Client
Uxrce_dds_client is a module command inside PX4 firmware, which doesn’t belong to ubuntu.
The client is running on PX4 flight control, and the agent is running on the companion computer. 

Architecture diagram:
```
Jetson / Ubuntu                   		Pixhawk / PX4 NuttX
--------------------              		--------------------
MicroXRCEAgent       <PORT>        	uxrce_dds_client
ROS 2                               		flight control、sensor、EKF
```

### How to get inside PX4 shell

- **NSH/MAVLink Shell **
    Inside Pixhawk, officially they are called MAVLink Shell or NSH console, and essentially they are the command line inside flight control.  When get signs like `nsh>` or `pxh>`, Then run `uxrce_dds_client start -t serial -d /dev/ttyS3 -b 921600` which is manually start PX4.

- **/dev/ttyS3**
    */dev/ttyS3* is the internal device name inside Pixhawk.

```
    Jetson: /dev/ttyUSB0
    Pixhawk: /dev/ttyS3

    Jetson			Pixhawk
    /dev/ttyUSB0		/dev/ttyS3
        TX ------------ RX
        RX ------------ TX
        GND ------------ GND
```

The suggested way to get the device number is throught QGroundControl UXRCE_DDS_CFG, and selet TELEM1 or TELEM2, not guess.

### QGroundControl
The easiest way to get into PX4 shell, is throught QGroundControl.
 
1. Use one typeC to USB-A cable to connect Pixhawk to PC
2. Install QGroundControl

    *Before installing QGroundControl for the first time:*
    - On the command prompt enter:
    ```
    sudo usermod -a -G dialout $USER
    sudo apt-get remove modemmanager -y
    sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
    sudo apt install libfuse2 -y
    sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y
    ```
    - Logout and login again to enable the change to user permissions.

    *To install QGroundControl:*
    - Download QGroundControl-x86_64.AppImage.
    - Install (and run) using the terminal commands:
    `chmod +x ./QGroundControl-x86_64.AppImage`
    `./QGroundControl-x86_64.AppImage`  (or double click)

3. Running QGroundControl
    ```
    QGroundControl
    -> Analyze Tools
    -> MAVLink Console
    ```
When seeing `nsh>`, run `uxrce_dds_client status`.
If the command exists, should return `INFO [xrce_dds_client] Running`. Or `INFO [xrce_dds_client] not running`, then decide if need manually start.

