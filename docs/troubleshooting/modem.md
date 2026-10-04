# Modem issue:

```
natasha@honor-linux:~$ python3 ~/.local/bin/mavproxy.py --master=/dev/ttyACM0
WARNING: You should uninstall ModemManager as it conflicts with APM and Pixhawk
Connect /dev/ttyACM0 source_system=255
Failed to connect to /dev/ttyACM0 : module 'serial' has no attribute 'SerialException'
natasha@honor-linux:~$ python3 ~/.local/bin/mavproxy.py --master=/dev/ttyACM0
WARNING: You should uninstall ModemManager as it conflicts with APM and Pixhawk
Connect /dev/ttyACM0 source_system=255
Failed to connect to /dev/ttyACM0 : module 'serial' has no attribute 'SerialException'
natasha@honor-linux:~$ systemctl status ModemManager
● ModemManager.service - Modem Manager
 	Loaded: loaded (/lib/systemd/system/ModemManager.service; enabled; vendor >
 	Active: active (running) since Wed 2026-07-29 00:30:58 PDT; 20h ago
   Main PID: 31406 (ModemManager)
  	Tasks: 3 (limit: 9115)
 	Memory: 3.0M
    	CPU: 743ms
 	CGroup: /system.slice/ModemManager.service
         	└─31406 /usr/sbin/ModemManager

Jul 29 00:54:30 honor-linux ModemManager[31406]: <info>  [base-manager] couldn'>
Jul 29 02:05:08 honor-linux ModemManager[31406]: <info>  [sleep-monitor-systemd>
Jul 29 12:17:23 honor-linux ModemManager[31406]: <info>  [sleep-monitor-systemd>
Jul 29 12:17:25 honor-linux ModemManager[31406]: <info>  [base-manager] couldn'>
Jul 29 12:17:25 honor-linux ModemManager[31406]: <info>  [base-manager] couldn'>
Jul 29 15:57:18 honor-linux ModemManager[31406]: <info>  [device /sys/devices/p>
Jul 29 15:57:18 honor-linux ModemManager[31406]: <warn>  [plugin/generic] could>
Jul 29 15:57:18 honor-linux ModemManager[31406]: <warn>  [base-manager] couldn'>
Jul 29 17:25:30 honor-linux ModemManager[31406]: <info>  [base-manager] couldn'>
Jul 29 17:25:35 honor-linux ModemManager[31406]: <info>  [base-manager] couldn'>
lines 1-20/20 (END)




nsh: sysinit: fopen failed: No such file or directory

NuttShell (NSH) NuttX-11.0.0
nsh> uxrce_dds_client status
INFO  [uxrce_dds_client] Running, disconnected
INFO  [uxrce_dds_client] Using transport:     serial
INFO  [uxrce_dds_client] timesync converged: false
uxrce_dds_client: cycle: 2316182 events, 7968064414us elapsed, 3440.17us avg, min 99us max 1013134us 3254.536us rms
uxrce_dds_client: cycle interval: 2316182 events, 3440.82us avg, min 100us max 1012000us 3186.251us rms
nsh> uxrce_dds_client start -t serial -d /dev/ttyS3 -b 921600
ERROR [uxrce_dds_client] Task already running
nsh> 



xrce_dds_client status
nsh: xrce_dds_client: command not found
nsh> uxrce_dds_client status
INFO  [uxrce_dds_client] Running, connected
INFO  [uxrce_dds_client] Using transport:     serial
INFO  [uxrce_dds_client] Payload tx:          66094 B/s
INFO  [uxrce_dds_client] Payload rx:          0 B/s
INFO  [uxrce_dds_client] timesync converged: true
uxrce_dds_client: cycle: 346855 events, 1266908326us elapsed, 3652.56us avg, min 100us max 68907us 3331.243us rms
uxrce_dds_client: cycle interval: 346857 events, 3653.68us avg, min 101us max 68907us 3331.306us rms



nsh> ver all
HW arch: PX4_FMU_V6C
HW type: V6C000001
HW version: 0x000
HW revision: 0x001
PX4 git-hash: 99c40407ffd7ac184e2d7b4b293f36f10fe561ef
PX4 version: Release 1.15.4 (17761535)
OS: NuttX
OS version: Release 11.0.0 (184549631)
OS git-hash: 5d74bc138955e6f010a38e0f87f34e9a9019aecc
Build datetime: Mar 20 2025 23:02:15
Build uri: localhost
Build variant: default
Toolchain: GNU GCC, 9.3.1 20200408 (release)
PX4GUID: 000600000000313836333233510e00330039
MCU: STM32H7[4|5]xxx, rev. V

nsh> commander check
INFO  [commander] Preflight check: FAILED
nsh> listener sensor_combined

TOPIC: sensor_combined
 sensor_combined
    timestamp: 9312672844 (0.004192 seconds ago)
    gyro_rad: [0.00235, 0.01092, -0.00437]
    gyro_integral_dt: 4560
    accelerometer_timestamp_relative: 0
    accelerometer_m_s2: [0.31528, 0.14602, -9.83629]
    accelerometer_integral_dt: 4337
    accelerometer_clipping: 0
    gyro_clipping: 0
    accel_calibration_count: 2
    gyro_calibration_count: 1
```

