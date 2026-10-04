
# Remote Connect to Jetson
https://www.jetson-ai-lab.com/tutorials/getting-started-with-jetson/

1. Find a typeC to USB-A cable to connect from Jetson to PC:
```
Username: p1234
Password: 1234
```
Use command `ssh p1234@192.168.55.1` to connect through USB-A cable, the address is for USB connection.
USB cable is only needed for the initial setup, the first connection.

2. Wifi configuration:
Configure the wifi connection through command: `sudo nmcli device wifi connect WiFi-name password XXXXXX`
After the log 'successfully activated with xxxxxx', type command `hostname -I` to check the IP address, ignore 192.168.55.1 which is for USB connection.

3. Remove the cable, connect with WIFI IP address.

## WiFi Setting with iPhone

***On iPhone:***

- Turn on **Maximize Compatibility**
By default, the hotspot is 5GHz/Wi-Fi 6, this button switch it to 2.4G.
```
sudo nmcli device wifi rescan
Sleep 3
nmcli device wifi list
```
- Check wifi ethernet:
`nmcli device`

- Scan nearby Wi-Fi:
`nmcli device wifi list`

- Check saved wifi connection:
`nmcli connection show`

- Connect hotspot:
`sudo nmcli device wifi connect "iPhone wifi name" password "xxxxxxxx"`
Check IP:
`Ip addr show wlan0` or `hostname -I`


## MicroXRCEAgent

Firstly check the serial port number, use `ls -l /dev/ttySUB*` to check. 
Or plug out pixhawk and plug in again, then `sudo dmesg -w`.
Here it is `/dev/ttyUSB0`.
`ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null`
`sudo MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600 -v 6`
> -v 6 is for detailed connection log.


