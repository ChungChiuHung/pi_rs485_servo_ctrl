# pi_rs485_servo_ctrl

This project is designed to control an AC servo motor from a Raspberry Pi 3B+
over RS485, using either a Web UI (Flask) or an OSC server.

> **Safety:** this code sends real commands to a physical servo motor. Make
> sure the motor and its load are clear of people and obstacles before
> starting any script. Note that `osc_2.py` writes to the driver as soon as it
> starts (it enables DI control via `PD16`), even before any OSC command is
> received.

# Project Structure

| Folder | Purpose |
|---|---|
| `servo_comm_shihlin/` | Type 2 motor (Shihlin SDE-series driver), 400W motor variant. Web UI (`app.py`) and OSC server (`osc_2.py`). |
| `servo_comm_shihlin_50W/` | Same as above, for the SDE-010A2 driver + SME-L00530 (50W) motor. |
| `servo_comm_shihlin_unified/` | Work in progress: merges the two Shihlin folders into one config-driven package. See its `README.md`. |
| `servo_communication/` | Type 1 motor (different brand). Web UI (`app.py`). |
| `web/` + `main.py` | Legacy entrypoint for the Type 1 Web UI; `servo_communication/app.py` supersedes it. `main.py` is a placeholder for a planned config-driven motor selector. |
| `examples/`, `tests/`, `tools/` | Development scratch code, tests and helper scripts. Not deployed. |
| `docs/` | Driver manuals (PDF). |

The two Shihlin folders share most code by copy, not by import. Only the
motor-specific settings differ:

| | `servo_comm_shihlin/` | `servo_comm_shihlin_50W/` |
|---|---|---|
| Baud rate | 115200 | 9600 |
| Pulses per output degree | 349525.33 | 116508.44 |
| Extra OSC commands | - | continuous motion, `/cancel_loop` |

All scripts use relative imports, so **always run them from inside their own
folder** (`cd <folder>` first).

# Hardware Requirements
1. Raspberry Pi 3B+
2. One of the following RS485 interfaces:
   - [RS485 CAN HAT](https://www.waveshare.com/wiki/RS485_CAN_HAT)
     ([PDF from waveshare.com](https://www.waveshare.com/w/upload/2/29/RS485-CAN-HAT-user-manuakl-en.pdf))
   - Industrial USB TO RS485 Bidirectional Converter (onboard original CH343G, with multi-protection circuits)

The serial port is detected automatically, trying `/dev/ttyS0`,
`/dev/ttyAMA0`, `/dev/serial0` and `/dev/ttyUSB0` in that order.

# AC Servo Motor Information
- [AC Servo Motor Type 1 Info.](https://amethyst-myrtle-52e.notion.site/Servo-Motor-Driver-2f7c21ac9d024b00933ec2252861ffcf)
- AC Servo Motor Type 2 Info. (Shihlin SDE series; PDF copies are also in `docs/`)
  - [zh[1]](https://www.seec.com.tw/Content/Goods/PdfViwer.aspx?SiteID=10&MmmID=655575436061077370&Msid=2022102818050639313&fd=GoodsDownload_Files&pname=SDE%E4%BC%BA%E6%9C%8D%E9%A9%85%E5%8B%95%E5%99%A8%E8%AA%AA%E6%98%8E%E6%9B%B8_V1.07.pdf)
  - [zh[2]](https://www.seec.com.tw/Content/Goods/PdfViwer.aspx?SiteID=10&MmmID=655575436061077370&Msid=2020082410220029759&fd=GoodsDownload_Files&pname=%E5%A3%AB%E6%9E%97%E9%9B%BB%E6%A9%9FSDE%E7%B0%A1%E6%98%93%E8%AA%AA%E6%98%8E%E6%9B%B8(%E4%B8%AD%E8%8B%B1)LE106D04204.pdf)
  - [en[3]](https://www.manualslib.com/products/Shihlin-Electric-Sde-040a2-10446073.html)
  - `docs/SDE_English_manual_UL_v107.pdf`

## Type 2 (Shihlin) Driver Communication Settings
The code uses Modbus ASCII, 8 data bits, no parity, 2 stop bits, device
number 1. Set the driver parameters to match (manual §9.2):

| Parameter | Meaning | `servo_comm_shihlin/` | `servo_comm_shihlin_50W/` |
|---|---|---|---|
| `PC20` (SNO) | Communication device number | `1` | `1` |
| `PC22` (BPS) | Protocol + baud rate | `0053h` (ASCII 8N2, 115200) | `0013h` (ASCII 8N2, 9600) |

`PC22` is marked (★) in the manual: power-cycle the driver after changing it.

# Configuring the RS485 CAN HAT
- [RS485 CAN HAT_ch](https://www.waveshare.net/wiki/RS485_CAN_HAT#.E5.89.8D.E7.BD.AE.E5.B7.A5.E4.BD.9C_2)
- [RS485 CAN HAT_uk](https://learn.sb-components.co.uk/RS485-CAN-HAT)

1. Update the system
   ```
   sudo apt-get update
   sudo apt-get upgrade
   ```
2. Open the UART port
   ```
   sudo raspi-config
   ```
   Select Interfacing Options -> Serial
   - Would you like a login shell to be accessible over serial? **No**
   - Would you like the serial port hardware to be enabled? **Yes**
3. Open `/boot/firmware/config.txt` (`/boot/config.txt` on Raspberry Pi OS
   Bullseye and older) and add the lines below to the end of the file
   ```
   [all]
   enable_uart=1
   dtparam=uart0=on
   dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25,spimaxfrequency=2000000
   dtoverlay=disable-bt
   ```
4. List the serial ports
   ```
   ls -l /dev/serial*
   ```
   ![image](https://github.com/ChungChiuHung/rpiWebServer_RS485_ServoCtrl/assets/52248840/9da6fa95-6cb4-4160-8ef5-387343c84b57)
5. Reboot
   ```
   sudo reboot
   ```
6. After restart, verify that the RS485 CAN HAT is detected during boot
   ```
   dmesg | grep -i '\(can\|spi\)'
   ```
   ![image](https://github.com/ChungChiuHung/rpiWebServer_RS485_ServoCtrl/assets/52248840/149436ad-a2ca-4dd2-9fa6-c44bf60b2702)

# Installation
```
git clone https://github.com/ChungChiuHung/pi_rs485_servo_ctrl.git
cd pi_rs485_servo_ctrl
pip3 install -r requirements.txt
```

> **Pushing changes back:** the HTTPS clone above works fine for pulling, but
> GitHub no longer accepts password authentication for `git push` over
> HTTPS. Either set up a [Personal access token](https://github.com/settings/tokens)
> as your Git credential, or switch the remote to SSH (requires an SSH key
> added to your GitHub account under Settings → SSH and GPG keys first):
> ```
> git remote set-url origin git@github.com:ChungChiuHung/pi_rs485_servo_ctrl.git
> ```

# Running the Web UI
```
cd servo_comm_shihlin/        # or servo_comm_shihlin_50W/, or servo_communication/
python3 app.py
```
Then open `http://<PI_IP>:5000` in a browser.

## HTTP API: `POST /alarm/clear`
Clears **AL.12 (Emergency stop)** only -- not a general-purpose Modbus
write endpoint. Available in `servo_comm_shihlin/app.py`.

Request body must include `{"confirm": true}`; without it, the request is
rejected with `400` and no command is sent to the drive.

```
curl -X POST http://<PI_IP>:5000/alarm/clear \
     -H "Content-Type: application/json" \
     -d '{"confirm": true}'
```

Response includes the alarm code read before and after the clear attempt
(`0` = no alarm), which mechanism actually cleared it, the caller's IP, and
a timestamp. Every call is logged (timestamp, before/after alarm code,
caller IP) regardless of outcome.

**Safety precondition -- read before using this endpoint.** Per the driver
manual (`docs/en_manual.txt`, AL.12 entry, ~line 10512), AL.12 means the
EMG (Emergency Stop) signal is active, and the manual's own remedy is to
*"Release the trigger after removal of some emergency conditions"* --
i.e. clear it only after the physical emergency condition has actually
been resolved.

The endpoint's primary clearing mechanism (`clear_alarm_12()`, the
existing function reused from `servo_control.py`) works by switching the
drive's DI input source to communication-control mode (`PD16`) and then
writing the virtual EMG DI bit to its "released" state (`PD25`/`ITST`).
**If a physical E-Stop circuit is still engaged when this is called, the
drive will report EMG as released anyway** -- the software cannot verify
the physical emergency condition is actually gone. This endpoint does not,
and cannot, perform that physical check; whoever calls it is responsible
for confirming the physical E-Stop condition is really resolved first.

If the primary mechanism doesn't bring the alarm code back to `0`, the
endpoint falls back to the driver's official "Alarm clearance" register
(`0x0130`, write `0x1EA5`; `docs/en_manual.txt` ~line 10230), which does
not touch DI control source or the virtual EMG state.

# Running the OSC Server
`osc_2.py` exists in both Shihlin folders. Pick the folder that matches the
connected motor (see [Project Structure](#project-structure)):
```
cd servo_comm_shihlin/        # or: cd servo_comm_shihlin_50W/
python3 osc_2.py --ip <PI_IP> --port_receive 5005
```
- `--ip` defaults to `10.12.1.107`, so pass the Pi's own IP address unless it
  happens to match.
- `--port_receive` defaults to `5005` (UDP).
- Feedback messages are sent to a TouchDesigner client whose address is
  hardcoded in `osc_2.py` (`TOUCHDESIGNER_IP = "10.12.1.164"`,
  `TOUCHDESIGNER_PORT = 5008`). Edit these to match your network.

## OSC Commands (received)
| Address | Arguments | Action |
|---|---|---|
| `/servo` | `1.0` / `0.0` | Servo on / off |
| `/clear` | - | Clear alarm AL.12 |
| `/set_point` | `angle` (degrees), `acc_time` (ms), `rpm` | Move to the target angle, measured from the software home position |
| `/back_home` | - | Return to the absolute home position (`abs_home_pos` in `servo_config.json`) |
| `/set_home` | - | Set the current position as the software home (angle = 0) |
| `/reset_initial_abs_position` | - | Write `PA29` (initial absolute position) on the driver |
| `/set_continous_motion` | `speed_rpm`, `acc_time` (ms), `enable` | 50W only: configure continuous (speed) motion |
| `/ctrl_continuous_motion` | `action` (`start`/`stop`), `CW_CCW` (`CW`/`CCW`) | 50W only: start/stop continuous motion |
| `/cancel_loop` | - | 50W only: stop the position reading loop |

## OSC Feedback (sent)
`/servo_on`, `/servo_off`, `/clear`, `/set_point`, `/back_home`,
`/set_home_position`, `/reset_initial_abs_position`, `/moving` (angle
difference while moving), `/motion_complete`; 50W only:
`/continuous_mode_start`, `/continuous_mode_stop`, `/cancel_loop` (current
angle).

# Running at Startup
Use **either** PM2 **or** systemd, not both, or the script will be started
twice and the two instances will compete for the serial port.

## Option A: [PM2](https://pm2.keymetrics.io/docs/usage/startup/)
1. Install PM2
   ```
   sudo apt-get update
   sudo apt-get install -y nodejs npm
   sudo npm install pm2 -g
   ```
2. Generate the startup script
   ```
   pm2 startup
   ```
   PM2 prints a `sudo env PATH=... pm2 startup ...` command specific to your
   system. Copy and run that exact command.
3. Start the script from its own folder
   ```
   pm2 start osc_2.py --name servo-osc --interpreter python3 \
       --cwd /path/to/pi_rs485_servo_ctrl/servo_comm_shihlin -- --ip <PI_IP>
   ```
4. Save the current PM2 process list
   ```
   pm2 save
   ```

## Option B: systemd
1. Create a service file
   ```
   sudo nano /etc/systemd/system/servo-osc.service
   ```
   `WorkingDirectory` is required: the scripts use relative imports and must
   run from inside their own folder.
   ```
   [Unit]
   Description=Servo OSC Server
   After=network-online.target
   Wants=network-online.target

   [Service]
   Type=simple
   WorkingDirectory=/path/to/pi_rs485_servo_ctrl/servo_comm_shihlin
   ExecStart=/usr/bin/python3 osc_2.py --ip <PI_IP>

   [Install]
   WantedBy=multi-user.target
   ```
2. Enable and start the service
   ```
   sudo systemctl daemon-reload                 # recognize the new service
   sudo systemctl enable servo-osc.service      # start automatically at boot
   sudo systemctl start servo-osc.service       # start right away
   sudo systemctl status servo-osc.service      # confirm it is active
   ```
3. Debugging: check the logs
   ```
   journalctl -u servo-osc.service
   ```

### Disabling the systemd Service
1. Disable and stop the service
   ```
   sudo systemctl disable servo-osc.service
   sudo systemctl stop servo-osc.service
   sudo systemctl status servo-osc.service
   ```
2. (Optional) Remove the service file
   ```
   sudo rm /etc/systemd/system/servo-osc.service
   sudo systemctl daemon-reload
   sudo systemctl reset-failed                  # clear leftover error states
   ```

# Configuring a Static IP for the Raspberry Pi
The OSC server listens on a fixed IP, so the Pi should use a static address.

1. Find the router IP (the first IP in the output) and the DNS server
   ```
   ip r | grep default
   cat /etc/resolv.conf                         # IP next to "nameserver"
   ```
2. Set the static IP. Replace `<STATICIP>`, `<ROUTERIP>` and `<DNSIP>`.

   **Raspberry Pi OS Bookworm and newer (NetworkManager):**
   ```
   nmcli connection show                        # find the connection name
   sudo nmcli connection modify "<CONNECTION>" \
       ipv4.method manual \
       ipv4.addresses <STATICIP>/24 \
       ipv4.gateway <ROUTERIP> \
       ipv4.dns <DNSIP>
   sudo nmcli connection up "<CONNECTION>"
   ```

   **Raspberry Pi OS Bullseye and older (dhcpcd):** edit `/etc/dhcpcd.conf`
   ```
   sudo nano /etc/dhcpcd.conf
   ```
   and add, with `<NETWORK>` set to `eth0` or `wlan0`:
   ```
   interface <NETWORK>
   static ip_address=<STATICIP>/24
   static routers=<ROUTERIP>
   static domain_name_servers=<DNSIP>
   ```
   then reboot:
   ```
   sudo reboot
   ```
3. Verify the IP
   ```
   hostname -I
   ```
