# BLE Fox Hunt

Pick a nearby BLE device and home in on it by signal strength. Built on the
same NimBLE observer scanning as the BLE Scanner app, and added to the menu as
**Fox Hunt**.

## Display

The round 240x240 screen has no status LEDs, so proximity is shown as a
coloured glow around the edge of the display that fades toward the centre and
pulses faster the closer you are:

| Band | Colour | Meaning |
|------|--------|---------|
| FREEZING | blue | weak / far |
| COLD | cyan | getting warmer |
| WARM | amber | close |
| HOT | red | very close |

The centre shows the band name, the smoothed RSSI in dBm, and the target name
with the raw RSSI underneath.

## Controls (touch)

- **Target dropdown** (top): choose a discovered device, or *Auto (strongest)*.
  The list refreshes as devices are found.
- **Tap left / right third** of the screen: lower / raise feedback volume
  (0–5; 0 = silent). The bezel edges are left clear.
- **Swipe in from the left or right bezel**: go back. A back chevron
  (top-left) does the same.
- **Gear** (top-right): smoothing, range tuning and feedback type.

## Feedback

A mix of haptic pulses and beeps, firing on each glow crest, so cadence and
pitch track the proximity band. Type (Off / Haptic / Beep / Buzz+Beep) and
volume are adjustable; haptic needs the DRV2605, beep needs the audio codec.

## Settings

- **Smoothing** — Low (fast, jumpy) / Medium / High (steady, laggy) RSSI
  averaging.
- **Range tune** — shifts all band thresholds by -10..+10 dB, to spread the
  bands out for close-range or long-range hunting.
- **Feedback** — the haptic/beep mix described above.

Settings persist while the firmware runs but reset on reboot.

## Notes

- Works with any BLE advertiser, including beacons that advertise every ~250 ms.
- Scanning is continuous and active while the app is open, and stops on exit.
