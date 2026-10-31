# BLE Fox Hunt

Pick a nearby BLE device and home in on it by signal strength. Built on the
factory card-page framework and the same NimBLE observer scanning as the BLE
Scanner app, and added to the menu as **Fox Hunt**.

## Display

The large **signal panel** at the top is the proximity indicator (there are no
status LEDs). It fills with the band colour and pulses — brighter and faster
the closer you are:

| Band | Colour | Meaning |
|------|--------|---------|
| FREEZING | blue | weak / far |
| COLD | cyan | getting warmer |
| WARM | amber | close |
| HOT | red | very close |

It shows the band name and the smoothed RSSI in dBm; the raw RSSI and target
name appear under the Target card.

## Controls

- **Target** dropdown: choose a discovered device to hunt, or *Auto
  (strongest)*. The list refreshes as devices are found.
- **Tap the left / right half of the signal panel**: lower / raise feedback
  volume (0–5; 0 = silent). The Volume slider does the same.
- **Swipe in from the left or right bezel** (or the back chevron): go back.

## Feedback

A mix of haptic pulses and beeps firing on each pulse crest, so cadence and
pitch track the proximity band. Set the mix under **Feedback → Type** (Off /
Haptic / Beep / Buzz+Beep); haptic needs the DRV2605, beep needs the audio
codec.

## Tuning

- **Smoothing** — Low (fast, jumpy) / Medium / High (steady, laggy) RSSI
  averaging.
- **Range dB** — shifts all band thresholds by -10..+10 dB, to spread the bands
  for close-range or long-range hunting.

Settings persist while the firmware runs and reset on reboot.

## Notes

- Works with any BLE advertiser, including beacons that advertise every ~250 ms.
- Scanning is continuous and active while the app is open, and stops on exit.
