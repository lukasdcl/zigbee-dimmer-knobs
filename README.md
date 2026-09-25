# Zigbee Rotary Dimmer Knob — combined proof of concept

A physical rotary knob that talks **directly** to a Zigbee bulb(s) over binding —
no coordinator, no hub or Home Assistant required once it's set up. Turn to
dim, press to toggle. If Zigbee2MQTT and Home Assistant are both switched
off, it keeps working. But can also be integrated into either, too. 

## Features

- **Direct binding** — commands go straight to your bulb(s) over Zigbee binding, no coordinator in the loop. Turn off Zigbee2MQTT/Home Assistant entirely and it keeps working. (endpoint 1)
- **Reverse binding (optional)** — bind your bulbs *back* to endpoint 2 and the knob picks up brightness/on-off changes made elsewhere (e.g. from Home Assistant) within a second or two.
- **Self-correcting brightness** — when multiple bulbs are bound, some of them missing a zigbee command doesn't leave bulbs out of sync forever: when you stop turning, one absolute "go to this level" pulls every bound bulb back into line.
- **Live tuning panel** — change common settings (brightness step size, fade time, debounce, etc.) over the serial console with `tune <setting> <value>`, no rebuild required; copy your favourites into `main/knob_config.h` to make them permanent.
- **Backs off under radio congestion** — automatically widens the gap between commands if a lot of sends are failing, and tightens it back up once conditions clear.
- **Turn** → relative Step-with-On/Off commands ("up by N", never "go to X").
  Turning up from off brings the bulb back at its previous brightness.
- **Press** → explicit On/Off Toggle (no brightness in it).

## Status

This is a working proof of concept, not a polished product. It builds and
runs on real hardware, bound to real bulb(s), but it hasn't seen extended
field use, isn't UL/CE tested, and the code has some rough edges noted below
under "Checked vs not checked".

I am developing **multi-knob** support, currently un-tested in the multi-knob branch of this repo.

## Hardware you need

- An ESP32-C6 board — developed against the **ESP32-C6-DevKitC-1**; a
  Seeed XIAO ESP32C6 also works with a pin remap
- A rotary encoder with a push switch — an **EC11** (or clone) for
  prototyping; a Bourns PEC11R is my intended final dimmer but not yet tested.
- An existing Zigbee network with a coordinator (this was built against
  Zigbee2MQTT, but anything that can bind endpoints should work)
- Optional, for a long cable run between the ESP32 and the encoder:
  2.2 kΩ external pull-up resistors (see `KNOB_USE_INTERNAL_PULLUPS` below)

## Wiring

| Encoder pin            | DevKit pin |
|------------------------|------------|
| 3-pin side, left       | GPIO18     |
| 3-pin side, middle     | GND        |
| 3-pin side, right      | GPIO19     |
| 2-pin side, one pin    | GPIO20     |
| 2-pin side, other pin  | GND        |

Built-in pull-ups are on by default (`KNOB_USE_INTERNAL_PULLUPS 1` in
`main/knob_config.h`). Set it to `0` if you're using external 2.2 kΩ
resistors for a longer cable run between the encoder and the board.

## Zigbee channel

By default this scans every standard 2.4GHz Zigbee channel (11–26) while
joining, the same as any normal Zigbee router — it'll find your network
whatever channel your coordinator picked, with nothing to configure.

If you know your coordinator's channel and want faster, more reliable
joining, you can lock to it: open `main/knob_config.h` and switch
`KNOB_ZB_CHANNEL_MASK` to the single-channel option (commented, right below
the default).

## One-time toolchain setup (ESP-IDF v5.5.4)

ESP-IDF is Espressif's own build toolkit — a different thing from ESPHome.
On macOS with Homebrew:

```bash
brew install cmake ninja dfu-util
mkdir -p ~/esp; cd ~/esp
git clone -b v5.5.4 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32c6
```

If it complains about the Python version: `brew install python@3.12`, then
repeat `./install.sh esp32c6`. On Linux, the same `install.sh` script works
the same way — see Espressif's own
[Get Started guide](https://docs.espressif.com/projects/esp-idf/en/stable/esp32c6/get-started/)
for platform-specific prerequisites.

## Every new terminal window

ESP-IDF isn't on your `PATH` permanently — each new terminal session needs
this once before `idf.py` will work. Bash or zsh (the macOS/Linux default):

```bash
. ~/esp/esp-idf/export.sh
```

Fish:

```fish
. ~/esp/esp-idf/export.fish
```

(`idf.py: command not found` means you forgot this step.)

## Build and flash

```bash
git clone <this-repo-url>
cd zb-dimmer-knob
idf.py set-target esp32c6
idf.py build
```

The first build downloads the Zigbee library automatically via the
component manager (see `dependencies.lock`). Plug the DevKit into the port
labelled **UART** and find it:

```bash
ls /dev/cu.usb*        # macOS
ls /dev/ttyUSB*        # Linux
```

Flash — `erase-flash` wipes any leftover firmware from the board, only
needed the first time:

```bash
idf.py -p /dev/cu.usbmodemXXXX erase-flash flash monitor
```

`Ctrl+]` leaves the monitor. `idf.py -p ... monitor` reopens it.

## Pair and bind

1. If your Zigbee controller has an old entry for this board, remove it.
2. **Permit join** on your network. The log shows `JOINED as a router`.
3. In your Zigbee controller: find the new device → **Bind** → source
   endpoint **1** → your bulb → tick **OnOff** and **LevelCtrl** → **Bind**.
4. Turn the knob and press it.
5. **The acid test:** stop your Zigbee controller entirely (e.g. the Z2M
   add-on). It should still work.

## Console commands (type into the serial monitor)

| Command | What it does |
|---|---|
| `state` | What the knob thinks the lights are doing |
| `tune` | Show all settings; `tune <name> <value>` changes one live |
| `stats` | Counts, send failures, reports received |
| `info` | Network address, PAN, channel |
| `on` / `off` / `toggle` | Test the binding without the knob |
| `step up 20` | One brightness step |
| `steer`, `factoryreset` | Join a network now; wipe and start over |

## How it behaves, and why

The knob holds its own **intent**: a brightness number and on/off state.
Every command comes from that, so all bound bulbs converge on the same
numbers whatever any individual one missed.

- **While turning:** relative Step commands, so it feels immediate.
- **When you stop:** one absolute "go to this level". Sending it twice
  changes nothing, so bulbs that missed steps are pulled back into line.
  This is what keeps several bulbs in sync. `tune land 0` disables it.
- **The button** sends an explicit On or Off, never Toggle. A missed
  Toggle would leave bulbs permanently out of sync with each other; an
  explicit command can only ever converge.
- **Dimming never switches a bulb off.** Below `floor` it lands exactly on
  minimum, so bulbs don't drop out one at a time near the bottom.
- **Turning up from off** comes on at minimum brightness. **Pressing the
  button** restores the bulb's own previous brightness.
- **At the top or bottom** it sends nothing at all.
- **If the radio struggles** (more than a quarter of sends failing) the
  gap between commands widens automatically, and narrows again once it
  clears.

## Optional: let bulbs tell the knob about outside changes

Endpoint 2 exists only to receive reports. Bind each **bulb's** On/Off and
LevelCtrl to this device's **endpoint 2**. Then a change made through your
Zigbee controller (e.g. Home Assistant) updates the knob's internal model
within a second or two.

When several bulbs report, the knob takes the middle value rather than an
average, so one stray bulb can't drag it. Reports arriving during or just
after a turn are ignored, since those are only echoes of the knob's own
commands.

None of this is required. Bulbs that can't report simply never appear, and
the knob carries on trusting its own intent. `stats` shows how many
reports have arrived, which tells you whether the reverse binding took.

## Tuning

`tune log 0` first — printing every command adds delay.

`tick` gap between commands, `units` brightness per encoder count (2 = 8
per click), `fade` tenths of a second each command fades over, `settle`
stillness before the landing command, `floor` where the bottom lands,
`rev` opposite counts needed to change direction, `debounce` button
stability time.

Settings reset on reboot; copy the ones you like into `main/knob_config.h`
to make them permanent.

## Checked vs not checked

Every Zigbee type and function name was checked against Espressif's SDK
2.x API reference, and all source files were compiled against stand-in
headers built from those docs, with warnings treated as errors. It has
been built and run against the real SDK on real hardware, bound to a real
bulb — but it hasn't seen extended field use. Most likely spot to need a
tweak on a different SDK version: the report handler, since the exact
shape of a report message can shift between SDK releases. If the build
fails inside it, try changing `KNOB_REPORT_STRUCT_STYLE` to `1` in
`main/knob_config.h`; if that also fails, set `KNOB_USE_REPORTS` to `0` —
everything except outside-change tracking still works.

Known limitation: encoder interrupts pause for a few milliseconds while
the Zigbee stack writes to flash, which can very occasionally drop a
single count.

## Contributing / issues

Pull requests and issue reports welcome — this was built for one specific
setup, so reports of what breaks on other hardware or SDK versions are
genuinely useful.
