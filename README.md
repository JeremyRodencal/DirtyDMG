# Overview

DirtyDMG is a quick and dirty gameboy emulator written in rust. There are many
excellent gameboy emulators out there, but this was started as a personal attempt
to learn about the Gameboy, emulation, and rust all at the same time!

As you would expect, it isn't (even remotely near) perfect, but it does work...

# Usage

Currently windows and linux are supported. There is not GUI (yet...) but for now,
a rom image can be loaded by passing it's path as the first command line argument.

## Basic Input
Emulator controls are:

|Keyboard Key| Gameboy Button|
|------------|---------------|
|z|B|
|x|A|
|c|Select|
|Enter|Start|
|Up Arrow|Dpad Up|
|Down Arrow|Dpad Down|
|Left Arrow|Dpad Left|
|Right Arrow|Dpad Right|

## Save States
The current state can be saved by pressing F5 during gameplay. Once a state has been saved, it can be loaded by pressing F8. Note that this state is not saved to disk, it is lost when the emulator is closed!

## Audio Channel Control
Each audio channel can be enabled or disabled by pressing the corresponding number key.

|Keyboard Key| Audio Channel|
|------------|--------------|
|1| Square Wave 1|
|2| Square Wave 2|
|3| Custom Waveform|
|4| Noise Channel|

# Internal Details

## Structure
This emulator is broken up into two major crates.
- dirtydmg-core (Library)
- dirtydmg-pc (Binary)

### dirtydmg-core

Dirtydmg core is as dependency pure as possible. It doesn't make OS calls. It doesn't access the file system.
It doesn't draw to the screen. It doesn't look for user input. All of those things are meant to be handled
by a separate module for a given platform. The idea is to promote portability in the core to eventually
support multiple platforms (Only PCs are supported for now, but we can dream!).

### dirtydmg-pc

A binary crate that provides support for both linux and windows. This is done by using SDL2 to handle
graphics, sound, input, etc. The rust standard library is used for all other OS interaction (files, etc).

## Emulation Strategy

This is just a simple interpreter. Each instruction executes atomically, then other peripherals are ticked
according to the number of clock cycles that were needed. There is a certain amount of inherent timing
inaccuracy in this, but the focus for now is on getting as much working with as little effort as possible.
In time, this may change as the desire for more compatibility drives improvements. 

Timing (in the PC binary) is done via audio. Once a fixed small audio buffer is full, the audio is handed
off to the audio driver, and the emulator runs until the next one is prepared. This leads to some input
latency, but performance is generally acceptable.
