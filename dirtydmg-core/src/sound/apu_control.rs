use super::{AudioChannel, AudioOutput};

pub struct ApuControl {
    pub nr50: u8,
    pub nr51: u8,
    pub nr52: u8,
}

impl ApuControl {
    pub const NR52_AUDIO_ENABLED_BITMASK:u8 = 0x80;

    pub fn new() -> ApuControl{
        ApuControl {
            nr50: 0,
            nr51: 0,
            nr52: 0
        }
    }

    // Bit 6-4 - SO2 output level (volume)  (0-7)
    pub fn right_channel_volume(&self) -> u8{
        (self.nr50 >> 4) & 0b111
    }

    /// Gets the volume for the left sound channel
    /// Bit 2-0 - SO1 output level (volume)  (0-7)
    pub fn left_channel_volume(&self) -> u8 {
        self.nr50 & 0b111
    }

    /// Checks if a channel is being sent to an output.
    /// 
    /// output - The audio output to be checked
    /// channel - The channel to check against the output.
    /// returns true if the channel is enabled for the output.
    pub fn is_channel_on_output(&self, output:AudioOutput, channel:AudioChannel) -> bool{
        // Bit 7 - Output sound 4 to SO2 terminal
        // Bit 6 - Output sound 3 to SO2 terminal
        // Bit 5 - Output sound 2 to SO2 terminal
        // Bit 4 - Output sound 1 to SO2 terminal
        // Bit 3 - Output sound 4 to SO1 terminal
        // Bit 2 - Output sound 3 to SO1 terminal
        // Bit 1 - Output sound 2 to SO1 terminal
        // Bit 0 - Output sound 1 to SO1 terminal
        let mut shift = (output as u8) * 4;
        shift += channel as u8;
        // We know what bit we want, sample it.
        (self.nr51 & (1 << shift)) != 0
    }

    /// Sets the value of the channel active status bit.
    pub fn set_audio_channel_active_status(&mut self, active:bool, channel:AudioChannel) {
        let mask = 1 << (channel as u8);
        if active {
            self.nr52 |= mask;
        } else {
            self.nr52 &= !mask;
        }
    }

    /// Sets the audio enable bit. Nothing more.
    pub fn set_audio_enable_bit(&mut self, active:bool) {
        let mask  = 0x80;
        if active {
            self.nr52 |= mask;
        } else {
            self.nr52 &= !mask;
        }
    }

}

impl Default for ApuControl {
    fn default() -> Self {
        Self::new()
    }
}