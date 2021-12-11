use crate::bus::BusRW;

enum AudioChannel {
    Channel1 = 0,
    Channel2 = 1,
    Channel3 = 2,
    Channel4 = 3,
}

struct Channel2{
    // Raw control registers
    nr21: u8,
    nr22: u8,
    nr23: u8,
    nr24: u8,

    length_remaining: u16,
    freq_counter_mod: u16,
    freq_counter: u16,
    duty_pos: u8,
    output: u8,
}

impl Channel2 {
    const duty_patterns:[u8;4] = [0x2, 0x6, 0x1E, 0x7E];

    fn new() -> Channel2{
        Channel2{
            nr21: 0,
            nr22: 0,
            nr23: 0,
            nr24: 0,

            length_remaining: 0,
            freq_counter_mod: 0,
            freq_counter: 0,
            duty_pos: 0,
            output: 0,
        }
    }
    
    fn clear(&mut self){
        self.nr21 = 0;
        self.nr22 = 0;
        self.nr23 = 0;
        self.nr24 = 0;
    }

    fn duty(&self) -> u8{
        self.nr21 >> 6
    }

    fn length(&self) -> u8 {
        self.nr21 & 0b0011_1111
    }

    fn env_initial_vol(&self) -> u8 {
        self.nr22 >> 4
    }

    fn env_dir(&self) -> u8 {
        (self.nr22 >> 3) & 1
    }

    fn sweep(&self) -> u8 {
        self.nr22 & 0b111
    }

    fn freq(&self) -> u16 {
        (((self.nr24 & 0b111) as u16) << 8) | (self.nr23) as u16
    }

    fn initial(&self) -> bool {
        (self.nr24 & 0b1000_0000) != 0
    }

    fn stop_on_length(&self) -> bool {
        (self.nr24 & 0b0100_0000) != 0
    }

    fn update_freq(&mut self){
        let fmod = (2048 - self.freq()) * 4;
        self.freq_counter_mod = fmod;
        self.freq_counter = fmod;
    }

    fn update_nr23(&mut self, value: u8){
        self.nr23 = value;
        self.update_freq();
    }

    fn update_nr24(&mut self, value: u8){
        self.nr24 = value & 0b1100_0111;
        self.update_freq();
    }

    fn tick(&mut self){
        // Just assume always on for now.
        let running = true; // TODO change this to account for length
        if running{
            // check for frequency pattern advance.
            if self.freq_counter > 0 {
                self.freq_counter -= 1;
            }

            if self.freq_counter == 0{
                // Set the counter to the mod
                self.freq_counter = self.freq_counter_mod;

                // advance the duty cycle
                self.duty_pos += 1;
                self.duty_pos %= 8;

                // Update the output.
                self.output = (Channel2::duty_patterns[self.duty() as usize] >> self.duty_pos) & 0x1;
            }
        }
    }
}

enum AudioOutput {
    Output1 = 0,
    Output2 = 1,
}

struct ApuControl {
    nr50: u8,
    nr51: u8,
    nr52: u8,
}

impl ApuControl {
    pub const NR52_AUDIO_ENABLED_BITMASK:u8 = 0x80;

    fn new() -> ApuControl{
        ApuControl {
            nr50: 0,
            nr51: 0,
            nr52: 0
        }
    }

    // Bit 6-4 - SO2 output level (volume)  (0-7)
    fn right_channel_volume(&self) -> u8{
        (self.nr50 >> 4) & 0b111
    }

    /// Gets the volume for the left sound channel
    /// Bit 2-0 - SO1 output level (volume)  (0-7)
    fn left_channel_volume(&self) -> u8 {
        self.nr50 & 0b111
    }

    /// Checks if a channel is being sent to an output.
    /// 
    /// output - The audio output to be checked
    /// channel - The channel to check against the output.
    /// returns true if the channel is enabled for the output.
    fn is_channel_on_output(&self, output:AudioOutput, channel:AudioChannel) -> bool{
        // Bit 7 - Output sound 4 to SO2 terminal
        // Bit 6 - Output sound 3 to SO2 terminal
        // Bit 5 - Output sound 2 to SO2 terminal
        // Bit 4 - Output sound 1 to SO2 terminal
        // Bit 3 - Output sound 4 to SO1 terminal
        // Bit 2 - Output sound 3 to SO1 terminal
        // Bit 1 - Output sound 2 to SO1 terminal
        // Bit 0 - Output sound 1 to SO1 terminal
        let mut shift = output as u8;         
        shift += channel as u8 * 4;
        // We know what bit we want, sample it.
        (self.nr51 & (1 << shift)) != 0
    }

    /// Sets the value of the channel active status bit.
    fn set_audio_channel_active_status(&mut self, active:bool, channel:AudioChannel) {
        let mask = 1 << (channel as u8);
        if active {
            self.nr52 |= mask;
        } else {
            self.nr52 &= !mask;
        }
    }

    /// Sets the audio enable bit. Nothing more.
    fn set_audio_enable_bit(&mut self, active:bool) {
        let mask  = 0x80;
        if active {
            self.nr52 |= mask;
        } else {
            self.nr52 &= !mask;
        }
    }

}


struct Apu {
    ch2: Channel2,
    ctrl: ApuControl,

    subtick_counter: u16,
    sample_ticks: u16,
    sample_subticks: u16,
    sample_subtick_rate: u16,
}

impl Apu {
    const NR21_ADDRESS:usize = 0xFF16;
    const NR22_ADDRESS:usize = 0xFF17;
    const NR23_ADDRESS:usize = 0xff18;
    const NR24_ADDRESS:usize = 0xff19;

    const NR50_ADDRESS:usize = 0xff23;
    const NR51_ADDRESS:usize = 0xff24;
    const NR52_ADDRESS:usize = 0xff25;

    pub fn new() -> Apu{
        Apu{ 
            ch2: Channel2::new(),
            ctrl: ApuControl::new(),
            sample_ticks: 95,
            sample_subticks: 1201,
            sample_subtick_rate: 11025,
            subtick_counter: 0
        }
    }

    pub fn tick(&mut self, ticks:u16){
        // TODO this is only a stub.
        for _ in 0..ticks {
            self.ch2.tick();
        }
    }

    fn enable_disable_audio(&mut self, enabled: bool) {
        if !enabled {
            self.ch2.clear();
            // TODO clear other channels.
        } else {
            self.ctrl.nr52 |= 0xF;
        }
    }
}

impl BusRW for Apu {
    fn bus_write8(&mut self, addr:usize, value:u8){
        match addr {
            Apu::NR21_ADDRESS => {
                self.ch2.nr21 = value;
            }
            Apu::NR22_ADDRESS => {
                self.ch2.nr22 = value;
            }
            Apu::NR23_ADDRESS => {
                self.ch2.update_nr23(value);
            }
            Apu::NR24_ADDRESS => {
                self.ch2.update_nr24(value)
            }

            /* Control Registers */
            Apu::NR50_ADDRESS => {
                self.ctrl.nr50 = value;
            }
            Apu::NR51_ADDRESS => {
                self.ctrl.nr51 = value;
            }
            Apu::NR52_ADDRESS => {
                self.ctrl.nr52 = value;
                // Handle audio enable or disable based on the bit.
                let enabled = value & ApuControl::NR52_AUDIO_ENABLED_BITMASK != 0;
                self.enable_disable_audio(enabled);
            }
            
            _ => {}
        }
    }

    fn bus_read8(&mut self, addr:usize) -> u8{
        match addr {
            Apu::NR21_ADDRESS => {
                // Pandocs seems to imply that only the top 2 bits are readable.
                self.ch2.nr21 & 0b1100_0000
            }
            Apu::NR22_ADDRESS => {
                self.ch2.nr22
            }
            // Write only  (low frequency bits)
            Apu::NR23_ADDRESS => { 0 }
            // Only bit 6 can be written
            Apu::NR24_ADDRESS => {
                self.ch2.nr24 & 0b0100_0000
            }

            // Control registers
            Apu::NR50_ADDRESS => {
                self.ctrl.nr50
            }
            Apu::NR51_ADDRESS => {
                self.ctrl.nr51
            }
            Apu::NR52_ADDRESS => {
                self.ctrl.nr52
            }

            // TODO - turn this into a panic
            _ => {0xFF}
        }
    }

    // Just satisfies trait.
    fn bus_write16(&mut self, _:usize, _:u16){}
    fn bus_read16(&mut self, _:usize) -> u16{panic!("APU 16 bit bus read should never happen")}
}

mod test{
    use super::*;

    #[test]
    fn channel2_set_frequency_low(){
        let mut apu = Apu::new();

        let low_freq = 0x5Au8;
        let expected_freq = (2048 - low_freq as u16) * 4;
        // Write the low 8 frequency bits.
        apu.bus_write8(Apu::NR23_ADDRESS, low_freq);
        assert_eq!(apu.ch2.nr23, low_freq);
        assert_eq!(apu.ch2.freq(), low_freq as u16);
        assert_eq!(apu.ch2.freq_counter_mod, expected_freq);
        assert_eq!(apu.ch2.freq_counter, expected_freq);
    }

    #[test]
    fn channel2_set_frequency_high(){
        let mut apu = Apu::new();

        let high_freq = 0x05u8;
        let expected_freq = (2048 - (0x5u16 << 8)) * 4;
        // Write the low 8 frequency bits.
        apu.bus_write8(Apu::NR24_ADDRESS, high_freq);
        assert_eq!(apu.ch2.nr24 & 0b111, high_freq);
        assert_eq!(apu.ch2.freq(), (high_freq as u16) << 8);
        assert_eq!(apu.ch2.freq_counter_mod, expected_freq);
        assert_eq!(apu.ch2.freq_counter, expected_freq);
    }

    #[test]
    fn channel2_freq_advance(){
        let mut apu = Apu::new();

        // Maximum frequency
        apu.bus_write8(Apu::NR23_ADDRESS, 0xFF);
        apu.bus_write8(Apu::NR24_ADDRESS, 0xFF);

        //Duty pattern 0
        apu.bus_write8(Apu::NR21_ADDRESS, 0b00 << 6);

        let expected_output = [0,1,0,0,0,0,0,0, 0,1,0,0,0,0,0,0,];
        for output in expected_output {
            assert_eq!(apu.ch2.output, output);
            apu.ch2.tick();
            apu.ch2.tick();
            apu.ch2.tick();
            apu.ch2.tick();
        }

        apu.bus_write8(Apu::NR21_ADDRESS, 0b01 << 6);
        let expected_output = [0,1,1,0,0,0,0,0];
        for output in expected_output {
            assert_eq!(apu.ch2.output, output);
            apu.ch2.tick();
            apu.ch2.tick();
            apu.ch2.tick();
            apu.ch2.tick();
        }

        apu.bus_write8(Apu::NR21_ADDRESS, 0b10 << 6);
        let expected_output = [0,1,1,1,1,0,0,0];
        for output in expected_output {
            assert_eq!(apu.ch2.output, output);
            apu.ch2.tick();
            apu.ch2.tick();
            apu.ch2.tick();
            apu.ch2.tick();
        }

        apu.bus_write8(Apu::NR21_ADDRESS, 0b11 << 6);
        let expected_output = [0,1,1,1,1,1,1,0];
        for output in expected_output {
            assert_eq!(apu.ch2.output, output);
            apu.ch2.tick();
            apu.ch2.tick();
            apu.ch2.tick();
            apu.ch2.tick();
        }

    }
}