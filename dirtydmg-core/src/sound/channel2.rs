pub struct Channel2{
    // Raw control registers
    pub nr21: u8,
    pub nr22: u8,
    pub nr23: u8,
    pub nr24: u8,

    pub length_remaining: u16,
    pub freq_counter_mod: u16,
    pub freq_counter: u16,
    pub duty_pos: u8,
    pub output: u8,
}

impl Channel2 {
    const duty_patterns:[u8;4] = [0x2, 0x6, 0x1E, 0x7E];

    pub fn new() -> Channel2{
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
    
    pub fn clear(&mut self){
        self.nr21 = 0;
        self.nr22 = 0;
        self.nr23 = 0;
        self.nr24 = 0;
    }

    pub fn duty(&self) -> u8{
        self.nr21 >> 6
    }

    pub fn length(&self) -> u8 {
        self.nr21 & 0b0011_1111
    }

    pub fn env_initial_vol(&self) -> u8 {
        self.nr22 >> 4
    }

    pub fn env_dir(&self) -> u8 {
        (self.nr22 >> 3) & 1
    }

    pub fn sweep(&self) -> u8 {
        self.nr22 & 0b111
    }

    pub fn freq(&self) -> u16 {
        (((self.nr24 & 0b111) as u16) << 8) | (self.nr23) as u16
    }

    pub fn initial(&self) -> bool {
        (self.nr24 & 0b1000_0000) != 0
    }

    pub fn stop_on_length(&self) -> bool {
        (self.nr24 & 0b0100_0000) != 0
    }

    pub fn update_freq(&mut self){
        let fmod = (2048 - self.freq()) * 4;
        self.freq_counter_mod = fmod;
        self.freq_counter = fmod;
    }

    pub fn update_nr23(&mut self, value: u8){
        self.nr23 = value;
        self.update_freq();
    }

    pub fn update_nr24(&mut self, value: u8){
        self.nr24 = value & 0b1100_0111;
        self.update_freq();
    }

    pub fn tick(&mut self){
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