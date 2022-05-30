pub struct Channel2{
    // Raw control registers
    pub nr21: u8,
    pub nr22: u8,
    pub nr23: u8,
    pub nr24: u8,

    // Shouldn't need to be public...
    pub length_counter: u16,
    pub envelope_counter: u8,
    pub freq_counter_mod: u16,
    pub freq_counter: u16,
    pub duty_pos: u8,

    pub output: u8,
    pub current_volume: u8,
    pub enabled: bool,
}

impl Channel2 {
    const DUTY_PATTERNS:[u8;4] = [0x2, 0x6, 0x1E, 0x7E];
    const LENGTH_TIMER_RELOAD:u16 = 64;

    pub fn new() -> Channel2{
        Channel2{
            nr21: 0,
            nr22: 0,
            nr23: 0,
            nr24: 0,

            length_counter: 0,
            envelope_counter: 0,
            freq_counter_mod: 0,
            freq_counter: 0,
            duty_pos: 0,
            output: 0,
            current_volume: 0,
            enabled: false,
        }
    }
    
    pub fn clear(&mut self){
        self.nr21 = 0;
        self.nr22 = 0;
        self.nr23 = 0;
        self.nr24 = 0;
        self.enabled = false;
        self.length_counter = 0;
        self.envelope_counter = 0;
        self.freq_counter_mod = 0;
        self.freq_counter = 0;
        self.duty_pos = 0;
        self.output = 0;
        self.current_volume = 0;
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

    pub fn env_period(&self) -> u8 {
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
    }
    pub fn update_nr22(&mut self, value: u8){
        self.nr22 = value;
        if value >> 4 == 0 {
            self.enabled = false;
        }
    }

    pub fn update_nr23(&mut self, value: u8){
        self.nr23 = value;
        self.update_freq();
    }

    pub fn update_nr24(&mut self, value: u8){
        self.nr24 = value & 0b1100_0111;
        self.update_freq();
        if value & 0x80 != 0{
            self.trigger();
        }
    }
    
    // Function to handle the enable "trigger" event.
    fn trigger(&mut self){
        // Channel is enabled (if volume is not zero)
        self.enabled = self.env_initial_vol() != 0;     
        self.length_counter = 0;
        self.update_freq();
        self.freq_counter = 0;
        self.envelope_counter = 0;
        self.current_volume = self.env_initial_vol();
    }

    pub fn tick(&mut self){
        // If the channel is currently enabled.
        if self.enabled {
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
                self.output = (Channel2::DUTY_PATTERNS[self.duty() as usize] >> self.duty_pos) & 0x1;
            }
        }
    }

    pub fn length_tick(&mut self) {
        // If the length counter is running
        if self.stop_on_length() {
            // increment the counter
            self.length_counter += 1;

            // If the length has expired.
            if self.length_counter >= Channel2::LENGTH_TIMER_RELOAD - self.length() as u16{
                self.length_counter = 0;
                self.enabled = false;
            }
        }
    }

    pub fn envelope_tick(&mut self) {
        // Quick abort if env is disabled.
        if self.env_period() == 0{
            return;
        }

        // tick the envelope counter;
        self.envelope_counter += 1;

        // If an envelope period has elapsed.
        if self.envelope_counter >= self.env_period() {
            // Reset the counter
            self.envelope_counter = 0;

            // Update the volume
            if self.env_dir() > 0 {
                if self.current_volume < 15{
                    self.current_volume += 1;
                }
            } else if self.current_volume != 0 {
                self.current_volume -= 1;
            }
        }
    }

    pub fn sample(&self) -> i8 {
        let mut sample = 0i8;
        if self.enabled{
            sample = self.current_volume as i8 * 2;
            if self.output == 0 { 
                sample *= -1;
            }
        }
        sample
    }
}

impl Default for Channel2 {
    fn default() -> Self {
        Self::new()
    }
}