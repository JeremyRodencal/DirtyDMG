use crate::bus::BusRW;

pub struct Channel3{
    // Raw control registers
    pub nr30: u8,
    pub nr31: u8,
    pub nr32: u8,
    pub nr33: u8,
    pub nr34: u8,

    pub sample_data: [u8;32],

    // Shouldn't need to be public...
    pub length_counter: u16,
    pub freq_counter_mod: u16,
    pub freq_counter: u16,
    pub sample_index: u8,

    pub output: i8,
    pub current_volume: u8,
    pub enabled: bool,
}

impl Channel3 {
    const LENGTH_TIMER_RELOAD:u16 = 256;
    const SAMPLE_COUNT:usize = 32;

    pub fn new() -> Channel3{
        Channel3{
            nr30: 0,
            nr31: 0,
            nr32: 0,
            nr33: 0,
            nr34: 0,

            sample_data: [0;32],

            length_counter: 0,
            freq_counter_mod: 0,
            freq_counter: 0,

            sample_index: 0,

            output: 0,
            current_volume: 0,
            enabled: false,
        }
    }
    
    pub fn clear(&mut self){
        self.nr30 = 0;
        self.nr31 = 0;
        self.nr32 = 0;
        self.nr33 = 0;
        self.nr34 = 0;

        self.length_counter = 0;
        self.freq_counter_mod = 0;
        self.freq_counter = 0;

        self.sample_index = 0;

        self.output = 0;
        self.current_volume = 0;
        self.enabled = false;
    }

    pub fn sound_enabled(&self) -> bool{
        self.nr30 >> 7 != 0
    }

    pub fn length(&self) -> u8 {
        self.nr31
    }

    pub fn volume_shift(&self) -> u8 {
        (self.nr32 >> 5) & 0b11
    }

    pub fn freq(&self) -> u16 {
        (((self.nr34 & 0b111) as u16) << 8) | (self.nr33) as u16
    }

    pub fn stop_on_length(&self) -> bool {
        (self.nr34 & 0b0100_0000) != 0
    }

    pub fn update_freq(&mut self){
        let fmod = (2048 - self.freq()) * 2;
        self.freq_counter_mod = fmod;
        self.freq_counter = fmod;
    }

    pub fn update_nr33(&mut self, value: u8){
        self.nr33 = value;
        self.update_freq();
    }

    pub fn update_nr24(&mut self, value: u8){
        self.nr34 = value & 0b1100_0111;
        self.update_freq();
        if value & 0x80 != 0{
            self.trigger();
        }
    }
    
    // Function to handle the enable "trigger" event.
    fn trigger(&mut self){
        // Channel is enabled (see length counter).
        self.enabled = true;     
        self.length_counter = 0;
        self.update_freq();
        self.sample_index = 0;
        //TODO more stuff here.
    }

    pub fn tick(&mut self){
        // If the channel is currently enabled.
        if self.enabled && self.sound_enabled() {
            // check for frequency pattern advance.
            if self.freq_counter > 0 {
                self.freq_counter -= 1;
            }

            if self.freq_counter == 0{
                // Set the counter to the mod
                self.freq_counter = self.freq_counter_mod;

                // advance the duty cycle
                self.sample_index += 1;
                self.sample_index %= Channel3::SAMPLE_COUNT as u8;

                // Update the output.
                self.output = if self.volume_shift() != 0 {
                    ((self.sample_data[self.sample_index as usize] * 4) as i16 - 30) as i8
                } else {
                    0
                };

                if self.volume_shift() > 1 {
                    self.output /= 1 << (self.volume_shift()-1);
                }
            }
        }
    }

    pub fn length_tick(&mut self) {
        // If the length counter is running
        if self.stop_on_length() {
            // increment the counter
            self.length_counter += 1;

            // If the length has expired.
            if self.length_counter >= Channel3::LENGTH_TIMER_RELOAD - self.length() as u16{
                self.length_counter = 0;
                self.enabled = false;
            }
        }
    }

    pub fn sample(&self) -> i8{
        let mut sample = 0i8;
        if self.enabled {
            sample = self.output;
        }
        sample
    }
}

impl BusRW for Channel3{
    fn bus_write8(&mut self, addr: usize, value:u8) {
        match addr {
            // Sample ram
            0xFF30..=0xFF3F  => {
                let index = (addr - 0xFF30) * 2;
                self.sample_data[index] = value >> 4;
                self.sample_data[index+1] = value & 0xF;
            }
            _ => {}
        }
    }

    fn bus_read8(&mut self, addr: usize) -> u8{
        match addr {
            // Sample ram
            0xFF30..=0xFF3F => {
                let index = (addr - 0xFF30) * 2;
                (self.sample_data[index] << 4) | self.sample_data[index+1]
            }
            _ => {0xFF}
        }
    }
}