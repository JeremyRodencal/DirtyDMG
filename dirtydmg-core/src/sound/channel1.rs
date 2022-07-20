use std::io::{Write, Read};
use byteorder::{WriteBytesExt, ReadBytesExt, LittleEndian};

#[derive(PartialEq, Debug)]
pub struct Channel1{
    // Raw control registers
    pub nr10: u8,
    pub nr11: u8,
    pub nr12: u8,
    pub nr13: u8,
    pub nr14: u8,

    // Shouldn't need to be public...
    pub length_counter: u16,
    pub envelope_counter: u8,
    pub freq_counter_mod: u16,
    pub freq_counter: u16,
    pub freq_shadow: u16,
    pub duty_pos: u8,
    sweep_counter: i8,
    pub output: u8,
    pub current_volume: u8,
    pub enabled: bool,
}

impl Channel1 {
    const DUTY_PATTERNS:[u8;4] = [0x2, 0x6, 0x1E, 0x7E];
    const LENGTH_TIMER_RELOAD:u16 = 64;

    pub fn new() -> Channel1{
        Channel1{
            nr10: 0,
            nr11: 0,
            nr12: 0,
            nr13: 0,
            nr14: 0,

            length_counter: 0,
            envelope_counter: 0,
            freq_counter_mod: 0,
            freq_counter: 0,
            freq_shadow:0,
            duty_pos: 0,
            output: 0,
            current_volume: 0,
            sweep_counter: 0,
            enabled: false,
        }
    }
    
    pub fn clear(&mut self){
        self.nr10 = 0;
        self.nr11 = 0;
        self.nr12 = 0;
        self.nr13 = 0;
        self.nr14 = 0;

        self.length_counter = 0;
        self.envelope_counter = 0;
        self.freq_counter_mod = 0;
        self.freq_counter = 0;
        self.freq_shadow =0;
        self.duty_pos = 0;
        self.output = 0;
        self.current_volume = 0;
        self.sweep_counter = 0;
        self.enabled = false;
    }

    fn sweep_period(&self) -> i8{
        ((self.nr10 >> 4) & 0b111) as i8
    }

    // True if the sweep is decreasing the frequency.
    fn sweep_decreasing(&self) -> bool{
        (self.nr10 & 0b1000) != 0
    }

    /// How many bits to shift on a sweep
    fn sweep_shift(&self) -> u16 {
        (self.nr10 & 0b111) as u16
    }

    pub fn duty(&self) -> u8{
        self.nr11 >> 6
    }

    pub fn length(&self) -> u8 {
        self.nr11 & 0b0011_1111
    }

    pub fn env_initial_vol(&self) -> u8 {
        self.nr12 >> 4
    }

    pub fn env_dir(&self) -> u8 {
        (self.nr12 >> 3) & 1
    }

    pub fn env_period(&self) -> u8 {
        self.nr12 & 0b111
    }

    pub fn freq(&self) -> u16 {
        (((self.nr14 & 0b111) as u16) << 8) | (self.nr13) as u16
    }

    pub fn initial(&self) -> bool {
        (self.nr14 & 0b1000_0000) != 0
    }

    pub fn stop_on_length(&self) -> bool {
        (self.nr14 & 0b0100_0000) != 0
    }

    pub fn update_freq(&mut self){
        let fmod = (2048 - self.freq()) * 4;
        self.freq_counter_mod = fmod;
    }

    pub fn set_freq(&mut self, value:u16){
        self.nr13 = value as u8;
        self.nr14 &= !0b111;
        self.nr14 |= (value >> 8) as u8 & 0b111;
    }

    pub fn update_nr12(&mut self, value: u8){
        self.nr12 = value;
        if self.nr12 >> 4 == 0 {
            self.disable();
        }
    }

    pub fn update_nr13(&mut self, value: u8){
        self.nr13 = value;
        self.freq_counter_mod = (2048 - self.freq()) * 4;
    }

    pub fn update_nr14(&mut self, value: u8){
        self.nr14 = value & 0b1100_0111;
        self.freq_counter_mod = (2048 - self.freq()) * 4;
        if value & 0x80 != 0{
            self.trigger();
        }
    }

    pub fn disable(&mut self){
        self.enabled = false;
    }
    
    // Function to handle the enable "trigger" event.
    fn trigger(&mut self){
        // Channel is enabled (see length counter).
        self.enabled = self.env_initial_vol() != 0;     
        self.length_counter = 0;
        self.update_freq();
        self.freq_shadow = self.freq();
        self.envelope_counter = 0;
        self.current_volume = self.env_initial_vol();
        self.sweep_counter = self.sweep_period();
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
                self.output = (Channel1::DUTY_PATTERNS[self.duty() as usize] >> self.duty_pos) & 0x1;
            }
        }
    }

    pub fn length_tick(&mut self) {
        // If the length counter is running
        if self.stop_on_length() {
            // increment the counter
            self.length_counter += 1;

            // If the length has expired.
            if self.length_counter >= Channel1::LENGTH_TIMER_RELOAD - self.length() as u16{
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

    pub fn sweep_tick(&mut self) {
        // Quick abort if sweep is disabled.
        if self.enabled{
            // Tick the sweep counter
            self.sweep_counter -= 1;

            // If the sweep counter has expired
            if self.sweep_counter <= 0 {
                // Reset the sweep period
                self.sweep_counter = self.sweep_period();
                if self.sweep_counter == 0 {
                    self.sweep_counter = 8;
                }

                if self.sweep_period() != 0 {
                    let mut f = self.freq_shadow;
                    let delta = f >> self.sweep_shift();
                    if self.sweep_decreasing(){
                        // Subtract off the delta
                        f -= delta;
                        self.freq_shadow = f;
                        self.set_freq(f);
                    } else {
                        // Add the delta to the frequency counter
                        f += delta;

                        // Disable the channel on overflow.
                        if f >= 2048 {
                            self.enabled = false;
                        } else {
                            self.freq_shadow = f;
                            self.set_freq(f);
                        }
                    }
                }
                    self.update_freq();
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

    pub fn serialize<T>(&self, writer: &mut T)
        where T: Write + ?Sized
    {
        // Raw control registers
        writer.write_u8(self.nr10).unwrap();
        writer.write_u8(self.nr11).unwrap();
        writer.write_u8(self.nr12).unwrap();
        writer.write_u8(self.nr13).unwrap();
        writer.write_u8(self.nr14).unwrap();

        // Shouldn't need to be public...
        writer.write_u16::<LittleEndian>(self.length_counter).unwrap();
        writer.write_u8(self.envelope_counter).unwrap();
        writer.write_u16::<LittleEndian>(self.freq_counter_mod).unwrap();
        writer.write_u16::<LittleEndian>(self.freq_counter).unwrap();
        writer.write_u16::<LittleEndian>(self.freq_shadow).unwrap();
        writer.write_u8(self.duty_pos).unwrap();
        writer.write_i8(self.sweep_counter).unwrap();
        writer.write_u8(self.output).unwrap();
        writer.write_u8(self.current_volume).unwrap();
        writer.write_u8(self.enabled as u8).unwrap();
    }

    pub fn deserialize<T>(&mut self, reader: &mut T)
        where T: Read + ?Sized
    {
        // Raw control registers
        self.nr10 = reader.read_u8().unwrap();
        self.nr11 = reader.read_u8().unwrap();
        self.nr12 = reader.read_u8().unwrap();
        self.nr13 = reader.read_u8().unwrap();
        self.nr14 = reader.read_u8().unwrap();

        // Shouldn't need to be public...
        self.length_counter = reader.read_u16::<LittleEndian>().unwrap();
        self.envelope_counter = reader.read_u8().unwrap();
        self.freq_counter_mod = reader.read_u16::<LittleEndian>().unwrap();
        self.freq_counter = reader.read_u16::<LittleEndian>().unwrap();
        self.freq_shadow = reader.read_u16::<LittleEndian>().unwrap();
        self.duty_pos = reader.read_u8().unwrap();
        self.sweep_counter = reader.read_i8().unwrap();
        self.output = reader.read_u8().unwrap();
        self.current_volume = reader.read_u8().unwrap();
        self.enabled = reader.read_u8().unwrap() != 0;
    }

}

impl Default for Channel1{
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn serialize_deserialize_loop()
    {
        let mut src = Channel1::new();
        src.nr10 = 1;
        src.nr11 = 2;
        src.nr12 = 3;
        src.nr13 = 4;
        src.nr14 = 5;
        src.length_counter = 6;
        src.envelope_counter = 7;
        src.freq_counter_mod = 8;
        src.freq_counter = 9;
        src.freq_shadow = 10;
        src.duty_pos = 11;
        src.sweep_counter = 12;
        src.output = 13;
        src.current_volume = 14;
        src.enabled = true;
        let src = src;

        let mut dst = Channel1::new();
        let mut data_buffer:Vec<u8> = Vec::new();
        src.serialize(&mut data_buffer);
        {
            let mut reader = &data_buffer[..];
            dst.deserialize(&mut reader);
        }

        assert_eq!(src, dst);
    }
}
