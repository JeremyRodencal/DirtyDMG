use std::io::{Read, Write};
use byteorder::{LittleEndian, ReadBytesExt, WriteBytesExt};

#[derive(PartialEq, Debug)]
pub struct Channel4{
    // Raw control registers
    pub nr41: u8,
    pub nr42: u8,
    pub nr43: u8,
    pub nr44: u8,

    // Shouldn't need to be public...
    pub length_counter: u16,
    pub envelope_counter: u8,
    pub freq_counter_mod: u16,
    pub freq_counter: u16,
    pub lfsr: u16,

    pub output: u8,
    pub current_volume: u8,
    pub enabled: bool,
}

impl Channel4 {
    const LENGTH_TIMER_RELOAD:u16 = 64;

    pub fn new() -> Channel4{
        Channel4{
            nr41: 0,
            nr42: 0,
            nr43: 0,
            nr44: 0,

            length_counter: 0,
            envelope_counter: 0,
            freq_counter_mod: 0,
            freq_counter: 0,
            lfsr: 0,
            output: 0,
            current_volume: 0,
            enabled: false,
        }
    }
    
    pub fn clear(&mut self){
        self.nr41 = 0;
        self.nr42 = 0;
        self.nr43 = 0;
        self.nr44 = 0;
        self.length_counter = 0;
        self.envelope_counter = 0;
        self.freq_counter_mod = 0;
        self.freq_counter = 0;
        self.lfsr = 0;
        self.output = 0;
        self.current_volume = 0;
        self.enabled = false;
    }

    pub fn length(&self) -> u8 {
        self.nr41 & 0b0011_1111
    }

    pub fn env_initial_vol(&self) -> u8 {
        self.nr42 >> 4
    }

    pub fn env_dir(&self) -> u8 {
        (self.nr42 >> 3) & 1
    }

    pub fn env_period(&self) -> u8 {
        self.nr42 & 0b111
    }

    pub fn shift_amount(&self) -> u8 {
        self.nr43 >> 4
    }

    pub fn shift_width_short(&self) -> bool {
        (self.nr43 & 0b1000) != 0
    }

    pub fn shift_div_code(&self) -> u8 {
        self.nr43 & 0b111
    }

    pub fn stop_on_length(&self) -> bool {
        (self.nr44 & 0b0100_0000) != 0
    }

    pub fn update_nr42(&mut self, value: u8) {
        self.nr42 = value;
        if value >> 4 == 0 {
            self.enabled = false;
        }
    }

    pub fn update_nr43(&mut self, value: u8) {
        self.nr43 = value;
        self.update_internal_freq_counters();
    }

    pub fn update_nr44(&mut self, value: u8) {
        self.nr44 = value & 0b1100_0000;
        if value & 0x80 != 0{
            self.trigger();
        }
    }

    /// Update the internal frequency counters
    pub fn update_internal_freq_counters(&mut self) {
        /* Pandocs has this:
            Frequency = 524288 Hz / r / 2^(s+1) ;For r=0 assume r=0.5 instead
            where 'r' is the bottom 3 bits of register NR43
            and 's' is the the top 4 bits of NR43.

            This really confused me for a while but I eventually figured out my solution below.
            First off, what is 534388 Hz? This is the cpu clock rate (2^22) divided by 8 (2^3),
            so that is where we start, a t cycle divider of 8.

            Next up is 'r'. R is further dividing the input clock, but with one extra detail.
            When r is zero, we are going to pretend that it is .5. That means it is going to
            double the rate of the input in that case. that means we roughtly have:
                divider = 8 * r;
                if divider == 0 { divider = 4} // 0 case, reduce the divider by a factor of 2.

            Finally the last bit. we need to further divide our signal by 2^(s+1).
            That ends up being pretty easy, it translates to multplying our divider by
            a power of two (aka, a bitwise left shift).
                divider <<= (s+1)

            We can do a tiny optimization here though. We can get rid of the +1 on the s
            if we just preemptively start with our divisor(s) increased by a factor of 2.
            That yeilds the final code below:
         */
        let mut d = 16 * self.shift_div_code() as u16;
        if d == 0 {
            d = 8 
        }
        d <<= self.shift_amount();

        self.freq_counter = d;
        self.freq_counter_mod = d;
    }
    
    // Function to handle the enable "trigger" event.
    fn trigger(&mut self){
        self.enabled = self.env_initial_vol() != 0;
        self.length_counter = 0;
        self.envelope_counter = 0;
        self.current_volume = self.env_initial_vol();
        self.update_internal_freq_counters();
        self.lfsr = 0x7FFF;
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
                // self.update_internal_freq_counters();

                // XOR bit - and 1 of lfsr
                let bit = ((self.lfsr>>1) ^ self.lfsr) & 1;

                // Shift the register, and place the bit into the end
                self.lfsr >>= 1;
                self.lfsr &= !(1 << 14);
                self.lfsr |= bit << 14;

                // If the register is configured to be "short", place the new bit in bit 6.
                if self.shift_width_short(){
                    self.lfsr &= !(1 << 6);
                    self.lfsr |= bit << 6;
                }

                // Update the output.
                self.output = (self.lfsr & 0x1) as u8;
            }
        }
    }

    pub fn length_tick(&mut self) {
        // If the length counter is running
        if self.stop_on_length() {
            // increment the counter
            self.length_counter += 1;

            // If the length has expired.
            if self.length_counter >= Channel4::LENGTH_TIMER_RELOAD - self.length() as u16{
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

    pub fn serialize<T>(&self, writer: &mut T)
        where T: Write + ?Sized
    {
        // Raw control registers
        writer.write_u8(self.nr41).unwrap();
        writer.write_u8(self.nr42).unwrap();
        writer.write_u8(self.nr43).unwrap();
        writer.write_u8(self.nr44).unwrap();

        // Internal registers
        writer.write_u16::<LittleEndian>(self.length_counter).unwrap();
        writer.write_u8(self.envelope_counter).unwrap();
        writer.write_u16::<LittleEndian>(self.freq_counter_mod).unwrap();
        writer.write_u16::<LittleEndian>(self.freq_counter).unwrap();
        writer.write_u16::<LittleEndian>(self.lfsr).unwrap();

        // output
        writer.write_u8(self.output).unwrap();
        writer.write_u8(self.current_volume).unwrap();
        writer.write_u8(self.enabled as u8).unwrap();
    }

    pub fn deserialize<T>(&mut self, reader: &mut T)
        where T: Read + ?Sized
    {
        // Raw control registers
        self.nr41 = reader.read_u8().unwrap();
        self.nr42 = reader.read_u8().unwrap();
        self.nr43 = reader.read_u8().unwrap();
        self.nr44 = reader.read_u8().unwrap();

        self.length_counter = reader.read_u16::<LittleEndian>().unwrap();
        self.envelope_counter = reader.read_u8().unwrap();
        self.freq_counter_mod = reader.read_u16::<LittleEndian>().unwrap();
        self.freq_counter = reader.read_u16::<LittleEndian>().unwrap();
        self.lfsr = reader.read_u16::<LittleEndian>().unwrap();

        self.output = reader.read_u8().unwrap();
        self.current_volume = reader.read_u8().unwrap();
        self.enabled = reader.read_u8().unwrap() != 0;
    }

}

impl Default for Channel4 {
    fn default() -> Self{
        Self::new()
    }
}


#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn serialize_deserialize_loop()
    {
        let mut src = Channel4::new();
        src.nr41 = 2;
        src.nr42 = 3;
        src.nr43 = 4;
        src.nr44 = 5;

        src.length_counter = 6;
        src.envelope_counter = 7;
        src.freq_counter_mod = 8;
        src.freq_counter = 9;
        src.lfsr = 0x1234;

        src.output = 13;
        src.current_volume = 14;
        src.enabled = true;
        let src = src;

        let mut dst = Channel4::new();
        let mut data_buffer:Vec<u8> = Vec::new();
        src.serialize(&mut data_buffer);
        {
            let mut reader = &data_buffer[..];
            dst.deserialize(&mut reader);
        }

        assert_eq!(src, dst);
    }
}