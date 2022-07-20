use std::io::{Read,Write};
use byteorder::{ReadBytesExt, WriteBytesExt, LittleEndian};
mod channel1;
mod channel2;
mod channel3;
mod channel4;
mod apu_control;
use crate::bus::BusRW;
pub use channel1::*;
pub use channel2::*;
pub use channel3::*;
pub use channel4::*;
pub use apu_control::*;

pub enum AudioChannel {
    Channel1 = 0,
    Channel2 = 1,
    Channel3 = 2,
    Channel4 = 3,
}

pub enum AudioOutput {
    Output1 = 0,
    Output2 = 1,
}

pub struct Apu {
    ch1: Channel1,
    ch2: Channel2,
    ch3: Channel3,
    ch4: Channel4,
    ctrl: ApuControl,

    frame_sequence_counter: u16,
    frame_length_counter: u16,
    frame_envelope_counter: u16,
    frame_sweep_counter: u16,

    subtick_counter: u16,
    sample_ticks: u16,

    sample: Option<(i8, i8)>,
    ch_user_enable: [bool;4],
}

impl Apu {
    const NR10_ADDRESS:usize = 0xFF10;
    const NR11_ADDRESS:usize = 0xFF11;
    const NR12_ADDRESS:usize = 0xFF12;
    const NR13_ADDRESS:usize = 0xFF13;
    const NR14_ADDRESS:usize = 0xFF14;

    const NR21_ADDRESS:usize = 0xFF16;
    const NR22_ADDRESS:usize = 0xFF17;
    const NR23_ADDRESS:usize = 0xff18;
    const NR24_ADDRESS:usize = 0xff19;

    const NR30_ADDRESS:usize = 0xFF1A;
    const NR31_ADDRESS:usize = 0xFF1B;
    const NR32_ADDRESS:usize = 0xFF1C;
    const NR33_ADDRESS:usize = 0xFF1D;
    const NR34_ADDRESS:usize = 0xFF1E;

    const NR41_ADDRESS:usize = 0xFF20;
    const NR42_ADDRESS:usize = 0xFF21;
    const NR43_ADDRESS:usize = 0xFF22;
    const NR44_ADDRESS:usize = 0xFF23;

    const NR50_ADDRESS:usize = 0xFF24;
    const NR51_ADDRESS:usize = 0xFF25;
    const NR52_ADDRESS:usize = 0xFF26;

    const FRAME_SEQUENCE_UPDATE_TICKS: u16 = 8192;
    const FRAME_SEQUENCE_LENGTH_TICKS: u16 = 2;
    const FRAME_SEQUENCE_ENVELOPE_TICKS: u16 = 8;
    const FRAME_SEQUENCE_SWEEP_TICKS: u16 = 4;

    // TODO - this is currently all hardcoded to 44.1khz sample rates.
    const SAMPLE_TICKS:u16 = 95;
    const SAMPLE_SUBTICK_INCREMENT: u16 = 1201;
    const SAMPLE_SUBTICK_LIMIT: u16 = 11025;

    pub fn new() -> Apu{
        Apu { 
            ch1: Channel1::new(),
            ch2: Channel2::new(),
            ch3: Channel3::new(),
            ch4: Channel4::new(),
            ctrl: ApuControl::new(),
            frame_sequence_counter: 0,
            frame_length_counter: 0,
            frame_envelope_counter: 0,
            frame_sweep_counter: 0,
            sample_ticks: 95,
            subtick_counter: 0,
            sample: None,
            ch_user_enable: [true;4],
        }
    }

    pub fn frame_sequencer_tick(&mut self){
        self.frame_sequence_counter += 1;
        if self.frame_sequence_counter >= Apu::FRAME_SEQUENCE_UPDATE_TICKS {
            self.frame_sequence_counter = 0;

            // Update the length counter.
            self.frame_length_counter += 1;
            if self.frame_length_counter >= Apu::FRAME_SEQUENCE_LENGTH_TICKS {
                self.frame_length_counter = 0;
                self.ch1.length_tick();
                self.ch2.length_tick();
                self.ch3.length_tick();
                self.ch4.length_tick();
            }

            // Update the envelope counter
            self.frame_envelope_counter += 1;
            if self.frame_envelope_counter >= Apu::FRAME_SEQUENCE_ENVELOPE_TICKS {
                self.frame_envelope_counter = 0;
                self.ch1.envelope_tick();
                self.ch2.envelope_tick();
                self.ch4.envelope_tick();
            }

            // Update the sweep counter
            self.frame_sweep_counter += 1;
            if self.frame_sweep_counter >= Apu::FRAME_SEQUENCE_SWEEP_TICKS {
                self.frame_sweep_counter = 0;
                self.ch1.sweep_tick();
            }
        }
    }

    fn generate_sample(&mut self){
        let mut left = 0i16;
        let mut right = 0i16;

        if self.ch_user_enable[AudioChannel::Channel1 as usize]{
            if self.ctrl.is_channel_on_output(AudioOutput::Output1, AudioChannel::Channel1)
            {
                left += self.ch1.sample() as i16;
            }
            if self.ctrl.is_channel_on_output(AudioOutput::Output2, AudioChannel::Channel1)
            {
                right += self.ch1.sample() as i16;
            }
        }

        if self.ch_user_enable[AudioChannel::Channel2 as usize]{
            if self.ctrl.is_channel_on_output(AudioOutput::Output1, AudioChannel::Channel2)
            {
                left += self.ch2.sample() as i16;
            }
            if self.ctrl.is_channel_on_output(AudioOutput::Output2, AudioChannel::Channel2)
            {
                right += self.ch2.sample() as i16;
            }
        }

        if self.ch_user_enable[AudioChannel::Channel3 as usize]{
            if self.ctrl.is_channel_on_output(AudioOutput::Output1, AudioChannel::Channel3)
            {
                left += self.ch3.sample() as i16;
            }
            if self.ctrl.is_channel_on_output(AudioOutput::Output2, AudioChannel::Channel3)
            {
                right += self.ch3.sample() as i16;
            }
        }

        if self.ch_user_enable[AudioChannel::Channel4 as usize]{
            if self.ctrl.is_channel_on_output(AudioOutput::Output1, AudioChannel::Channel4)
            {
                left += self.ch4.sample() as i16;
            }
            if self.ctrl.is_channel_on_output(AudioOutput::Output2, AudioChannel::Channel4)
            {
                right += self.ch4.sample() as i16;
            }
        }

        // TODO respect volume register

        fn trim(samp: i16) -> i8{
            if samp > 127{
                127
            }
            else if samp < -128 {
                -128
            }
            else {
                samp as i8
            }
        }

        self.sample = Some((trim(left), trim(right)));
    }

    fn sample_tick(&mut self) {
        // Increment ticks
        self.sample_ticks += 1;

        // If enough ticks have occured for a sample to be generated.
        if self.sample_ticks >= Apu::SAMPLE_TICKS {

            // Increment the subticks
            self.subtick_counter += Apu::SAMPLE_SUBTICK_INCREMENT;
            if self.subtick_counter >= Apu::SAMPLE_SUBTICK_LIMIT {
                // The subticks have added up to an entire tick.
                // clear the stubticks, but don't take a sample yet.
                self.subtick_counter -= Apu::SAMPLE_SUBTICK_LIMIT;
            }
            // No subtick overflow to delay for. Take a sample.
            else {
                self.sample_ticks = 0;
                self.generate_sample();
            }
        }
    }

    pub fn get_sample(&mut self) -> Option<(i8, i8)>{
        let sample = self.sample;
        self.sample = None;

        sample
    }

    pub fn tick(&mut self, ticks:u16){
        for _ in 0..ticks {
            self.ch1.tick();
            self.ch2.tick();
            self.ch3.tick();
            self.ch4.tick();
            self.frame_sequencer_tick();
            self.sample_tick();
        }
    }

    fn enable_disable_audio(&mut self, enabled: bool) {
        if !enabled {
            self.ch1.clear();
            self.ch2.clear();
            self.ch3.clear();
            self.ch4.clear();
        } else {
            self.ctrl.nr52 |= 0xF;
        }
    }

    pub fn user_set_channel_enable(&mut self, channel: AudioChannel, enabled: bool){
        self.ch_user_enable[channel as usize] = enabled;
    }

    pub fn user_set_channel_enable_toggle(&mut self, channel: AudioChannel){
        let channel = channel as usize;
        self.ch_user_enable[channel as usize] = !self.ch_user_enable[channel as usize];
    }

    pub fn serialize<T>(&self, writer:&mut T)
        where T: Write + ?Sized
    {
        self.ch1.serialize(writer);
        self.ch2.serialize(writer);
        self.ch3.serialize(writer);
        self.ch4.serialize(writer);
        self.ctrl.serialize(writer);

        writer.write_u16::<LittleEndian>(self.frame_sequence_counter).unwrap();
        writer.write_u16::<LittleEndian>(self.frame_length_counter).unwrap();
        writer.write_u16::<LittleEndian>(self.frame_envelope_counter).unwrap();
        writer.write_u16::<LittleEndian>(self.frame_sweep_counter).unwrap();

        writer.write_u16::<LittleEndian>(self.subtick_counter).unwrap();
        writer.write_u16::<LittleEndian>(self.sample_ticks).unwrap();
    }

    pub fn deserialize<T>(&mut self, reader:&mut T)
        where T: Read + ?Sized
    {
        self.ch1.deserialize(reader);
        self.ch2.deserialize(reader);
        self.ch3.deserialize(reader);
        self.ch4.deserialize(reader);
        self.ctrl.deserialize(reader);

        self.frame_sequence_counter = reader.read_u16::<LittleEndian>().unwrap();
        self.frame_length_counter = reader.read_u16::<LittleEndian>().unwrap();
        self.frame_envelope_counter = reader.read_u16::<LittleEndian>().unwrap();
        self.frame_sweep_counter = reader.read_u16::<LittleEndian>().unwrap();

        self.subtick_counter = reader.read_u16::<LittleEndian>().unwrap();
        self.sample_ticks = reader.read_u16::<LittleEndian>().unwrap();
    }
        
}

impl BusRW for Apu {
    fn bus_write8(&mut self, addr:usize, value:u8){
        match addr {
            // Channel 1
            Apu::NR10_ADDRESS => {
                self.ch1.nr10 = value;
            }
            Apu::NR11_ADDRESS => {
                self.ch1.nr11 = value;
            }
            Apu::NR12_ADDRESS => {
                self.ch1.update_nr12(value);
            }
            Apu::NR13_ADDRESS => {
                self.ch1.update_nr13(value);
            }
            Apu::NR14_ADDRESS => {
                self.ch1.update_nr14(value);
            }

            // Channel 2
            Apu::NR21_ADDRESS => {
                // println!("NR21: {:#02X}", value);
                self.ch2.nr21 = value;
            }
            Apu::NR22_ADDRESS => {
                // println!("NR22: {:#02X}", value);
                self.ch2.update_nr22(value);
            }
            Apu::NR23_ADDRESS => {
                // println!("NR23: {:#02X}", value);
                self.ch2.update_nr23(value);
            }
            Apu::NR24_ADDRESS => {
                // println!("NR24: {:#02X}", value);
                self.ch2.update_nr24(value)
            }

            // Channel 3
            Apu::NR30_ADDRESS => {
                self.ch3.nr30 = value & 0x80;
                self.ch3.update_nr30(value);
            }
            Apu::NR31_ADDRESS => {
                self.ch3.nr31 = value;
            }
            Apu::NR32_ADDRESS => {
                self.ch3.nr32 = value & 0b0110_0000;
            }
            Apu::NR33_ADDRESS => {
                self.ch3.update_nr33(value)
            }
            Apu::NR34_ADDRESS => {
                self.ch3.update_nr24(value)
            }

            // Channel 4
            Apu::NR41_ADDRESS => {
                self.ch4.nr41 = value
            }
            Apu::NR42_ADDRESS => {
                self.ch4.update_nr42(value);
            }
            Apu::NR43_ADDRESS => {
                self.ch4.update_nr43(value)
            }
            Apu::NR44_ADDRESS => {
                self.ch4.update_nr44(value)
            }

            /* Control Registers */
            Apu::NR50_ADDRESS => {
                // println!("NR50: {:#02X}", value);
                self.ctrl.nr50 = value;
            }
            Apu::NR51_ADDRESS => {
                // println!("NR51: {:#02X}", value);
                self.ctrl.nr51 = value;
            }
            Apu::NR52_ADDRESS => {
                // println!("NR52: {:#02X}", value);
                self.ctrl.nr52 = value;
                // Handle audio enable or disable based on the bit.
                let enabled = value & ApuControl::NR52_AUDIO_ENABLED_BITMASK != 0;
                self.enable_disable_audio(enabled);
            }

            0xFF30..=0xFF3F => {
                self.ch3.bus_write8(addr, value);
            }
            
            _ => {}
        }
    }

    fn bus_read8(&mut self, addr:usize) -> u8{
        match addr {
            // Channel 1
            Apu::NR10_ADDRESS => {
                self.ch1.nr10
            }
            Apu::NR11_ADDRESS => {
                self.ch1.nr11 & 0b1100_0000
            }
            Apu::NR12_ADDRESS => {
                self.ch1.nr12
            }
            Apu::NR13_ADDRESS => { 0 }
            // Only bit 6 can be read.
            Apu::NR14_ADDRESS => {
                self.ch1.nr14 & 0b0100_0000
            }

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

            Apu::NR30_ADDRESS => {
                self.ch3.nr30 & 0x80
            }
            Apu::NR31_ADDRESS => {
                0xFF // write only
            }
            Apu::NR32_ADDRESS => {
                self.ch3.nr32 & 0b0110_0000
            }
            Apu::NR33_ADDRESS => {
                0xFF // write only
            }
            Apu::NR34_ADDRESS => {
                self.ch3.nr34 & 0b0100_0000
            }

            // Channel 4
            Apu::NR41_ADDRESS => {
                0xFF
            }
            Apu::NR42_ADDRESS => {
                self.ch4.nr42
            }
            Apu::NR43_ADDRESS => {
                self.ch4.nr43
            }
            Apu::NR44_ADDRESS => {
                self.ch4.nr44 & 0b0100_0000
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

            0xFF30..=0xFF3F => {
                self.ch3.bus_read8(addr)
            }

            // TODO - turn this into a panic
            _ => {0xFF}
        }
    }
}

impl Default for Apu {
    fn default() -> Self {
        Self::new()
    }
}


#[cfg(test)]
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
    }

    #[test]
    fn channel2_freq_advance(){
        let mut apu = Apu::new();

        // Enable DAC
        apu.bus_write8(Apu::NR22_ADDRESS, 0xF0);

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