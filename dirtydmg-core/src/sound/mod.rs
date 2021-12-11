mod channel2;
mod apu_control;
use crate::bus::BusRW;
pub use channel2::*;
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

struct Apu {
    ch2: Channel2,
    ctrl: ApuControl,

    frame_sequence_counter: u16,
    frame_length_counter: u16,
    frame_envelope_counter: u16,
    frame_sweep_counter: u16,

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

    const FRAME_SEQUENCE_UPDATE_TICKS: u16 = 8192;
    const FRAME_SEQUENCE_LENGTH_TICKS: u16 = 2;
    const FRAME_SEQUENCE_ENVELOPE_TICKS: u16 = 8;
    const FRAME_SEQUENCE_SWEEP_TICKS: u16 = 4;

    pub fn new() -> Apu{
        Apu { 
            ch2: Channel2::new(),
            ctrl: ApuControl::new(),
            frame_sequence_counter: 0,
            frame_length_counter: 0,
            frame_envelope_counter: 0,
            frame_sweep_counter: 0,
            sample_ticks: 95,
            sample_subticks: 1201,
            sample_subtick_rate: 11025,
            subtick_counter: 0
        }
    }

    pub fn frame_sequencer_tick(&mut self){
        self.frame_sequence_counter += 1;
        if self.frame_sequence_counter > Apu::FRAME_SEQUENCE_UPDATE_TICKS {
            // Update the length counter.
            self.frame_length_counter += 1;
            if self.frame_length_counter >= Apu::FRAME_SEQUENCE_LENGTH_TICKS {
                self.frame_length_counter = 0;
                self.ch2.length_tick();
                // TODO tick all channels.
            }

            // Update the envelope counter
            self.frame_envelope_counter += 1;
            if self.frame_envelope_counter >= Apu::FRAME_SEQUENCE_ENVELOPE_TICKS {
                self.frame_envelope_counter = 0;
                self.ch2.envelope_tick();
                // TODO tick all channels.
            }

            // Update the sweep counter
            self.frame_sweep_counter += 1;
            if self.frame_sweep_counter >= Apu::FRAME_SEQUENCE_SWEEP_TICKS {
                self.frame_sweep_counter = 0;
                // TODO tick all channels.
            }
        }
    }

    pub fn tick(&mut self, ticks:u16){
        // TODO this is only a stub.
        for _ in 0..ticks {
            self.ch2.tick();
            self.frame_sequencer_tick();
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