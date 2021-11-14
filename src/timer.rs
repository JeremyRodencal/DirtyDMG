use crate::bus::BusRW;
use crate::interrupt::InterruptStatus;

const DIV_REG_ADDR:usize  = 0xFF04;
const TIMA_REG_ADDR:usize = 0xFF05;
const TMA_REG_ADDR:usize  = 0xFF06;
const TAC_REG_ADDR:usize  = 0xFF07;

pub struct TimerUnit {
    // Raw registers
    div:u8,
    tima:u8,
    tma:u8,
    tac:u8,

    // Derived from tac
    ticks_per_inc: u16,
    enabled: bool,

    // Timing functions.
    tima_inc_counter: u16,
    div_inc_counter: u16,
}

impl TimerUnit {
    const ticks_per_div:u16 = 256;

    pub fn new() -> TimerUnit {
        TimerUnit {
            div: 0,
            tima: 0,
            tma: 0,
            tac: 0,
            ticks_per_inc: 1024,
            enabled: false,
            tima_inc_counter: 0,
            div_inc_counter: 0
        }
    }

    pub fn update(&mut self, cpu_ticks:u16 , is:&mut InterruptStatus) {
        // Update div counter 
        // TODO account for stop instruction.
        self.div_inc_counter += cpu_ticks;
        if self.div_inc_counter >= TimerUnit::ticks_per_div {
            let ticks = self.div_inc_counter / TimerUnit::ticks_per_div;
            self.div = self.div.wrapping_add(ticks as u8);
            self.div_inc_counter %= TimerUnit::ticks_per_div;
        }

        if self.enabled{
            // Update the tima counter
            // TODO account for stop instruction.
            self.tima_inc_counter += cpu_ticks;
            if self.tima_inc_counter >= self.ticks_per_inc {
                let ticks = self.tima_inc_counter / self.ticks_per_inc;
                self.tima_inc_counter %= self.ticks_per_inc;

                // If we are wrapping
                if ticks + self.tima as u16 >= 256 {
                    self.tima = self.tima.wrapping_add(ticks as u8) + self.tma;
                    is.request_timer();
                } else {
                    self.tima += ticks as u8;
                }
            }
        }
    }
}

impl BusRW for TimerUnit {

    fn bus_write8(&mut self, addr: usize, value:u8){
       match addr {
            DIV_REG_ADDR => {
                self.div = 0;
            }
            TIMA_REG_ADDR => {
                println!("TIMA write: {}", value);
                self.tima = value;
            }
            TMA_REG_ADDR => {
                self.tma = value;
            }
            TAC_REG_ADDR => {
                self.tac = value;
                self.ticks_per_inc = 
                    match value & 0b11 {
                        0 => 1024,
                        1 => 16,
                        2 => 64,
                        3 => 256,
                        _ => {0}
                    };
                self.enabled = value & 0b100 != 0;

                println!("Timer enabled: {}", self.enabled);
                println!("Timer rate: {}", self.ticks_per_inc);
            }
            _ => {panic!("TimerUnit: Unknown read at address {:#X}", addr);}
       } 
    }

    fn bus_read8(&mut self, addr: usize) -> u8 {
        match addr {
            DIV_REG_ADDR => {
                self.div
            }
            TIMA_REG_ADDR => {
                println!("tima read: {}", self.tima);
                self.tima
            }
            TMA_REG_ADDR => {
                self.tma
            }
            TAC_REG_ADDR => {
                self.tac
            }
            _ => {panic!("TimerUnit: Unknown read at address {:#X}", addr);}
        }
    }

    fn bus_write16(&mut self, addr:usize, value:u16) {
        self.bus_write8(addr, value as u8);
        self.bus_write8(addr+1, (value >> 8) as u8);
    }

    fn bus_read16(&mut self, addr:usize) -> u16 {
        self.bus_read8(addr) as u16 | 
        ((self.bus_read8(addr+1) as u16) << 8) 
    }
}

#[cfg(test)]
mod test{
    use super::*;
    use crate::interrupt::InterruptStatus;

    fn get_test_pack() -> (TimerUnit, InterruptStatus){
        let mut tu = TimerUnit::new();
        let mut is = InterruptStatus::new();
        is.isrmask = 0xFF;
        (tu, is)
    }

    #[test]
    fn timer_no_tick_until_enabled() {
        let (mut tu, mut is) = get_test_pack();

        tu.bus_write8(super::TAC_REG_ADDR, 0x03);
        tu.update(4, &mut is);
        assert_eq!(tu.tima_inc_counter, 0);

        tu.bus_write8(super::TAC_REG_ADDR, 0x5);
        tu.update(4, &mut is);
        assert_eq!(tu.tima_inc_counter, 4);
    }

    #[test]
    fn timer_tick_at_correct_freq() {
        let (mut tu, mut is) = get_test_pack();
        tu.bus_write8(super::TAC_REG_ADDR, 0b101);

        tu.update(15, &mut is);
        assert_eq!(tu.tima, 0);
        tu.update(1, &mut is);
        assert_eq!(tu.tima, 1);
    }

    #[test]
    fn timer_reset_to_tma(){
        let (mut tu, mut is) = get_test_pack();
        tu.bus_write8(super::TAC_REG_ADDR, 0b100);
        let tma = 0x23;
        tu.bus_write8(super::TMA_REG_ADDR, tma);

        for _ in 0..255{
            tu.update(1024, &mut is);
        }
        assert_eq!(tu.bus_read8(super::TIMA_REG_ADDR), 255);

        // Enough here to tick twice, should wrap tick into tma.
        tu.update(2049, &mut is);
        assert_eq!(tu.tima, tma+1);

        // Partial tick of 1023 should cause update due to one extra tick.
        tu.update(1023, &mut is);
        assert_eq!(tu.tima, tma+2);
    }
}