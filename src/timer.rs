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
            self.div = self.div.wrapping_add(1);
            self.div_inc_counter -= TimerUnit::ticks_per_div;
        }

        if self.enabled{
            // Update the tima counter
            // TODO account for stop instruction.
            self.tima_inc_counter += cpu_ticks;
            if self.tima_inc_counter >= self.ticks_per_inc {
                self.tima_inc_counter -= self.ticks_per_inc;

                // if tima register will wrap
                if self.tima == 0xFF {
                    // Reset to mod, and continue
                    self.tima = self.tma;
                    is.request_timer();
                }
                // Else increment normally.
                else {
                    self.tima += 1;
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
    #[test]
    #[ignore]
    fn timer_module_needs_tests() {
        assert!(false);
    }
}