use crate::bus::BusRW;
use crate::interrupt::InterruptStatus;

const SB_REG_ADDR:usize = 0xFF01;
const SC_REG_ADDR:usize = 0xFF02;

pub struct SerialUnit{
    sb: u8,
    sc: u8,
    input: u8,
    output: Option<u8>,
    cycle_count: u32
}

impl SerialUnit {
    const CYCLES_PER_TICK:u32 = 128; // 128 m-cycles per bit tick.
    const CYCLES_PER_OPERATION:u32 = SerialUnit::CYCLES_PER_TICK * 8;

    const SC_ACTIVE_MASK:u8 = 0b1000_0000;
    const SC_SPEED_MASK:u8 = 0b0000_0010;
    const SC_MASTER_MASK:u8 = 0b0000_0010;

    fn is_transfer_active(&self) -> bool {
        return (self.sc & SerialUnit::SC_ACTIVE_MASK) != 0;
    }

    fn is_highspeed(&self) -> bool {
        return (self.sc & SerialUnit::SC_SPEED_MASK) != 0;
    }

    fn is_master(&self) -> bool {
        return (self.sc & SerialUnit::SC_MASTER_MASK) != 0;
    }

    /// # Read but do not clear the last output byte.
    fn peak_output(&self) -> Option<u8>{
        return self.output;
    }

    /// # Read and clear the last output byte.
    fn get_output(&mut self) -> Option<u8>{
        let ret = self.output;
        self.output = None;
        return ret;
    }

    pub fn update(&mut self, cycles: u32, is:&mut InterruptStatus) {
        // If transfer is active
        if self.is_transfer_active() {
            // Add ticks to cycle_count.
            self.cycle_count += cycles;
            // If enough ticks have elapsed
            if self.cycle_count >= SerialUnit::CYCLES_PER_OPERATION{
                // Trigger a serial interrupt.
                is.request_serial();
                self.output = Some(self.sb);
                // Clear the sb byte
                self.sb = 0;
                // Clear the transfer active flag
                self.sb ^= SerialUnit::SC_ACTIVE_MASK;
                // Clear the cycle count
                self.cycle_count = 0;
            }
        }
    }

    pub fn new() -> SerialUnit{
        SerialUnit{
            sb: 0,
            sc: 0,
            input: 0,
            output: None,
            cycle_count: 0,
        }
    }
}

impl BusRW for SerialUnit {
    fn bus_write8(&mut self, addr:usize, value:u8) {
        match addr {
            SB_REG_ADDR => {
                self.sb = value;
            }
            SC_REG_ADDR => {
                self.sc = value;
            }
            _ => {
                panic!("SerialUnit unexpected write at address: {}", addr);
            }
        }
    }

    fn bus_write16(&mut self, addr:usize, value:u16) {
        self.bus_write8(addr, value as u8);
        self.bus_write8(addr+1, (value >> 8) as u8);
    }

    fn bus_read8(&mut self, addr:usize) -> u8 {
        match addr {
            SB_REG_ADDR => {
                self.sb
            }
            SC_REG_ADDR => {
                self.sc
            }
            _ => {
                panic!("SerialUnit unexpected read at address: {}", addr);
            }
        }
    }

    fn bus_read16(&mut self, addr:usize) -> u16 {
        self.bus_read8(addr) as u16 | 
        ((self.bus_read8(addr+1) as u16) << 8) 
    }
}

#[cfg(test)]
mod test{
    use super::*;

    fn get_test_pack() -> (SerialUnit, InterruptStatus){
        let mut unit = SerialUnit::new();
        let mut is = InterruptStatus::new();
        is.isrmask = 0xFF;
        return (unit, is);
    }

    fn send_data(stu:&mut SerialUnit, data:u8) {
        stu.bus_write8(0xFF01, data);
        stu.bus_write8(0xFF02, 0x81);
    }

    #[test]
    fn interrupt_on_send_complete(){
        let (mut stu, mut is) = get_test_pack();
        send_data(&mut stu, 0xA5);

        // No isr yet
        stu.update(SerialUnit::CYCLES_PER_OPERATION-1, &mut is);
        assert_eq!(is.is_serial_active(), false);

        // Now isr should trigger
        stu.update(1, &mut is);
        assert_eq!(is.is_serial_active(), true);
    }

    #[test]
    fn output_on_send_complete(){
        let (mut stu, mut is) = get_test_pack();
        send_data(&mut stu, 0xA5);

        assert_eq!(stu.get_output(), None);
        stu.update(SerialUnit::CYCLES_PER_OPERATION, &mut is);

        assert_eq!(stu.get_output(), Some(0xA5));
        assert_eq!(stu.get_output(), None);
    }

}
