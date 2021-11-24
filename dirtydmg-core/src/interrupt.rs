use crate::bus::BusRW;

pub struct InterruptStatus {
    /// Interrupt status request byte. Use the assorted XXX_MASK constants to check for requests.
    pub isrreq: u8,

    /// Interrupt enable/disable mask.
    pub isrmask: u8,
}

impl InterruptStatus {
    const VBLANK_MASK:u8 =  0b1;
    const LCDSTAT_MASK:u8 = 0b10;
    const TIMER_MASK:u8 =   0b100;
    const SERIAL_MASK:u8 =  0b1000;
    const JOYPAD_MASK:u8 =  0b10000;
    const ISR_EN_ADDR:usize = 0xFFFF;
    const ISR_FLAG_ADDR:usize = 0xFF0F;

    /// Constructs a new InterruptStatus object
    /// 
    /// All interrupts are clear and interrupts are disabled.
    /// 
    pub fn new() -> InterruptStatus{
        InterruptStatus{
            isrreq: 0,
            isrmask: 0,
        }
    }

    /// Requests a vblank interrupt.
    pub fn request_vblank(&mut self){
        self.isrreq |= InterruptStatus::VBLANK_MASK;
    }

    /// Requests an lcdstat interrupt.
    pub fn request_lcdstat(&mut self){
        self.isrreq |= InterruptStatus::LCDSTAT_MASK;
    }

    /// Requests a timer interrupt.
    pub fn request_timer(&mut self){
        self.isrreq |= InterruptStatus::TIMER_MASK;
    }

    /// Requests a serial interrupt.
    pub fn request_serial(&mut self){
        self.isrreq |= InterruptStatus::SERIAL_MASK;
    }

    /// Requests a joypad interrupt.
    pub fn request_joypad(&mut self){
        self.isrreq |= InterruptStatus::JOYPAD_MASK;
    }

    /// Clears the vblank interrupt status.
    pub fn clear_vblank(&mut self){
        self.isrreq &= !InterruptStatus::VBLANK_MASK;
    }

    /// Clears the LCD status interrupt status.
    pub fn clear_lcdstat(&mut self){
        self.isrreq &= !InterruptStatus::LCDSTAT_MASK;
    }

    /// Clears the timer interrupt status.
    pub fn clear_timer(&mut self){
        self.isrreq &= !InterruptStatus::TIMER_MASK;
    }

    /// Clears the interrupt status.
    pub fn clear_serial(&mut self){
        self.isrreq &= !InterruptStatus::SERIAL_MASK;
    }

    /// Clears the joypad status.
    pub fn clear_joypad(&mut self) {
        self.isrreq = !InterruptStatus::JOYPAD_MASK;
    }

    /// Checks if the vblank interrupt is active
    pub fn is_vblank_active(&self) -> bool {
        return self.isrreq & self.isrmask & InterruptStatus::VBLANK_MASK > 0;
    }

    /// Checks if the lcdstat interrupt is active
    pub fn is_lcdstat_active(&self) -> bool {
        return self.isrreq & self.isrmask & InterruptStatus::LCDSTAT_MASK > 0;
    }

    /// Checks if the timer interrupt is active
    pub fn is_timer_active(&self) -> bool {
        return self.isrreq & self.isrmask & InterruptStatus::TIMER_MASK > 0;
    }

    /// Checks if the serial interrupt is active
    pub fn is_serial_active(&self) -> bool {
        return self.isrreq & self.isrmask & InterruptStatus::SERIAL_MASK > 0;
    }

    /// Checks if the joypad interrupt is active
    pub fn is_joypad_active(&self) -> bool {
        return self.isrreq & self.isrmask & InterruptStatus::JOYPAD_MASK > 0;
    }
}

impl BusRW for InterruptStatus {

    fn bus_read8(&mut self, addr: usize) -> u8 {
        match addr {
            InterruptStatus::ISR_EN_ADDR => {
                self.isrmask
            },
            InterruptStatus::ISR_FLAG_ADDR => {
                self.isrreq
            }
            _ => {
                panic!("Interrupt status bus fault reading from {:04X}", addr);
            }
        }
    }

    fn bus_write8(&mut self, addr: usize, value: u8) {
        match addr {
            InterruptStatus::ISR_EN_ADDR => {
                self.isrmask = value;
            },
            InterruptStatus::ISR_FLAG_ADDR => {
                self.isrreq = value;
            },
            _ => {
                panic!("Interrupt status bus fault writing to {:04X}", addr);
            }
        }
    }

    /// 16 bit reads are not supported for InterruptStatus.
    fn bus_read16(&mut self, addr: usize) -> u16 {
        panic!("Interrupt status bus fault reading from {:04X}", addr);
    }

    /// 16 bit writes are not supported for InterruptStatus.
    fn bus_write16(&mut self, addr: usize, _value: u16) {
        panic!("Interrupt status bus fault writing to {:04X}", addr);
    }

}

#[cfg(test)]
mod test {
    use super::*;

    // Gets a new InterruptStatus object with all interrupts enabled.
    fn get_isr_all_enabled()-> InterruptStatus {
        let mut isr = InterruptStatus::new();
        isr.bus_write8(InterruptStatus::ISR_EN_ADDR, 0xFF);
        return isr;
    }

    // Gets a new InterruptStatus object with all interrupts active.
    fn get_isr_all_active()-> InterruptStatus {
        let mut isr = InterruptStatus::new();
        isr.bus_write8(InterruptStatus::ISR_FLAG_ADDR, 0xFF);
        return isr;
    }

    impl InterruptStatus{
        /// TEST function to read the ISR request flags from the bus.
        pub fn read_isr_flag(&mut self) -> u8{
            return self.bus_read8(InterruptStatus::ISR_FLAG_ADDR);
        }

        /// TEST function to read the ISR enable mask from the bus.
        pub fn read_isr_mask(&mut self) -> u8{
            return self.bus_read8(InterruptStatus::ISR_EN_ADDR);
        }

        /// TEST writes the isr mask.
        pub fn write_isr_mask(&mut self, value:u8){
            self.bus_write8(InterruptStatus::ISR_EN_ADDR, value);
        }
    }

    #[test]
    fn test_request_vblank_enabled() {
        let mut isr = get_isr_all_enabled();

        // when 
        isr.request_vblank();

        // then
        assert_eq!(isr.read_isr_flag(), InterruptStatus::VBLANK_MASK);
        assert_eq!(isr.is_vblank_active(), true);
    }

    #[test]
    fn test_request_lcdstat_enabled() {
        let mut isr = get_isr_all_enabled();

        // when
        isr.request_lcdstat();

        // then
        assert_eq!(isr.read_isr_flag(), InterruptStatus::LCDSTAT_MASK);
        assert_eq!(isr.is_lcdstat_active(), true);
    }

    #[test]
    fn test_request_timer_enabled() {
        let mut isr = get_isr_all_enabled();

        // when
        isr.request_timer();

        // then
        assert_eq!(isr.read_isr_flag(), InterruptStatus::TIMER_MASK);
        assert_eq!(isr.is_timer_active(), true);
    }

    #[test]
    fn test_request_serial_enabled() {
        let mut isr = get_isr_all_enabled();

        // when
        isr.request_serial();

        // then
        assert_eq!(isr.read_isr_flag(), InterruptStatus::SERIAL_MASK);
        assert_eq!(isr.is_serial_active(), true);
    }

    #[test]
    fn test_request_joypad_enabled() {
        let mut isr = get_isr_all_enabled();

        // when
        isr.request_joypad();

        // then
        assert_eq!(isr.read_isr_flag(), InterruptStatus::JOYPAD_MASK);
        assert_eq!(isr.is_joypad_active(), true);
    }

    #[test]
    fn test_request_vblank_disabled() {
        let mut isr = get_isr_all_enabled();
        isr.write_isr_mask(!InterruptStatus::VBLANK_MASK);

        // when 
        isr.request_vblank();

        // then
        assert_eq!(isr.read_isr_flag(), InterruptStatus::VBLANK_MASK);
        assert_eq!(isr.is_vblank_active(), false);
    }

    #[test]
    fn test_request_lcdstat_disabled() {
        let mut isr = get_isr_all_enabled();
        isr.write_isr_mask(!InterruptStatus::LCDSTAT_MASK);

        // when
        isr.request_lcdstat();

        // then
        assert_eq!(isr.read_isr_flag(), InterruptStatus::LCDSTAT_MASK);
        assert_eq!(isr.is_lcdstat_active(), false);
    }

    #[test]
    fn test_request_timer_disabled() {
        let mut isr = get_isr_all_enabled();
        isr.write_isr_mask(!InterruptStatus::TIMER_MASK);

        // when
        isr.request_timer();

        // then
        assert_eq!(isr.read_isr_flag(), InterruptStatus::TIMER_MASK);
        assert_eq!(isr.is_timer_active(), false);
    }

    #[test]
    fn test_request_serial_disabled() {
        let mut isr = get_isr_all_enabled();
        isr.write_isr_mask(!InterruptStatus::SERIAL_MASK);

        // when
        isr.request_serial();

        // then
        assert_eq!(isr.read_isr_flag(), InterruptStatus::SERIAL_MASK);
        assert_eq!(isr.is_timer_active(), false);
    }

    #[test]
    fn test_request_joypad_disabled() {
        let mut isr = get_isr_all_enabled();
        isr.write_isr_mask(!InterruptStatus::JOYPAD_MASK);

        // when
        isr.request_joypad();

        // then
        assert_eq!(isr.read_isr_flag(), InterruptStatus::JOYPAD_MASK);
        assert_eq!(isr.is_joypad_active(), false);
    }

    #[test]
    fn test_clear_vblank_interrupt() {
        let mut isr = get_isr_all_active();
        isr.clear_vblank();
        assert_eq!(isr.read_isr_flag(), !InterruptStatus::VBLANK_MASK);
    }
    
    #[test]
    fn test_clear_lcdstat_interrupt(){
        let mut isr = get_isr_all_active();
        isr.clear_vblank();
        assert_eq!(isr.read_isr_flag(), !InterruptStatus::VBLANK_MASK);
    }

    #[test]
    fn test_clear_timer_interrupt(){
        let mut isr = get_isr_all_active();
        isr.clear_timer();
        assert_eq!(isr.read_isr_flag(), !InterruptStatus::TIMER_MASK);
    }

    #[test]
    fn test_clear_serial_interrupt(){
        let mut isr = get_isr_all_active();
        isr.clear_serial();
        assert_eq!(isr.read_isr_flag(), !InterruptStatus::SERIAL_MASK);
    }

    #[test]
    fn test_clear_joypad_interrupt(){
        let mut isr = get_isr_all_active();
        isr.clear_joypad();
        assert_eq!(isr.read_isr_flag(), !InterruptStatus::JOYPAD_MASK);
    }
}
