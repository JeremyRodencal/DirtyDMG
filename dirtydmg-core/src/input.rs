use crate::bus::BusRW;

pub enum Button {
    A,
    B,
    Select,
    Start,
    Up,
    Down,
    Left,
    Right,
}

pub struct Gamepad{
    buttons_raw: u8,
    dpad_raw: u8,
    dpad_active: bool,
    isr_req: bool,
}

impl Gamepad{

    const DPAD_ENABLE_MASK:u8 = 0b0001_0000;
    const BTN_ENABLE_MASK:u8  = 0b0010_0000;
    const BTN_A_RIGHT_MASK:u8     = 0b1;
    const BTN_B_LEFT_MASK:u8      = 0b10;
    const BTN_SELECT_UP_MASK:u8   = 0b100;
    const BTN_START_DOWN_MASK:u8 = 0b1000;

    pub fn new()->Gamepad{
        Gamepad{
            buttons_raw: 0xF,
            dpad_raw: 0xF,
            dpad_active: false,
            isr_req: false,
        }
    }

    fn is_dpad(button:&Button) -> bool{
        match button {
            Button::Up | Button::Down | Button::Left | Button::Right => {true}
            _ => false
        }
    }

    fn get_mask(button:&Button) -> u8 {
        match button{
            Button::A | Button::Right => {
                Gamepad::BTN_A_RIGHT_MASK
            }
            Button::B | Button::Left => {
                Gamepad::BTN_B_LEFT_MASK
            }
            Button::Start | Button::Down => {
                Gamepad::BTN_START_DOWN_MASK
            }
            Button::Select | Button::Up => {
                Gamepad::BTN_SELECT_UP_MASK
            }
        }
    }

    pub fn press_btn(&mut self, btn:Button) {
        self.isr_req = true;
        let mask = Gamepad::get_mask(&btn);
        if Gamepad::is_dpad(&btn) {
            self.dpad_raw &= !mask;
        } else{
            self.buttons_raw &= !mask;
        }
    }

    pub fn release_btn(&mut self, btn:Button) {
        let mask = Gamepad::get_mask(&btn);
        if Gamepad::is_dpad(&btn) {
            self.dpad_raw |= mask;
        } else{
            self.buttons_raw |= mask;
        }
    }
}

impl BusRW for Gamepad{

    fn bus_read8(&mut self, _addr: usize) -> u8{
        if self.dpad_active{
            self.dpad_raw
        }
        else {
            self.buttons_raw
        }
    }

    fn bus_write8(&mut self, _addr: usize, value: u8) {
        if value & Gamepad::DPAD_ENABLE_MASK == 0{
            self.dpad_active = true;
        }
        else if value & Gamepad::BTN_ENABLE_MASK == 0{
            self.dpad_active = false;
        }
    }

    // These are not practical, just here to satisfy the trait.
    fn bus_read16(&mut self, _addr: usize) -> u16{0}
    fn bus_write16(&mut self, _addr: usize, _val: u16){}
}