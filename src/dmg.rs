use std::rc::Rc;
use std::cell::RefCell;
use crate::cpu::{Cpu};
use crate::ram::Ram;
use crate::bus::{Bus, BusItem};
use crate::interrupt::InterruptStatus;
use crate::cartrige::{Cartrige};
use crate::ppu::{PPU};
use crate::serial::{SerialUnit};

pub struct Dmg{
    pub cpu:Cpu,
    ram: Rc<RefCell<Ram>>,
    bus: Bus,
    isr: Rc<RefCell<InterruptStatus>>,
    cart: Rc<RefCell<Cartrige>>,
    ppu: Rc<RefCell<PPU>>,
}

impl Dmg {

    pub fn new() -> Dmg {
        // components that will live on the bus
        let ram = Rc::new(RefCell::new(Ram::new(0x2000, 0xC000)));
        let isr = Rc::new(RefCell::new(InterruptStatus::new()));
        let cart = Rc::new(RefCell::new(Cartrige::new()));
        let ppu = Rc::new(RefCell::new(PPU::new()));
        let stu = Rc::new(RefCell::new(SerialUnit::new()));

        // Map components to the bus.
        let mut bus = Bus::new();
        bus.add_item(BusItem::new(0xC000, 0xDFFF, ram.clone()));
        bus.add_item(BusItem::new(0xFF0F, 0xFF0F, isr.clone()));
        bus.add_item(BusItem::new(0xFFFF, 0xFFFF, isr.clone()));
        bus.add_item(BusItem::new(0x0000, 0x7FFF, cart.clone()));
        bus.add_item(BusItem::new(0xA000, 0xBFFF, cart.clone()));
        bus.add_item(BusItem::new(0xFF01, 0xFF02, stu.clone()));

        Dmg {
            cpu: Cpu::new(),
            ram,
            bus,
            isr,
            cart,
            ppu
        }
    }

    // Attempts to load the specified rom file into the system.
    pub fn load_rom(&mut self, data: &[u8]) -> Result<(), String>{
        self.cart.as_ref().borrow_mut().load_rom(data)
    }
}
