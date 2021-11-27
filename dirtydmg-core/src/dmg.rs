use std::rc::Rc;
use std::cell::RefCell;
use crate::cpu::{Cpu};
use crate::ram::Ram;
use crate::bus::{Bus, BusItem};
use crate::interrupt::InterruptStatus;
use crate::cartrige::{Cartrige};
use crate::ppu::{PPU};
use crate::serial::{SerialUnit};
use crate::timer::TimerUnit;

//DEBUG! should not stay!
use std::io::stdout;
use std::io::Write;

pub struct Dmg{
    pub cpu:Cpu,
    ram: Rc<RefCell<Ram>>,
    bus: Bus,
    isr: Rc<RefCell<InterruptStatus>>,
    cart: Rc<RefCell<Cartrige>>,
    pub ppu: Rc<RefCell<PPU>>,
    stu: Rc<RefCell<SerialUnit>>,
    tu: Rc<RefCell<TimerUnit>>,
}

impl Dmg {

    pub fn new() -> Dmg {
        // components that will live on the bus
        let ram = Rc::new(RefCell::new(Ram::new(0x2000, 0xC000)));
        let zero_page = Rc::new(RefCell::new(Ram::new(127, 0xFF80)));
        let isr = Rc::new(RefCell::new(InterruptStatus::new()));
        let cart = Rc::new(RefCell::new(Cartrige::new()));
        let ppu = Rc::new(RefCell::new(PPU::new()));
        let stu = Rc::new(RefCell::new(SerialUnit::new()));
        let tu = Rc::new(RefCell::new(TimerUnit::new()));


        // Map components to the bus.
        let mut bus = Bus::new();
        bus.add_item(BusItem::new(0x0000, 0x7FFF, cart.clone()));
        bus.add_item(BusItem::new(0x8000, 0x9FFF, ppu.clone()));
        bus.add_item(BusItem::new(0xA000, 0xBFFF, cart.clone()));
        bus.add_item(BusItem::new(0xC000, 0xDFFF, ram.clone()));
        bus.add_item(BusItem::new(0xFF01, 0xFF02, stu.clone()));
        bus.add_item(BusItem::new(0xFF04, 0xFF07, tu.clone()));
        bus.add_item(BusItem::new(0xFF0F, 0xFF0F, isr.clone()));
        bus.add_item(BusItem::new(0xFF40, 0xFF4B, ppu.clone()));
        bus.add_item(BusItem::new(0xFF80, 0xFFFE, zero_page.clone()));
        bus.add_item(BusItem::new(0xFFFF, 0xFFFF, isr.clone()));

        let mut cpu = Cpu::new();
        cpu.reg.a = 0x01;
        cpu.reg.f = 0xB0;
        cpu.reg.b = 0;
        cpu.reg.c = 0x13;
        Dmg {
            cpu,
            ram,
            bus,
            isr,
            cart,
            ppu,
            stu,
            tu
        }
    }

    // Attempts to load the specified rom file into the system.
    pub fn load_rom(&mut self, data: &[u8]) -> Result<(), String>{
        self.cpu.reg.pc = 0x100;
        self.cart.as_ref().borrow_mut().load_rom(data)
    }

    pub fn update(&mut self) {

        self.cpu.handle_interrupts(&mut self.bus, &mut self.isr.as_ref().borrow_mut());
        let cycles = self.cpu.update(&mut self.bus);
        self.ppu.as_ref().borrow_mut().execute_ticks(cycles as u16 * 4, &mut self.bus, &mut self.isr.as_ref().borrow_mut());
        {
            let mut stu = self.stu.as_ref().borrow_mut();
            stu.update(cycles as u32, &mut self.isr.as_ref().borrow_mut());
            match stu.get_output() {
                Some(x) => {
                    print!("{} ", x as char);
                    stdout().flush().unwrap();
                }
                None => {}
            };
        }
        // Update the timer unit with cpu ticks (not machine cycles)
        self.tu.as_ref().borrow_mut().update(
            cycles as u16 * 4, 
            &mut self.isr.as_ref().borrow_mut());
    }
}
