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
use crate::input::{Gamepad, Button};
use crate::sound::Apu;

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
    gamepad: Rc<RefCell<Gamepad>>,
    pub apu: Rc<RefCell<Apu>>,
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
        let gamepad =  Rc::new(RefCell::new(Gamepad::new()));
        let apu = Rc::new(RefCell::new(Apu::new()));


        // Map components to the bus.
        let mut bus = Bus::new();
        bus.add_item(BusItem::new(0x0000, 0x7FFF, cart.clone()));
        bus.add_item(BusItem::new(0x8000, 0x9FFF, ppu.clone()));
        bus.add_item(BusItem::new(0xA000, 0xBFFF, cart.clone()));
        bus.add_item(BusItem::new(0xC000, 0xDFFF, ram.clone()));
        bus.add_item(BusItem::new(0xFE00, 0xFE9F, ppu.clone()));
        bus.add_item(BusItem::new(0xFF00, 0xFF00, gamepad.clone()));
        //bus.add_item(BusItem::new(0xFF01, 0xFF02, stu.clone()));
        bus.add_item(BusItem::new(0xFF04, 0xFF07, tu.clone()));
        bus.add_item(BusItem::new(0xFF0F, 0xFF0F, isr.clone()));
        bus.add_item(BusItem::new(0xFF10, 0xFF3F, apu.clone()));
        bus.add_item(BusItem::new(0xFF40, 0xFF4B, ppu.clone()));
        bus.add_item(BusItem::new(0xFF80, 0xFFFE, zero_page));
        bus.add_item(BusItem::new(0xFFFF, 0xFFFF, isr.clone()));

        let mut cpu = Cpu::new();
        cpu.reg.a = 0x01;
        cpu.reg.f = 0xB0;
        cpu.reg.b = 0;
        cpu.reg.c = 0x13;
        cpu.reg.e = 0xD8;
        cpu.reg.h = 0x01;
        cpu.reg.l = 0x4D;
        cpu.reg.sp = 0xFFFE;
        Dmg {
            cpu,
            ram,
            bus,
            isr,
            cart,
            ppu,
            stu,
            tu,
            gamepad,
            apu,
        }
    }

    // Attempts to load the specified rom file into the system.
    pub fn load_rom(&mut self, data: &[u8]) -> Result<(), String>{
        self.cpu.reg.pc = 0x100;
        self.cart.as_ref().borrow_mut().load_rom(data)
    }

    pub fn load_sram(&mut self, data: &[u8]){
        self.cart.as_ref().borrow_mut().load_ram(data);
    }

    pub fn get_sram(&self, buffer: &mut Vec<u8>){
        self.cart.as_ref().borrow_mut().get_ram(buffer);
    }

    pub fn update(&mut self) {

        self.cpu.handle_interrupts(&mut self.bus, &mut self.isr.as_ref().borrow_mut());
        let cycles = self.cpu.update(&mut self.bus);
        self.ppu.as_ref().borrow_mut().execute_ticks(cycles as u16 * 4, &mut self.bus, &mut self.isr.as_ref().borrow_mut());
        {
            let mut stu = self.stu.as_ref().borrow_mut();
            stu.update(cycles as u32, &mut self.isr.as_ref().borrow_mut());
            if let Some(x) = stu.get_output() {
                print!("{} ", x as char);
                stdout().flush().unwrap();
            };
        }
        // Update the timer unit with cpu ticks (not machine cycles)
        self.tu.as_ref().borrow_mut().update(
            cycles as u16 * 4, 
            &mut self.isr.as_ref().borrow_mut());
        self.apu.as_ref().borrow_mut().tick(cycles as u16 * 4);
    }

    pub fn input(&mut self, btn:Button, pressed:bool) {
        if pressed {
            self.isr.as_ref().borrow_mut().request_joypad();
            self.gamepad.as_ref().borrow_mut().press_btn(btn);
        } else {
            self.gamepad.as_ref().borrow_mut().release_btn(btn);
        }
    }
}

impl Default for Dmg {
    fn default() -> Self {
        Self::new()
    }
}
