use std::rc::Rc;
use std::cell::RefCell;
use crate::cpu::{Cpu};
use crate::ram::Ram;
use crate::bus::{Bus, BusItem};

pub struct Dmg{
    pub cpu:Cpu,
    ram: Rc<RefCell<Ram>>,
    bus: Bus
}

impl Dmg {

    pub fn new() -> Dmg {
        let ram = Rc::new(RefCell::new(Ram::new(0x2000, 0xC000)));
        let mut bus = Bus::new();
        bus.add_item(BusItem::new(0xC000, 0xDFFF, ram.clone()));
        Dmg {
            cpu: Cpu::new(),
            ram,
            bus 
        }
    }

    pub fn do_test(&mut self){
        println!("Writing 2 to location 0x2000");
        self.cpu.write_mem_test(&mut self.bus, 0xC000, 2);
        println!(
            "Reading from location 0x2000: {}", 
            self.cpu.read_mem_test(&mut self.bus, 0xC000));
    }
}
