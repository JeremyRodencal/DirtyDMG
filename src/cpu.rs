use std::num::Wrapping;
use byteorder::{ByteOrder, LittleEndian};
use crate::bus::{BusRW};

#[allow(dead_code)]
#[derive(Debug)]
#[derive(Clone, Copy)]
enum Register {
    A, F,
    B, C,
    D, E,
    H, L,
    AF,
    BC,
    DE,
    HL,
    SP
}

#[allow(dead_code)]
struct Regs {
    a:u8,
    f:u8,
    b:u8,
    c:u8,
    d:u8,
    e:u8,
    h:u8,
    l:u8,
    sp: u16,
    pc: u16,
}

#[allow(dead_code)]
impl Regs {

    fn new()->Regs {
        Regs{
            a:0, f:0,
            b:0, c:0, 
            d:0, e:0,
            h:0, l:0,
            sp: 0,
            pc: 0
        }
    }

    fn read8(&self, reg:Register) -> u8
    {
        match reg
        {
            Register::A => self.a,
            Register::F => self.f,
            Register::B => self.b,
            Register::C => self.c,
            Register::D => self.d,
            Register::E => self.e,
            Register::H => self.h,
            Register::L => self.l,
            _ => panic!("Invalid 8bit register read of {:?}", reg)
        }
    }

    fn read16(&self, reg:Register) -> u16 {
        match reg {
            Register::AF => (self.a as u16) << 8 | (self.f as u16),
            Register::BC => (self.b as u16) << 8 | (self.c as u16),
            Register::DE => (self.d as u16) << 8 | (self.e as u16),
            Register::HL => (self.h as u16) << 8 | (self.l as u16),
            _ => panic!("Invalid 16 bit register read {:?}", reg)
        }
    }

    fn write8(&mut self, reg:Register, value:u8) {
        match reg {
            Register::A => self.a = value,
            Register::F => self.f = value,
            Register::B => self.b = value,
            Register::C => self.c = value,
            Register::D => self.d = value,
            Register::E => self.e = value,
            Register::H => self.h = value,
            Register::L => self.l = value,
            _ => panic!("Invalid 8bit register write of {}=>{:?}", value, reg)
        }
    }

    fn write16(&mut self, reg:Register, value:u16) {
        match reg{
            Register::AF => { 
                self.a = (value >> 8) as u8;
                self.f = value as u8;
            },
            Register::BC => {
                self.b = (value >> 8) as u8;
                self.c = value as u8;
            }
            Register::DE => {
                self.d = (value >> 8) as u8;
                self.e = value as u8;
            }
            Register::HL => {
                self.h = (value >> 8) as u8;
                self.l = value as u8;
            }
            _ => panic!("Invalid 8bit register write of {}=>{:?}", value, reg)
        }
    }
}

#[allow(dead_code)]
pub struct Cpu {
    reg: Regs
}

#[allow(dead_code)]
impl Cpu {
    pub fn new()->Cpu {
        Cpu{
            reg:Regs::new()
        }
    }

    pub fn write_mem_test(&mut self, bus:&mut impl BusRW, addr: usize, value: u8)
    {
        self.reg.a = value;
        bus.bus_write8(addr, value);
    }
    
    pub fn read_mem_test(&mut self, bus:&mut impl BusRW, addr: usize)->u8
    {
        self.reg.a = bus.bus_read8(addr);
        return self.reg.a
    }

    fn execute_instruction(&mut self, bus:&mut impl BusRW, data: &[u8]) -> u8
    {
        let opcode = data[0];
        let instruction = &INSTRUCTION_TABLE[opcode as usize];

        self.execute_operation(bus, data, &instruction.op);
        //TODO do something with cycle length

        return instruction.cycles;
    }

    fn execute_operation(&mut self, bus:&mut impl BusRW, data: &[u8], op:&Operation)
    {
        use Operation::*;
        match op {
            Nop => (),
            Stop => { panic!("Stop is not")},

            // Move a register value into another register.
            LdRR{dst, src} => { 
                self.reg.write8(
                    *dst, 
                    self.reg.read8(*src)
                );
            },

            // A register from memory transfer.
            LdRM{dst, src} => {
                self.reg.write8(
                    *dst, 
                    bus.bus_read8(
                        self.reg.read16(*src) as usize
                    )
                );
            },

            // A 16bit immediate load into a 16 bit register.
            LdR16I16{dst} => {
                self.reg.write16(
                    *dst, 
                    LittleEndian::read_u16(&data[1..])
                );
            },

            // Store a register into memory pointed to by 16bit register.
            LdMR16{dst, src} => {
                bus.bus_write8(
                    self.reg.read16(*dst) as usize,
                    self.reg.read8(*src)
                );
            },

            // Load a register with an 8 bit immediate value.
            LdRI{dst} => {
                self.reg.write8(
                    *dst,
                    data[1]
                );
            },

            // Save a 16 bit register to an immediate address.
            LdI16R16{src} => {
                bus.bus_write16(
                    LittleEndian::read_u16(&data[1..]) as usize,
                    self.reg.read16(*src)
                );
            },

            // Increment a 16 bit register.
            IncR16{dst} => {
                let i = self.reg.read16(*dst).wrapping_add(1);
                self.reg.write16(*dst, i);
            },

            // Increment an 8 bit register.
            IncR{dst} => {
                let i = self.reg.read8(*dst).wrapping_add(1);
                self.reg.write8(*dst, i);
            },
            
            // Decrement an 8 bit register
            DecR{dst} => {
                let i = self.reg.read8(*dst).wrapping_sub(1);
                self.reg.write8(*dst, i);
            },

            // Decrement a 16 bit register
            DecR16{dst} => {
                let i = self.reg.read16(*dst).wrapping_sub(1);
                self.reg.write16(*dst, i);
            },

            // // Rotate A once to the left, and carry bit 7 into the carry flag.
            // Rlca,
            // // Rotate A once to the right and carry bit 0 into the carry flag.
            // Rrca,

            // // Add two 16 bit registers.
            // AddR16R16{dst:Register, src:Register},
            _ => panic!("not implemented")
        }
    }
}

#[allow(dead_code)]
enum Operation {
    // A no operation.
    Nop,
    // A stop operation
    Stop,

    // A register to register transfer.
    LdRR{dst:Register, src:Register},
    // A register from memory transfer.
    LdRM{dst:Register, src:Register},
    // A 16bit immediate load into a 16 bit register.
    LdR16I16{dst:Register},
    // Store a register into memory pointed to by 16bit register.
    LdMR16{dst:Register, src:Register},
    // Load a register with an 8 bit immediate value.
    LdRI{dst:Register},
    // Save a 16 bit register to an immediate address.
    LdI16R16{src:Register},

    // Increment a 16 bit register.
    IncR16{dst:Register},
    // Increment an 8 bit register.
    IncR{dst:Register},
    
    // Decrement an 8 bit register
    DecR{dst:Register},
    // Decrement a 16 bit register
    DecR16{dst:Register},

    // Rotate A once to the left, and carry bit 7 into the carry flag.
    Rlca,
    // Rotate A once to the right and carry bit 0 into the carry flag.
    Rrca,

    // Add two 16 bit registers.
    AddR16R16{dst:Register, src:Register},
}

#[allow(dead_code)]
struct Instruction{
    op:Operation,
    length:u8,
    cycles:u8,
}

#[allow(dead_code)]
const INSTRUCTION_TABLE: [Instruction;256] = [
    // 0x00 NOP
    Instruction{op:Operation::Nop, length:1, cycles:1},
    // 0x01 LD BC, d16
    Instruction{op:Operation::LdR16I16{dst:Register::BC}, length:3, cycles:3},
    // 0x02 LD (BC), A
    Instruction{op:Operation::LdMR16{dst:Register::BC, src:Register::A}, length:1, cycles:2},
    // 0x03 INC BC
    Instruction{op:Operation::IncR16{dst:Register::BC}, length: 1, cycles:2},
    // 0x04 INC B
    Instruction{op:Operation::IncR{dst:Register::B}, length:1, cycles:1},
    // 0x05 DEC B 1, 1
    Instruction{op:Operation::DecR{dst:Register::B}, length:1, cycles:1},
    // 0x06 LD B, D8 2, 2
    Instruction{op:Operation::LdRI{dst:Register::B}, length:2, cycles:2},
    // 0x07 RLCA 1, 1
    Instruction{op:Operation::Rlca, length: 1, cycles:1},
    // 0x08 LD (A16), SP 3, 5
    Instruction{op:Operation::LdI16R16{src:Register::SP}, length:3, cycles:5},
    // 0x09 ADD HL, BC 1, 2
    Instruction{op:Operation::AddR16R16{dst:Register::HL, src:Register::BC}, length:1, cycles:2},
    // 0x0A LD A, (BC) 1, 2
    Instruction{op:Operation::LdRM{dst:Register::A, src:Register::BC}, length:1, cycles:2},
    // 0x0B DEC BC  1, 2
    Instruction{op:Operation::DecR16{dst:Register::BC}, length:1, cycles:2},
    // 0x0C INC C   1, 1
    Instruction{op:Operation::IncR{dst:Register::C}, length:1, cycles:1},
    // 0x0D DEC C   1, 1
    Instruction{op:Operation::DecR{dst:Register::C}, length:1, cycles:1},
    // 0x0E LD C d8 2, 2
    Instruction{op:Operation::LdRI{dst: Register::C}, length:1, cycles:1},
    // 0x0F RRCA    1, 1
    Instruction{op:Operation::Nop, length:1, cycles:1},

    // Temporary NOPs
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1},
];