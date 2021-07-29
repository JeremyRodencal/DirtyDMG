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
    const CARRY_FLAG:u8  = 0b0001_0000;
    const HCARRY_FLAG:u8 = 0b0010_0000;
    const SUB_FLAG:u8    = 0b0100_0000;
    const ZERO_FLAG:u8   = 0b1000_0000;

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
            Register::SP => self.sp,
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

    fn read_pc(&mut self, bus:&mut impl BusRW) -> u8
    {
        let value = bus.bus_read8(self.reg.pc as usize);
        self.reg.pc += 1;
        return value;
    }

    fn execute_instruction(&mut self, bus:&mut impl BusRW) -> u8
    {
        // Read the opcode, fetch the instruction details, and read in the entire instruction.
        let opcode = self.read_pc(bus);
        let instruction = &INSTRUCTION_TABLE[opcode as usize];
        let mut data = [opcode, 0, 0];
        for i in 1..1+(instruction.length-1)
        {
            data[i as usize] = self.read_pc(bus);
        }

        // Execute the operation in the instruction
        self.execute_operation(bus, &data, &instruction.op);

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
                let initial = self.reg.read8(*dst);
                let result = self.reg.read8(*dst).wrapping_add(1);
                self.reg.write8(*dst, result);

                // Only the carry flag is retained.
                self.reg.f = self.reg.f & Regs::CARRY_FLAG;

                // If there was a half carry
                if (initial ^ result) & 0x10 > 0 {
                    self.reg.f = self.reg.f | Regs::HCARRY_FLAG;
                }
                // If the result was zero
                if result == 0 {
                    self.reg.f = self.reg.f | Regs::ZERO_FLAG;
                }
            },
            
            // Decrement an 8 bit register
            DecR{dst} => {
                let initial = self.reg.read8(*dst);
                let result = initial.wrapping_sub(1);
                self.reg.write8(*dst, result);
                // Preserve the carry flaga and set the sub flag
                self.reg.f &= Regs::CARRY_FLAG;
                self.reg.f |= Regs::SUB_FLAG;

                // Set zero flag if needed.
                if result == 0 {
                    self.reg.f |= Regs::ZERO_FLAG
                }

                // set the half carry flag if needed.
                if  (initial ^ result) & 0x10 > 0 {
                    self.reg.f |= Regs::HCARRY_FLAG
                }
            },

            // Decrement a 16 bit register
            DecR16{dst} => {
                let i = self.reg.read16(*dst).wrapping_sub(1);
                self.reg.write16(*dst, i);
            },

            // Rotate A once to the left, and carry bit 7 into the carry flag.
            Rlca => {
                self.reg.a = self.reg.a.rotate_left(1);
                self.reg.f = 
                if self.reg.a & 0x01 == 1{
                    Regs::CARRY_FLAG
                } else {0}
            },


            // Rotate A once to the right and carry bit 0 into the carry flag.
            Rrca => {
                self.reg.a = self.reg.a.rotate_right(1);
                self.reg.f = 
                    if self.reg.a & 0x80 == 0x80 {
                        Regs::CARRY_FLAG
                    } else {
                        0
                    }
            },

            // // Add two 16 bit registers.
            AddR16R16{dst, src} => {
                let initial = self.reg.read16(*dst);
                let result = initial.wrapping_add(self.reg.read16(*src));
                self.reg.write16(*dst, result);

                // Zero flag is preserved.
                self.reg.f &= Regs::ZERO_FLAG;

                // Half carry only occurs on most significant byte.
                if (initial ^ result) & 0x1000 > 0 {
                    self.reg.f = self.reg.f | Regs::HCARRY_FLAG;
                }
                // Set carry flag if needed.
                if result < initial {
                    self.reg.f = self.reg.f | Regs::CARRY_FLAG;
                }
            }
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
    Instruction{op:Operation::LdRI{dst: Register::C}, length:2, cycles:2},
    // 0x0F RRCA    1, 1
    Instruction{op:Operation::Rrca, length:1, cycles:1},

    // Temporary NOPs
    // 0x1X
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

    // 0x2X
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

    // 0x3X
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

    // 0x4X
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

    // 0x5X
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
    
    // 0x6X
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

    // 0x7X
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

    // 0x8X
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

    // 0x9X
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

    // 0xAX
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

    // 0xBX
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

    // 0xCX
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

    // 0xDX
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

    // 0xEX
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

    // 0xFX
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

#[cfg(test)]
mod test {
    use super::*;
    use crate::bus::{BusRW};
    use crate::ram::Ram;

    fn get_ram() -> Ram {
        Ram::new(0xFFFF, 0)
    }

    fn load_into_ram(ram:&mut Ram, inst: &[u8])
    {
        let mut addr = 0;
        for val in inst {
            ram.bus_write8(addr, *val);
            addr += 1;
        }
    }

    #[test]
    fn cpu_0x00()
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();

        let data = [0 as u8];
        load_into_ram(&mut ram, &data);
        assert_eq!(cpu.execute_instruction(&mut ram), 1);
        assert_eq!(cpu.reg.pc, 1);
    }

    #[test]
    fn cpu_0x01()
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        let data = [0x01, 0x01, 0x02];
        load_into_ram(&mut ram, &data);

        let cycles = cpu.execute_instruction(&mut ram);
        assert_eq!(cycles, 3);
        assert_eq!(cpu.reg.pc, 3);
        assert_eq!(cpu.reg.read16(Register::BC), 0x0201);
    }

    #[test]
    fn cpu_0x02()
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();

        let data = [0x02];
        load_into_ram(&mut ram, &data);

        cpu.reg.write16(Register::BC, 0x1234);
        cpu.reg.write8(Register::A, 0xAB);
        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 2);
        assert_eq!(cpu.reg.pc, 1);
        assert_eq!(ram.bus_read8(0x1234), 0xAB);
    }

    #[test]
    fn cpu_0x03()
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();

        let data = [0x03];
        load_into_ram(&mut ram, &data);
        cpu.reg.write16(Register::BC, 0x1234);
        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 2);
        assert_eq!(cpu.reg.pc, 1);
        assert_eq!(cpu.reg.read16(Register::BC), 0x1235);
    }

    #[test]
    fn cpu_0x04()
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();

        let data = [0x04];
        load_into_ram(&mut ram, &data);
        cpu.reg.b = 0xFF;
        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 1);
        assert_eq!(cpu.reg.pc, 1);
        assert_eq!(cpu.reg.b, 0x00);
        assert_eq!(cpu.reg.f, Regs::ZERO_FLAG | Regs::HCARRY_FLAG);

        cpu.reg.b = 0x0F;
        cpu.reg.f = Regs::CARRY_FLAG; // instruction must not modify carry flag.
        cpu.reg.pc = 0;
        cpu.execute_instruction(&mut ram);
        assert_eq!(cpu.reg.b, 0x10);
        assert_eq!(cpu.reg.f, Regs::CARRY_FLAG | Regs::HCARRY_FLAG);
    }

    #[test]
    fn cpu_0x05()
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();

        let data = [0x05];
        load_into_ram(&mut ram, &data);
        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 1);
        assert_eq!(cpu.reg.pc, 1);
        assert_eq!(cpu.reg.b, 0xFF);
        assert_eq!(cpu.reg.f, Regs::HCARRY_FLAG | Regs::SUB_FLAG);

        cpu.reg.pc = 0;
        cpu.reg.b  = 1;
        cpu.execute_instruction(&mut ram);
        assert_eq!(cpu.reg.b, 0);
        assert_eq!(cpu.reg.f, Regs::SUB_FLAG | Regs::ZERO_FLAG);
    }

    #[test]
    fn cpu_0x06()
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();

        let data = [0x06, 0x43];
        load_into_ram(&mut ram, &data);
        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 2);
        assert_eq!(cpu.reg.pc, 2);
        assert_eq!(cpu.reg.b, 0x43);
    }

    #[test]
    fn cpu_0x07()
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        
        let data = [0x07];
        load_into_ram(&mut ram, &data);
        cpu.reg.a = 0xAA;
        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 1);
        assert_eq!(cpu.reg.pc, 1);
        assert_eq!(cpu.reg.a, 0x55);
        assert_eq!(cpu.reg.f, Regs::CARRY_FLAG);

        cpu.reg.a = 0x55;
        cpu.reg.pc = 0;
        cpu.execute_instruction(&mut ram);
        assert_eq!(cpu.reg.a, 0xAA);
        assert_eq!(cpu.reg.f, 0);
    }

    #[test]
    fn cpu_0x08()
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();

        // Save SP to immediate address 0x1000
        let data = [0x08, 0x00, 0x10];
        load_into_ram(&mut ram, &data);
        cpu.reg.sp = 0x0102;

        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 5);
        assert_eq!(cpu.reg.pc, 3);
        assert_eq!(ram.bus_read16(0x1000), cpu.reg.sp);
    }

    #[test]
    fn cpu_0x09()
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();

        // Add HL and BC, store in HL
        let data = [0x09];
        load_into_ram(&mut ram, &data);
        cpu.reg.write16(Register::HL, 0xFF);
        cpu.reg.write16(Register::BC, 0xFF);
        
        cpu.reg.f = 0xF0;
        cpu.reg.write16(Register::HL, 0xFF);
        cpu.reg.write16(Register::BC, 0xFF);
        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 2);
        assert_eq!(cpu.reg.pc, 1);
        assert_eq!(cpu.reg.read16(Register::HL), 0xFF + 0xFF);
        assert_eq!(cpu.reg.f, Regs::ZERO_FLAG);

        cpu.reg.pc = 0;
        cpu.reg.f = 0;
        cpu.reg.write16(Register::HL, 0xFFFF);
        cpu.reg.write16(Register::BC, 0x0001);
        cpu.execute_instruction(&mut ram);

        assert_eq!(cpu.reg.read16(Register::HL), 0);
        assert_eq!(cpu.reg.f, Regs::CARRY_FLAG | Regs::HCARRY_FLAG);
    }

    #[test]
    fn cpu_0x0A() {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();

        let instruction = [0x0A];
        load_into_ram(&mut ram, &instruction);

        let address = 0xABCD;
        let expected_value = 143;
        ram.bus_write8(address, expected_value);
        cpu.reg.write16(Register::BC, address as u16);

        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 2);
        assert_eq!(cpu.reg.pc, 1);
        assert_eq!(cpu.reg.a, expected_value);
    }

    #[test]
    fn cpu_0x0B() {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();

        let instruction = [0x0B];
        load_into_ram(&mut ram, &instruction);

        let flags = 0xF0;
        cpu.reg.f = flags;
        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 2);
        assert_eq!(cpu.reg.pc, 1);
        assert_eq!(cpu.reg.read16(Register::BC), 0xFFFF);
        assert_eq!(cpu.reg.f, flags);
    }

    #[test]
    fn cpu_0x0C() {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();

        let data = [0x0C];
        load_into_ram(&mut ram, &data);
        cpu.reg.c = 0xFF;
        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 1);
        assert_eq!(cpu.reg.pc, 1);
        assert_eq!(cpu.reg.b, 0x00);
        assert_eq!(cpu.reg.f, Regs::ZERO_FLAG | Regs::HCARRY_FLAG);

        cpu.reg.c = 0x0F;
        cpu.reg.f = Regs::CARRY_FLAG; // instruction must not modify carry flag.
        cpu.reg.pc = 0;
        cpu.execute_instruction(&mut ram);
        assert_eq!(cpu.reg.c, 0x10);
        assert_eq!(cpu.reg.f, Regs::CARRY_FLAG | Regs::HCARRY_FLAG);
    }

    #[test]
    fn cpu_0x0D() {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();

        let data = [0x0D];
        load_into_ram(&mut ram, &data);

        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 1);
        assert_eq!(cpu.reg.pc, 1);
        assert_eq!(cpu.reg.c, 0xFF);
        assert_eq!(cpu.reg.f, Regs::HCARRY_FLAG | Regs::SUB_FLAG);

        cpu.reg.pc = 0;
        cpu.reg.c  = 1;
        cpu.execute_instruction(&mut ram);

        assert_eq!(cpu.reg.c, 0);
        assert_eq!(cpu.reg.f, Regs::SUB_FLAG | Regs::ZERO_FLAG);
    }

    #[test]
    fn cpu_0x0E() {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();

        let data = [0x0E, 0x23];
        load_into_ram(&mut ram, &data);
        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 2);
        assert_eq!(cpu.reg.pc, 2);
        assert_eq!(cpu.reg.c, 0x23);
    }

    #[test]
    fn cpu_0x0F() {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();

        let data = [0x0F];
        load_into_ram(&mut ram, &data);

        cpu.reg.a = 0x55;
        cpu.reg.f = 0xF0;

        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 1);
        assert_eq!(cpu.reg.pc, 1);
        assert_eq!(cpu.reg.a, 0xAA);
        assert_eq!(cpu.reg.f, Regs::CARRY_FLAG);

        cpu.reg.pc = 0;
        cpu.reg.a = 0xAA;
        cpu.reg.f = 0xF0;
        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cpu.reg.f, 0);
        assert_eq!(cpu.reg.a, 0x55);
    }
}