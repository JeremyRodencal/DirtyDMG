use std::num::Wrapping;
use byteorder::{ByteOrder, LittleEndian};
use crate::bus::{BusRW};

#[allow(dead_code)]
#[derive(Debug)]
#[derive(Clone, Copy)]
#[derive(PartialEq)]
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
    reg: Regs,
    busy_cycles: u32, // Number of cycles until ready to execute
    cycle: u64,       // Current cycle number
}

#[allow(dead_code)]
impl Cpu {
    pub fn new()->Cpu {
        Cpu{
            reg:Regs::new(),
            busy_cycles: 0,
            cycle: 0
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

        self.busy_cycles = instruction.cycles as u32;

        // Execute the operation in the instruction
        self.execute_operation(bus, &data, &instruction.op);
        return self.busy_cycles as u8;
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
                let addend = self.reg.read16(*src);
                let result = initial.wrapping_add(self.reg.read16(*src));
                self.reg.write16(*dst, result);

                // Zero flag is preserved.
                self.reg.f &= Regs::ZERO_FLAG;

                // Half carry only occurs on most significant byte.
                if (initial ^ addend) & 0x1000 != result & 0x1000 {
                    self.reg.f = self.reg.f | Regs::HCARRY_FLAG;
                }
                // Set carry flag if needed.
                if result < initial {
                    self.reg.f = self.reg.f | Regs::CARRY_FLAG;
                }
            },

            Rla => {
                // Determine the carry in for bit zero.
                let carry_in = 
                    if self.reg.f & Regs::CARRY_FLAG > 0 {1}
                    else {0};
                // Determine if the carry flag should be set.
                self.reg.f = 
                    if self.reg.a & 0x80 > 0 {Regs::CARRY_FLAG}
                    else {0};

                // Shift A over, then OR in the bottom bit.
                self.reg.a <<= 1;
                self.reg.a |= carry_in;
            },

            Rra => {
                let carry_in = (self.reg.f & Regs::CARRY_FLAG) << 3;
                self.reg.f = (self.reg.a & 0x01) << 4;
                self.reg.a >>= 1;
                self.reg.a |= carry_in;
            },

            Jr{cond} => {
                let take_branch = match cond {
                    JumpCondition::Always => true,
                    JumpCondition::Z =>  self.reg.f & Regs::ZERO_FLAG  != 0,
                    JumpCondition::Nz => self.reg.f & Regs::ZERO_FLAG  == 0,
                    JumpCondition::C =>  self.reg.f & Regs::CARRY_FLAG != 0,
                    JumpCondition::Nc => self.reg.f & Regs::CARRY_FLAG != 0,
                };
                if take_branch {
                    // get the signed jump offset, then add it to the program counter.
                    let jump_offset = data[1] as i8;
                    self.reg.pc = self.reg.pc.wrapping_add(jump_offset as u16);
                    self.busy_cycles += 1;
                }
            },

            LdMR16Mv{add} => {
                let addr = self.reg.read16(Register::HL);
                bus.bus_write8(addr as usize, self.reg.a);
                self.reg.write16(Register::HL, addr.wrapping_add(*add as i16 as u16));
            }

            LdRMMv{add} => {
                let addr = self.reg.read16(Register::HL);
                self.reg.a = bus.bus_read8(addr as usize);
                self.reg.write16(Register::HL, addr.wrapping_add(*add as i16 as u16));
            },

            Cpl => {
                self.reg.a ^= 0xFF;
                self.reg.f |= Regs::SUB_FLAG | Regs::HCARRY_FLAG;
            }

            _ => panic!("not implemented")
        }
    }
}

#[allow(dead_code)]
enum JumpCondition {
    Always, // Always jump
    Z,      // Jump if zero flag set
    Nz,     // Jump if zero flag not set
    C,      // Jump if carry flag set
    Nc,     // Jump if carry flag not set
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
    // Store the A register into memory pointed to by the HL register, and modify HL.
    LdMR16Mv{add:i8},
    // Read into the A register from memory pointed to by the HL register, and modify HL.
    LdRMMv{add:i8},
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

    // Rotate A once to the left, and carry bit 7 into the carry flag and bit zero.
    Rlca,
    // Rotate A once to the right and carry bit 0 into the carry flag.
    Rrca,
    // Rotate A once to the left, rotate carry into bit 0, rotate bit 7 into the carry flag
    Rla,
    // Rotate A once to the right, rotate carry into bit 7, rotate bit zero into the carry flag
    Rra,

    // Add two 16 bit registers.
    AddR16R16{dst:Register, src:Register},

    // Correct binary coded decimal.
    Daa,

    // One's compliment of register A.
    Cpl,

    // Jump relative
    Jr{cond: JumpCondition},
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

    // 0x10 Stop
    Instruction{op:Operation::Stop, length:2, cycles:1},
    // 0x11 LD DE, d16
    Instruction{op:Operation::LdR16I16{dst: Register::DE}, length:3, cycles:3},
    // 0x12 LD (DE), A
    Instruction{op:Operation::LdMR16{dst:Register::DE, src:Register::A}, length:1, cycles:2},
    // 0x13 Inc DE
    Instruction{op:Operation::IncR16{dst:Register::DE}, length:1, cycles:2},
    // 0x14 Inc D
    Instruction{op:Operation::IncR{dst:Register::D}, length:1, cycles:1},
    // 0x15 Dec D
    Instruction{op:Operation::DecR{dst:Register::D}, length:1, cycles:1},
    // 0x16 Ld D, d8
    Instruction{op:Operation::LdRI{dst:Register::D}, length:2, cycles:2},
    // 0x17 Rla
    Instruction{op:Operation::Rla, length:1, cycles:1},
    // 0x18 Jd s8
    Instruction{op:Operation::Jr{cond: JumpCondition::Always}, length:2, cycles:2},
    // 0x19 Add HL, DE
    Instruction{op:Operation::AddR16R16{dst:Register::HL, src:Register::DE}, length:1, cycles:2},
    // 0x1A Ld A, (DE)
    Instruction{op:Operation::LdRM{dst:Register::A, src:Register::DE}, length:1, cycles:2},
    // 0x1B Dec DE
    Instruction{op:Operation::DecR16{dst:Register::DE}, length:1, cycles:2},
    // 0x1C Inc E
    Instruction{op:Operation::IncR{dst:Register::E}, length:1, cycles:1},
    // 0x1D Dec E
    Instruction{op:Operation::DecR{dst:Register::E}, length:1, cycles:1},
    // 0x1E Ld E, d8
    Instruction{op:Operation::LdRI{dst:Register::E}, length:2, cycles:2},
    // 0x1F Rra
    Instruction{op:Operation::Rra, length:1, cycles:1},

    // 0x2X
    // 0x20 Jr NZ, s8
    Instruction{op:Operation::Jr{cond: JumpCondition::Nz}, length:2, cycles:2},
    // 0x21 Ld HL, d16
    Instruction{op:Operation::LdR16I16{dst: Register::HL}, length:3, cycles:3},
    // 0x22 Ld (HL+), A
    Instruction{op:Operation::LdMR16Mv{add:1}, length:1, cycles:2},
    // 0x23 Inc HL
    Instruction{op:Operation::IncR16{dst: Register::HL}, length:1, cycles:2},
    // 0x24 Inc H
    Instruction{op:Operation::IncR{dst: Register::H}, length:1, cycles:1},
    // 0x25 Dec H
    Instruction{op:Operation::DecR{dst: Register::H}, length:1, cycles:1},
    // 0x26 Ld H, d8
    Instruction{op:Operation::LdRI{dst: Register::H}, length:2, cycles:2},
    // 0x27 Daa
    Instruction{op:Operation::Daa, length:1, cycles:1},
    // 0x28 Jr Z, s8
    Instruction{op:Operation::Jr{cond:JumpCondition::Z}, length:2, cycles:2},
    // 0x29 Add HL, HL
    Instruction{op:Operation::AddR16R16{dst:Register::HL, src:Register::HL}, length:1, cycles:2},
    // 0x2A Ld A (HL+)
    Instruction{op:Operation::LdRMMv{add:1}, length:1, cycles:2},
    // 0x2B Dec HL
    Instruction{op:Operation::DecR16{dst:Register::HL}, length:1, cycles:2},
    // 0x2C Inc L
    Instruction{op:Operation::IncR{dst:Register::L}, length:1, cycles:1},
    // 0x2D Dec L
    Instruction{op:Operation::DecR{dst:Register::L}, length:1, cycles:1},
    // 0x2E Ld L, d8
    Instruction{op:Operation::LdRI{dst:Register::L}, length:2, cycles:2},
    // 0x2F Cpl A
    Instruction{op:Operation::Cpl, length:1, cycles:1},

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

    fn test_op_ldR16I16(inst: &[u8], dst:Register, value:u16)
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        load_into_ram(&mut ram, &inst);

        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 3);
        assert_eq!(cpu.reg.pc, 3);
        assert_eq!(cpu.reg.read16(dst), value);
    }

    fn test_op_ldMR16(inst: &[u8], dst:Register, src:Register, addr:u16, value:u8)
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();

        load_into_ram(&mut ram, &inst);

        cpu.reg.write16(dst, addr);
        cpu.reg.write8(src, value);
        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 2);
        assert_eq!(cpu.reg.pc, 1);
        assert_eq!(ram.bus_read8(addr as usize), value);
    }

    fn test_op_incR16(inst: &[u8], dst:Register)
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();

        load_into_ram(&mut ram, &inst);
        cpu.reg.write16(dst, 0x1234);
        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 2);
        assert_eq!(cpu.reg.pc, 1);
        assert_eq!(cpu.reg.read16(dst), 0x1235);
    }

    fn test_op_incR(inst: &[u8], dst:Register)
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();

        load_into_ram(&mut ram, &inst);
        cpu.reg.write8(dst, 0xff);
        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 1);
        assert_eq!(cpu.reg.pc, 1);
        assert_eq!(cpu.reg.read8(dst), 0x00);
        assert_eq!(cpu.reg.f, Regs::ZERO_FLAG | Regs::HCARRY_FLAG);

        cpu.reg.write8(dst, 0x0F);
        cpu.reg.f = Regs::CARRY_FLAG; // instruction must not modify carry flag.
        cpu.reg.pc = 0;
        cpu.execute_instruction(&mut ram);
        assert_eq!(cpu.reg.read8(dst), 0x10);
        assert_eq!(cpu.reg.f, Regs::CARRY_FLAG | Regs::HCARRY_FLAG);
    }

    fn test_op_decR(inst: &[u8], dst:Register)
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();

        load_into_ram(&mut ram, &inst);
        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 1);
        assert_eq!(cpu.reg.pc, 1);
        assert_eq!(cpu.reg.read8(dst), 0xFF);
        assert_eq!(cpu.reg.f, Regs::HCARRY_FLAG | Regs::SUB_FLAG);

        cpu.reg.pc = 0;
        cpu.reg.write8(dst, 1);
        cpu.execute_instruction(&mut ram);
        assert_eq!(cpu.reg.read8(dst), 0);
        assert_eq!(cpu.reg.f, Regs::SUB_FLAG | Regs::ZERO_FLAG);
    }

    fn test_op_ldRI(inst: &[u8], dst:Register, value:u8)
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();

        load_into_ram(&mut ram, &inst);
        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 2);
        assert_eq!(cpu.reg.pc, 2);
        assert_eq!(cpu.reg.read8(dst), value);
    }

    fn test_op_addr16r16(inst: &[u8], dst:Register, src:Register )
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();

        // Add HL and BC, store in HL
        load_into_ram(&mut ram, &inst);
        cpu.reg.write16(dst, 0xFF);
        cpu.reg.write16(src, 0xFF);
        
        cpu.reg.f = 0xF0;
        cpu.reg.write16(dst, 0xFF);
        cpu.reg.write16(src, 0xFF);
        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 2);
        assert_eq!(cpu.reg.pc, 1);
        assert_eq!(cpu.reg.read16(dst), 0xFF + 0xFF);
        assert_eq!(cpu.reg.f, Regs::ZERO_FLAG);

        cpu.reg.pc = 0;
        cpu.reg.f = 0;
        cpu.reg.write16(dst, 0xFFFF);

        // Special case when source and destination registers are the same
        let result = 
            if src != dst {
                cpu.reg.write16(src, 0x0001);
                0
            } else {0xFFFE};
        
        cpu.execute_instruction(&mut ram);
        assert_eq!(cpu.reg.read16(dst), result);
        assert_eq!(cpu.reg.f, Regs::CARRY_FLAG | Regs::HCARRY_FLAG);
    }

    fn test_op_ldRM(inst: &[u8], dst:Register, src:Register, addr: u16, value: u8)
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();

        load_into_ram(&mut ram, &inst);

        ram.bus_write8(addr as usize, value);
        cpu.reg.write16(src, addr);

        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 2);
        assert_eq!(cpu.reg.pc, 1);
        assert_eq!(cpu.reg.read8(dst), value);
    }

    fn test_op_decR16(inst: &[u8], dst:Register)
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();

        load_into_ram(&mut ram, &inst);

        let flags = 0xF0;
        cpu.reg.f = flags;
        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 2);
        assert_eq!(cpu.reg.pc, 1);
        assert_eq!(cpu.reg.read16(dst), 0xFFFF);
        assert_eq!(cpu.reg.f, flags);
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
        let data = [0x01, 0x01, 0x02];
        test_op_ldR16I16(&data, Register::BC, 0x0201);
    }

    #[test]
    fn cpu_0x02()
    {
        let data = [0x02];
        test_op_ldMR16(&data, Register::BC, Register::A, 0x1234, 0xAB)
    }

    #[test]
    fn cpu_0x03()
    {
        let data = [0x03];
        test_op_incR16(&data, Register::BC);
    }

    #[test]
    fn cpu_0x04()
    {
        let data = [0x04];
        test_op_incR(&data, Register::B);
    }

    #[test]
    fn cpu_0x05()
    {
        let data = [0x05];
        test_op_decR(&data, Register::B);
    }

    #[test]
    fn cpu_0x06()
    {
        let data = [0x06, 0x43];
        test_op_ldRI(&data, Register::B, 0x43);
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
        let data = [0x09];
        test_op_addr16r16(&data, Register::HL, Register::BC);
    }

    #[test]
    fn cpu_0x0A() 
    {
        let instruction = [0x0A];
        test_op_ldRM(&instruction, Register::A, Register::BC, 0xABCD, 143);
    }

    #[test]
    fn cpu_0x0B() {
        let instruction = [0x0B];
        test_op_decR16(&instruction, Register::BC);
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

    #[test]
    #[ignore]
    fn cpu_0x10()
    {
        // TODO test stop instruction
    }

    #[test]
    fn cpu_0x11()
    {
        let data = [0x11, 0xFF, 0xEE];
        test_op_ldR16I16(&data, Register::DE, 0xEEFF);
    }

    #[test]
    fn cpu_0x12()
    {
        let data = [0x12];
        test_op_ldMR16(&data, Register::DE, Register::A, 0x4321, 0x99);
    }

    #[test]
    fn cpu_0x13()
    {
        let data = [0x13];
        test_op_incR16(&data, Register::DE);
    }

    #[test]
    fn cpu_0x14()
    {
        let data = [0x14];
        test_op_incR(&data, Register::D);
    }

    #[test]
    fn cpu_0x15()
    {
        let data = [0x15];
        test_op_decR(&data, Register::D);
    }

    #[test]
    fn cpu_0x16()
    {
        let data = [0x16, 0xBE];
        test_op_ldRI(&data, Register::D, 0xBE);
    }

    #[test]
    fn cpu_0x17()
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        let data = [0x17];
        load_into_ram(&mut ram, &data);

        // Set A to alternating bit pattern, set carry flag.
        // Carry flag is set, so bit zero must be set after operation.
        // Entire flags register should be cleared after operation.
        cpu.reg.a = 0b_0101_0101;
        cpu.reg.f = Regs::ZERO_FLAG | Regs::SUB_FLAG | Regs::CARRY_FLAG | Regs::HCARRY_FLAG;

        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 1);
        assert_eq!(cpu.reg.pc, 1);
        assert_eq!(cpu.reg.a, 0b1010_1011);
        assert_eq!(cpu.reg.f, 0);

        // Set A to an alternating bit pattern, clear carry flag.
        // MSB is 1 so carry flag must be set.
        cpu.reg.a = 0b_1010_1010;
        cpu.reg.f = 0;
        cpu.reg.pc = 0;

        cpu.execute_instruction(&mut ram);

        assert_eq!(cpu.reg.a, 0b0101_0100);
        assert_eq!(cpu.reg.f, Regs::CARRY_FLAG);
    }

    #[test]
    fn cpu_0x18()
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();

        let inst_addr = 0x7E;

        // Jump to 127 bytes after instruction;
        let inst = [0x18, 0x7F];
        ram.bus_write8(inst_addr, inst[0]);
        ram.bus_write8(inst_addr + 1, inst[1]);

        cpu.reg.pc = inst_addr as u16;
        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 3);
        assert_eq!(cpu.reg.pc, 0x7E + 2 + 0x7F);

        // write a -128 byte as the jump offset.
        ram.bus_write8(inst_addr+1, 0x80);
        cpu.reg.pc = inst_addr as u16;

        cpu.execute_instruction(&mut ram);

        assert_eq!(cpu.reg.pc, 0x7E + 2 - 128);
    }

    #[test]
    fn cpu_0x19()
    {
        let inst = [0x19];
        test_op_addr16r16(&inst, Register::HL, Register::DE);
    }

    #[test]
    fn cpu_0x1A()
    {
        let inst = [0x1A];
        test_op_ldRM(&inst, Register::A, Register::DE, 0xBEAF, 0xBC);
    }

    #[test]
    fn cpu_0x1B()
    {
        let inst = [0x1B];
        test_op_decR16(&inst, Register::DE);
    }

    #[test]
    fn cpu_0x1C()
    {
        let inst = [0x1C];
        test_op_incR(&inst, Register::E);
    }

    #[test]
    fn cpu_0x1D()
    {
        let inst = [0x1D];
        test_op_decR(&inst, Register::E);
    }

    #[test]
    fn cpu_0x1E()
    {
        let inst = [0x1E, 0xFE];
        test_op_ldRI(&inst, Register::E, 0xFE);
    }

    #[test]
    fn cpu_0x1F()
    {
        let inst = [0x1F];

        let mut cpu = Cpu::new();
        let mut ram = get_ram();

        load_into_ram(&mut ram, &inst);
        cpu.reg.f = 0xF0;
        cpu.reg.a = 0b1010_1010;

        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 1);
        assert_eq!(cpu.reg.pc, 1);
        assert_eq!(cpu.reg.a, 0b1101_0101);
        assert_eq!(cpu.reg.f, 0);

        cpu.reg.pc = 0;
        cpu.reg.a = 0b0101_0101;
        cpu.reg.f = 0;
        cpu.execute_instruction(&mut ram);

        assert_eq!(cpu.reg.a, 0b0010_1010);
        assert_eq!(cpu.reg.f, Regs::CARRY_FLAG);
    }

    #[test]
    fn cpu_0x20()
    {
        // JR NZ, 3
        let inst = [0x20, 3];

        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        load_into_ram(&mut ram, &inst);

        cpu.reg.f = 0; // clear all flags.
        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 3);
        assert_eq!(cpu.reg.pc, 5); // branch was taken because zero flag was false.

        cpu.reg.f = Regs::ZERO_FLAG;
        cpu.reg.pc = 0;
        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 2);
        assert_eq!(cpu.reg.pc, 2); // branch not taken because zero flag was true;
    }

    #[test]
    fn cpu_0x21()
    {
        // Ld HL, 0xCAFE
        let inst = [0x21, 0xFE, 0xCA];
        test_op_ldR16I16(&inst, Register::HL, 0xCAFE);
    }

    #[test]
    fn cpu_0x22()
    {
        // Ld (HL+), A
        let inst = [0x22];
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        load_into_ram(&mut ram, &inst);
        
        cpu.reg.write16(Register::HL, 0x1234);
        cpu.reg.a = 91;
        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 2);
        assert_eq!(cpu.reg.pc, 1);
        assert_eq!(cpu.reg.read16(Register::HL), 0x1235);
        assert_eq!(ram.bus_read8(0x1234), 91);
    }

    #[test]
    fn cpu_0x23()
    {
        // Inc HL
        let inst = [0x23];
        test_op_incR16(&inst, Register::HL);
    }

    #[test]
    fn cpu_0x24()
    {
        // Inc H
        let inst = [0x24];
        test_op_incR(&inst, Register::H);
    }

    #[test]
    fn cpu_0x25()
    {
        // Dec H
        test_op_decR(&[0x25], Register::H);
    }

    #[test]
    fn cpu_0x26()
    {
        // Ld H, d8
        test_op_ldRI(&[0x26, 0x23], Register::H, 0x23);
    }

    #[test]
    #[ignore]
    fn cpu_0x27()
    {
        //Daa
        assert_eq!(1, 2);
    }

    #[test]
    fn cpu_0x28()
    {
        // Jr Z, s8
        let inst = [0x28, -2i8 as u8];
        let mut ram = get_ram();
        let mut cpu = Cpu::new();
        load_into_ram(&mut ram, &inst);

        cpu.reg.f = Regs::ZERO_FLAG; // Branch is taken
        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 3);
        assert_eq!(cpu.reg.pc, 0); // branch taken, jump back 2 yeilds zero.

        cpu.reg.f = 0; // Branch is not taken
        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 2);
        assert_eq!(cpu.reg.pc, 2); // branch was not taken, no jump.
    }

    #[test]
    fn cpu_0x29()
    {
        test_op_addr16r16(&[0x29], Register::HL, Register::HL);
    }

    #[test]
    fn cpu_0x2A()
    {
        // Ld A, (HL+)
        let inst = [0x2A];
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        load_into_ram(&mut ram, &inst);
        
        cpu.reg.write16(Register::HL, 0x1234);
        ram.bus_write8(0x1234, 91);
        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 2);
        assert_eq!(cpu.reg.pc, 1);
        assert_eq!(cpu.reg.read16(Register::HL), 0x1235);
        assert_eq!(cpu.reg.a, 91);
    }

    #[test]
    fn cpu_0x2B()
    {
        // Dec HL
        test_op_decR16(&[0x2B], Register::HL);
    }

    #[test]
    fn cpu_0x2C()
    {
        // Inc L
        test_op_incR(&[0x2C], Register::L);
    }

    #[test]
    fn cpu_0x2D()
    {
        // Dec L
        test_op_decR(&[0x2D], Register::L);
    }

    #[test]
    fn cpu_0x2E()
    {
        // Ld L, d8
        test_op_ldRI(&[0x2E, 233], Register::L, 233);
    }

    #[test]
    fn cpu_0x2F()
    {
        // Cpl
        let inst = [0x2F];
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        load_into_ram(&mut ram, &inst);

        cpu.reg.a = 0x55;
        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cpu.reg.a, 0xAA);
        assert_eq!(cpu.reg.f, Regs::SUB_FLAG | Regs::HCARRY_FLAG);
        assert_eq!(cpu.reg.pc, 1);
        assert_eq!(cycles, 1);
    }
}
