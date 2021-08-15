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
            },
            Register::DE => {
                self.d = (value >> 8) as u8;
                self.e = value as u8;
            },
            Register::HL => {
                self.h = (value >> 8) as u8;
                self.l = value as u8;
            },
            Register::SP => {
                self.sp = value;
            },
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

            // Increment memory pointed to by register.
            IncM{dst} => {
                let addr = self.reg.read16(*dst) as usize;
                let initial = bus.bus_read8(addr);
                let result = initial.wrapping_add(1);
                bus.bus_write8(addr, result);

                self.reg.f &= Regs::CARRY_FLAG;

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

            // Decrement a memory location addressed by 16 bit register dst
            DecM{dst} => {
                let addr = self.reg.read16(*dst) as usize;
                let initial = bus.bus_read8(addr);
                let result = initial.wrapping_sub(1);
                bus.bus_write8(addr, result);

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
                    JumpCondition::Nc => self.reg.f & Regs::CARRY_FLAG == 0,
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

            LdMI{dst} => {
                let addr= self.reg.read16(*dst) as usize;
                bus.bus_write8(addr, data[1]);
            },

            Cpl => {
                self.reg.a ^= 0xFF;
                self.reg.f |= Regs::SUB_FLAG | Regs::HCARRY_FLAG;
            },

            Scf => {
                self.reg.f &= Regs::ZERO_FLAG;
                self.reg.f |= Regs::CARRY_FLAG;
            },

            Ccf => {
                self.reg.f &= Regs::ZERO_FLAG | Regs::CARRY_FLAG;
                self.reg.f ^= Regs::CARRY_FLAG;
            },

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
    // Store an 8 bit immediate into memory pointed to by a 16 bit register.
    LdMI{dst:Register},

    // Increment a 16 bit register.
    IncR16{dst:Register},
    // Increment an 8 bit register.
    IncR{dst:Register},
    // Increment a memory location addressed by 16 bit register.
    IncM{dst:Register},
    
    // Decrement an 8 bit register
    DecR{dst:Register},
    // Decrement a 16 bit register
    DecR16{dst:Register},
    // Decrement a memory location addressed by a 16 bit register
    DecM{dst:Register},

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
    // Set carry flag.
    Scf,
    // Clear carry flag
    Ccf,

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
    // 0x30 Jr NC, s8
    Instruction{op:Operation::Jr{cond:JumpCondition::Nc}, length:2, cycles:2},
    // 0x31 Ld SP, d16
    Instruction{op:Operation::LdR16I16{dst: Register::SP}, length:3, cycles:3},
    // 0x32 Ld (HL-), A
    Instruction{op:Operation::LdMR16Mv{add: -1}, length:1, cycles:2},
    // 0x33 Inc SP
    Instruction{op:Operation::IncR16{dst: Register::SP}, length:1, cycles:2},
    // 0x34 Inc (HL)
    Instruction{op:Operation::IncM{dst:Register::HL}, length:1, cycles:3},
    // 0x35 Dec (HL)
    Instruction{op:Operation::DecM{dst:Register::HL}, length:1, cycles:3},
    // 0x36 Ld (HL), d8
    Instruction{op:Operation::LdMI{dst:Register::HL}, length:2, cycles:3},
    // 0x37 Scf
    Instruction{op:Operation::Scf, length:1, cycles:1},
    // 0x38 Jr C, s8
    Instruction{op:Operation::Jr{cond:JumpCondition::C}, length:2, cycles:2},
    // 0x39 Add HL, SP
    Instruction{op:Operation::AddR16R16{dst:Register::HL, src:Register::SP}, length:1, cycles:2},
    // 0x3A Ld A, (HL-)
    Instruction{op:Operation::LdRMMv{add:-1}, length:1, cycles:2},
    // 0x3B Dec SP
    Instruction{op:Operation::DecR16{dst:Register::SP}, length:1, cycles:2},
    // 0x3C Inc A
    Instruction{op:Operation::IncR{dst:Register::A}, length:1, cycles:1},
    // 0x3D Dec A
    Instruction{op:Operation::DecR{dst:Register::A}, length:1, cycles:1},
    // 0x3E Ld A, d8
    Instruction{op:Operation::LdRI{dst:Register::A}, length:2, cycles:2},
    // 0x3F Ccf
    Instruction{op:Operation::Ccf, length:1, cycles:1},

    // 0x4X
    // 0x40 Ld B, B
    Instruction{op:Operation::LdRR{dst:Register::B, src:Register::B}, length:1, cycles:1},
    // 0x41 Ld B, C
    Instruction{op:Operation::LdRR{dst:Register::B, src:Register::C}, length:1, cycles:1},
    // 0x42 Ld B, D
    Instruction{op:Operation::LdRR{dst:Register::B, src:Register::D}, length:1, cycles:1},
    // 0x43 Ld B, E
    Instruction{op:Operation::LdRR{dst:Register::B, src:Register::E}, length:1, cycles:1},
    // 0x44 Ld B, H
    Instruction{op:Operation::LdRR{dst:Register::B, src:Register::H}, length:1, cycles:1},
    // 0x45 Ld B, L
    Instruction{op:Operation::LdRR{dst:Register::B, src:Register::L}, length:1, cycles:1},
    // 0x46 Ld B, (HL)
    Instruction{op:Operation::LdRM{dst:Register::B, src:Register::HL}, length:1, cycles:2},
    // 0x47 Ld B, A
    Instruction{op:Operation::LdRR{dst:Register::B, src:Register::A}, length:1, cycles:1},
    // 0x48 Ld C, B
    Instruction{op:Operation::LdRR{dst:Register::C, src:Register::B}, length:1, cycles:1},
    // 0x49 Ld C, C
    Instruction{op:Operation::LdRR{dst:Register::C, src:Register::C}, length:1, cycles:1},
    // 0x4A Ld C, D
    Instruction{op:Operation::LdRR{dst:Register::C, src:Register::D}, length:1, cycles:1},
    // 0x4B Ld C, E
    Instruction{op:Operation::LdRR{dst:Register::C, src:Register::E}, length:1, cycles:1},
    // 0x4C Ld C, H
    Instruction{op:Operation::LdRR{dst:Register::C, src:Register::H}, length:1, cycles:1},
    // 0x4D Ld C, L
    Instruction{op:Operation::LdRR{dst:Register::C, src:Register::L}, length:1, cycles:1},
    // 0x4E Ld C, (HL)
    Instruction{op:Operation::LdRM{dst:Register::C, src:Register::HL}, length:1, cycles:2},
    // 0x4F Ld C, A
    Instruction{op:Operation::LdRR{dst:Register::C, src:Register::A}, length:1, cycles:1},

    // 0x5X
    // 0x50 Ld D, B
    Instruction{op:Operation::LdRR{dst:Register::D, src:Register::B}, length:1, cycles:1},
    // 0x51 Ld D, C
    Instruction{op:Operation::LdRR{dst:Register::D, src:Register::C}, length:1, cycles:1},
    // 0x52 Ld D, D
    Instruction{op:Operation::LdRR{dst:Register::D, src:Register::D}, length:1, cycles:1},
    // 0x53 Ld D, E
    Instruction{op:Operation::LdRR{dst:Register::D, src:Register::E}, length:1, cycles:1},
    // 0x54 Ld D, H
    Instruction{op:Operation::LdRR{dst:Register::D, src:Register::H}, length:1, cycles:1},
    // 0x55 Ld D, L
    Instruction{op:Operation::LdRR{dst:Register::D, src:Register::L}, length:1, cycles:1},
    // 0x56 Ld D, (HL)
    Instruction{op:Operation::LdRM{dst:Register::D, src:Register::HL}, length:1, cycles:2},
    // 0x57 Ld D, A
    Instruction{op:Operation::LdRR{dst:Register::D, src:Register::A}, length:1, cycles:1},
    // 0x58 Ld E, B
    Instruction{op:Operation::LdRR{dst:Register::E, src:Register::B}, length:1, cycles:1},
    // 0x59 Ld E, C
    Instruction{op:Operation::LdRR{dst:Register::E, src:Register::C}, length:1, cycles:1},
    // 0x5A Ld E, D
    Instruction{op:Operation::LdRR{dst:Register::E, src:Register::D}, length:1, cycles:1},
    // 0x5B Ld E, E
    Instruction{op:Operation::LdRR{dst:Register::E, src:Register::E}, length:1, cycles:1},
    // 0x5C Ld E, H
    Instruction{op:Operation::LdRR{dst:Register::E, src:Register::H}, length:1, cycles:1},
    // 0x5D Ld E, L
    Instruction{op:Operation::LdRR{dst:Register::E, src:Register::L}, length:1, cycles:1},
    // 0x5E Ld E, (HL)
    Instruction{op:Operation::LdRM{dst:Register::E, src:Register::HL}, length:1, cycles:2},
    // 0x5F Ld E, A
    Instruction{op:Operation::LdRR{dst:Register::E, src:Register::A}, length:1, cycles:1},
    
    // 0x6X
    // 0x60 Ld D, B
    Instruction{op:Operation::LdRR{dst:Register::H, src:Register::B}, length:1, cycles:1},
    // 0x61 Ld D, C
    Instruction{op:Operation::LdRR{dst:Register::H, src:Register::C}, length:1, cycles:1},
    // 0x62 Ld D, D
    Instruction{op:Operation::LdRR{dst:Register::H, src:Register::D}, length:1, cycles:1},
    // 0x63 Ld D, E
    Instruction{op:Operation::LdRR{dst:Register::H, src:Register::E}, length:1, cycles:1},
    // 0x64 Ld D, H
    Instruction{op:Operation::LdRR{dst:Register::H, src:Register::H}, length:1, cycles:1},
    // 0x65 Ld D, L
    Instruction{op:Operation::LdRR{dst:Register::H, src:Register::L}, length:1, cycles:1},
    // 0x66 Ld D, (HL)
    Instruction{op:Operation::LdRM{dst:Register::H, src:Register::HL}, length:1, cycles:2},
    // 0x67 Ld D, A
    Instruction{op:Operation::LdRR{dst:Register::H, src:Register::A}, length:1, cycles:1},
    // 0x68 Ld E, B
    Instruction{op:Operation::LdRR{dst:Register::L, src:Register::B}, length:1, cycles:1},
    // 0x69 Ld E, C
    Instruction{op:Operation::LdRR{dst:Register::L, src:Register::C}, length:1, cycles:1},
    // 0x6A Ld E, D
    Instruction{op:Operation::LdRR{dst:Register::L, src:Register::D}, length:1, cycles:1},
    // 0x6B Ld E, E
    Instruction{op:Operation::LdRR{dst:Register::L, src:Register::E}, length:1, cycles:1},
    // 0x6C Ld E, H
    Instruction{op:Operation::LdRR{dst:Register::L, src:Register::H}, length:1, cycles:1},
    // 0x6D Ld E, L
    Instruction{op:Operation::LdRR{dst:Register::L, src:Register::L}, length:1, cycles:1},
    // 0x6E Ld E, (HL)
    Instruction{op:Operation::LdRM{dst:Register::L, src:Register::HL}, length:1, cycles:2},
    // 0x6F Ld E, A
    Instruction{op:Operation::LdRR{dst:Register::L, src:Register::A}, length:1, cycles:1},

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

    fn test_op_ldRR(inst: &[u8], dst:Register, src:Register)
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();

        load_into_ram(&mut ram, &inst);

        let value = 0x55;
        cpu.reg.write8(dst, 0xAA);
        cpu.reg.write8(src, value);
        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 1);
        assert_eq!(cpu.reg.pc, 1);
        assert_eq!(cpu.reg.read8(dst), value);
        assert_eq!(cpu.reg.read8(src), value);
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

    #[test]
    fn cpu_0x30()
    {
        // Jr NC, s8
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        load_into_ram(&mut ram, &[0x30, -1i8 as u8]);

        // All flags cleared, branch is taken.
        cpu.reg.f = 0;
        let cycles = cpu.execute_instruction(&mut ram);
        assert_eq!(cpu.reg.pc, 1);
        assert_eq!(cycles, 3);

        // Carry flag set, branch is not taken.
        cpu = Cpu::new();
        cpu.reg.f = Regs::CARRY_FLAG;
        let cycles = cpu.execute_instruction(&mut ram);
        assert_eq!(cpu.reg.pc, 2);
        assert_eq!(cycles, 2);
    }

    #[test]
    fn cpu_0x31()
    {
        // LD SP, d16
        test_op_ldR16I16(&[0x31, 0xAA, 0xBB], Register::SP, 0xBBAA);
    }

    #[test]
    fn cpu_0x32()
    {
        // LD (HL-), A
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        load_into_ram(&mut ram, &[0x32]);
        
        cpu.reg.write16(Register::HL, 0x1234);
        cpu.reg.a = 91;
        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 2);
        assert_eq!(cpu.reg.pc, 1);
        assert_eq!(cpu.reg.read16(Register::HL), 0x1233);
        assert_eq!(ram.bus_read8(0x1234), 91);
    }

    #[test]
    fn cpu_0x33()
    {
        // Inc SP
        test_op_incR16(&[0x33], Register::SP);
    }

    #[test]
    fn cpu_0x34()
    {
        // Inc (HL)
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        load_into_ram(&mut ram, &[0x34]);

        let address = 0x1000;
        ram.bus_write16(address, 0xFF);
        cpu.reg.f = 0xF0;
        cpu.reg.write16(Register::HL, address as u16);

        let cycles = cpu.execute_instruction(&mut ram);
        assert_eq!(cycles, 3);
        assert_eq!(cpu.reg.pc, 1);
        assert_eq!(ram.bus_read8(address), 0);
        assert_eq!(cpu.reg.f, Regs::HCARRY_FLAG | Regs::CARRY_FLAG | Regs::ZERO_FLAG);

        cpu.reg.pc = 0;
        cpu.execute_instruction(&mut ram);
        assert_eq!(cpu.reg.f, Regs::CARRY_FLAG); // carry still unchanged from before.
        assert_eq!(ram.bus_read8(address), 0x01);

        cpu.reg.pc = 0;
        cpu.reg.f = 0;
        ram.bus_write8(address, 0x0F);
        cpu.execute_instruction(&mut ram);
        assert_eq!(cpu.reg.f, Regs::HCARRY_FLAG); // Half carry set
        assert_eq!(ram.bus_read8(address), 0x10);
    }

    #[test]
    fn cpu_0x35()
    {
        // Dec (HL)
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        load_into_ram(&mut ram, &[0x35]);

        let address = 0x1001;
        ram.bus_write16(address, 0);
        cpu.reg.f = Regs::CARRY_FLAG;
        cpu.reg.write16(Register::HL, address as u16);

        let cycles = cpu.execute_instruction(&mut ram);
        assert_eq!(cycles, 3);
        assert_eq!(cpu.reg.pc, 1);
        assert_eq!(ram.bus_read8(address), 0xFF);
        assert_eq!(cpu.reg.f, Regs::HCARRY_FLAG | Regs::CARRY_FLAG | Regs::SUB_FLAG);

        cpu.reg.pc = 0;
        cpu.reg.f = 0;
        ram.bus_write8(address, 0x01);
        cpu.execute_instruction(&mut ram);
        assert_eq!(cpu.reg.f, Regs::SUB_FLAG | Regs::ZERO_FLAG);
        assert_eq!(ram.bus_read8(address), 0x00);
    }

    #[test]
    fn cpu_0x36()
    {
        // LD (HL), d8
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        let value = 99u8;
        let address = 0xFF;
        load_into_ram(&mut ram, &[0x36, value]);

        cpu.reg.write16(Register::HL, address);
        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 3);
        assert_eq!(cpu.reg.pc, 2);
        assert_eq!(ram.bus_read8(address as usize), value);
    }

    #[test]
    fn cpu_0x37()
    {
        // SCF
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        load_into_ram(&mut ram, &[0x37]);
        
        cpu.reg.f = Regs::HCARRY_FLAG | Regs::ZERO_FLAG | Regs::SUB_FLAG;
        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 1);
        assert_eq!(cpu.reg.f, Regs::CARRY_FLAG | Regs::ZERO_FLAG);
    }

    #[test]
    fn cpu_0x38()
    {
        // Jr C, i8
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        load_into_ram(&mut ram, &[0x38, 4]);

        // Branch was not taken
        cpu.reg.f = 0;
        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 2);
        assert_eq!(cpu.reg.pc, 2);

        // Branch was taken
        cpu.reg.f = Regs::CARRY_FLAG;
        cpu.reg.pc = 0;
        let cycles = cpu.execute_instruction(&mut ram);
        
        assert_eq!(cycles, 3);
        assert_eq!(cpu.reg.pc, 6);
    }

    #[test]
    fn cpu_0x39()
    {
        // ADD HL, SP
        test_op_addr16r16(&[0x39], Register::HL, Register::SP);
    }

    #[test]
    fn cpu_0x3A()
    {
        // LD A (HL-)
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        load_into_ram(&mut ram, &[0x3A]);
        
        let address = 0xFF;
        let value = 0x91;
        cpu.reg.write16(Register::HL, address);
        ram.bus_write8(address as usize, value);
        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 2);
        assert_eq!(cpu.reg.pc, 1);
        assert_eq!(cpu.reg.read16(Register::HL), address - 1);
        assert_eq!(cpu.reg.a, value);
    }

    #[test]
    fn cpu_0x3B()
    {
        // Dec SP
        test_op_decR16(&[0x3B], Register::SP);
    }

    #[test]
    fn cpu_0x3C()
    {
        // Inc A
        test_op_incR(&[0x3C], Register::A);
    }

    #[test]
    fn cpu_0x3D()
    {
        // Dec A
        test_op_decR(&[0x3D], Register::A);
    }

    #[test]
    fn cpu_0x3E()
    {
        test_op_ldRI(&[0x3E, 0x01], Register::A, 0x01);
    }

    #[test]
    fn cpu_0x3F()
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        load_into_ram(&mut ram, &[0x3F]);

        cpu.reg.f = 0xFF;
        let cycles = cpu.execute_instruction(&mut ram);
        assert_eq!(cycles, 1);
        assert_eq!(cpu.reg.pc, 1);
        assert_eq!(cpu.reg.f, Regs::ZERO_FLAG);

        cpu.reg.pc = 0;
        cpu.reg.f = 0x00;
        cpu.execute_instruction(&mut ram);
        assert_eq!(cpu.reg.f, Regs::CARRY_FLAG);
    }

    #[test]
    fn cpu_0x40()
    {
        test_op_ldRR(&[0x40], Register::B, Register::B);
    }

    #[test]
    fn cpu_0x41()
    {
        test_op_ldRR(&[0x41], Register::B, Register::C);
    }

    #[test]
    fn cpu_0x42()
    {
        test_op_ldRR(&[0x42], Register::B, Register::D);
    }

    #[test]
    fn cpu_0x43()
    {
        test_op_ldRR(&[0x43], Register::B, Register::E);
    }

    #[test]
    fn cpu_0x44()
    {
        test_op_ldRR(&[0x44], Register::B, Register::H);
    }

    #[test]
    fn cpu_0x45()
    {
        test_op_ldRR(&[0x45], Register::B, Register::L);
    }

    #[test]
    fn cpu_0x46()
    {
        test_op_ldRM(&[0x46], Register::B, Register::HL, 0x4325, 23);
    }

    #[test]
    fn cpu_0x47()
    {
        test_op_ldRR(&[0x47], Register::B, Register::A);
    }

    #[test]
    fn cpu_0x48()
    {
        test_op_ldRR(&[0x48], Register::C, Register::B);
    }

    #[test]
    fn cpu_0x49()
    {
        test_op_ldRR(&[0x49], Register::C, Register::C);
    }

    #[test]
    fn cpu_0x4A()
    {
        test_op_ldRR(&[0x4A], Register::C, Register::D);
    }

    #[test]
    fn cpu_0x4B()
    {
        test_op_ldRR(&[0x4B], Register::C, Register::E);
    }

    #[test]
    fn cpu_0x4C()
    {
        test_op_ldRR(&[0x4C], Register::C, Register::H);
    }

    #[test]
    fn cpu_0x4D()
    {
        test_op_ldRR(&[0x4d], Register::C, Register::L);
    }

    #[test]
    fn cpu_0x4E()
    {
        test_op_ldRM(&[0x4E], Register::C, Register::HL, 0x1432, 9);
    }

    #[test]
    fn cpu_0x4F()
    {
        test_op_ldRR(&[0x4F], Register::C, Register::A);
    }

    #[test]
    fn cpu_0x50()
    {
        test_op_ldRR(&[0x50], Register::D, Register::B)
    }

    #[test]
    fn cpu_0x51()
    {
        test_op_ldRR(&[0x51], Register::D, Register::C)
    }

    #[test]
    fn cpu_0x52()
    {
        test_op_ldRR(&[0x52], Register::D, Register::D)
    }

    #[test]
    fn cpu_0x53()
    {
        test_op_ldRR(&[0x53], Register::D, Register::E);
    }

    #[test]
    fn cpu_0x54()
    {
        test_op_ldRR(&[0x54], Register::D, Register::H);
    }

    #[test]
    fn cpu_0x55()
    {
        test_op_ldRR(&[0x55], Register::D, Register::L);
    }

    #[test]
    fn cpu_0x56()
    {
        test_op_ldRM(&[0x56], Register::D, Register::HL, 0x12, 0x32);
    }

    #[test]
    fn cpu_0x57()
    {
        test_op_ldRR(&[0x57], Register::D, Register::A);
    }

    #[test]
    fn cpu_0x58()
    {
        test_op_ldRR(&[0x58], Register::E, Register::B);
    }

    #[test]
    fn cpu_0x59()
    {
        test_op_ldRR(&[0x59], Register::E, Register::C);
    }

    #[test]
    fn cpu_0x5A()
    {
        test_op_ldRR(&[0x5A], Register::E, Register::D);
    }

    #[test]
    fn cpu_0x5B()
    {
        test_op_ldRR(&[0x5B], Register::E, Register::E);
    }

    #[test]
    fn cpu_0x5C()
    {
        test_op_ldRR(&[0x5C], Register::E, Register::H);
    }

    #[test]
    fn cpu_0x5D()
    {
        test_op_ldRR(&[0x5D], Register::E, Register::L);
    }

    #[test]
    fn cpu_0x5E()
    {
        test_op_ldRM(&[0x5E], Register::E, Register::HL, 0x9874, 9);
    }

    #[test]
    fn cpu_0x5F()
    {
        test_op_ldRR(&[0x5F], Register::E, Register::A);
    }

    #[test]
    fn cpu_0x60()
    {
        test_op_ldRR(&[0x60], Register::H, Register::B);
    }

    #[test]
    fn cpu_0x61()
    {
        test_op_ldRR(&[0x61], Register::H, Register::C);
    }

    #[test]
    fn cpu_0x62()
    {
        test_op_ldRR(&[0x62], Register::H, Register::D);
    }

    #[test]
    fn cpu_0x63()
    {
        test_op_ldRR(&[0x63], Register::H, Register::E);
    }

    #[test]
    fn cpu_0x64()
    {
        test_op_ldRR(&[0x64], Register::H, Register::H);
    }

    #[test]
    fn cpu_0x65()
    {
        test_op_ldRR(&[0x65], Register::H, Register::L);
    }
     
    #[test]
    fn cpu_0x66()
    {
        test_op_ldRM(&[0x66], Register::H, Register::HL, 0x3549, 34);
    }

    #[test]
    fn cpu_0x67()
    {
        test_op_ldRR(&[0x67], Register::H, Register::A);
    }

    #[test]
    fn cpu_0x68()
    {
        test_op_ldRR(&[0x68], Register::L, Register::B);
    }

    #[test]
    fn cpu_0x69()
    {
        test_op_ldRR(&[0x69], Register::L, Register::C);
    }

    #[test]
    fn cpu_0x6A()
    {
        test_op_ldRR(&[0x6A], Register::L, Register::D);
    }

    #[test]
    fn cpu_0x6B()
    {
        test_op_ldRR(&[0x6B], Register::L, Register::E);
    }

    #[test]
    fn cpu_0x6C()
    {
        test_op_ldRR(&[0x6C], Register::L, Register::H);
    }

    #[test]
    fn cpu_0x6D()
    {
        test_op_ldRR(&[0x6D], Register::L, Register::L);
    }

    #[test]
    fn cpu_0x6E()
    {
        test_op_ldRM(&[0x6E], Register::L, Register::HL, 123, 38);
    }

    #[test]
    fn cpu_0x6F()
    {
        test_op_ldRR(&[0x6F], Register::L, Register::A);
    }

}
