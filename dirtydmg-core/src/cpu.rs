use byteorder::{ByteOrder, LittleEndian};
use crate::interrupt::InterruptStatus;
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

pub struct Regs {
    pub a:u8,
    pub f:u8,
    pub b:u8,
    pub c:u8,
    pub d:u8,
    pub e:u8,
    pub h:u8,
    pub l:u8,
    pub sp: u16,
    pub pc: u16,
}

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
            Register::F => self.f = value & 0xF0,
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
                self.f = value as u8 & 0xF0;
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
enum CpuMode {
    Running,
    Halted,
    Stopped,
}

#[allow(dead_code)]
pub struct Cpu {
    /// CPU registers.
    pub reg: Regs,
    /// Tracks cycles until the cpu will be ready to execute again.
    busy_cycles: u32, 
    /// Tracks the current cycle number.
    cycle: u64,       
    /// Tracks if interrupts are enabled or disabled.
    isr_en: bool,
    /// Tracks if the next "instruction" is the start of an ISR.
    isr_pending: bool,
    /// Tracks if the cpu is running halted or stopped.
    mode: CpuMode,
}

#[allow(dead_code)]
impl Cpu {
    const ISR_OVERHEAD_CYCLES:u8 = 5;
    const VBLANK_ISR_ADDR:u16 = 0x40;
    const LCD_ISR_ADDR:u16    = 0x48;
    const TIMER_ISR_ADDR:u16  = 0x50;
    const SERIAL_ISR_ADDR:u16 = 0x58;
    const JOYPAD_ISR_ADDR:u16 = 0x60;

    pub fn new()->Cpu {
        Cpu {
            reg:Regs::new(),
            busy_cycles: 0,
            cycle: 0,
            isr_en: false,
            isr_pending: false,
            mode: CpuMode::Running,
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
        self.reg.a
    }

    fn read_pc(&mut self, bus:&mut impl BusRW) -> u8
    {
        let value = bus.bus_read8(self.reg.pc as usize);
        self.reg.pc += 1;
        value
    }

    fn test_condition(&mut self, cond:JumpCondition) -> bool
    {
        match cond {
            JumpCondition::Always => true,
            JumpCondition::Z =>  self.reg.f & Regs::ZERO_FLAG  != 0,
            JumpCondition::Nz => self.reg.f & Regs::ZERO_FLAG  == 0,
            JumpCondition::C =>  self.reg.f & Regs::CARRY_FLAG != 0,
            JumpCondition::Nc => self.reg.f & Regs::CARRY_FLAG == 0,
        }
    }

    /// # Attempts to start an ISR.
    /// 
    /// If interrupts are enabled, the isr is started.
    /// If interrupts are not enabled, the isr is not started.
    /// If the cpu is halted, the cpu is always returned to the running mode.
    /// 
    /// Returns true if the ISR was started, false if it was not.
    fn start_isr(&mut self, bus: &mut impl BusRW, addr: u16) -> bool
    {
        let started = self.isr_en;
        if started {
            // Write the current PC to the stack
            self.reg.sp = self.reg.sp.wrapping_sub(2);
            bus.bus_write16(self.reg.sp as usize, self.reg.pc);

            // Update the program counter
            self.reg.pc = addr;

            // Disable interrupts
            self.isr_en = false;
        }

        // If we are currently halted, return to running.
        if let CpuMode::Halted = self.mode {
            self.mode = CpuMode::Running;
        }

        started
    }

    /// Checks for and handles pending interrupts
    /// 
    /// Returns the number of busy cycles if an ISR is started, or zero if no interrupt started.
    #[allow(clippy::collapsible_if)]
    fn update_interrupts(&mut self, bus: &mut impl BusRW, is: &mut InterruptStatus) -> bool
    {
        let mut started:bool = false;

        // Check for interrupts if enabled.
        if is.is_vblank_active()
        {
            if self.start_isr(bus, Cpu::VBLANK_ISR_ADDR){
                started = true;
                is.clear_vblank();
            }
        }
        else if is.is_lcdstat_active()
        {
            if self.start_isr(bus, Cpu::LCD_ISR_ADDR){
                started = true;
                is.clear_lcdstat();
            }
        }
        else if is.is_timer_active()
        {
            if self.start_isr(bus, Cpu::TIMER_ISR_ADDR){
                started = true;
                is.clear_timer();
            }
        }
        else if is.is_serial_active()
        {
            if self.start_isr(bus, Cpu::SERIAL_ISR_ADDR){
                started = true;
                is.clear_serial();
            }
        }
        else if is.is_joypad_active()
        {
            if self.start_isr(bus, Cpu::JOYPAD_ISR_ADDR){
                started = true;
                is.clear_joypad();
            }
        }

        started
    }

    pub fn handle_interrupts(&mut self, bus: &mut impl BusRW, is: &mut InterruptStatus){
        if self.update_interrupts(bus, is)
        {
            self.busy_cycles = Cpu::ISR_OVERHEAD_CYCLES as u32;
            self.isr_pending = true;
        }
    }

    pub fn update(&mut self, bus: &mut impl BusRW) -> u8
    {
        // Handle any pending interrupts
        if self.isr_pending
        {
            self.busy_cycles = Cpu::ISR_OVERHEAD_CYCLES as u32;
            self.isr_pending = false;
        // Not starting an interrupt.
        } else { 
        
            // Execute depending on the current cpu mode.
            match self.mode {
                CpuMode::Running => {self.execute_instruction(bus);}
                CpuMode::Halted => { self.busy_cycles = 1; }
                _ => {panic!("Cpu is in an unimplemented mode");}
            }
        }

        self.busy_cycles as u8
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
        // println!("[{:#0X},{:#0X},{:#0X}]{:?} @ {:#04X}", data[0], data[1], data[2], instruction.op, self.reg.pc);
        self.execute_operation(bus, &data, &instruction.op);
        self.busy_cycles as u8
    }

    fn subc(&mut self, sub:u8){
        let sub = sub as u16;
        let minuend = self.reg.a as u16 | 0x100;
        let carry = ((self.reg.f & Regs::CARRY_FLAG) >> 4) as u16;

        let result = minuend - sub - carry;
        self.reg.a = result as u8;

        // Clear all flag bits while setting sub flag.
        self.reg.f = Regs::SUB_FLAG;

        // Check for carry flag
        if result & 0x100 == 0{
            self.reg.f |= Regs::CARRY_FLAG;
        }

        // Check for the half carry flag
        if (minuend ^ sub) & 0x10 != (result & 0x10){
            self.reg.f |= Regs::HCARRY_FLAG;
        }

        // Check for the zero flag
        if self.reg.a == 0 {
            self.reg.f |= Regs::ZERO_FLAG;
        }
    }

    fn rl(&mut self, val: u8)->u8
    {
        let result = (val << 1) | ((self.reg.f & Regs::CARRY_FLAG) >> 4);

        // Clear, then set the zero flag if needed.
        self.reg.f = 
            if result == 0 {Regs::ZERO_FLAG}
            else {0};

        // Check for the carry flag
        if (val & 0x80) != 0 {
            self.reg.f |= Regs::CARRY_FLAG;
        }

        result
    }

    fn rr(&mut self, val: u8) -> u8{
        let result = val >> 1 | ((self.reg.f & Regs::CARRY_FLAG) << 3);

        // Clear, then set the zero flag if needed.
        self.reg.f = 
            if result == 0 {Regs::ZERO_FLAG}
            else {0};

        // Check for the carry flag
        if (val & 0x01) != 0 {
            self.reg.f |= Regs::CARRY_FLAG;
        }

        result
    }

    fn sla(&mut self, val: u8) -> u8 {
        let result = val << 1;

        // Clear, then set the zero flag if needed.
        self.reg.f = 
            if result == 0 {Regs::ZERO_FLAG}
            else {0};

        // Check for the carry flag
        if (val & 0x80) != 0 {
            self.reg.f |= Regs::CARRY_FLAG;
        }

        result
    }

    fn sra(&mut self, val: u8) -> u8 {
        let result = (val >> 1) | (val & 0x80);

        // Clear, then set the zero flag if needed.
        self.reg.f = 
            if result == 0 {Regs::ZERO_FLAG}
            else {0};

        // Check for the carry flag
        if (val & 0x01) != 0 {
            self.reg.f |= Regs::CARRY_FLAG;
        }

        result
    }

    fn srl(&mut self, val: u8) -> u8 {
        let result = val >> 1;

        // Clear, then set the zero flag if needed.
        self.reg.f = 
            if result == 0 {Regs::ZERO_FLAG}
            else {0};

        // Check for the carry flag
        if (val & 0x01) != 0 {
            self.reg.f |= Regs::CARRY_FLAG;
        }

        result
    }

    fn bit(&mut self, val:u8, index:u8) {
        // retain the Carry flag, set the half carry flag, clear all others.
        self.reg.f &= Regs::CARRY_FLAG;
        self.reg.f |= Regs::HCARRY_FLAG;

        // set the zero flag if the index bit is not set.
        if (val & (1<<index)) == 0 {
            self.reg.f |= Regs::ZERO_FLAG;
        }
    }

    fn execute_operation(&mut self, bus:&mut impl BusRW, data: &[u8], op:&Operation)
    {
        use Operation::*;
        match op {
            Nop => (),
            Stop => { 
                panic!("Stop instruction @ {:#4X}", self.reg.pc)
            },

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

            // Save a register to the zero page addressed by an immediate value.
            LdZIR{src} => {
                bus.bus_write8(
                    0xFF00usize + data[1] as usize, 
                    self.reg.read8(*src));
            },

            // Save a register to the zero page addressed by another register
            LdZRR{dst, src} => {
                bus.bus_write8(
                    0xFF00usize + self.reg.read8(*dst) as usize,
                    self.reg.read8(*src)
                );
            }

            // Save a register into the memory addressed by a 16 bit immediate value
            LdMI16R{src} => {
                let address = LittleEndian::read_u16(&data[1..]) as usize;
                bus.bus_write8( address, self.reg.read8(*src));
            },

            // Load a register from zero page ram addressed by an 8 bit immediate value
            LdRZI{dst} => {
                let address = data[1] as usize + 0xFF00;
                self.reg.write8(*dst, bus.bus_read8(address));
            },

            // Load a register from zero page ram addressed by an 8 bit register.
            LdRZR{src, dst} => {
                let address = self.reg.read8(*src) as usize + 0xFF00;
                self.reg.write8(*dst, bus.bus_read8(address));
            },

            // Load the stack pointer with the content of HL.
            LdSpHl => {
                self.reg.sp = self.reg.read16(Register::HL);
            },

            // Load a register with the value in memory addressed by a 16 bit immediate.
            LdRMI16{dst} => {
                let address = LittleEndian::read_u16(&data[1..]) as usize;
                self.reg.write8(*dst, bus.bus_read8(address));
            },

            // Increment a 16 bit register.
            IncR16{dst} => {
                let i = self.reg.read16(*dst).wrapping_add(1);
                self.reg.write16(*dst, i);
            },

            // Add a signed 8 bit immediate to the stack pointer.
            StkMvI => {
                let initial = self.reg.sp;
                let addend = data[1] as i8 as u16;
                let result = initial.wrapping_add(addend);
                self.reg.write16(Register::HL, result);

                self.reg.f = 0;
                // If the addend was negative.
                if addend & 0x8000 != 0{
                    if (result & 0xFF) <= (initial & 0xFF)
                    {
                        self.reg.f |= Regs::CARRY_FLAG;
                    }
                    if (result & 0xF) <= (initial & 0xF)
                    {
                        self.reg.f |= Regs::HCARRY_FLAG;
                    }
                } 
                // The addend was positive.
                else {
                    if (initial ^ addend) & 0x10 != result & 0x10 {
                        self.reg.f |= Regs::HCARRY_FLAG;
                    }
                    if result & 0xFF < initial & 0xFF {
                        self.reg.f |= Regs::CARRY_FLAG;
                    }
                }
            },

            // Increment an 8 bit register.
            IncR{dst} => {
                let initial = self.reg.read8(*dst);
                let result = self.reg.read8(*dst).wrapping_add(1);
                self.reg.write8(*dst, result);

                // Only the carry flag is retained.
                self.reg.f &= Regs::CARRY_FLAG;

                // If there was a half carry
                if (initial ^ result) & 0x10 > 0 {
                     self.reg.f |= Regs::HCARRY_FLAG;
                }
                // If the result was zero
                if result == 0 {
                    self.reg.f |= Regs::ZERO_FLAG;
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
                    self.reg.f |= Regs::HCARRY_FLAG;
                }
                // If the result was zero
                if result == 0 {
                    self.reg.f |= Regs::ZERO_FLAG;
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
                    self.reg.f |= Regs::HCARRY_FLAG;
                }
                // Set carry flag if needed.
                if result < initial {
                    self.reg.f |= Regs::CARRY_FLAG;
                }
            },

            AddR16I{dst} => {
                let initial = self.reg.read16(*dst);
                let addend = data[1] as i8 as u16;
                let result = initial.wrapping_add(addend);
                self.reg.write16(*dst, result);

                self.reg.f = 0;
                // If the addend was negative.
                if addend & 0x8000 != 0{
                    if (result & 0xFF) <= (initial & 0xFF)
                    {
                        self.reg.f |= Regs::CARRY_FLAG;
                    }
                    if (result & 0xF) <= (initial & 0xF)
                    {
                        self.reg.f |= Regs::HCARRY_FLAG;
                    }
                } 
                // The addend was positive.
                else {
                    if (initial ^ addend) & 0x10 != result & 0x10 {
                        self.reg.f |= Regs::HCARRY_FLAG;
                    }
                    if result & 0xFF < initial & 0xFF {
                        self.reg.f |= Regs::CARRY_FLAG;
                    }
                }
            },

            AddR{src} => {
                let initial = self.reg.a;
                let addend = self.reg.read8(*src);
                self.reg.a  = self.reg.a.wrapping_add(addend);
                
                self.reg.f = 0;

                // Check for carry flag
                if self.reg.a < initial {
                    self.reg.f |= Regs::CARRY_FLAG;
                }

                // Check for the half carry flag
                if (initial ^ addend) & 0x10 != (self.reg.a & 0x10) {
                    self.reg.f |= Regs::HCARRY_FLAG;
                }

                // Check for the zero flag
                if self.reg.a == 0 {
                    self.reg.f |= Regs::ZERO_FLAG;
                }
            }

            AddCR{src} => {
                let initial = self.reg.a as u16;
                let addend_a = self.reg.read8(*src) as u16;
                let addend_b = (self.reg.f & Regs::CARRY_FLAG) as u16 >> 4;

                self.reg.f = 0;

                let result = initial + addend_a + addend_b;
                self.reg.a = result as u8;

                // Check for half carry.
                if (initial ^ addend_a) & 0x10 != (result & 0x10){
                    self.reg.f |= Regs::HCARRY_FLAG;
                }

                // Set carry if combined value overflows.
                if result > 0xFF  {
                    self.reg.f |= Regs::CARRY_FLAG;
                }

                // Check for the zero flag
                if self.reg.a == 0 {
                    self.reg.f |= Regs::ZERO_FLAG;
                }
            },

            AddM => {
                let initial = self.reg.a;
                let addend = bus.bus_read8(self.reg.read16(Register::HL) as usize);
                self.reg.a  = self.reg.a.wrapping_add(addend);
                
                // clear flags
                self.reg.f = 0;

                // Check for carry flag
                if self.reg.a < initial {
                    self.reg.f |= Regs::CARRY_FLAG;
                }

                // Check for the half carry flag
                if (initial ^ addend) & 0x10 != (self.reg.a & 0x10) {
                    self.reg.f |= Regs::HCARRY_FLAG;
                }

                // Check for the zero flag
                if self.reg.a == 0 {
                    self.reg.f |= Regs::ZERO_FLAG;
                }
            },

            AddI => {
                let initial = self.reg.a;
                let addend = data[1];
                self.reg.a  = self.reg.a.wrapping_add(addend);
                
                // clear flags
                self.reg.f = 0;

                // Check for carry flag
                if self.reg.a < initial {
                    self.reg.f |= Regs::CARRY_FLAG;
                }

                // Check for the half carry flag
                if (initial ^ addend) & 0x10 != (self.reg.a & 0x10) {
                    self.reg.f |= Regs::HCARRY_FLAG;
                }

                // Check for the zero flag
                if self.reg.a == 0 {
                    self.reg.f |= Regs::ZERO_FLAG;
                }
            }

            AddCM => {
                let initial = self.reg.a as u16;
                let addend_a =bus.bus_read8(self.reg.read16(Register::HL) as usize) as u16;
                let addend_b = (self.reg.f & Regs::CARRY_FLAG) as u16 >> 4;

                self.reg.f = 0;

                let result = initial + addend_a + addend_b;
                self.reg.a = result as u8;

                // Check for half carry.
                if (initial ^ addend_a) & 0x10 != (result & 0x10){
                    self.reg.f |= Regs::HCARRY_FLAG;
                }

                // Set carry if combined value overflows.
                if result > 0xFF  {
                    self.reg.f |= Regs::CARRY_FLAG;
                }

                // Check for the zero flag
                if self.reg.a == 0 {
                    self.reg.f |= Regs::ZERO_FLAG;
                }
            }

            AddCI => {
                let initial = self.reg.a as u16;
                let addend_a = data[1] as u16;
                let addend_b = (self.reg.f & Regs::CARRY_FLAG) as u16 >> 4;

                self.reg.f = 0;

                let result = initial + addend_a + addend_b;
                self.reg.a = result as u8;

                // Check for half carry.
                if (initial ^ addend_a) & 0x10 != (result & 0x10){
                    self.reg.f |= Regs::HCARRY_FLAG;
                }

                // Set carry if combined value overflows.
                if result > 0xFF  {
                    self.reg.f |= Regs::CARRY_FLAG;
                }

                // Check for the zero flag
                if self.reg.a == 0 {
                    self.reg.f |= Regs::ZERO_FLAG;
                }
            }

            SubR{src} => {
                let minuend = self.reg.a;
                let sub = self.reg.read8(*src);
                self.reg.a = self.reg.a.wrapping_sub(sub);

                self.reg.f = Regs::SUB_FLAG;

                // Check for carry flag
                if self.reg.a > minuend {
                    self.reg.f |= Regs::CARRY_FLAG;
                }

                // Check for the half carry flag
                if (self.reg.a << 4) > (minuend << 4){
                    self.reg.f |= Regs::HCARRY_FLAG;
                }

                // Check for the zero flag
                if self.reg.a == 0 {
                    self.reg.f |= Regs::ZERO_FLAG;
                }
            }

            SubI => {
                let minuend = self.reg.a;
                let sub = data[1];
                self.reg.a = self.reg.a.wrapping_sub(sub);

                self.reg.f = Regs::SUB_FLAG;

                // Check for carry flag
                if self.reg.a > minuend {
                    self.reg.f |= Regs::CARRY_FLAG;
                }

                // Check for the half carry flag
                if (self.reg.a << 4) > (minuend << 4){
                    self.reg.f |= Regs::HCARRY_FLAG;
                }

                // Check for the zero flag
                if self.reg.a == 0 {
                    self.reg.f |= Regs::ZERO_FLAG;
                }
            }

            SubM => {
                let minuend = self.reg.a;
                let sub = bus.bus_read8(self.reg.read16(Register::HL) as usize);
                self.reg.a = self.reg.a.wrapping_sub(sub);

                self.reg.f = Regs::SUB_FLAG;

                // Check for carry flag
                if self.reg.a > minuend {
                    self.reg.f |= Regs::CARRY_FLAG;
                }

                // Check for the half carry flag
                if (self.reg.a << 4) > (minuend << 4){
                    self.reg.f |= Regs::HCARRY_FLAG;
                }

                // Check for the zero flag
                if self.reg.a == 0 {
                    self.reg.f |= Regs::ZERO_FLAG;
                }
            }

            SbcR {src} => {
                self.subc(self.reg.read8(*src));
            }

            SbcM => {
                self.subc(bus.bus_read8(self.reg.read16(Register::HL) as usize));
            },

            SbcI => {
                self.subc(data[1]);
            }

            AndR{src} => {
                self.reg.a &= self.reg.read8(*src);
                self.reg.f = Regs::HCARRY_FLAG;
                if self.reg.a == 0 {
                    self.reg.f |= Regs::ZERO_FLAG;
                }
            }

            AndM => {
                self.reg.a &= bus.bus_read8(self.reg.read16(Register::HL) as usize);
                self.reg.f = Regs::HCARRY_FLAG;
                if self.reg.a == 0 {
                    self.reg.f |= Regs::ZERO_FLAG;
                }
            },

            AndI => {
                self.reg.a &= data[1];
                self.reg.f = Regs::HCARRY_FLAG;
                if self.reg.a == 0 {
                    self.reg.f |= Regs::ZERO_FLAG;
                }
            },

            XorR{src} => {
                self.reg.a ^= self.reg.read8(*src);
                self.reg.f = 0;
                if self.reg.a == 0 {
                    self.reg.f |= Regs::ZERO_FLAG;
                }
            }

            XorM => {
                self.reg.a ^= bus.bus_read8(self.reg.read16(Register::HL) as usize);
                self.reg.f = 0;
                if self.reg.a == 0 {
                    self.reg.f |= Regs::ZERO_FLAG;
                }
            }

            XorI => {
                self.reg.a ^= data[1];
                self.reg.f = 0;
                if self.reg.a == 0 {
                    self.reg.f |= Regs::ZERO_FLAG;
                }
            },

            OrR{src} => {
                self.reg.a |= self.reg.read8(*src);
                self.reg.f = 0;
                if self.reg.a == 0 {
                    self.reg.f |= Regs::ZERO_FLAG;
                }
            }

            OrM => {
                self.reg.a |= bus.bus_read8(self.reg.read16(Register::HL) as usize);
                self.reg.f = 0;
                if self.reg.a == 0 {
                    self.reg.f |= Regs::ZERO_FLAG;
                }
            },

            OrI => {
                self.reg.a |= data[1];
                self.reg.f = 0;
                if self.reg.a == 0 {
                    self.reg.f |= Regs::ZERO_FLAG;
                }
            }

            CpR{src} => {
                let minuend = self.reg.a;
                let sub = self.reg.read8(*src);
                let result = self.reg.a.wrapping_sub(sub);

                self.reg.f = Regs::SUB_FLAG;

                // Check for carry flag
                if result > minuend {
                    self.reg.f |= Regs::CARRY_FLAG;
                }

                // Check for the half carry flag
                if (result << 4) > (minuend << 4){
                    self.reg.f |= Regs::HCARRY_FLAG;
                }

                // Check for the zero flag
                if result == 0 {
                    self.reg.f |= Regs::ZERO_FLAG;
                }
            }

            CpM => {
                let minuend = self.reg.a;
                let sub = bus.bus_read8(self.reg.read16(Register::HL) as usize);
                let result = self.reg.a.wrapping_sub(sub);

                self.reg.f = Regs::SUB_FLAG;

                // Check for carry flag
                if result > minuend {
                    self.reg.f |= Regs::CARRY_FLAG;
                }

                // Check for the half carry flag
                if (result << 4) > (minuend << 4){
                    self.reg.f |= Regs::HCARRY_FLAG;
                }

                // Check for the zero flag
                if result == 0 {
                    self.reg.f |= Regs::ZERO_FLAG;
                }
            }

            CpI => {
                let minuend = self.reg.a;
                let sub = data[1];
                let result = self.reg.a.wrapping_sub(sub);

                self.reg.f = Regs::SUB_FLAG;

                // Check for carry flag
                if result > minuend {
                    self.reg.f |= Regs::CARRY_FLAG;
                }

                // Check for the half carry flag
                if (result << 4) > (minuend << 4){
                    self.reg.f |= Regs::HCARRY_FLAG;
                }

                // Check for the zero flag
                if result == 0 {
                    self.reg.f |= Regs::ZERO_FLAG;
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
                if self.test_condition(*cond) {
                    // get the signed jump offset, then add it to the program counter.
                    let jump_offset = data[1] as i8;
                    self.reg.pc = self.reg.pc.wrapping_add(jump_offset as u16);
                    self.busy_cycles += 1;
                }
            },

            Jp{cond} => {
                if self.test_condition(*cond){
                    self.reg.pc = LittleEndian::read_u16(&data[1..]);
                    self.busy_cycles += 1;
                }
            },

            JpHL => {
                self.reg.pc = self.reg.read16(Register::HL);
            },

            Call{cond} => {
                if self.test_condition(*cond) {
                    self.reg.sp = self.reg.sp.wrapping_sub(2);
                    bus.bus_write16(self.reg.sp as usize, self.reg.pc);
                    self.reg.pc = LittleEndian::read_u16(&data[1..]);
                    self.busy_cycles += 3;
                }
            },

            Ret{cond} => {
                if self.test_condition(*cond) {
                    let target = bus.bus_read16(self.reg.sp as usize);
                    self.reg.pc = target;
                    self.reg.sp = self.reg.sp.wrapping_add(2);
                    self.busy_cycles += 3;
                }
            },

            Reti => {
                // Pop the stack and jump to the address.
                let target = bus.bus_read16(self.reg.sp as usize);
                self.reg.pc = target;
                self.reg.sp += 2;

                // Enable interrupts
                self.isr_en = true;
            },

            Rst{index} => {
                // Push the PC to the stack.
                self.reg.sp = self.reg.sp.wrapping_sub(2);
                bus.bus_write16(self.reg.sp as usize, self.reg.pc);
                // Jump to the index location.
                self.reg.pc = *index as u16 * 8;
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

            LdMR{dst, src} =>{
                let addr = self.reg.read16(*dst) as usize;
                bus.bus_write8(addr, self.reg.read8(*src));
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

            Pop{dst} => {
                self.reg.write16(*dst, bus.bus_read16(self.reg.sp as usize));
                self.reg.sp += 2;
            },

            Push{src} => {
                self.reg.sp = self.reg.sp.wrapping_sub(2);
                bus.bus_write16(self.reg.sp as usize, self.reg.read16(*src));
            },

            Di => {
                self.isr_en = false;
            },

            Ei => {
                self.isr_en = true;
            }

            //0xCB instructions.
            CBExt => {
                // 0xCB instructions are very well structured.
                // The low 3 bits are the argument register/address,
                // and the high 8 bits are the operation code.
                let opcode = (data[1] & 0b_1111_1000) >> 3;
                let target = CB_TARGET_TABLE[(data[1] & 0b_0000_0111) as usize];
                let operation = 
                    // when target is HL, that means this is a memory based operation.
                    if target == Register::HL {
                        match opcode {
                            0x00 => RlcM,
                            0x01 => RrcM,
                            0x02 => RlM,
                            0x03 => RrM,
                            0x04 => SlaM,
                            0x05 => SraM,
                            0x06 => SwapM,
                            0x07 => SrlM,
                            0x08 => BitM{index:0},
                            0x09 => BitM{index:1},
                            0x0A => BitM{index:2},
                            0x0B => BitM{index:3},
                            0x0C => BitM{index:4},
                            0x0D => BitM{index:5},
                            0x0E => BitM{index:6},
                            0x0F => BitM{index:7},
                            0x10 => ResM{index:0},
                            0x11 => ResM{index:1},
                            0x12 => ResM{index:2},
                            0x13 => ResM{index:3},
                            0x14 => ResM{index:4},
                            0x15 => ResM{index:5},
                            0x16 => ResM{index:6},
                            0x17 => ResM{index:7},
                            0x18 => SetM{index:0},
                            0x19 => SetM{index:1},
                            0x1A => SetM{index:2},
                            0x1B => SetM{index:3},
                            0x1C => SetM{index:4},
                            0x1D => SetM{index:5},
                            0x1E => SetM{index:6},
                            0x1F => SetM{index:7},
                            _ => panic!("not implemented")
                        }
                    }
                    // target is a normal register.
                    else {
                        match opcode {
                            0x00 => RlcR{src:target},
                            0x01 => RrcR{src:target},
                            0x02 => RlR{src:target},
                            0x03 => RrR{src:target},
                            0x04 => SlaR{src:target},
                            0x05 => SraR{src:target},
                            0x06 => SwapR{src:target},
                            0x07 => SrlR{src:target},
                            0x08 => BitR{index:0, src:target},
                            0x09 => BitR{index:1, src:target},
                            0x0A => BitR{index:2, src:target},
                            0x0B => BitR{index:3, src:target},
                            0x0C => BitR{index:4, src:target},
                            0x0D => BitR{index:5, src:target},
                            0x0E => BitR{index:6, src:target},
                            0x0F => BitR{index:7, src:target},
                            0x10 => ResR{index:0, src:target},
                            0x11 => ResR{index:1, src:target},
                            0x12 => ResR{index:2, src:target},
                            0x13 => ResR{index:3, src:target},
                            0x14 => ResR{index:4, src:target},
                            0x15 => ResR{index:5, src:target},
                            0x16 => ResR{index:6, src:target},
                            0x17 => ResR{index:7, src:target},
                            0x18 => SetR{index:0, src:target},
                            0x19 => SetR{index:1, src:target},
                            0x1A => SetR{index:2, src:target},
                            0x1B => SetR{index:3, src:target},
                            0x1C => SetR{index:4, src:target},
                            0x1D => SetR{index:5, src:target},
                            0x1E => SetR{index:6, src:target},
                            0x1F => SetR{index:7, src:target},
                            _ => panic!("not implemented")
                        }
                    };

                // Execute the decoded operation.
                self.execute_operation(bus, data, &operation);
            }

            RlcR{src} => {
                let val = self.reg.read8(*src);

                // Clear, then set the zero flag if needed.
                self.reg.f = 
                    if val == 0 {Regs::ZERO_FLAG}
                    else {0};
                
                self.reg.write8(*src, val.rotate_left(1));

                // Check for the carry flag
                if (val & 0x80) != 0 {
                    self.reg.f |= Regs::CARRY_FLAG;
                }
            },

            RlcM => {
                let addr = self.reg.read16(Register::HL) as usize;
                let val = bus.bus_read8(addr);

                // Clear, then set the zero flag if needed.
                self.reg.f = 
                    if val == 0 {Regs::ZERO_FLAG}
                    else {0};
                
                bus.bus_write8(addr, val.rotate_left(1));

                // Check for the carry flag
                if (val & 0x80) != 0 {
                    self.reg.f |= Regs::CARRY_FLAG;
                }

                self.busy_cycles += 2;
            },

            RrcR{src} => {
                let val = self.reg.read8(*src);

                // Clear, then set the zero flag if needed.
                self.reg.f = 
                    if val == 0 {Regs::ZERO_FLAG}
                    else {0};
                
                self.reg.write8(*src, val.rotate_right(1));

                // Check for the carry flag
                if (val & 0x01  ) != 0 {
                    self.reg.f |= Regs::CARRY_FLAG;
                }
            },

            RrcM => {
                let addr = self.reg.read16(Register::HL) as usize;
                let val = bus.bus_read8(addr);

                // Clear, then set the zero flag if needed.
                self.reg.f = 
                    if val == 0 {Regs::ZERO_FLAG}
                    else {0};
                
                bus.bus_write8(addr, val.rotate_right(1));

                // Check for the carry flag
                if (val & 0x01) != 0 {
                    self.reg.f |= Regs::CARRY_FLAG;
                }

                self.busy_cycles += 2;
            },

            RlR{src} => {
                let val = self.rl(self.reg.read8(*src));
                self.reg.write8(*src, val);
            },

            RlM => {
                let addr = self.reg.read16(Register::HL) as usize;
                let val = self.rl(bus.bus_read8(addr));
                bus.bus_write8(addr, val);
                self.busy_cycles += 2;
            },

            RrR{src} => {
                let result = self.rr(self.reg.read8(*src));
                self.reg.write8(*src, result);
            },

            RrM => {
                let addr = self.reg.read16(Register::HL) as usize;
                let result = self.rr(bus.bus_read8(addr));
                bus.bus_write8(addr, result);
                self.busy_cycles += 2;
            },

            SlaR{src} => {
                let val = self.reg.read8(*src);
                let result = self.sla(val);
                self.reg.write8( *src, result);
            },

            SlaM => {
                let addr = self.reg.read16(Register::HL) as usize;
                let val = bus.bus_read8(addr);
                let result = self.sla(val);
                bus.bus_write8(addr, result);
                self.busy_cycles += 2;
            }

            SraR{src} => {
                let val = self.reg.read8(*src);
                let result = self.sra(val);
                self.reg.write8( *src, result);
            },

            SraM => {
                let addr = self.reg.read16(Register::HL) as usize;
                let val = bus.bus_read8(addr);
                let result = self.sra(val);
                bus.bus_write8(addr, result);
                self.busy_cycles += 2;
            },

            SwapR{src} => {
                let val = self.reg.read8(*src);
                self.reg.write8( *src, (val >> 4) | (val << 4));

                // Clear, then set the zero flag if needed.
                self.reg.f = 
                    if val == 0 {Regs::ZERO_FLAG}
                    else {0};
            },

            SwapM => {
                let addr = self.reg.read16(Register::HL) as usize;
                let val = bus.bus_read8(addr);
                bus.bus_write8(addr, (val >> 4) | (val << 4));

                // Clear, then set the zero flag if needed.
                self.reg.f = 
                    if val == 0 {Regs::ZERO_FLAG}
                    else {0};

                self.busy_cycles += 2;
            },

            SrlR{src} => {
                let val = self.reg.read8(*src);
                let result = self.srl(val);
                self.reg.write8( *src, result);
            },

            SrlM => {
                let addr = self.reg.read16(Register::HL) as usize;
                let val = bus.bus_read8(addr);
                let result = self.srl(val);
                bus.bus_write8(addr, result);
                self.busy_cycles += 2;
            },
            
            BitR{index, src} => {
                let val = self.reg.read8(*src);
                self.bit(val, *index);
            },

            BitM{index} => {
                let val = bus.bus_read8(self.reg.read16(Register::HL) as usize);
                self.bit(val, *index);
                self.busy_cycles += 2;
            },

            ResR{index, src} => {
                let val = self.reg.read8(*src);
                self.reg.write8(*src, val & !(1<<index));
            },

            ResM{index} => {
                let addr = self.reg.read16(Register::HL) as usize;
                let val = bus.bus_read8(addr);
                bus.bus_write8(addr, val & !(1<<index));

                self.busy_cycles += 2;
            },

            SetR{index, src} => {
                let val = self.reg.read8(*src);
                self.reg.write8(*src, val | (1<<index));
            },

            SetM{index} => {
                let addr = self.reg.read16(Register::HL) as usize;
                let val = bus.bus_read8(addr);
                bus.bus_write8(addr, val | (1<<index));
                self.busy_cycles += 2;
            },

            Daa => {
                let sub = self.reg.f & Regs::SUB_FLAG != 0;
                let mut cflag = false;

                if !sub {  
                    // after an addition, adjust if (half-)carry occurred or if result is out of bounds
                    if (self.reg.f & Regs::CARRY_FLAG != 0) || (self.reg.a > 0x99) { 
                        self.reg.a = self.reg.a.wrapping_add(0x60);
                        cflag = true;
                    }
                    if (self.reg.f & Regs::HCARRY_FLAG != 0) || (self.reg.a & 0x0f) > 0x09 {
                        self.reg.a = self.reg.a.wrapping_add(0x6);
                    }
                } else {  // after a subtraction, only adjust if (half-)carry occurred
                    if self.reg.f & Regs::CARRY_FLAG != 0 { 
                        self.reg.a = self.reg.a.wrapping_sub(0x60);
                        cflag = true;
                    }
                    if self.reg.f & Regs::HCARRY_FLAG != 0 { 
                        self.reg.a = self.reg.a.wrapping_sub(0x6);
                    }
                }

                self.reg.f &= Regs::SUB_FLAG;
                if self.reg.a == 0 {
                    self.reg.f |= Regs::ZERO_FLAG;
                }
                if cflag {
                    self.reg.f |= Regs::CARRY_FLAG;
                }
            }

            Halt => {
                self.mode = CpuMode::Halted;
            },
        }
    }

}

impl Default for Cpu {
    fn default() -> Self {
        Self::new()
    }
}

#[allow(dead_code)]
#[derive(Clone, Copy, Debug)]
enum JumpCondition {
    Always, // Always jump
    Z,      // Jump if zero flag set
    Nz,     // Jump if zero flag not set
    C,      // Jump if carry flag set
    Nc,     // Jump if carry flag not set
}

#[allow(dead_code)]
#[derive(Debug)]
enum Operation {
    // A no operation.
    Nop,
    // A stop operation.
    Stop,
    // A halt operation.
    Halt,

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
    // Store an 8bit register into memory addressed by dst.
    LdMR{dst:Register, src:Register},
    // Read into the A register from memory pointed to by the HL register, and modify HL.
    LdRMMv{add:i8},
    // Load a register with an 8 bit immediate value.
    LdRI{dst:Register},
    // Save a 16 bit register to an immediate address.
    LdI16R16{src:Register},
    // Store an 8 bit immediate into memory pointed to by a 16 bit register.
    LdMI{dst:Register},
    // Store register into zero page memory addressed by a 8 bit immediate value.
    LdZIR{src: Register},
    // Store register into zero page memory addressed by an 8 bit register.
    LdZRR{dst: Register, src:Register},
    // Load a register from zero page memory addressed by an 8 bit immediate value.
    LdRZI{dst: Register},
    // Load a register from zero page memory addressed by an 8 bit register.
    LdRZR{dst: Register, src:Register},
    // Load a register from 
    LdRMI16{dst: Register},
    // Store a register into memory addressed by a 16 bit immediate value.
    LdMI16R{src: Register},
    // Add a signed 8 bit immediate value to sp and store in HL.
    StkMvI,
    // Set the stack pointer to the value in HL.
    LdSpHl,

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
    // Add an 8 bit immediate to a 16 bit register.
    AddR16I{dst:Register},
    // Add a register to the accumulator
    AddR{src:Register},
    // Add a byte addressed by HL into the accumulator
    AddM,
    // Add an immediate byte to the accumulator
    AddI,
    // Add a register and the carry flag to the accumulator
    AddCR{src:Register},
    // Add a byte addressed by HL and the carry flag to the accumulator
    AddCM,
    // Add an immediate byte and the carry flag to the accumulator
    AddCI,
    // Subtract a register from the accumulator
    SubR{src:Register},
    // Subtract memory locaton address by HL and the carry flag from the accumulator.
    SubM,
    // Subtract an immediate byte from the accumulator
    SubI,
    // Subtract a register and the carry flag from the accumulator
    SbcR{src:Register},
    // Subtract value in memory addressed by HL and the carry flag from the accumulator.
    SbcM,
    // Subtract an imediate value and the carry flag from the accumulator
    SbcI,
    // Bitwise AND the accumulator against a register.
    AndR{src:Register},
    // Bitwise AND the accumulator against a memory value addressed by HL.
    AndM,
    // Bitwise AND the accumulator agains an immediate value.
    AndI,
    // Bitwise XOR the accumulator against a register.
    XorR{src:Register},
    // Bitwise XOR the accumulator against a memory value addressed by HL.
    XorM,
    // Bitwise XOR the accumulator against an immeidate value.
    XorI,
    // Bitwise OR the accumulator against a register.
    OrR{src:Register},
    // Bitwise XOR the accumulator against a memory value addressed by HL.
    OrM,
    // Bitwise OR the accumulator against an immediate value.
    OrI,
    // Compare a register against the accumulator
    CpR{src:Register},
    // Compare a memory location addressed by HL to the accumulator.
    CpM,
    // Compare an immediate value to the accumulator.
    CpI,

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
    // Jump
    Jp{cond: JumpCondition},
    // Jump to address in HL.
    JpHL,
    // Return
    Ret{cond: JumpCondition},
    // Return from interrupt
    Reti,
    // Call a routine
    Call{cond: JumpCondition},
    // Reset to a a handler in zero page.
    Rst{index: u8},

    // Pushes a 16 bit register onto the stack
    Push{src: Register},
    // Pops the stack into a 16 bit register.
    Pop{dst: Register},

    // Disable interrupts
    Di,
    // Enable interrupts
    Ei,

    // A placeholder operation for the 0xCB series of multibyte instructions.
    CBExt,

    RlcR{src:Register},
    RlcM,

    RrcR{src:Register},
    RrcM,

    RlR{src:Register},
    RlM,

    RrR{src:Register},
    RrM,

    SlaR{src:Register},
    SlaM,

    SraR{src:Register},
    SraM,

    SwapR{src:Register},
    SwapM,

    SrlR{src:Register},
    SrlM,

    BitR{index:u8, src:Register},
    BitM{index:u8},

    ResR{index:u8, src:Register},
    ResM{index:u8},

    SetR{index:u8, src:Register},
    SetM{index:u8},
}

#[allow(dead_code)]
struct Instruction{
    op:Operation,
    length:u8,
    cycles:u8,
}

#[allow(dead_code)]
const CB_TARGET_TABLE: [Register;8] = [
    Register::B,
    Register::C,
    Register::D,
    Register::E,
    Register::H,
    Register::L,
    Register::HL, // special case, memory addressed by HL.
    Register::A,
];

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
    // 0x70 Ld (HL), B
    Instruction{op:Operation::LdMR{dst:Register::HL, src:Register::B}, length:1, cycles:2},
    // 0x71 Ld (HL), C
    Instruction{op:Operation::LdMR{dst:Register::HL, src:Register::C}, length:1, cycles:2},
    // 0x72 Ld (HL), D
    Instruction{op:Operation::LdMR{dst:Register::HL, src:Register::D}, length:1, cycles:2},
    // 0x73 Ld (HL), E
    Instruction{op:Operation::LdMR{dst:Register::HL, src:Register::E}, length:1, cycles:2},
    // 0x74 Ld (HL), H
    Instruction{op:Operation::LdMR{dst:Register::HL, src:Register::H}, length:1, cycles:2},
    // 0x75 Ld (HL), L
    Instruction{op:Operation::LdMR{dst:Register::HL, src:Register::L}, length:1, cycles:2},
    // 0x76 Halt
    Instruction{op:Operation::Halt, length:1, cycles:1},
    // 0x77 Ld (HL), A
    Instruction{op:Operation::LdMR{dst:Register::HL, src:Register::A}, length:1, cycles:2},
    // 0x78 Ld A, B
    Instruction{op:Operation::LdRR{dst:Register::A, src:Register::B}, length:1, cycles:1},
    // 0x79 Ld A, C
    Instruction{op:Operation::LdRR{dst:Register::A, src:Register::C}, length:1, cycles:1},
    // 0x7A Ld A, D
    Instruction{op:Operation::LdRR{dst:Register::A, src:Register::D}, length:1, cycles:1},
    // 0x7B Ld A, E
    Instruction{op:Operation::LdRR{dst:Register::A, src:Register::E}, length:1, cycles:1},
    // 0x7C Ld A, H
    Instruction{op:Operation::LdRR{dst:Register::A, src:Register::H}, length:1, cycles:1},
    // 0x7D Ld A, L
    Instruction{op:Operation::LdRR{dst:Register::A, src:Register::L}, length:1, cycles:1},
    // 0x7E Ld A, (HL)
    Instruction{op:Operation::LdRM{dst:Register::A, src:Register::HL}, length:1, cycles:2},
    // 0x7F Ld A, A
    Instruction{op:Operation::LdRR{dst:Register::A, src:Register::A}, length:1, cycles:1},

    // 0x8X
    Instruction{op:Operation::AddR{src:Register::B}, length:1, cycles:1},
    Instruction{op:Operation::AddR{src:Register::C}, length:1, cycles:1},
    Instruction{op:Operation::AddR{src:Register::D}, length:1, cycles:1},
    Instruction{op:Operation::AddR{src:Register::E}, length:1, cycles:1},
    Instruction{op:Operation::AddR{src:Register::H}, length:1, cycles:1},
    Instruction{op:Operation::AddR{src:Register::L}, length:1, cycles:1},
    Instruction{op:Operation::AddM, length:1, cycles:2},
    Instruction{op:Operation::AddR{src:Register::A}, length:1, cycles:1},
    Instruction{op:Operation::AddCR{src:Register::B}, length:1, cycles:1},
    Instruction{op:Operation::AddCR{src:Register::C}, length:1, cycles:1},
    Instruction{op:Operation::AddCR{src:Register::D}, length:1, cycles:1},
    Instruction{op:Operation::AddCR{src:Register::E}, length:1, cycles:1},
    Instruction{op:Operation::AddCR{src:Register::H}, length:1, cycles:1},
    Instruction{op:Operation::AddCR{src:Register::L}, length:1, cycles:1},
    Instruction{op:Operation::AddCM, length:1, cycles:2},
    Instruction{op:Operation::AddCR{src:Register::A}, length:1, cycles:1},

    // 0x9X
    Instruction{op:Operation::SubR{src:Register::B}, length:1, cycles:1},
    Instruction{op:Operation::SubR{src:Register::C}, length:1, cycles:1},
    Instruction{op:Operation::SubR{src:Register::D}, length:1, cycles:1},
    Instruction{op:Operation::SubR{src:Register::E}, length:1, cycles:1},
    Instruction{op:Operation::SubR{src:Register::H}, length:1, cycles:1},
    Instruction{op:Operation::SubR{src:Register::L}, length:1, cycles:1},
    Instruction{op:Operation::SubM,                  length:1, cycles:2},
    Instruction{op:Operation::SubR{src:Register::A}, length:1, cycles:1},
    Instruction{op:Operation::SbcR{src:Register::B}, length:1, cycles:1},
    Instruction{op:Operation::SbcR{src:Register::C}, length:1, cycles:1},
    Instruction{op:Operation::SbcR{src:Register::D}, length:1, cycles:1},
    Instruction{op:Operation::SbcR{src:Register::E}, length:1, cycles:1},
    Instruction{op:Operation::SbcR{src:Register::H}, length:1, cycles:1},
    Instruction{op:Operation::SbcR{src:Register::L}, length:1, cycles:1},
    Instruction{op:Operation::SbcM,                  length:1, cycles:2},
    Instruction{op:Operation::SbcR{src:Register::A}, length:1, cycles:1},

    // 0xAX
    Instruction{op:Operation::AndR{src:Register::B}, length:1, cycles:1},
    Instruction{op:Operation::AndR{src:Register::C}, length:1, cycles:1},
    Instruction{op:Operation::AndR{src:Register::D}, length:1, cycles:1},
    Instruction{op:Operation::AndR{src:Register::E}, length:1, cycles:1},
    Instruction{op:Operation::AndR{src:Register::H}, length:1, cycles:1},
    Instruction{op:Operation::AndR{src:Register::L}, length:1, cycles:1},
    Instruction{op:Operation::AndM,                  length:1, cycles:2},
    Instruction{op:Operation::AndR{src:Register::A}, length:1, cycles:1},
    Instruction{op:Operation::XorR{src:Register::B}, length:1, cycles:1},
    Instruction{op:Operation::XorR{src:Register::C}, length:1, cycles:1},
    Instruction{op:Operation::XorR{src:Register::D}, length:1, cycles:1},
    Instruction{op:Operation::XorR{src:Register::E}, length:1, cycles:1},
    Instruction{op:Operation::XorR{src:Register::H}, length:1, cycles:1},
    Instruction{op:Operation::XorR{src:Register::L}, length:1, cycles:1},
    Instruction{op:Operation::XorM,                  length:1, cycles:2},
    Instruction{op:Operation::XorR{src:Register::A}, length:1, cycles:1},

    // 0xBX
    Instruction{op:Operation::OrR{src:Register::B}, length:1, cycles:1},
    Instruction{op:Operation::OrR{src:Register::C}, length:1, cycles:1},
    Instruction{op:Operation::OrR{src:Register::D}, length:1, cycles:1},
    Instruction{op:Operation::OrR{src:Register::E}, length:1, cycles:1},
    Instruction{op:Operation::OrR{src:Register::H}, length:1, cycles:1},
    Instruction{op:Operation::OrR{src:Register::L}, length:1, cycles:1},
    Instruction{op:Operation::OrM,                  length:1, cycles:2},
    Instruction{op:Operation::OrR{src:Register::A}, length:1, cycles:1},

    Instruction{op:Operation::CpR{src:Register::B}, length:1, cycles:1},
    Instruction{op:Operation::CpR{src:Register::C}, length:1, cycles:1},
    Instruction{op:Operation::CpR{src:Register::D}, length:1, cycles:1},
    Instruction{op:Operation::CpR{src:Register::E}, length:1, cycles:1},
    Instruction{op:Operation::CpR{src:Register::H}, length:1, cycles:1},
    Instruction{op:Operation::CpR{src:Register::L}, length:1, cycles:1},
    Instruction{op:Operation::CpM                 , length:1, cycles:2},
    Instruction{op:Operation::CpR{src:Register::A}, length:1, cycles:1},

    // 0xCX
    Instruction{op:Operation::Ret{cond:JumpCondition::Nz},      length:1, cycles:2},
    Instruction{op:Operation::Pop{dst:Register::BC},            length:1, cycles:3},
    Instruction{op:Operation::Jp{cond:JumpCondition::Nz},       length:3, cycles:3},
    Instruction{op:Operation::Jp{cond:JumpCondition::Always},   length:3, cycles:3},
    Instruction{op:Operation::Call{cond:JumpCondition::Nz},     length:3, cycles:3},
    Instruction{op:Operation::Push{src:Register::BC},           length:1, cycles:4},
    Instruction{op:Operation::AddI,                             length:2, cycles:2},
    Instruction{op:Operation::Rst{index:0},                     length:1, cycles:4},
    Instruction{op:Operation::Ret{cond:JumpCondition::Z},       length:1, cycles:2},
    Instruction{op:Operation::Ret{cond:JumpCondition::Always},  length:1, cycles:2},
    Instruction{op:Operation::Jp{cond:JumpCondition::Z},        length:3, cycles:3},
    Instruction{op:Operation::CBExt,                            length:2, cycles:2},
    Instruction{op:Operation::Call{cond:JumpCondition::Z},      length:3, cycles:3},
    Instruction{op:Operation::Call{cond:JumpCondition::Always}, length:3, cycles:3},
    Instruction{op:Operation::AddCI,                            length:2, cycles:2},
    Instruction{op:Operation::Rst{index:1},                     length:1, cycles:4},

    // 0xDX
    Instruction{op:Operation::Ret{cond:JumpCondition::Nc},  length:1, cycles:2},
    Instruction{op:Operation::Pop{dst:Register::DE},        length:1, cycles:3},
    Instruction{op:Operation::Jp{cond:JumpCondition::Nc},   length:3, cycles:3},
    Instruction{op:Operation::Nop, length:1, cycles:1}, // Crash?
    Instruction{op:Operation::Call{cond:JumpCondition::Nc}, length:3, cycles:3},
    Instruction{op:Operation::Push{src:Register::DE},       length:1, cycles:4},
    Instruction{op:Operation::SubI,                         length:2, cycles:2},
    Instruction{op:Operation::Rst{index:2},                 length:1, cycles:4},
    Instruction{op:Operation::Ret{cond:JumpCondition::C},   length:1, cycles:2},
    Instruction{op:Operation::Reti,                         length:1, cycles:4},
    Instruction{op:Operation::Jp{cond:JumpCondition::C},    length:3, cycles:3},
    Instruction{op:Operation::Nop, length:1, cycles:1}, // Crash?
    Instruction{op:Operation::Call{cond:JumpCondition::C},  length:3, cycles:3}, 
    Instruction{op:Operation::Nop, length:1, cycles:1}, // Crash?
    Instruction{op:Operation::SbcI,                         length:2, cycles:2},
    Instruction{op:Operation::Rst{index:3},                 length:1, cycles:4},

    // 0xEX
    Instruction{op:Operation::LdZIR{src:Register::A}, length:2, cycles:3},
    Instruction{op:Operation::Pop{dst:Register::HL}, length:1, cycles:3},
    Instruction{op:Operation::LdZRR{dst:Register::C, src:Register::A}, length:1, cycles:2},
    Instruction{op:Operation::Nop, length:1, cycles:1}, // Crash?
    Instruction{op:Operation::Nop, length:1, cycles:1}, // Crash?
    Instruction{op:Operation::Push{src:Register::HL}, length:1, cycles:4},
    Instruction{op:Operation::AndI, length:2, cycles:2},
    Instruction{op:Operation::Rst{index:4}, length:1, cycles:4},
    Instruction{op:Operation::AddR16I{dst:Register::SP}, length:2, cycles:4},
    Instruction{op:Operation::JpHL, length:1, cycles:1},
    Instruction{op:Operation::LdMI16R{src:Register::A}, length:3, cycles:4},
    Instruction{op:Operation::Nop, length:1, cycles:1}, // Crash?
    Instruction{op:Operation::Nop, length:1, cycles:1}, // Crash?
    Instruction{op:Operation::Nop, length:1, cycles:1}, // Crash?
    Instruction{op:Operation::XorI, length:2, cycles:2},
    Instruction{op:Operation::Rst{index:5}, length:1, cycles:4},

    // 0xFX
    Instruction{op:Operation::LdRZI{dst:Register::A}, length:2, cycles:3},
    Instruction{op:Operation::Pop{dst:Register::AF}, length:1, cycles:3},
    Instruction{op:Operation::LdRZR{dst:Register::A, src:Register::C}, length:1, cycles:2},
    Instruction{op:Operation::Di, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1}, // Crash?
    Instruction{op:Operation::Push{src:Register::AF}, length:1, cycles:4},
    Instruction{op:Operation::OrI, length:2, cycles:2},
    Instruction{op:Operation::Rst{index:6}, length:1, cycles:4},
    Instruction{op:Operation::StkMvI, length:2, cycles:3},
    Instruction{op:Operation::LdSpHl, length:1, cycles:2},
    Instruction{op:Operation::LdRMI16{dst:Register::A}, length:3, cycles:4},
    Instruction{op:Operation::Ei, length:1, cycles:1},
    Instruction{op:Operation::Nop, length:1, cycles:1}, // Crash?
    Instruction{op:Operation::Nop, length:1, cycles:1}, // Crash?
    Instruction{op:Operation::CpI, length:2, cycles:2},
    Instruction{op:Operation::Rst{index:7}, length:1, cycles:4},
];

#[cfg(test)]
#[allow(non_snake_case)]
mod test {
    use super::*;
    use crate::bus::{BusRW};
    use crate::ram::Ram;

    struct TestPack {
        cpu: Cpu,
        ram: Ram,
    }

    #[derive(Clone, Copy)]
    enum AddressingMode {
        Register{src:Register},
        RegisterMem{src:Register, addr:usize},
    }

    impl TestPack {
        /// Constructs a new test package with a fresh cpu and ram.
        fn new() -> TestPack {
            TestPack{
                cpu: Cpu::new(),
                ram: Ram::new(0x10000, 0),
            }
        }

        /// Executes an instruction on the CPU
        fn execute_instruction(&mut self) {
            self.cpu.execute_instruction(&mut self.ram);
        }

        /// Checks the number of busy cycles on the cpu.
        fn check_cycles(&self, expected: u8) {
            assert_eq!(expected, self.cpu.busy_cycles as u8);
        }

        /// Checks the current program counter value.
        fn check_length(&self, expected: u16) {
            assert_eq!(expected, self.cpu.reg.pc);
        }

        /// Checks the current flags register
        fn check_flags(&self, expected:u8) {
            assert_eq!(expected, self.cpu.reg.f);
        }
        
        /// Loads instruction data into address 0 of "ram".
        fn load_instruction(&mut self, inst: &[u8]) {
            let mut addr = 0;
            for val in inst {
                self.ram.bus_write8(addr, *val);
                addr += 1;
            }
        }

        /// Resets the cpu to default values.
        fn reset_cpu(&mut self) {
            self.cpu = Cpu::new();
        }

        /// Sets a value specified by the addressing mode.
        fn set_value(&mut self, mode:AddressingMode, value:u8) {
            match mode {
                AddressingMode::Register{src} => {
                    self.cpu.reg.write8(src, value);
                },
                AddressingMode::RegisterMem{src, addr} => {
                    self.cpu.reg.write16(src, addr as u16);
                    self.ram.bus_write8(
                        self.cpu.reg.read16(src) as usize,
                        value);
                },
            }
        }

        /// Gets a value specified by the addressing mode.
        fn get_value(&mut self, mode:AddressingMode) -> u8{
            match mode {
                AddressingMode::Register{src} => {
                    self.cpu.reg.read8(src)
                },
                AddressingMode::RegisterMem{src, addr} => {
                    self.cpu.reg.write16(src, addr as u16);
                    self.ram.bus_read8(
                        self.cpu.reg.read16(src) as usize)
                }
            }
        }
    }

    fn get_ram() -> Ram {
        Ram::new(0x10000, 0)
    }

    fn load_into_ram(ram:&mut Ram, inst: &[u8])
    {
        let mut addr = 0;
        for val in inst {
            ram.bus_write8(addr, *val);
            addr += 1;
        }
    }

    fn get_memory_HL_setter(address: usize) ->  impl Fn(&mut Cpu, &mut Ram, u8)
    {
        move |cpu:&mut Cpu, ram:&mut Ram, value:u8| {
            cpu.reg.write16(Register::HL, address as u16);
            ram.bus_write8(address, value);
        }
    }

    fn get_memory_HL_getter(address:usize) -> impl Fn(&mut Cpu, &mut Ram) -> u8
    {
        move |cpu:&mut Cpu, ram: &mut Ram| {
            cpu.reg.write16(Register::HL, address as u16);
            ram.bus_read8(address)
        }
    }
    
    fn get_register8_setter(target:Register) -> impl Fn(&mut Cpu, &mut Ram, u8)
    {
        move |cpu:&mut Cpu, _:&mut Ram, value:u8| {
            cpu.reg.write8(target, value);
        }
    }

    fn get_register8_getter(target:Register) -> impl Fn(&mut Cpu, &mut Ram) -> u8
    {
        move |cpu:&mut Cpu, _:&mut Ram| {
            cpu.reg.read8(target)
        }
    }

    fn get_immediate_u8_setter(pc_offset:u8) -> impl Fn(&mut Cpu, &mut Ram, u8)
    {
        move |cpu:&mut Cpu, ram:&mut Ram, value:u8| {
            let target =cpu.reg.pc + pc_offset as u16;
            ram.bus_write8(target as usize, value);
        }
    }

    const CB_ADDRESSING_MODES: [AddressingMode;8] = [ 
        AddressingMode::Register{src:Register::B},
        AddressingMode::Register{src:Register::C},
        AddressingMode::Register{src:Register::D},
        AddressingMode::Register{src:Register::E},
        AddressingMode::Register{src:Register::H},
        AddressingMode::Register{src:Register::L},
        AddressingMode::RegisterMem{src:Register::HL, addr:0x4324},
        AddressingMode::Register{src:Register::A},
    ];

    // Test helpers for the cpu.
    impl Cpu{
        fn set_jump_condition(&mut self, cond:JumpCondition)
        {
            match cond {
                JumpCondition::Always => {},
                JumpCondition::Z  => {self.reg.f |= Regs::ZERO_FLAG},
                JumpCondition::Nz => {self.reg.f &= !Regs::ZERO_FLAG},
                JumpCondition::C  => {self.reg.f |= Regs::CARRY_FLAG},
                JumpCondition::Nc  => {self.reg.f &= !Regs::CARRY_FLAG},
            };
        }

        fn clear_jump_condition(&mut self, cond:JumpCondition) -> bool
        {
            match cond {
                JumpCondition::Always => false,
                JumpCondition::Nz  => {self.reg.f |= Regs::ZERO_FLAG;  true},
                JumpCondition::Z => {self.reg.f &= !Regs::ZERO_FLAG;   true},
                JumpCondition::Nc  => {self.reg.f |= Regs::CARRY_FLAG; true},
                JumpCondition::C  => {self.reg.f &= !Regs::CARRY_FLAG; true},
            }
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
    
    fn test_op_ldMR(inst: &[u8], dst:Register, src:Register)
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();

        load_into_ram(&mut ram, &inst);

        let address = 0x343;
        let value = match src {
            Register::H => 0x3,
            Register::L => 0x43,
            _ => 0xA5
        };
        cpu.reg.write8(src, value);
        cpu.reg.write16(dst, address);

        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 2);
        assert_eq!(cpu.reg.pc, 1);
        assert_eq!(ram.bus_read8(address as usize), value);
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

    fn test_op_add<T>(inst: &[u8], set_src:T, cycle_count:u8)
        where T: Fn(&mut Cpu, &mut Ram, u8)
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        load_into_ram(&mut ram, &inst);
        cpu.reg.a = 1;
        set_src(&mut cpu, &mut ram, 0);
        let self_reference = cpu.reg.a != 1;

        // Set all flags, to verify they change
        cpu.reg.f = 0xFF;
        cpu.reg.a = 0x00;
        set_src(&mut cpu, &mut ram, 0);

        let cycles = cpu.execute_instruction(&mut ram);
        assert_eq!(cycles, cycle_count);
        assert_eq!(cpu.reg.pc, inst.len() as u16);
        assert_eq!(cpu.reg.a, 0);
        assert_eq!(cpu.reg.f, Regs::ZERO_FLAG);

        cpu = Cpu::new();
        cpu.reg.a = 0xFF;


        // Deal with case where dst and src are the same.
        let expected = if self_reference == false {
            set_src(&mut cpu, &mut ram, 0xF);
            0x0E
        } else { 0xFE };

        cpu.execute_instruction(&mut ram);
        assert_eq!(cpu.reg.a, expected);
        assert_eq!(cpu.reg.f, Regs::CARRY_FLAG | Regs::HCARRY_FLAG);
        
        if self_reference == false
        {
            cpu = Cpu::new();
            cpu.reg.a = 0x02;
            set_src(&mut cpu, &mut ram, 0x2E);
            cpu.execute_instruction(&mut ram);
            assert_eq!(cpu.reg.a, 0x30);
            assert_eq!(cpu.reg.f, Regs::HCARRY_FLAG);
        }
    }

    fn test_op_addr(inst: &[u8], src:Register)
    {
        test_op_add(&inst, get_register8_setter(src), 1);
    }

    fn test_op_addi(inst: &[u8])
    {
        test_op_add(&inst, get_immediate_u8_setter(1), 2);
    }

    fn test_op_addc<T>(inst: &[u8], set_src:T, cycle_count:u8)
        where T: Fn(&mut Cpu, &mut Ram, u8)
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        load_into_ram(&mut ram, &inst);
        set_src(&mut cpu, &mut ram, 1);
        let self_reference = cpu.reg.a != 0;

        // set the carry flag
        cpu.reg.a = 0;
        set_src(&mut cpu, &mut ram, 0xFF);
        cpu.reg.f = Regs::CARRY_FLAG;

        let mut expected = 23;
        if !self_reference{
            cpu.reg.a = 23; // Just a nonzero value.
        } else {
            expected = 0xFF;
        }
        let cycles = cpu.execute_instruction(&mut ram);
        assert_eq!(cpu.reg.pc, inst.len() as u16);
        assert_eq!(cycles, cycle_count);
        assert_eq!(cpu.reg.a, expected);
    }

    fn test_op_addcr(inst: &[u8], src:Register)
    {
        test_op_addc(&inst, get_register8_setter(src), 1);
    }

    fn test_op_addci(inst: &[u8])
    {
        test_op_addc(&inst, get_immediate_u8_setter(1), 2);
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

    fn test_op_sub<T>(inst: &[u8], set_src:T, cycle_count:u8)
        where T: Fn(&mut Cpu, &mut Ram, u8)
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        load_into_ram(&mut ram, &inst);
        
        set_src(&mut cpu, &mut ram, 2);
        let self_reference = cpu.reg.a == 2;
        cpu.reg.a = if !self_reference {0} else {2};
        let result = if self_reference {0} else {0xFE};
        let expected_flags = if self_reference {
            Regs::ZERO_FLAG |Regs::SUB_FLAG
        } else {
            Regs::CARRY_FLAG | Regs::HCARRY_FLAG | Regs::SUB_FLAG
        };
        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, cycle_count);
        assert_eq!(cpu.reg.pc, inst.len() as u16);
        assert_eq!(cpu.reg.a, result);
        assert_eq!(cpu.reg.f, expected_flags);
    }

    fn test_op_subr(inst: &[u8], src:Register)
    {
        test_op_sub(&inst, get_register8_setter(src), 1);
    }

    fn test_op_subi(inst: &[u8])
    {
        test_op_sub(&inst, get_immediate_u8_setter(1), 2);
    }

    fn test_op_sbc<T>(inst:&[u8], set_src:T, cycles:u8)
        where T: Fn(&mut Cpu, &mut Ram, u8)
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        load_into_ram(&mut ram, inst);

        let init_a = 2;
        cpu.reg.f = Regs::CARRY_FLAG;
        cpu.reg.a = init_a;

        set_src(&mut cpu, &mut ram, 1);
        let self_referenced = cpu.reg.a != init_a;


        let executed_cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(executed_cycles, cycles);
        assert_eq!(cpu.reg.pc, inst.len() as u16);

        if self_referenced {
            // Self referenced case is 
            assert_eq!(cpu.reg.a, 255);
            assert_eq!(cpu.reg.f, Regs::SUB_FLAG | Regs::CARRY_FLAG | Regs::HCARRY_FLAG);
        } else {
            assert_eq!(cpu.reg.a, 0);
            assert_eq!(cpu.reg.f, Regs::SUB_FLAG | Regs::ZERO_FLAG);
        }


        if self_referenced == false {
            // set carry flag
            cpu = Cpu::new();
            cpu.reg.a = 2;
            cpu.reg.f = Regs::CARRY_FLAG;
            set_src(&mut cpu, &mut ram, 2);

            cpu.execute_instruction(&mut ram);

            assert_eq!(cpu.reg.a, 255);
            assert_eq!(cpu.reg.f, Regs::SUB_FLAG | Regs::CARRY_FLAG | Regs::HCARRY_FLAG);
        }

        // Carry flag not set, half carry.
        if self_referenced == false {
            cpu = Cpu::new();
            cpu.reg.a = 0x11;
            set_src(&mut cpu, &mut ram, 3);

            cpu.execute_instruction(&mut ram);

            assert_eq!(cpu.reg.a, 0x0E);
            assert_eq!(cpu.reg.f, Regs::SUB_FLAG | Regs::HCARRY_FLAG);
        }

        // Carry flag not set, no carry
        cpu = Cpu::new();
        cpu.reg.a = 0x13;
        let expected_a = if self_referenced {0} else {0x10};
        let expected_flags = if self_referenced {
            Regs::SUB_FLAG | Regs::ZERO_FLAG
        } else {
            Regs::SUB_FLAG
        };

        set_src(&mut cpu, &mut ram, 3);

        cpu.execute_instruction(&mut ram);

        assert_eq!(cpu.reg.a, expected_a);
        assert_eq!(cpu.reg.f, expected_flags);
    }

    fn test_op_sbcr(inst: &[u8], src:Register)
    {
        test_op_sbc(&inst, get_register8_setter(src), 1);
    }

    fn test_op_sbcm(inst: &[u8])
    {
        test_op_sbc(&inst, get_memory_HL_setter(0x3245), 2);
    }

    fn test_op_and<T>(inst: &[u8], set_src:T, cycleCount:u8)
        where T: Fn(&mut Cpu, &mut Ram, u8)
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        load_into_ram(&mut ram, inst);

        // Evaluates to zero case (except self reference)
        cpu.reg.a = 0xAA;
        set_src(&mut cpu, &mut ram, 0x55);
        let self_referenced = cpu.reg.a != 0xAA;
        let expected_a = if self_referenced {cpu.reg.a} else {0};
        let expected_flags = if self_referenced {Regs::HCARRY_FLAG} 
                             else {Regs::HCARRY_FLAG | Regs::ZERO_FLAG};

        let cycles = cpu.execute_instruction(&mut ram);
        assert_eq!(cycles, cycleCount);
        assert_eq!(cpu.reg.pc, inst.len() as u16);
        assert_eq!(cpu.reg.a, expected_a);
        assert_eq!(cpu.reg.f, expected_flags);

        // Non zero case.
        cpu = Cpu::new();
        cpu.reg.a = 0xFF;
        set_src(&mut cpu, &mut ram, 0x0F);
        let expected_a = if self_referenced {cpu.reg.a} else {0x0F};
        let expected_flags = Regs::HCARRY_FLAG;

        cpu.execute_instruction(&mut ram);
        assert_eq!(cpu.reg.a, expected_a);
        assert_eq!(cpu.reg.f, expected_flags);
    }

    fn test_op_andr(inst: &[u8], src:Register)
    {
        test_op_and(&inst, get_register8_setter(src), 1);
    }

    fn test_op_andm(inst: &[u8])
    {
        test_op_and(&inst, get_memory_HL_setter(0x5431), 2);
    }

    fn test_op_xor<T>(inst: &[u8], set_src:T, cycleCount:u8)
        where T: Fn(&mut Cpu, &mut Ram, u8)
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        load_into_ram(&mut ram, inst);

        // Evaluates to zero case (except self reference)
        cpu.reg.a = 0xAA;
        set_src(&mut cpu, &mut ram, 0x55);
        let self_referenced = cpu.reg.a != 0xAA;
        let expected_a = if self_referenced {0} else {0xFF};
        let expected_flags = if self_referenced {Regs::ZERO_FLAG} 
                             else {0};

        let cycles = cpu.execute_instruction(&mut ram);
        assert_eq!(cycles, cycleCount);
        assert_eq!(cpu.reg.pc, inst.len() as u16);
        assert_eq!(cpu.reg.a, expected_a);
        assert_eq!(cpu.reg.f, expected_flags);

        // Non zero case.
        cpu = Cpu::new();
        cpu.reg.a = 0xFF;
        set_src(&mut cpu, &mut ram, 0x0F);
        let expected_a = if self_referenced {0} else {0xF0};
        let expected_flags = if self_referenced {Regs::ZERO_FLAG} else {0};

        cpu.execute_instruction(&mut ram);
        assert_eq!(cpu.reg.a, expected_a);
        assert_eq!(cpu.reg.f, expected_flags);
    }

    fn test_op_xorr(inst: &[u8], src:Register)
    {
        test_op_xor(&inst, get_register8_setter(src), 1);
    }

    fn test_op_xorm(inst: &[u8])
    {
        test_op_xor(&inst, get_memory_HL_setter(0xFF), 2);
    }

    fn test_op_or<T>(inst: &[u8], set_src:T, cycleCount:u8)
        where T: Fn(&mut Cpu, &mut Ram, u8)
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        load_into_ram(&mut ram, inst);

        // Evaluates to zero case (except self reference)
        cpu.reg.a = 0xAA;
        set_src(&mut cpu, &mut ram, 0x55);
        let self_referenced = cpu.reg.a != 0xAA;
        let expected_a = if self_referenced {0x55} else {0xFF};
        let expected_flags = 0;

        let cycles = cpu.execute_instruction(&mut ram);
        assert_eq!(cycles, cycleCount);
        assert_eq!(cpu.reg.pc, inst.len() as u16);
        assert_eq!(cpu.reg.a, expected_a);
        assert_eq!(cpu.reg.f, expected_flags);

        // Non zero case.
        cpu = Cpu::new();
        cpu.reg.a = 0xF0;
        set_src(&mut cpu, &mut ram, 0x01);
        let expected_a = if self_referenced {0x01} else {0xF1};
        let expected_flags = 0;

        cpu.execute_instruction(&mut ram);
        assert_eq!(cpu.reg.a, expected_a);
        assert_eq!(cpu.reg.f, expected_flags);
    }

    fn test_op_orr(inst: &[u8], src:Register)
    {
        test_op_or(&inst, get_register8_setter(src), 1);
    }

    fn test_op_orm(inst: &[u8])
    {
        test_op_or(&inst, get_memory_HL_setter(0xFF), 2);
    }

    fn test_op_cp<T>(inst: &[u8], set_src:T, cycle_count:u8)
        where T: Fn(&mut Cpu, &mut Ram, u8)
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        load_into_ram(&mut ram, &inst);
        
        // euqal case
        let equal_value = 9;
        set_src(&mut cpu, &mut ram, equal_value);
        let self_reference = cpu.reg.a != 0;
        cpu.reg.a = equal_value;
        let expected_flags = Regs::ZERO_FLAG | Regs::SUB_FLAG;

        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, cycle_count);
        assert_eq!(cpu.reg.pc, inst.len() as u16);
        assert_eq!(cpu.reg.a, equal_value);
        assert_eq!(cpu.reg.f, expected_flags);

        // not equal case ( not possible to test with self reference case)
        if self_reference == false{
            cpu = Cpu::new();
            cpu.reg.a = 9;
            set_src(&mut cpu, &mut ram, 10);
            let expected_flags = Regs::CARRY_FLAG | Regs::HCARRY_FLAG | Regs::SUB_FLAG;

            cpu.execute_instruction(&mut ram);

            assert_eq!(cpu.reg.a, 9);
            assert_eq!(cpu.reg.f, expected_flags);
        }
    }

    fn test_op_cpr(inst: &[u8], src:Register)
    {
        test_op_cp(&inst, get_register8_setter(src), 1);
    }

    fn test_op_cpm(inst: &[u8])
    {
        test_op_cp(&inst, get_memory_HL_setter(234), 2);
    }

    fn test_op_ret(inst: &[u8], cond:JumpCondition)
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        load_into_ram(&mut ram, &inst);

        // set the jump condition, and setup the stack and jump address
        cpu.set_jump_condition(cond);
        let stack_ptr = 0x6;
        let jump_addr = 0xCAFE;
        cpu.reg.write16(Register::SP, stack_ptr);
        ram.bus_write16(stack_ptr as usize, jump_addr);

        // Execute the code.
        let cycles = cpu.execute_instruction(&mut ram);

        // Should take 5 cycles and jump to the address on the stack.
        assert_eq!(cycles, 5);
        assert_eq!(cpu.reg.pc, jump_addr);
        assert_eq!(cpu.reg.sp, stack_ptr + 2);

        // If the jump condition can be cleared, test the no branch case.
        cpu = Cpu::new();
        if cpu.clear_jump_condition(cond) {
            // Set the 
            cpu.reg.write16(Register::SP, stack_ptr);
            ram.bus_write16(stack_ptr as usize, jump_addr);

            let cycles = cpu.execute_instruction(&mut ram);

            assert_eq!(cycles, 2);
            assert_eq!(cpu.reg.pc, 1);
            assert_eq!(cpu.reg.sp, stack_ptr);
        }
    }

    fn test_op_pop(inst: &[u8], dst:Register)
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        load_into_ram(&mut ram, inst);

        // place a value into memory at the stack pointer address
        let address = 0x1234;
        let value = 0xBEEF;
        cpu.reg.sp = address;
        ram.bus_write16(address as usize, value);

        let cycles = cpu.execute_instruction(&mut ram);

        // 3 cycles, 1 byte long, stack increases by 2, dst contains value from ram.
        assert_eq!(cycles, 3);
        assert_eq!(cpu.reg.pc, 1);
        assert_eq!(cpu.reg.sp, address + 2);
        if dst != Register::AF{
            assert_eq!(cpu.reg.read16(dst), value);
        }
        else {
            assert_eq!(cpu.reg.read16(dst), value & 0xFFF0);
        }
    }
    
    fn test_op_jp(inst: &[u8], cond:JumpCondition)
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        load_into_ram(&mut ram, &inst);
        assert_eq!(inst.len(), 3);
        
        // set the jump condition, and setup the stack and jump address
        cpu.set_jump_condition(cond);
        let jump_addr = LittleEndian::read_u16(&inst[1..]);

        // Execute the code.
        let cycles = cpu.execute_instruction(&mut ram);

        // Should take 4 cycles and jump to the immediate address
        assert_eq!(cycles, 4);
        assert_eq!(cpu.reg.pc, jump_addr);

        // If the jump condition can be cleared, test the no branch case.
        cpu = Cpu::new();
        if cpu.clear_jump_condition(cond) {
            let cycles = cpu.execute_instruction(&mut ram);

            // Only 3 cycles if no branch is taken.
            assert_eq!(cycles, 3);
            assert_eq!(cpu.reg.pc, 3);
        }
    }

    fn test_op_call(inst: &[u8], cond:JumpCondition)
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        load_into_ram(&mut ram, &inst);
        assert_eq!(inst.len(), 3);
        let jump_addr = LittleEndian::read_u16(&inst[1..]);
        
        // set the jump condition, and setup the stack and jump address
        cpu.set_jump_condition(cond);
        let initial_sp = 5;
        cpu.reg.sp = initial_sp;

        // Execute the code.
        let cycles = cpu.execute_instruction(&mut ram);

        // Must take 6 cycles and jump to the immediate address.
        // The stack pointer must decrease by 2.
        // The program counter value must be on the stack.
        assert_eq!(cycles, 6);
        assert_eq!(cpu.reg.pc, jump_addr);
        assert_eq!(cpu.reg.sp, initial_sp - 2);
        assert_eq!(ram.bus_read16(cpu.reg.sp as usize), 3);

        // If the jump condition can be cleared, test the no branch case.
        cpu = Cpu::new();
        cpu.reg.sp = initial_sp;
        if cpu.clear_jump_condition(cond) {
            let cycles = cpu.execute_instruction(&mut ram);

            // Only 3 cycles if no branch is taken, no jump,
            // and the stack pointer remains unchanged.
            assert_eq!(cycles, 3);
            assert_eq!(cpu.reg.pc, 3);
            assert_eq!(cpu.reg.sp, initial_sp);
        }
    }

    fn test_op_push(inst: &[u8], src:Register)
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        load_into_ram(&mut ram, inst);

        // place a value into register and init stack pointer
        let address = 0x1234;
        let value = 0xBEEF;
        cpu.reg.sp = address;
        cpu.reg.write16(src, value);

        let cycles = cpu.execute_instruction(&mut ram);

        // 4 cycles, 1 byte long, stack decreases by 2, ram contains register value.
        assert_eq!(cycles, 4);
        assert_eq!(cpu.reg.pc, 1);
        assert_eq!(cpu.reg.sp, address - 2);
        if src != Register::AF{
            assert_eq!(ram.bus_read16(cpu.reg.sp as usize), value);
        } else {
            assert_eq!(ram.bus_read16(cpu.reg.sp as usize), value & 0xFFF0);
        }
    }

    fn test_op_rst(inst: &[u8], index: u8)
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        load_into_ram(&mut ram, inst);

        // Set the stack pointer
        let initial_stack = 10;
        let expected_pc:u16 = index as u16 * 8;
        cpu.reg.sp = initial_stack;

        cpu.execute_instruction(&mut ram);

        // Stack pointer push back 2 bytes.
        // Stack must contain program counter after instruction
        // PC must to to 8 byte indexed location in zero page.
        assert_eq!(cpu.reg.sp, initial_stack - 2);
        assert_eq!(ram.bus_read16(cpu.reg.sp as usize), 1);
        assert_eq!(cpu.reg.pc, expected_pc);
    }

    fn test_op_rlc<S, G>(inst: &[u8], set: S, get: G, cycles: u8)
        where S: Fn(&mut Cpu, &mut Ram, u8),
              G: Fn(&mut Cpu, &mut Ram)->u8
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        load_into_ram(&mut ram, inst);

        /////// Test No Carry Flag Case ////////

        // load a base value into the test value source.
        let val = 0x18u8;
        set(&mut cpu, &mut ram, val);

        // Execute the code under test.
        let cycles_count = cpu.execute_instruction(&mut ram);
        let result_val = get(&mut cpu, &mut ram);

        assert_eq!(cycles_count, cycles);
        assert_eq!(cpu.reg.pc, 2);
        assert_eq!(result_val, 0x30);
        assert_eq!(cpu.reg.f, 0);

        //////// Test Carry Flag Case ////////

        // Reset the CPU
        cpu = Cpu::new();

        let val = 0x81;
        set(&mut cpu, &mut ram, val);

        cpu.execute_instruction(&mut ram);

        let result_val = get(&mut cpu, &mut ram);

        assert_eq!(result_val, 0x03);
        assert_eq!(cpu.reg.f, Regs::CARRY_FLAG);

        //////// Test Zero Flag Case ////////
        cpu = Cpu::new();
        set(&mut cpu, &mut ram, 0);

        cpu.execute_instruction(&mut ram);

        assert_eq!(get(&mut cpu, &mut ram), 0);
        assert_eq!(cpu.reg.f, Regs::ZERO_FLAG);
    }

    fn test_op_rrc<S, G>(inst: &[u8], set: S, get: G, cycles: u8)
        where S: Fn(&mut Cpu, &mut Ram, u8),
              G: Fn(&mut Cpu, &mut Ram)->u8
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        load_into_ram(&mut ram, inst);

        /////// Test No Carry Flag Case ////////

        // load a base value into the test value source.
        let val = 0x18u8;
        set(&mut cpu, &mut ram, val);

        // Execute the code under test.
        let cycles_count = cpu.execute_instruction(&mut ram);
        let result_val = get(&mut cpu, &mut ram);

        assert_eq!(cycles_count, cycles);
        assert_eq!(cpu.reg.pc, 2);
        assert_eq!(result_val, 0b_0000_1100);
        assert_eq!(cpu.reg.f, 0);

        //////// Test Carry Flag Case ////////

        // Reset the CPU
        cpu = Cpu::new();

        let val = 0x71;
        set(&mut cpu, &mut ram, val);

        cpu.execute_instruction(&mut ram);

        let result_val = get(&mut cpu, &mut ram);

        assert_eq!(result_val, 0b_1011_1000);
        assert_eq!(cpu.reg.f, Regs::CARRY_FLAG);

        //////// Test Zero Flag Case ////////
        cpu = Cpu::new();
        set(&mut cpu, &mut ram, 0);

        cpu.execute_instruction(&mut ram);

        assert_eq!(get(&mut cpu, &mut ram), 0);
        assert_eq!(cpu.reg.f, Regs::ZERO_FLAG);
    }

    fn test_op_rl(inst: &[u8], addrMode:AddressingMode, cycles:u8)
    {
        // Setup test state.
        let mut tp = TestPack::new();
        tp.load_instruction(inst);

        // Carry in and out case.
        let carry_out_val = 0b_1000_0100;
        let expected_val = 0b_0000_1001;
        tp.set_value(addrMode, carry_out_val);
        tp.cpu.reg.f = Regs::CARRY_FLAG;

        tp.execute_instruction();
        tp.check_cycles(cycles);
        tp.check_length(inst.len() as u16);

        assert_eq!(tp.get_value(addrMode), expected_val);
        assert_eq!(tp.cpu.reg.f, Regs::CARRY_FLAG);

        // Carry in no carry out.
        tp.reset_cpu();
        let no_carry_out_val = 0b_0100_0100;
        let expected_val = 0b_1000_1001;
        tp.set_value(addrMode, no_carry_out_val);
        tp.cpu.reg.f = Regs::CARRY_FLAG;

        tp.execute_instruction();

        assert_eq!(tp.get_value(addrMode), expected_val);
        assert_eq!(tp.cpu.reg.f, 0);

        // No Carry out or in.
        tp.reset_cpu();
        let no_carry_out_val = 0b_0100_0100;
        let expected_val = 0b_1000_1000;
        tp.set_value(addrMode, no_carry_out_val);

        tp.execute_instruction();

        assert_eq!(tp.get_value(addrMode), expected_val);
        assert_eq!(tp.cpu.reg.f, 0);

        // Zero case
        tp.reset_cpu();
        tp.set_value(addrMode, 0);
        tp.execute_instruction();
        assert_eq!(tp.cpu.reg.f, Regs::ZERO_FLAG);
    }

    fn test_op_rr(inst: &[u8], addrMode:AddressingMode, cycles:u8)
    {
        // Setup test state.
        let mut tp = TestPack::new();
        tp.load_instruction(inst);

        // Carry in and out case.
        let carry_out_val = 0b_1000_0001;
        let expected_val = 0b_1100_0000;
        tp.set_value(addrMode, carry_out_val);
        tp.cpu.reg.f = Regs::CARRY_FLAG;

        tp.execute_instruction();
        tp.check_cycles(cycles);
        tp.check_length(inst.len() as u16);

        assert_eq!(tp.get_value(addrMode), expected_val);
        assert_eq!(tp.cpu.reg.f, Regs::CARRY_FLAG);

        // Carry in no carry out.
        tp.reset_cpu();
        let no_carry_out_val = 0b_0100_0100;
        let expected_val = 0b_1010_0010;
        tp.set_value(addrMode, no_carry_out_val);
        tp.cpu.reg.f = Regs::CARRY_FLAG;

        tp.execute_instruction();

        assert_eq!(tp.get_value(addrMode), expected_val);
        assert_eq!(tp.cpu.reg.f, 0);

        // No Carry out or in.
        tp.reset_cpu();
        let no_carry_out_val = 0b_0100_0010;
        let expected_val =     0b_0010_0001;
        tp.set_value(addrMode, no_carry_out_val);

        tp.execute_instruction();

        assert_eq!(tp.get_value(addrMode), expected_val);
        assert_eq!(tp.cpu.reg.f, 0);

        // Zero case
        tp.reset_cpu();
        tp.set_value(addrMode, 0);
        tp.execute_instruction();
        assert_eq!(tp.cpu.reg.f, Regs::ZERO_FLAG);
    }

    fn test_op_sla(inst: &[u8], addrMode: AddressingMode, cycles:u8)
    {
        // Setup test state.
        let mut tp = TestPack::new();
        tp.load_instruction(inst);

        // Carry out case.
        let carry_out_val = 0b_1100_0000;
        let expected_val = 0b_1000_0000;
        tp.set_value(addrMode, carry_out_val);
        tp.cpu.reg.f = Regs::CARRY_FLAG;

        tp.execute_instruction();
        tp.check_cycles(cycles);
        tp.check_length(inst.len() as u16);

        assert_eq!(tp.get_value(addrMode), expected_val);
        assert_eq!(tp.cpu.reg.f, Regs::CARRY_FLAG);

        // No carry out.
        tp.reset_cpu();
        let no_carry_out_val = 0b_0100_0110;
        let expected_val = 0b_1000_1100;
        tp.set_value(addrMode, no_carry_out_val);
        tp.cpu.reg.f = Regs::CARRY_FLAG;

        tp.execute_instruction();

        assert_eq!(tp.get_value(addrMode), expected_val);
        assert_eq!(tp.cpu.reg.f, 0);

        // Zero case
        tp.reset_cpu();
        tp.set_value(addrMode, 0);
        tp.execute_instruction();
        assert_eq!(tp.cpu.reg.f, Regs::ZERO_FLAG);
    }

    fn test_op_sra(inst: &[u8], addrMode: AddressingMode, cycles:u8)
    {
        // Setup test state.
        let mut tp = TestPack::new();
        tp.load_instruction(inst);

        // Carry out case, negative value case.
        let carry_out_val = 0b_1000_0001;
        let expected_val = 0b_1100_0000;
        tp.set_value(addrMode, carry_out_val);

        tp.execute_instruction();
        tp.check_cycles(cycles);
        tp.check_length(inst.len() as u16);

        assert_eq!(tp.get_value(addrMode), expected_val);
        assert_eq!(tp.cpu.reg.f, Regs::CARRY_FLAG);

        // No carry out, positive value case.
        tp.reset_cpu();
        let no_carry_out_val = 0b_0100_0110;
        let expected_val =     0b_0010_0011;
        tp.set_value(addrMode, no_carry_out_val);
        tp.cpu.reg.f = Regs::CARRY_FLAG;

        tp.execute_instruction();

        assert_eq!(tp.get_value(addrMode), expected_val);
        assert_eq!(tp.cpu.reg.f, 0);

        // Zero case
        tp.reset_cpu();
        tp.set_value(addrMode, 0);
        tp.execute_instruction();
        assert_eq!(tp.cpu.reg.f, Regs::ZERO_FLAG);
    }

    fn test_op_swap(inst: &[u8], addr_mode:AddressingMode, cycles:u8) {
        // Create a test package and load the instruction.
        let mut tp = TestPack::new();
        tp.load_instruction(inst);

        // Test non zero.
        let value = 0xAB;
        let expected = 0xBA;
        tp.cpu.reg.f = 0xFF; // Only to make sure it is cleared.
        tp.set_value(addr_mode, value);

        tp.execute_instruction();
        tp.check_cycles(cycles);
        tp.check_length(inst.len() as u16);

        assert_eq!(tp.get_value(addr_mode), expected);
        assert_eq!(tp.cpu.reg.f, 0);

        ///////
        // Test zero
        tp.reset_cpu();
        tp.set_value(addr_mode, 0);

        tp.execute_instruction();

        assert_eq!(tp.get_value(addr_mode), 0);
        assert_eq!(tp.cpu.reg.f, Regs::ZERO_FLAG);
    }

    fn test_op_srl(inst: &[u8], addrMode: AddressingMode, cycles:u8)
    {
        // Setup test state.
        let mut tp = TestPack::new();
        tp.load_instruction(inst);

        // Carry out case
        let carry_out_val = 0b_1000_0011;
        let expected_val = 0b_0100_0001;
        tp.set_value(addrMode, carry_out_val);

        tp.execute_instruction();
        tp.check_cycles(cycles);
        tp.check_length(inst.len() as u16);

        assert_eq!(tp.get_value(addrMode), expected_val);
        assert_eq!(tp.cpu.reg.f, Regs::CARRY_FLAG);

        // No carry out case
        tp.reset_cpu();
        let no_carry_out_val = 0b_0100_0110;
        let expected_val =     0b_0010_0011;
        tp.set_value(addrMode, no_carry_out_val);
        tp.cpu.reg.f = Regs::CARRY_FLAG;

        tp.execute_instruction();

        assert_eq!(tp.get_value(addrMode), expected_val);
        assert_eq!(tp.cpu.reg.f, 0);

        // Zero case
        tp.reset_cpu();
        tp.set_value(addrMode, 0);
        tp.execute_instruction();
        assert_eq!(tp.cpu.reg.f, Regs::ZERO_FLAG);
    }

    fn test_op_bit(inst: &[u8], index:u8, addr_mode:AddressingMode, cycles:u8) {
        // Setup test state.
        let mut tp = TestPack::new();
        tp.load_instruction(inst);

        ////////
        // Bit set case.
        tp.set_value(addr_mode, 1<<index);
        tp.cpu.reg.f = Regs::CARRY_FLAG; // only the check that it is not cleared.

        tp.execute_instruction();

        tp.check_cycles(cycles);
        tp.check_length(inst.len() as u16);
        tp.check_flags(Regs::HCARRY_FLAG | Regs::CARRY_FLAG);

        ////////
        // Bit cleared case
        tp.reset_cpu();
        tp.set_value(addr_mode, !(1<<index));

        tp.execute_instruction();

        tp.check_flags(Regs::HCARRY_FLAG | Regs::ZERO_FLAG);
    }

    fn test_op_res(inst: &[u8], index:u8, addr_mode:AddressingMode, cycles:u8)
    {
        let mut tp = TestPack::new();
        tp.load_instruction(inst);

        let value = 0xFF;
        tp.set_value(addr_mode, value);

        tp.execute_instruction();

        tp.check_cycles(cycles);
        tp.check_length(inst.len() as u16);
        assert_eq!(tp.get_value(addr_mode), !(1<<index));
    }

    fn test_op_set(inst: &[u8], index:u8, addr_mode:AddressingMode, cycles:u8)
    {
        let mut tp = TestPack::new();
        tp.load_instruction(inst);
        tp.set_value(addr_mode, 0);

        tp.execute_instruction();

        tp.check_cycles(cycles);
        tp.check_length(inst.len() as u16);
        assert_eq!(tp.get_value(addr_mode), 1<<index);
    }

    fn run_daa_test_case(flags:u8, val:u8, e_flags:u8, e_val:u8) {
        let inst = [0x27];
        let mut ram = get_ram();
        let mut cpu = Cpu::new();
        load_into_ram(&mut ram, &inst);

        cpu.reg.a = val;
        cpu.reg.f = flags;

        cpu.execute_instruction(&mut ram);

        assert_eq!(cpu.reg.f, e_flags, 
            "Flags didn't match flags: {:#X}, e_flags: {:#X}\nval: {:#X}, e_val: {:#X}\n", 
            cpu.reg.f, e_flags, 
            cpu.reg.a, e_val);
        assert_eq!(cpu.reg.a, e_val, 
            "Flags didn't match flags: {:#X}, e_flags: {:#X}\nval: {:#X}, e_val: {:#X}\n", 
            cpu.reg.f, e_flags, 
            cpu.reg.a, e_val);
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
        cpu.execute_instruction(&mut ram);

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
    fn cpu_0x27()
    {
        // Sets zero flag if result is zero.
        run_daa_test_case(0, 0, Regs::ZERO_FLAG, 0);
        // Leaves sub flag alone
        run_daa_test_case(Regs::SUB_FLAG, 1, Regs::SUB_FLAG, 1);

        // Wraps lower nibble in half carry add case.
        run_daa_test_case(Regs::HCARRY_FLAG, 0x12, 0, 0x18);
        // Wraps lower nibble in no half carry overflow case.
        run_daa_test_case(0, 0xB, 0, 0x11);
        // Wraps lower nibble in half carry subtract case
        run_daa_test_case(Regs::HCARRY_FLAG | Regs::SUB_FLAG, 0xF, Regs::SUB_FLAG, 0x09);

        // Wraps Upper nibble in carry case
        run_daa_test_case(0, 0x9A, Regs::CARRY_FLAG | Regs::ZERO_FLAG, 0x00);
        run_daa_test_case(Regs::SUB_FLAG | Regs::HCARRY_FLAG | Regs::CARRY_FLAG, 0xFF, 
                          Regs::CARRY_FLAG | Regs::SUB_FLAG, 0x99);
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

    #[test]
    fn cpu_0x70()
    {
        test_op_ldMR(&[0x70], Register::HL, Register::B);
    }

    #[test]
    fn cpu_0x71()
    {
        test_op_ldMR(&[0x71], Register::HL, Register::C);
    }

    #[test]
    fn cpu_0x72()
    {
        test_op_ldMR(&[0x72], Register::HL, Register::D);
    }

    #[test]
    fn cpu_0x73()
    {
        test_op_ldMR(&[0x73], Register::HL, Register::E);
    }

    #[test]
    fn cpu_0x74()
    {
        test_op_ldMR(&[0x74], Register::HL, Register::H);
    }

    #[test]
    fn cpu_0x75()
    {
        test_op_ldMR(&[0x75], Register::HL, Register::L);
    }

    #[test]
    #[ignore]
    fn cpu_0x76()
    {
        // Halt
    }

    #[test]
    fn cpu_0x77()
    {
        test_op_ldMR(&[0x77], Register::HL, Register::A);
    }

    #[test]
    fn cpu_0x78()
    {
        test_op_ldRR(&[0x78], Register::A, Register::B);
    }

    #[test]
    fn cpu_0x79()
    {
        test_op_ldRR(&[0x79], Register::A, Register::C);
    }

    #[test]
    fn cpu_0x7A()
    {
        test_op_ldRR(&[0x7A], Register::A, Register::D);
    }

    #[test]
    fn cpu_0x7B()
    {
        test_op_ldRR(&[0x7B], Register::A, Register::E);
    }

    #[test]
    fn cpu_0x7C()
    {
        test_op_ldRR(&[0x7C], Register::A, Register::H);
    }

    #[test]
    fn cpu_0x7D()
    {
        test_op_ldRR(&[0x7D], Register::A, Register::L);
    }

    #[test]
    fn cpu_0x7E()
    {
        test_op_ldRM(&[0x7E], Register::A, Register::HL, 4564, 23);
    }

    #[test]
    fn cpu_0x7F()
    {
        test_op_ldRR(&[0x7F], Register::A, Register::A);
    }

    #[test]
    fn cpu_0x80()
    {
        test_op_addr(&[0x80], Register::B);
    }

    #[test]
    fn cpu_0x81()
    {
        test_op_addr(&[0x81], Register::C);
    }

    #[test]
    fn cpu_0x82()
    {
        test_op_addr(&[0x82], Register::D);
    }

    #[test]
    fn cpu_0x83()
    {
        test_op_addr(&[0x83], Register::E);
    }

    #[test]
    fn cpu_0x84()
    {
        test_op_addr(&[0x84], Register::H);
    }

    #[test]
    fn cpu_0x85()
    {
        test_op_addr(&[0x85], Register::L);
    }

    #[test]
    fn cpu_0x86()
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();

        // Load the instruction into ram
        load_into_ram(&mut ram, &[0x86]);

        // Setup ram with a test value;
        let address = 0x1000;
        let value = 1;
        let expected = 0;
        cpu.reg.a = 0xFF;
        cpu.reg.f = 0xFF;
        cpu.reg.write16(Register::HL, address as u16);
        ram.bus_write8(address, value);
        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 2);
        assert_eq!(cpu.reg.pc, 1);
        assert_eq!(cpu.reg.a, expected);
        assert_eq!(cpu.reg.f, Regs::ZERO_FLAG | Regs::CARRY_FLAG | Regs::HCARRY_FLAG);

        // Add values that will cause a half carry.
        let value = 0x9;
        cpu = Cpu::new();
        cpu.reg.a = 0x7;
        cpu.reg.write16(Register::HL, address as u16);
        ram.bus_write8(address, value);
        cpu.execute_instruction(&mut ram);

        assert_eq!(cpu.reg.a, 0x10);
        assert_eq!(cpu.reg.f, Regs::HCARRY_FLAG);
    }

    #[test]
    fn cpu_0x87()
    {
        test_op_addr(&[0x87], Register::A);
    }

    #[test]
    fn cpu_0x88()
    {
        test_op_addcr(&[0x88], Register::B);
    }

    #[test]
    fn cpu_0x89()
    {
        test_op_addcr(&[0x89], Register::C);
    }

    #[test]
    fn cpu_0x8A()
    {
        test_op_addcr(&[0x8A], Register::D);
    }

    #[test]
    fn cpu_0x8B()
    {
        test_op_addcr(&[0x8B], Register::E);
    }

    #[test]
    fn cpu_0x8C()
    {
        test_op_addcr(&[0x8C], Register::H);
    }

    #[test]
    fn cpu_0x8D()
    {
        test_op_addcr(&[0x8D], Register::L);
    }

    #[test]
    fn cpu_0x8E()
    {
        test_op_addc(&[0x8E], get_memory_HL_setter(0x32), 2);
    }

    #[test]
    fn cpu_0x8F()
    {
        test_op_addcr(&[0x8F], Register::A);
    }

    #[test]
    fn cpu_0x90()
    {
        test_op_subr(&[0x90], Register::B);
    }

    #[test]
    fn cpu_0x91()
    {
        test_op_subr(&[0x91], Register::C);
    }

    #[test]
    fn cpu_0x92()
    {
        test_op_subr(&[0x92], Register::D);
    }

    #[test]
    fn cpu_0x93()
    {
        test_op_subr(&[0x93], Register::E);
    }

    #[test]
    fn cpu_0x94()
    {
        test_op_subr(&[0x94], Register::H);
    }

    #[test]
    fn cpu_0x95()
    {
        test_op_subr(&[0x95], Register::L);
    }

    #[test]
    fn cpu_0x96()
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        load_into_ram(&mut ram, &[0x96]);
        
        let address = 0x2398;
        let value = 18;
        let expected = 255;
        let expected_flags = Regs::CARRY_FLAG | Regs::HCARRY_FLAG | Regs::SUB_FLAG;

        ram.bus_write16(address, value);
        cpu.reg.write16(Register::HL, address as u16);
        cpu.reg.a = 17;

        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 2);
        assert_eq!(cpu.reg.pc, 1);
        assert_eq!(cpu.reg.a, expected);
        assert_eq!(cpu.reg.f, expected_flags);
    }

    #[test]
    fn cpu_0x97()
    {
        test_op_subr(&[0x97], Register::A);
    }

    #[test]
    fn cpu_0x98()
    {
        test_op_sbcr(&[0x98], Register::B);
    }

    #[test]
    fn cpu_0x99()
    {
        test_op_sbcr(&[0x99], Register::C);
    }

    #[test]
    fn cpu_0x9A()
    {
        test_op_sbcr(&[0x9A], Register::D);
    }

    #[test]
    fn cpu_0x9B()
    {
        test_op_sbcr(&[0x9B], Register::E);
    }

    #[test]
    fn cpu_0x9C()
    {
        test_op_sbcr(&[0x9C], Register::H);
    }

    #[test]
    fn cpu_0x9D()
    {
        test_op_sbcr(&[0x9D], Register::L);
    }

    #[test]
    fn cpu_0x9E()
    {
        test_op_sbcm(&[0x9E]);
    }

    #[test]
    fn cpu_0x9F()
    {
        test_op_sbcr(&[0x9F], Register::A);
    }

    #[test]
    fn cpu_0xA0()
    {
        test_op_andr(&[0xA0], Register::B);
    }

    #[test]
    fn cpu_0xA1()
    {
        test_op_andr(&[0xA1], Register::C);
    }

    #[test]
    fn cpu_0xA2()
    {
        test_op_andr(&[0xA2], Register::D);
    }

    #[test]
    fn cpu_0xA3()
    {
        test_op_andr(&[0xA3], Register::E);
    }

    #[test]
    fn cpu_0xA4()
    {
        test_op_andr(&[0xA4], Register::H);
    }

    #[test]
    fn cpu_0xA5()
    {
        test_op_andr(&[0xA5], Register::L);
    }

    #[test]
    fn cpu_0xA6()
    {
        test_op_andm(&[0xA6]);
    }

    #[test]
    fn cpu_0xA7()
    {
        test_op_andr(&[0xA7], Register::A);
    }

    #[test]
    fn cpu_0xA8()
    {
        test_op_xorr(&[0xA8], Register::B);
    }

    #[test]
    fn cpu_0xA9()
    {
        test_op_xorr(&[0xA9], Register::C);
    }

    #[test]
    fn cpu_0xAA()
    {
        test_op_xorr(&[0xAA], Register::D);
    }

    #[test]
    fn cpu_0xAB()
    {
        test_op_xorr(&[0xAB], Register::E);
    }

    #[test]
    fn cpu_0xAC()
    {
        test_op_xorr(&[0xAC], Register::H);
    }

    #[test]
    fn cpu_0xAD()
    {
        test_op_xorr(&[0xAD], Register::L);
    }

    #[test]
    fn cpu_0xAE()
    {
        test_op_xorm(&[0xAE]);
    }

    #[test]
    fn cpu_0xAF()
    {
        test_op_xorr(&[0xAF], Register::A);
    }

    #[test]
    fn cpu_0xB0()
    {
        test_op_orr(&[0xB0], Register::B);
    }

    #[test]
    fn cpu_0xB1()
    {
        test_op_orr(&[0xB1], Register::C);
    }

    #[test]
    fn cpu_0xB2()
    {
        test_op_orr(&[0xB2], Register::D);
    }

    #[test]
    fn cpu_0xB3()
    {
        test_op_orr(&[0xB3], Register::E);
    }

    #[test]
    fn cpu_0xB4()
    {
        test_op_orr(&[0xB4], Register::H);
    }

    #[test]
    fn cpu_0xB5()
    {
        test_op_orr(&[0xB5], Register::L);
    }

    #[test]
    fn cpu_0xB6()
    {
        test_op_orm(&[0xB6]);
    }

    #[test]
    fn cpu_0xB7()
    {
        test_op_orr(&[0xB7], Register::A)
    }

    #[test]
    fn cpu_0xB8()
    {
        test_op_cpr(&[0xB8], Register::B);
    }

    #[test]
    fn cpu_0xB9()
    {
        test_op_cpr(&[0xB9], Register::C);
    }

    #[test]
    fn cpu_0xBA()
    {
        test_op_cpr(&[0xBA], Register::D);
    }

    #[test]
    fn cpu_0xBB()
    {
        test_op_cpr(&[0xBB], Register::E);
    }

    #[test]
    fn cpu_0xBC()
    {
        test_op_cpr(&[0xBC], Register::H);
    }

    #[test]
    fn cpu_0xBD()
    {
        test_op_cpr(&[0xBD], Register::L);
    }

    #[test]
    fn cpu_0xBE()
    {
        test_op_cpm(&[0xBE]);
    }

    #[test]
    fn cpu_0xBF()
    {
        test_op_cpr(&[0xBF], Register::A);
    }

    #[test]
    fn cpu_0xC0()
    {
        // Return on not zero.
        test_op_ret(&[0xC0], JumpCondition::Nz);
    }

    #[test]
    fn cpu_0xC1()
    {
        test_op_pop(&[0xC1], Register::BC);
    }

    #[test]
    fn cpu_0xC2()
    {
        test_op_jp(&[0xC2, 1, 3], JumpCondition::Nz);
    }

    #[test]
    fn cpu_0xC3()
    {
        test_op_jp(&[0xC3, 2, 4], JumpCondition::Always);
    }

    #[test]
    fn cpu_0xC4()
    {
        test_op_call(&[0xC4, 83, 34], JumpCondition::Nz);
    }

    #[test]
    fn cpu_0xC5()
    {
        test_op_push(&[0xC5], Register::BC);
    }

    #[test]
    fn cpu_0xC6()
    {
        test_op_addi(&[0xC6, 00]);
    }

    #[test]
    fn cpu_0xC7()
    {
        test_op_rst(&[0xC7], 0);
    }

    #[test]
    fn cpu_0xC8()
    {
        test_op_ret(&[0xC8], JumpCondition::Z);
    }

    #[test]
    fn cpu_0xC9()
    {
        test_op_ret(&[0xC9], JumpCondition::Always);
    }

    #[test]
    fn cpu_0xCA()
    {
        test_op_jp(&[0xCA, 3, 4], JumpCondition::Z);
    }

    // The 0xCBXX family of instruction tests are after the single byte instructions.

    #[test]
    fn cpu_0xCC()
    {
        test_op_call(&[0xCC, 32, 23],  JumpCondition::Z);
    }

    #[test]
    fn cpu_0xCD()
    {
        test_op_call(&[0xCD, 0xFF, 0xEE],  JumpCondition::Always);
    }

    #[test]
    fn cpu_0xCE()
    {
        test_op_addci(&[0xCE, 0]);
    }

    #[test]
    fn cpu_0xCF()
    {
        test_op_rst(&[0xCF], 1);
    }

    #[test]
    fn cpu_0xD0()
    {
        test_op_ret(&[0xD0], JumpCondition::Nc);
    }

    #[test]
    fn cpu_0xD1()
    {  
         test_op_pop(&[0xD1], Register::DE);
    }

    #[test]
    fn cpu_0xD2()
    {
        test_op_jp(&[0xD2, 1, 2], JumpCondition::Nc);
    }

    #[test]
    fn cpu_0xD3()
    {
        // Crash?
    }

    #[test]
    fn cpu_0xD4()
    {
        test_op_call(&[0xD4, 1, 2], JumpCondition::Nc);
    }

    #[test]
    fn cpu_0xD5()
    {
        test_op_push(&[0xD5], Register::DE);
    }

    #[test]
    fn cpu_0xD6()
    {
        test_op_subi(&[0xD6, 0]);
    }

    #[test]
    fn cpu_0xD7()
    {
        test_op_rst(&[0xD7], 2);
    }

    #[test]
    fn cpu_0xD8()
    {
        test_op_ret(&[0xD8], JumpCondition::C);
    }

    #[test]
    fn cpu_0xD9()
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        load_into_ram(&mut ram, &[0xD9]);

        // Write a return address to the tip of the stack pointer
        let ret_addr = 0x123;
        let stack_ptr:u16 = 0x200;
        ram.bus_write16(stack_ptr as usize, ret_addr);
        cpu.reg.sp = stack_ptr;
        cpu.isr_en = false;

        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 4);
        assert_eq!(cpu.reg.pc, ret_addr);
        assert_eq!(cpu.reg.sp, stack_ptr + 2);
        assert_eq!(cpu.isr_en, true);
    }

    #[test]
    fn cpu_0xDA()
    {
        test_op_jp(&[0xDA, 0, 0], JumpCondition::C);
    }

    #[test]
    fn cpu_0xDB()
    {
        // Crash?
    }

    #[test]
    fn cpu_0xDC()
    {
        test_op_call(&[0xDC, 0, 0], JumpCondition::C);
    }

    #[test]
    fn cpu_0xDD()
    {
        // Crash?
    }

    #[test]
    fn cpu_0xDE()
    {
        test_op_sbc(&[0xDE, 0], get_immediate_u8_setter(1), 2);
    }

    #[test]
    fn cpu_0xDF()
    {
        test_op_rst(&[0xDF], 3);
    }

    #[test]
    fn cpu_0xE0()
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        let address = 0x22;
        load_into_ram(&mut ram, &[0xE0, address]);

        let value = 3;
        cpu.reg.a = value;

        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 3);
        assert_eq!(cpu.reg.pc, 2);
        assert_eq!(value, ram.bus_read8(address as usize + 0xFF00));
    }

    #[test]
    fn cpu_0xE1()
    {
        test_op_pop(&[0xE1], Register::HL);
    }

    #[test]
    fn cpu_0xE2()
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        load_into_ram(&mut ram, &[0xE2]);

        let offset = 0xFF;
        let value = 0xA5;

        cpu.reg.c = offset as u8;
        cpu.reg.a = value;

        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 2);
        assert_eq!(cpu.reg.pc, 1);
        assert_eq!(ram.bus_read8(0xff00usize + offset as usize), value);
    }

    #[test]
    fn cpu_0xE3()
    {
        // Crash?
    }

    #[test]
    fn cpu_0xE4()
    {
        // Crash?
    }

    #[test]
    fn cpu_0xE5()
    {
        test_op_push(&[0xE5], Register::HL);
    }

    #[test]
    fn cpu_0xE6()
    {
        test_op_and(&[0xE6, 0], get_immediate_u8_setter(1), 2);
    }

    #[test]
    fn cpu_0xE7()
    {
        test_op_rst(&[0xE7], 4);

    }

    #[test]
    fn cpu_0xE8()
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        // Subtract 1 from SP
        load_into_ram(&mut ram, &[0xE8, 0x80]);
        
        cpu.reg.f = 0xF0;
        cpu.reg.sp = 0x1234;
        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 4);
        assert_eq!(cpu.reg.pc, 2);
        assert_eq!(cpu.reg.sp, 0x1234 - 128);
        assert_eq!(cpu.reg.f, Regs::HCARRY_FLAG);

        load_into_ram(&mut ram, &[0xE8, 0x7F]);
        cpu = Cpu::new();
        cpu.reg.write16(Register::SP, 0x1234);
        cpu.execute_instruction(&mut ram);

        assert_eq!(cpu.reg.sp, 0x1234 + 127);
        assert_eq!(cpu.reg.f, Regs::HCARRY_FLAG);
    }

    #[test]
    fn cpu_0xE9()
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        load_into_ram(&mut ram, &[0xE9]);

        let address = 0x3265;
        cpu.reg.write16(Register::HL, address);

        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 1);
        assert_eq!(cpu.reg.pc, address);
    }

    #[test]
    fn cpu_0xEA()
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        load_into_ram(&mut ram, &[0xEA, 0x34, 0x12]);

        let value = 0x87;
        cpu.reg.a = value;

        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 4);
        assert_eq!(cpu.reg.pc, 3);
        assert_eq!(ram.bus_read8(0x1234), value);
    }

    #[test]
    fn cpu_0xEB()
    {
        // Crash?
    }

    #[test]
    fn cpu_0xEC()
    {
        // Crash?
    }

    #[test]
    fn cpu_0xED()
    {
        // Crash?
    }

    #[test]
    fn cpu_0xEE()
    {
        test_op_xor(&[0xEE, 0], get_immediate_u8_setter(1), 2);
    }

    #[test]
    fn cpu_0xEF()
    {
        test_op_rst(&[0xEF], 5);
    }

    #[test]
    fn cpu_0xF0()
    {
        // Ld A, (a8) // Zero page load.
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        load_into_ram(&mut ram, &[0xF0, 0x35]);
        let address = 0xFF35u16;
        let value = 93;
        ram.bus_write8(address as usize, value);

        let cycles = cpu.execute_instruction(&mut ram);
        
        assert_eq!(cycles, 3);
        assert_eq!(cpu.reg.pc, 2);
        assert_eq!(cpu.reg.a, value);
    }

    #[test]
    fn cpu_0xF1()
    {
        test_op_pop(&[0xF1], Register::AF);
    }

    #[test]
    fn cpu_0xF2()
    {
        // Ld A, (c) // Zero page load.
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        load_into_ram(&mut ram, &[0xF2]);
        let address = 0xFF35u16;
        let value = 93;
        ram.bus_write8(address as usize, value);
        cpu.reg.c = 0x35;

        let cycles = cpu.execute_instruction(&mut ram);
        
        assert_eq!(cycles, 2);
        assert_eq!(cpu.reg.pc, 1);
        assert_eq!(cpu.reg.a, value);
    }

    #[test]
    fn cpu_0xF3()
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        load_into_ram(&mut ram, &[0xF3]);

        cpu.isr_en = true;
        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 1);
        assert_eq!(cpu.reg.pc, 1);
        assert_eq!(cpu.isr_en, false);
    }

    #[test]
    fn cpu_0xF4()
    {
        // Crash?
    }

    #[test]
    fn cpu_0xF5()
    {
        test_op_push(&[0xF5], Register::AF);
    }

    #[test]
    fn cpu_0xF6()
    {
        test_op_or(&[0xF6, 0], get_immediate_u8_setter(1), 2);
    }

    #[test]
    fn cpu_0xF7()
    {
        test_op_rst(&[0xF7], 6);
    }

    #[test]
    fn cpu_0xF8()
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        load_into_ram(&mut ram, &[0xF8, 0x80]);

        cpu.reg.sp = 0x100;
        cpu.reg.f = 0xFF;
        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 3);
        assert_eq!(cpu.reg.pc, 2);
        assert_eq!(cpu.reg.read16(Register::HL), 0x100 - 128);
        assert_eq!(cpu.reg.f, Regs::HCARRY_FLAG);

        cpu = Cpu::new();
        load_into_ram(&mut ram, &[0xF8, 0x7F]);
        cpu.reg.sp = 0x100;

        cpu.execute_instruction(&mut ram);

        assert_eq!(cpu.reg.read16(Register::HL),  0x100 + 0x7F);
        assert_eq!(cpu.reg.f, 0);
    }

    #[test]
    fn cpu_0xF9()
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        load_into_ram(&mut ram, &[0xF9]);

        let value = 0x5432;
        cpu.reg.write16(Register::HL, value);

        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 2);
        assert_eq!(cpu.reg.pc, 1);
        assert_eq!(cpu.reg.sp, value);
    }

    #[test]
    fn cpu_0xFA()
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        load_into_ram(&mut ram, &[0xFA, 0x01, 0x02]);
        let address = 0x0201 as usize;

        let value = 23;
        ram.bus_write8(address, value);

        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 4);
        assert_eq!(cpu.reg.pc, 3);
        assert_eq!(cpu.reg.a, value);
    }

    #[test]
    fn cpu_0xFB()
    {
        let mut cpu = Cpu::new();
        let mut ram = get_ram();
        load_into_ram(&mut ram, &[0xFB]);

        cpu.isr_en = false;
        let cycles = cpu.execute_instruction(&mut ram);

        assert_eq!(cycles, 1);
        assert_eq!(cpu.reg.pc, 1);
        assert_eq!(cpu.isr_en, true);
    }

    #[test]
    fn cpu_0xFC()
    {
        // crash ?
    }

    #[test]
    fn cpu_0xFD()
    {
        // crash ?
    }

    #[test]
    fn cpu_0xFE()
    {
        test_op_cp(&[0xFE, 0], get_immediate_u8_setter(1), 2);
    }

    #[test]
    fn cpu_0xFF()
    {
        test_op_rst(&[0xFF], 7);
    }

    #[test]
    fn cpu_0xCB_00_to_07()
    {
        for x in 0..8{
            let target = CB_TARGET_TABLE[x as usize];
            println!("Testing 0xCB{:02X}", x);

            if target != Register::HL{
                test_op_rlc(
                    &[0xCB, x], 
                    get_register8_setter(target), 
                    get_register8_getter(target),
                    2);
            }
            else {
                test_op_rlc(
                    &[0xCB, x], 
                    get_memory_HL_setter(0x1234), 
                    get_memory_HL_getter(0x1234),
                    4);
            }
        }
    }

    #[test]
    fn cpu_0xCB_08_to_0F()
    {
        for x in 8..16{
            let target = CB_TARGET_TABLE[(x & 0x7) as usize];
            println!("Testing 0xCB{:02X}", x);

            if target != Register::HL{
                test_op_rrc(
                    &[0xCB, x], 
                    get_register8_setter(target), 
                    get_register8_getter(target),
                    2);
            }
            else {
                test_op_rrc(
                    &[0xCB, x], 
                    get_memory_HL_setter(0x1235), 
                    get_memory_HL_getter(0x1235),
                    4);
            }
        }
    }

    #[test]
    fn cpu_0xCB_10_to_17(){
        for x in 0..8 {
            println!("testing 0xCB{:02X}", 0x10|x);
            let mode = CB_ADDRESSING_MODES[x];
            let cycles = 
                if let AddressingMode::RegisterMem{..} = mode { 4 } 
                else { 2 };
            test_op_rl(
                &[0xCB, 0x10|(x as u8)],
                mode, 
                cycles);
        }
    }

    #[test]
    fn cpu_0xCB_18_to_1F(){
        for x in 0x18..0x1Fu8{
            println!("Testing 0xCB{:02X}", x);
            let mode = CB_ADDRESSING_MODES[(x&0b111) as usize];
            let cycles = 
                if let AddressingMode::RegisterMem{..} = mode { 4 } 
                else { 2 };
            test_op_rr(&[0xCB, x], mode, cycles);
        }
    }

    #[test]
    fn cpu_0xCB_20_to_27(){
        for x in 0x20..0x27u8{
            println!("testing 0xCB{:02X}", x);
            let mode = CB_ADDRESSING_MODES[(x&0b111) as usize];
            let cycles = 
                if let AddressingMode::RegisterMem{..} = mode { 4 } 
                else { 2 };
            test_op_sla(&[0xCB, x], mode, cycles);
        }
    }

    #[test]
    fn cpu_0xCB_28_to_2F(){
        for x in 0x28..0x2Fu8{
            println!("testing 0xCB{:02X}", x);
            let mode = CB_ADDRESSING_MODES[(x&0b111) as usize];
            let cycles = 
                if let AddressingMode::RegisterMem{..} = mode { 4 } 
                else { 2 };
            test_op_sra(&[0xCB, x], mode, cycles);
        }
    }

    #[test]
    fn cpu_0xCB_30_to_37(){
        for x in 0x30..0x37{
            println!("testing 0xCB{:02X}", x);
            let mode = CB_ADDRESSING_MODES[(x&0b111) as usize];
            let cycles = 
                if let AddressingMode::RegisterMem{..} = mode { 4 } 
                else { 2 };
            test_op_swap(&[0xCB, x], mode, cycles);
        }
    }

    #[test]
    fn cpu_0xCB_38_to_3F(){
        for x in 0x38..0x3F{
            println!("testing 0xCB{:02X}", x);
            let mode = CB_ADDRESSING_MODES[(x&0b111) as usize];
            let cycles = 
                if let AddressingMode::RegisterMem{..} = mode { 4 } 
                else { 2 };
            test_op_srl(&[0xCB, x], mode, cycles);
        }
    }

    #[test]
    fn cpu_0xCB_40_to_7F(){
        for x in 0x40..0x7F{
            println!("testing 0xCB{:02X}", x);
            let mode = CB_ADDRESSING_MODES[(x&0b111) as usize];
            let index = (x - 0x40)/8;
            let cycles = 
                if let AddressingMode::RegisterMem{..} = mode { 4 } 
                else { 2 };
            test_op_bit(&[0xCB, x], index, mode, cycles);
        }
    }

    #[test]
    fn cpu_0xCB_80_to_BF(){
        for x in 0x80..0xBF{
            println!("testing 0xCB{:02X}", x);
            let mode = CB_ADDRESSING_MODES[(x&0b111) as usize];
            let index = (x - 0x80)/8;
            let cycles = 
                if let AddressingMode::RegisterMem{..} = mode { 4 } 
                else { 2 };
            test_op_res(&[0xCB, x], index, mode, cycles);
        }
    }

    #[test]
    fn cpu_0xCB_C0_to_FF(){
        for x in 0xC0..0xFF{
            println!("testing 0xCB{:02X}", x);
            let mode = CB_ADDRESSING_MODES[(x&0b111) as usize];
            let index = (x - 0xC0)/8;
            let cycles = 
                if let AddressingMode::RegisterMem{..} = mode { 4 } 
                else { 2 };
            test_op_set(&[0xCB, x], index, mode, cycles);
        }
    }

    fn test_isr(is: &mut InterruptStatus, addr: u16)
    {
        // Setup components
        let mut cpu = Cpu::new();
        cpu.isr_en = true;
        let mut ram = get_ram();

        cpu.handle_interrupts(&mut ram, is);
        let cycles = cpu.update(&mut ram);

        assert_eq!(cpu.isr_en, false);
        assert_eq!(cpu.reg.pc, addr);
        assert_eq!(cycles, 5);
    }

    #[test]
    fn cpu_isr_vblank(){
        // Request a vblank interrupt
        let mut is = InterruptStatus::new();
        is.isrmask = 0xFF;
        is.request_vblank();

        test_isr(&mut is, 0x40);

        assert_eq!(is.is_vblank_active(), false);
    }

    #[test]
    fn cpu_isr_lcd() {
        // Request a vblank interrupt
        let mut is = InterruptStatus::new();
        is.isrmask = 0xFF;
        is.request_lcdstat();

        test_isr(&mut is, 0x48);

        assert_eq!(is.is_lcdstat_active(), false);
    }

    #[test]
    fn cpu_isr_timer() {
        // Request a vblank interrupt
        let mut is = InterruptStatus::new();
        is.isrmask = 0xFF;
        is.request_timer();

        test_isr(&mut is, 0x50);

        assert_eq!(is.is_timer_active(), false);
    }

    #[test]
    fn cpu_isr_serial() {
        // Request a vblank interrupt
        let mut is = InterruptStatus::new();
        is.isrmask = 0xFF;
        is.request_serial();

        test_isr(&mut is, 0x58);

        assert_eq!(is.is_serial_active(), false);
    }

    #[test]
    fn cpu_isr_joypad() {
        // Request a vblank interrupt
        let mut is = InterruptStatus::new();
        is.isrmask = 0xFF;
        is.request_joypad();

        test_isr(&mut is, 0x60);

        assert_eq!(is.is_joypad_active(), false);
    }

    #[test]
    fn cpu_update_noIsrIfDisabled()
    {
        // Disable interrupts on the cpu
        let mut cpu = Cpu::new();
        cpu.isr_en = false;

        // Enable and set all the interrupts.
        let mut is = InterruptStatus::new();
        is.isrmask = 0xFF;
        is.isrreq = 0xFF;

        // Ram is blank
        let mut ram = get_ram();

        cpu.handle_interrupts(&mut ram, &mut is);
        let cycles = cpu.update(&mut ram);

        // Cpu should have executed a NOP since ram is zeroed.
        assert_eq!(cycles, 1);
        assert_eq!(cpu.reg.pc, 1);
    }
}
