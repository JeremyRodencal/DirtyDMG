use super::*;

enum BankMode {
    Mode16KRom,
    Mode4KRom,
}

pub struct Mbc3Cart {
    mode: BankMode,
    rom_offset: usize,
    ram_offset: usize, 
    ram_enabled: bool,
    is_ram_mode: bool,
    rom_bank: u8,
    ram_bank: u8,
}

impl Mbc3Cart {
    const ROM_BANK_SIZE:usize = 0x4000;
    const RAM_BANK_SIZE:usize = 0x2000;

    const ROM_BANK_0_START_ADDR: usize = 0x0000;
    const ROM_BANK_0_END_ADDR: usize = 0x3FFF;
    const ROM_BANK_SWITCH_START_ADDR: usize = 0x4000;
    const ROM_BANK_SWITCH_END_ADDR: usize = 0x7FFF;
    const RAM_START_ADDR: usize = 0xA000;
    const RAM_END_ADDR: usize = 0xBFFF;

    const RAM_ENABLE_START_ADDR: usize = 0x0000;
    const RAM_ENABLE_END_ADDR: usize = 0x1FFF;
    const BANK_CFG_0_START_ADDR: usize = 0x2000;
    const BANK_CFG_0_END_ADDR: usize = 0x3FFF;
    const BANK_CFG_1_START_ADDR: usize = 0x4000;
    const BANK_CFG_1_END_ADDR: usize = 0x5FFF;
    const MODE_SEL_START_ADDR: usize = 0x6000;
    const MODE_SEL_END_ADDR: usize = 0x7FFF;

    pub fn new() -> Mbc3Cart {
        let mut mapper = Mbc3Cart {
            mode:BankMode::Mode16KRom,
            rom_offset:0,
            ram_offset:0,
            ram_enabled: false,
            is_ram_mode: false,
            rom_bank: 0,
            ram_bank: 0,
        };
        mapper.update_ram_bank_offset();
        mapper.update_rom_bank_offset();

        mapper
    }

    fn update_rom_bank_offset(&mut self) {
        self.rom_offset = (self.rom_bank as usize) * Mbc3Cart::ROM_BANK_SIZE;
    }

    fn update_ram_bank_offset(&mut self) {
        let bank = if self.is_ram_mode {self.ram_bank}
                    else {0};
        self.ram_offset = (bank as usize) * Mbc3Cart::RAM_BANK_SIZE;
    }
}

impl MapperRW for Mbc3Cart {

    fn read(&mut self, ram: &mut [u8], rom: &mut [u8], addr:u16) -> u8{
        let addr = addr as usize;
        match addr {
            // Bank zero area
            Mbc3Cart::ROM_BANK_0_START_ADDR..=Mbc3Cart::ROM_BANK_0_END_ADDR => {
                rom[addr]
            }
            // Switchable bank area
            Mbc3Cart::ROM_BANK_SWITCH_START_ADDR..=Mbc3Cart::ROM_BANK_SWITCH_END_ADDR => {
                rom[self.rom_offset + addr - Mbc3Cart::ROM_BANK_SWITCH_START_ADDR]
            }
            // Ram area
            Mbc3Cart::RAM_START_ADDR..=Mbc3Cart::RAM_END_ADDR => {
                ram[self.ram_offset + addr - Mbc3Cart::RAM_START_ADDR]
            }
            // For any other address, return 0xFF.
            _ => {
                0xFF
            }
        }
    }

    fn write(&mut self, ram:&mut [u8], _:&mut [u8], addr:u16, value:u8) {
        let addr = addr as usize;
        match addr {
            // Ram enable register write
            Mbc3Cart::RAM_ENABLE_START_ADDR..=Mbc3Cart::RAM_ENABLE_END_ADDR => {
                // Only the magic value of 0xA enables ram.
                // Any other value disables ram
                self.ram_enabled = value & 0xF == 0xA;
            }
            // Bank cfg 0 (7bit rom bank select)
            Mbc3Cart::BANK_CFG_0_START_ADDR..=Mbc3Cart::BANK_CFG_0_END_ADDR => {
                let mut bits = value & 0x7F;
                // Zero is not allowed to be written to the lowest bank bits.
                if bits == 0 { bits += 1; }
                self.rom_bank = bits;
                self.update_rom_bank_offset();
            }
            // Bank cfg 1 (upper 2 rom bank bits, or 2 ram bank bits)
            Mbc3Cart::BANK_CFG_1_START_ADDR..=Mbc3Cart::BANK_CFG_1_END_ADDR => {
                match value {
                    // Ram bank select
                    0..=4 => {
                        self.ram_bank = value;
                        self.update_ram_bank_offset();
                    }
                    // RTC register select
                    0x8..=0xC => {
                        // TODO 
                    }
                    // Unmapped
                    _ => {}
                }
            }
            // Latch clock data
            Mbc3Cart::MODE_SEL_START_ADDR..=Mbc3Cart::MODE_SEL_END_ADDR => {
                // TODO
            }
            // Ram region
            Mbc3Cart::RAM_START_ADDR..=Mbc3Cart::RAM_END_ADDR => {
                let index = addr + self.ram_offset - Mbc3Cart::RAM_START_ADDR;
                if self.ram_enabled && index < ram.len(){
                    ram[index] = value;
                }
            }
            // Should not ever happen
            _ => {
                panic!("Write to unknown address in MBC3 cart {:#04X}", addr);
            }
        }
    }
}

#[cfg(test)]
mod test {
    //TODO write unit tests for this module.
    #[test]
    #[ignore]
    fn mmc1_needs_to_be_tested(){
        assert!(false);
    }
}