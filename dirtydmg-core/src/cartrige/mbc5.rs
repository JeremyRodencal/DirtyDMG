use super::*;

enum BankMode {
    Mode16KRom,
    Mode4KRom,
}

pub struct Mbc5Cart {
    mode: BankMode,
    rom_offset: usize,
    ram_offset: usize, 
    ram_enabled: bool,
    low_bank_bits: u8,
    high_bank_bits: u8,
    ram_bank: u8,
}

impl Mbc5Cart {
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
    const BANK_CFG_0_END_ADDR: usize = 0x2FFF;
    const BANK_CFG_1_START_ADDR: usize = 0x3000;
    const BANK_CFG_1_END_ADDR: usize = 0x3FFF;
    const BANK_CFG_2_START_ADDR: usize = 0x4000;
    const BANK_CFG_2_END_ADDR: usize = 0x5FFF;
    const MODE_SEL_START_ADDR: usize = 0x6000;
    const MODE_SEL_END_ADDR: usize = 0x7FFF;

    pub fn new() -> Mbc5Cart {
        let mut mapper = Mbc5Cart {
            mode:BankMode::Mode16KRom,
            rom_offset:0,
            ram_offset:0,
            ram_enabled: false,
            low_bank_bits: 1,
            high_bank_bits: 0,
            ram_bank: 0,
        };
        mapper.update_ram_bank_offset();
        mapper.update_rom_bank_offset();
        
        // Return the mapper
        mapper
    }

    fn update_rom_bank_offset(&mut self) {
        let bank = 
            ((self.high_bank_bits as u16) << 8) 
            |(self.low_bank_bits) as u16;
        self.rom_offset = (bank as usize) * Mbc5Cart::ROM_BANK_SIZE;
    }

    fn update_ram_bank_offset(&mut self) {
        self.ram_offset = (self.ram_bank as usize) * Mbc5Cart::RAM_BANK_SIZE;
    }
}

impl MapperRW for Mbc5Cart {

    fn read(&mut self, ram: &mut [u8], rom: &mut [u8], addr:u16) -> u8{
        let addr = addr as usize;
        match addr {
            // Bank zero area
            Mbc5Cart::ROM_BANK_0_START_ADDR..=Mbc5Cart::ROM_BANK_0_END_ADDR => {
                rom[addr]
            }
            // Switchable bank area
            Mbc5Cart::ROM_BANK_SWITCH_START_ADDR..=Mbc5Cart::ROM_BANK_SWITCH_END_ADDR => {
                rom[self.rom_offset + addr - Mbc5Cart::ROM_BANK_SWITCH_START_ADDR]
            }
            // Ram area
            Mbc5Cart::RAM_START_ADDR..=Mbc5Cart::RAM_END_ADDR => {
                ram[self.ram_offset + addr - Mbc5Cart::RAM_START_ADDR]
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
            Mbc5Cart::RAM_ENABLE_START_ADDR..=Mbc5Cart::RAM_ENABLE_END_ADDR => {
                // Only the magic value of 0xA enables ram.
                // Any other value disables ram
                self.ram_enabled = 
                    value & 0xF == 0xA;
            }
            // Bank cfg 0 (lower 8 rom bank bits)
            Mbc5Cart::BANK_CFG_0_START_ADDR..=Mbc5Cart::BANK_CFG_0_END_ADDR => {
                self.low_bank_bits = value;
                self.update_rom_bank_offset();
            }
            // Bank cfg 1 (high bank bit)
            Mbc5Cart::BANK_CFG_1_START_ADDR..=Mbc5Cart::BANK_CFG_1_END_ADDR => {
                self.high_bank_bits = value & 0b1;
                self.update_rom_bank_offset();
            }
            // Ram bank number
            Mbc5Cart::BANK_CFG_2_START_ADDR..=Mbc5Cart::BANK_CFG_2_END_ADDR => {
                self.ram_bank = value & 0xF;
                self.update_ram_bank_offset();
            }
            // Ram region
            Mbc5Cart::RAM_START_ADDR..=Mbc5Cart::RAM_END_ADDR => {
                let index = addr + self.ram_offset - Mbc5Cart::RAM_START_ADDR;
                if self.ram_enabled && index < ram.len(){
                    ram[index] = value;
                }
            }
            // Should not ever happen
            _ => {
                panic!("Write to unknown address in MBC5 cart {:#4X}", addr);
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