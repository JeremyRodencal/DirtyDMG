use super::*;

enum BankMode {
    Mode16KRom,
    Mode4KRom,
}

pub struct Mbc2Cart {
    mode: BankMode,
    rom_offset: usize,
    ram_enabled: bool,
    is_ram_mode: bool,
}

impl Mbc2Cart {
    const ROM_BANK_SIZE:usize = 0x4000;
    const RAM_BANK_SIZE:usize = 0x2000;

    const ROM_BANK_0_START_ADDR: usize = 0x0000;
    const ROM_BANK_0_END_ADDR: usize = 0x3FFF;
    const ROM_BANK_SWITCH_START_ADDR: usize = 0x4000;
    const ROM_BANK_SWITCH_END_ADDR: usize = 0x7FFF;
    const RAM_START_ADDR: usize = 0xA000;
    const RAM_END_ADDR: usize = 0xBFFF;

    const CFG_0_REG_START_ADDR: usize = 0x0000;
    const CFG_0_REG_END_ADDR:   usize = 0x3FFF;
    const CFG_0_REG_ROM_BANK_SELECT_BIT: usize = 0x100;

    pub fn new() -> Mbc2Cart {
        let mut mapper = Mbc2Cart {
            mode:BankMode::Mode16KRom,
            rom_offset:0,
            ram_enabled: false,
            is_ram_mode: false,
        };
        mapper.update_rom_bank_offset(1);
        
        // Return the mapper
        mapper
    }

    fn update_rom_bank_offset(&mut self, mut bank: u8) {
        // Only use the lower 4 bank bits. Bank zero must become a 1.
        bank &= 0xF;
        if bank == 0 {
            bank += 1;
        }

        self.rom_offset = (bank as usize) * Mbc2Cart::ROM_BANK_SIZE;
    }
}

impl MapperRW for Mbc2Cart {

    fn read(&mut self, ram: &mut [u8], rom: &mut [u8], addr:u16) -> u8{
        let addr = addr as usize;
        match addr {
            // Bank zero area
            Mbc2Cart::ROM_BANK_0_START_ADDR..=Mbc2Cart::ROM_BANK_0_END_ADDR => {
                rom[addr]
            }
            // Switchable bank area
            Mbc2Cart::ROM_BANK_SWITCH_START_ADDR..=Mbc2Cart::ROM_BANK_SWITCH_END_ADDR => {
                rom[self.rom_offset + addr - Mbc2Cart::ROM_BANK_SWITCH_START_ADDR]
            }
            // Ram area
            Mbc2Cart::RAM_START_ADDR..=Mbc2Cart::RAM_END_ADDR => {
                ram[(addr - Mbc2Cart::RAM_START_ADDR ) & 0x1FF]
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
            // Bank cfg 1 (upper 2 rom bank bits, or 2 ram bank bits)
            Mbc2Cart::CFG_0_REG_START_ADDR ..= Mbc2Cart::CFG_0_REG_END_ADDR => {

                // If this is a ROM bank selection
                if addr & Mbc2Cart::CFG_0_REG_ROM_BANK_SELECT_BIT != 0 {
                    self.update_rom_bank_offset(value);
                }
                // This is a RAM enable or disable operation.
                else {
                    self.ram_enabled = value == 0xA;
                }
            }
            
            // Ram region
            Mbc2Cart::RAM_START_ADDR ..= Mbc2Cart::RAM_END_ADDR => {
                if self.ram_enabled{
                    let index = (addr & 0x1FF) - Mbc2Cart::RAM_START_ADDR;
                    if index < ram.len(){
                        // Can only use 4 bit words, top 4 are "undefined".
                        ram[index] = value | 0xF0;
                    }
                }
            }

            // Should not ever happen
            _ => {
                println!("Write to unknown address in MBC2 cart {:#4X}", addr);
                // panic!("Write to unknown address in MBC2 cart {:#4X}", addr);
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