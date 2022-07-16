use super::MapperRW;
enum BankMode {
    Mode16KRom,
    Mode4KRom,
}

pub struct Mbc1Cart {
    mode: BankMode,
    rom_offset: usize,
    ram_offset: usize, 
    ram_enabled: bool,
    is_ram_mode: bool,
    low_bank_bits: u8,
    high_bank_bits: u8,
}

impl Mbc1Cart {
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

    pub fn new() -> Mbc1Cart {
        let mut mapper = Mbc1Cart {
            mode:BankMode::Mode16KRom,
            rom_offset:0,
            ram_offset:0,
            ram_enabled: false,
            is_ram_mode: false,
            low_bank_bits: 1,
            high_bank_bits: 0,
        };
        mapper.update_ram_bank_offset();
        mapper.update_rom_bank_offset();

        mapper
    }

    fn update_rom_bank_offset(&mut self) {
        let mut bank = self.low_bank_bits;
        if self.is_ram_mode == false {
            bank |= self.high_bank_bits << 5;
        }
        self.rom_offset = (bank as usize) * Mbc1Cart::ROM_BANK_SIZE;
    }

    fn update_ram_bank_offset(&mut self) {
        let bank = if self.is_ram_mode {self.high_bank_bits}
                        else {0};
        self.ram_offset = (bank as usize) * Mbc1Cart::RAM_BANK_SIZE;
    }
}

impl MapperRW for Mbc1Cart {

    fn read(&mut self, ram: &mut [u8], rom: &mut [u8], addr:u16) -> u8{
        let addr = addr as usize;
        match addr {
            // Bank zero area
            Mbc1Cart::ROM_BANK_0_START_ADDR..=Mbc1Cart::ROM_BANK_0_END_ADDR => {
                rom[addr]
            }
            // Switchable bank area
            Mbc1Cart::ROM_BANK_SWITCH_START_ADDR..=Mbc1Cart::ROM_BANK_SWITCH_END_ADDR => {
                rom[self.rom_offset + addr - Mbc1Cart::ROM_BANK_SWITCH_START_ADDR]
            }
            // Ram area
            Mbc1Cart::RAM_START_ADDR..=Mbc1Cart::RAM_END_ADDR => {
                ram[self.ram_offset + addr - Mbc1Cart::RAM_START_ADDR]
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
            Mbc1Cart::RAM_ENABLE_START_ADDR..=Mbc1Cart::RAM_ENABLE_END_ADDR => {
                // Only the magic value of 0xA enables ram.
                // Any other value disables ram
                self.ram_enabled = value & 0xF == 0xA; 
            }
            // Bank cfg 0 (lower 5 rom bank bits)
            Mbc1Cart::BANK_CFG_0_START_ADDR..=Mbc1Cart::BANK_CFG_0_END_ADDR => {
                let mut bits = value & 0x1F;
                // Zero is not allowed to be written to the lowest bank bits.
                if bits == 0 { bits += 1;}
                self.low_bank_bits = bits;
                self.update_rom_bank_offset();
            }
            // Bank cfg 1 (upper 2 rom bank bits, or 2 ram bank bits)
            Mbc1Cart::BANK_CFG_1_START_ADDR..=Mbc1Cart::BANK_CFG_1_END_ADDR => {
                self.high_bank_bits = value & 0b11;
                self.update_ram_bank_offset();
                self.update_rom_bank_offset();
            }
            // Set the RAM or ROM banking mode.
            Mbc1Cart::MODE_SEL_START_ADDR..=Mbc1Cart::MODE_SEL_END_ADDR => {
                self.is_ram_mode = value & 0x1 != 0;
            }
            // Ram region
            Mbc1Cart::RAM_START_ADDR..=Mbc1Cart::RAM_END_ADDR => {
                let index = addr + self.ram_offset - Mbc1Cart::RAM_START_ADDR;
                if index < ram.len(){
                    ram[index] = value;
                }
            }
            // Should not ever happen
            _ => {
                panic!("Write to unknown address in MBC1 cart {:#4X}", addr);
            }
        }
    }

    fn serialize(&self, _writer: &mut dyn std::io::Write) {
        todo!();
    }

    fn deserialize(&mut self, _reader: &mut dyn std::io::Read) {
        todo!();
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