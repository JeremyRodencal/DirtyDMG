use crate::bus::BusRW;

trait MapperRW{
    /// Reads a single byte from the mapper.
    fn read(&mut self, rom:&mut [u8], ram:&mut [u8], addr:u16) -> u8;

    /// Writes a single byte to the mapper.
    fn write(&mut self, rom:&mut [u8], ram:&mut [u8], addr:u16, value:u8);
}


enum MapperType{
    Rom,
    Mbc1,
    Mbc2,
    Mmm01,
    Mbc3,
    Mbc5,
    Mbc6,
    Mbc7,
}

struct CartInfo{
    /// The memory mapper used by the cart.
    mapper:MapperType,
    /// The rom size on the cart.
    rom_size: usize,
    /// The ram size on the cart (zero if none).
    ram_size: usize,
    /// True if a battery is present.
    battery: bool,
    /// True if a timer is present.
    timer: bool,
    /// True if rumble is present.
    rumble: bool,
}

impl CartInfo{
    /// Offset of the cart type byte in the ROM header
    pub const HEADER_TYPE_OFFSET: usize = 0x147;
    /// Offset of the ROM size byte in the ROM header
    pub const HEADER_ROM_SIZE_OFFSET: usize = 0x148;
    /// Offset of the RAM size byte in the ROM header.
    pub const HEADER_RAM_SIZE_OFFSET: usize = 0x149;
    pub const HEADER_LENGTH:usize = 0x150;

    fn from_header(header: &[u8]) -> Result<CartInfo, String> {

        // error out early if the header data is too short.
        if header.len() < CartInfo::HEADER_LENGTH {
            return Err("ROM header was too short.".to_owned());
        }

        // Construct a mutable info object to populate.
        let mut info = CartInfo {
            mapper:MapperType::Rom,
            rom_size:0,
            ram_size:0,
            battery: false,
            timer: false,
            rumble: false,
        };

        // Obtain info from the cartrige type byte
        match header[CartInfo::HEADER_TYPE_OFFSET] {
            0x00 => { }
            0x01 => {info.mapper = MapperType::Mbc1;},
            0x02 => {info.mapper = MapperType::Mbc1;},
            0x03 => {info.mapper = MapperType::Mbc1;
                     info.battery = true},
            0x1B => {info.mapper = MapperType::Mbc5;
                     info.battery = true},
            // TODO Complete cartrige type parsing.
            value => {
                return Err(format!("Unsupported cartrige type: {:#02X}", value).to_owned());
            }
        };

        // Get the cartrige rom size.
        match header[CartInfo::HEADER_ROM_SIZE_OFFSET]{
            0x00 => {info.rom_size = 32 * 0x400;}
            0x01 => {info.rom_size = 64 * 0x400;}
            0x02 => {info.rom_size = 128 * 0x400;}
            0x03 => {info.rom_size = 256 * 0x400;}
            0x04 => {info.rom_size = 512 * 0x400;}
            0x05 => {info.rom_size = 1 * 0x100000;}
            0x06 => {info.rom_size = 2 * 0x100000;}
            0x07 => {info.rom_size = 4 * 0x100000;}
            0x08 => {info.rom_size = 8 * 0x100000;}
            value => {
                return Err(format!("Unsupported cartrige rom size: {:02X}", value).to_owned())
            }
        };

        // Get the cartrige ram size.
        match header[CartInfo::HEADER_RAM_SIZE_OFFSET] {
            0x00 => {info.ram_size = 0;}
            0x01 => {info.ram_size = 2 * 0x400;}
            0x02 => {info.ram_size = 8 * 0x400;}
            0x03 => {info.ram_size = 32 * 0x400;}
            0x04 => {info.ram_size = 128 * 0x400;}
            0x05 => {info.ram_size = 64 * 0x400;}
            value => {
                return Err(format!("Unsupported cartrige ram size: {:02X}", value).to_owned());
            }
        };

        return Ok(info);
    }
}

pub struct Cartrige
{
    rom: Vec<u8>,
    ram: Vec<u8>,
    mapper: Box<dyn MapperRW>,
}

impl Cartrige {
    pub fn new() -> Cartrige{
        return Cartrige {
            ram: vec![0u8, 0],
            rom: vec![0u8, 0],
            mapper: Box::new(RawCart{}),
        }
    }

    pub fn from_data(rom_data: &[u8]) -> Result<Cartrige, String>
    {
        if rom_data.len() > 0x8000{
            return Err("Rom data was too long.".to_owned());
        }

        let info = CartInfo::from_header(rom_data)?;

        let ram: Vec<u8> = vec![0;info.ram_size];
        let rom: Vec<u8> = Vec::from(&rom_data[0..info.rom_size]);

        let mapper:Box<dyn MapperRW> = 
            match info.mapper {
                MapperType::Rom => {
                    Box::new(RawCart{})
                },
                MapperType::Mbc1 => {
                    Box::new(mbc1::Mbc1Cart::new())
                },
                _ => panic!("Unsupported mapper type")
            };

        let cart = Cartrige {
            mapper,
            rom,
            ram,
        };

        return Ok(cart);
    }

    pub fn load_rom(&mut self, rom_data: &[u8]) -> Result<(), String>
    {
        let info = CartInfo::from_header(rom_data)?;

        let ram: Vec<u8> = vec![0;info.ram_size];
        let rom: Vec<u8> = Vec::from(&rom_data[0..info.rom_size]);

        let mapper:Box<dyn MapperRW> = 
            match info.mapper {
                MapperType::Rom => {
                    Box::new(RawCart{})
                },
                MapperType::Mbc1 => {
                    Box::new(mbc1::Mbc1Cart::new())
                },
                MapperType::Mbc5 => {
                    Box::new(mbc5::Mbc5Cart::new())
                }
                _ => panic!("Unsupported mapper type")
            };

        self.rom = rom;
        self.ram = ram;
        self.mapper = mapper;

        return Ok(());
    }
}

impl BusRW for Cartrige {
    fn bus_read8(&mut self, addr:usize) -> u8
    {
        self.mapper.as_mut().read(&mut self.ram[..], &mut self.rom[..], addr as u16)
    }

    fn bus_write8(&mut self, addr:usize, value:u8)
    {
        self.mapper.as_mut().write(&mut self.ram[..], &mut self.rom[..], addr as u16, value);
    }

    fn bus_read16(&mut self, addr: usize) -> u16
    {
        let addr = addr as u16;
        let mut value: u16 = self.mapper.as_mut().read(&mut self.ram[..], &mut self.rom[..], addr) as u16;
        value |= (self.mapper.as_mut().read(&mut self.ram[..], &mut self.rom[..], addr+1) as u16) << 8;
        return value;
    }

    fn bus_write16(&mut self, addr: usize, value: u16)
    {
        let addr = addr as u16;
        self.mapper.as_mut().write(&mut self.ram[..], &mut self.rom[..], addr, (value & 0xFF) as u8);
        self.mapper.as_mut().write(&mut self.ram[..], &mut self.rom[..], addr+1, (value >> 8) as u8);
    }
}

struct RawCart { }

impl MapperRW for RawCart {

    fn read(&mut self, _ram: &mut [u8], rom: &mut [u8], addr:u16) -> u8{
        match addr {
            // Return values strait up if they are in the rom area.
            0x0000..=0x7FFF => {
                return rom[addr as usize];
            }
            // For any other address, return 0xFF.
            _ => {
                0xFF
            }
        }
    }

    fn write(&mut self, _:&mut [u8], _:&mut [u8], _:u16, _:u8) {
        // Nothing to do, ROM only.
    }

}


mod mbc1 {
    use super::*;

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
            return mapper;
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
                    return rom[addr];
                }
                // Switchable bank area
                Mbc1Cart::ROM_BANK_SWITCH_START_ADDR..=Mbc1Cart::ROM_BANK_SWITCH_END_ADDR => {
                    return rom[self.rom_offset + addr - Mbc1Cart::ROM_BANK_SWITCH_START_ADDR];
                }
                // Ram area
                Mbc1Cart::RAM_START_ADDR..=Mbc1Cart::RAM_END_ADDR => {
                    return ram[self.ram_offset + addr - Mbc1Cart::RAM_START_ADDR];
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
                    self.ram_enabled = 
                        if value & 0xF == 0xA { true }
                        else { false };
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
}

mod mbc5 {
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
            return mapper;
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
                    return rom[addr];
                }
                // Switchable bank area
                Mbc5Cart::ROM_BANK_SWITCH_START_ADDR..=Mbc5Cart::ROM_BANK_SWITCH_END_ADDR => {
                    return rom[self.rom_offset + addr - Mbc5Cart::ROM_BANK_SWITCH_START_ADDR];
                }
                // Ram area
                Mbc5Cart::RAM_START_ADDR..=Mbc5Cart::RAM_END_ADDR => {
                    return ram[self.ram_offset + addr - Mbc5Cart::RAM_START_ADDR];
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
                        if value & 0xF == 0xA { true }
                        else { false };
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
}
