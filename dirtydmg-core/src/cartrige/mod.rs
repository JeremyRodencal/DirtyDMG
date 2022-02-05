mod mbc1;
mod mbc2;
mod mbc3;
mod mbc5;
mod no_mapper;
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
            0x05 => {info.mapper = MapperType::Mbc2;},
            0x06 => {info.mapper = MapperType::Mbc2;
                     info.battery = true},
            0x13 => {info.mapper = MapperType::Mbc3;
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
        // Special case for MBC2 mapper with integrated RAM
        if let MapperType::Mbc2 = info.mapper{
            info.ram_size = 512;
        }

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
            mapper: Box::new(no_mapper::NoMapper{}),
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
                    Box::new(no_mapper::NoMapper{})
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
                    Box::new(no_mapper::NoMapper{})
                },
                MapperType::Mbc1 => {
                    Box::new(mbc1::Mbc1Cart::new())
                },
                MapperType::Mbc2 => {
                    Box::new(mbc2::Mbc2Cart::new())
                }
                MapperType::Mbc3 => {
                    Box::new(mbc3::Mbc3Cart::new())
                }
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

    pub fn load_ram(&mut self, sram: &[u8]) {
        if sram.len() == self.ram.len(){
            self.ram.clone_from_slice(sram)
        }
    }

    pub fn get_ram(&self, buffer: &mut Vec<u8>) {
        buffer.clear();
        buffer.clone_from(&self.ram);
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
}

