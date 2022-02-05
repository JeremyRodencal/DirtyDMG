use super::MapperRW;

pub struct NoMapper { }

impl MapperRW for NoMapper {

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