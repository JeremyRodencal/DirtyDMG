use super::MapperRW;

pub struct NoMapper { }

impl MapperRW for NoMapper {

    fn read(&mut self, _ram: &mut [u8], rom: &mut [u8], addr:u16) -> u8{
        match addr {
            // Return values strait up if they are in the rom area.
            0x0000..=0x7FFF => {
                rom[addr as usize]
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

    /// Serializes mapper state into a writer
    fn serialize(&self, _writer: &mut dyn std::io::Write) { 
        // There is no state to serialize when there is no mapper.
    }

    /// Deserailizes mapper state from a reader
    fn deserialize(&mut self, _reader: &mut dyn std::io::Read) { 
        // There is no state to deserailize when there is no mapper.
    }

}