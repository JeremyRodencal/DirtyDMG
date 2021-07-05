use crate::bus::{BusRW};
use byteorder::{ByteOrder, LittleEndian};

// Plain old read write memory, no special actions.
pub struct Ram {
	storage: Vec<u8>,
	start:	usize
}

impl Ram {
	pub fn new(size:usize, start_address:usize) -> Ram {
		Ram{
			storage: vec![0 as u8;size],
			start: start_address
		}
	}
}

impl BusRW for Ram {
	fn bus_read8(&mut self, addr:usize) -> u8
	{
		self.storage[addr - self.start]
	}

	fn bus_read16(&mut self, addr:usize) -> u16
	{
		LittleEndian::read_u16(&self.storage[addr - self.start..])
	}

	fn bus_write8(&mut self, addr:usize, value:u8)
	{
		self.storage[addr - self.start] = value;
	}

	fn bus_write16(&mut self, addr:usize, value:u16)
	{
		LittleEndian::write_u16(&mut self.storage[addr - self.start..], value);
	}
}
