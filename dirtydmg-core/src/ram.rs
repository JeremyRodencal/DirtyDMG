use crate::bus::{BusRW};

// Plain old read write memory, no special actions.
pub struct Ram {
	storage: Vec<u8>,
	start:	usize
}

impl Ram {
	pub fn new(size:usize, start_address:usize) -> Ram {
		Ram{
			storage: vec![0u8; size],
			start: start_address
		}
	}
}

impl BusRW for Ram {
	fn bus_read8(&mut self, addr:usize) -> u8
	{
		self.storage[addr - self.start]
	}

	fn bus_write8(&mut self, addr:usize, value:u8)
	{
		self.storage[addr - self.start] = value;
	}
}
