use crate::bus::{BusRW};
use std::io::Write;
use std::io::Read;

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

	/// Serializes this ram data into a stream
	/// 
	/// ## Details
	/// 
	/// This doesn't not serialize things like the size and start_address
	/// of this object, just the data.
	/// 
	/// ## Arguments
	/// 
	/// * `writer` - A writer to serialize the ram data into.
	pub fn serialize<T>(&self, writer: &mut T)
		where T : Write + ?Sized
	{
		writer.write_all(self.storage.as_slice()).unwrap();
	}

	/// Deserializes data into this ram object
	/// 
	/// ## Details
	/// 
	/// This function will attempt to read in the exact size of this
	/// ram object from the reader. If the correct number of bytes cannot
	/// be read, this function will panic.
	/// 
	/// ## Arguments
	/// 
	/// * `reader` - The reader to deserialize data from
	pub fn deserialize<T>(&mut self, reader: &mut T)
		where T : Read + ?Sized
	{
		// Read the bytes in.
		let read_size = reader.read(&mut self.storage).unwrap();
		if read_size != self.storage.len()
		{
			panic!("Failed to read in all data");
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

#[cfg(test)]
mod test {

use super::*;

	#[test]
	fn test_serilizes_and_deserializes()
	{
		//////// Setup ////////
		let mut ram = Ram::new(10, 20);
		for x in 20u8..30
		{
			ram.bus_write8(x as usize, x);
		}

		//////// Execute ////////
		let mut buffer = [0u8;10];
		{
			let mut writer = &mut buffer[..];
			ram.serialize(&mut writer);
		}
		let mut deseriailzed = Ram::new(10, 20);
		{
			let mut reader = &buffer[..];
			deseriailzed.deserialize(&mut reader);
		}

		//////// Check ////////
		assert_eq!(ram.storage, deseriailzed.storage);
	}
}