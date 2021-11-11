mod dmg;
mod cpu;
mod bus;
mod ram;
mod interrupt;
mod cartrige;
mod ppu;
mod serial;

use std::env;
use std::io::{Error, ErrorKind};
use std::io::Read;
use std::fs;

use dmg::Dmg;

fn load_file(filepath: &str) -> Result<Vec<u8>, std::io::Error>
{
    let mut rom_file = fs::File::open(filepath)?;
    let meta = rom_file.metadata()?;
    
    // If the file is too larger
    if meta.len() > (8 * 1024 * 1024) {
        return Err(std::io::Error::new(ErrorKind::Other, "ROM file is too large."));
    }
    else if meta.len() < 32 * 1024 {
        return Err(std::io::Error::new(ErrorKind::Other, "ROM file is too small."));
    }

    let mut rom_data: Vec<u8> = Vec::new();
    rom_file.read_to_end(&mut rom_data).unwrap();

    return Ok(rom_data);
}

fn main() {

    let args:Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        println!("At least one command line argument must be provided.");
        return;
    }
    let path = &args[1];
    println!("ROM PATH: {}", path);

    let mut dmg = Dmg::new();

    // Load the rom into the DMG.
    let rom_data = load_file(path);
    match rom_data {
        Err(x) => {
            println!("Error loading rom file: {}", x);
            return;
        },
        Ok(x) => {
            dmg.load_rom(&x).unwrap();
        }
    }
}
