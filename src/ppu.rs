use crate::bus::{BusRW};

/// Overall size of the ram block used for tile sets.
const TILESET_RAM:usize = 0x1800;
/// The total number of tiles in the tile sets.
const TILESET_COUNT:usize = 0x180;
/// The starting RAM address of the tile sets.
const TILESET_START_ADDRESS:usize = 0x8000;
/// The ending RAM address of the tile sets.
const TILESET_END_ADDRESS:usize = TILESET_START_ADDRESS + TILESET_RAM - 1;

/// The width and height of a tile in pixels
const TILE_DIMENSION:usize = 8;
/// The number of bytes in an individual tile
const TILE_SIZE:usize = 16;

/// A tile map is 32x32 tiles.
const TILEMAP_DIMENSION: usize = 32;
/// The number of tiles in a tile map.
const TILEMAP_ITEM_COUNT: usize = TILEMAP_DIMENSION * TILEMAP_DIMENSION;
/// The number of tile maps on the system
const TILEMAPS_COUNT: usize = 2;
/// The ram size of all tile maps combined.
const TILEMAPS_SIZE: usize = TILEMAPS_COUNT * TILEMAP_ITEM_COUNT;
/// The starting address of all tile set data
const TILEMAP_START_ADDRESS:usize = 0x9800;
const TILEMAP_A_START_ADDRESS: usize = 0x9800;
const TILEMAP_A_END_ADDRESS: usize = 0x9BFF;
const TILEMAP_B_START_ADDRESS: usize = 0x9C00;
const TILEMAP_B_END_ADDRESS: usize = 0x9FFF;
/// The ending address of tile set data
const TILEMAP_END_ADDRESS:usize = 0x9FFF;

/// The number of tiles in OAM memory.
const OAM_SPRITE_COUNT: usize = 40;
/// The size of an individual sprite in memory.
const OAM_SPRITE_SIZE: usize = 4;
/// The amount of ram used for OAM.
const OAM_RAM_SIZE:usize = OAM_SPRITE_COUNT * OAM_SPRITE_SIZE;
const OAM_START_ADDRESS:usize = 0xFE00;
const OAM_END_ADDRESS:usize = OAM_START_ADDRESS + OAM_RAM_SIZE;

#[derive(Clone, Copy)]
/// Structure to hold tile pixel data in an easily accessable format.
struct Tile {
    /// tile pixel data.
    pixel: [[u8;TILE_DIMENSION];TILE_DIMENSION]
}

impl Tile {
    /// Reads pixel from the tile set.
    fn read_pixel(&self, x: u8, y: u8) -> u8
    {
        self.pixel[y as usize][x as usize]
    }

    /// Writes a pixel in the tile set.
    fn write_pixel(&mut self, x: u8, y: u8, value: u8)
    {
        self.pixel[y as usize][x as usize] = value;
    }

    // Updates a row of a tile with either msb or lsb pixel data.
    pub fn update_row(&mut self, mut data:u8, y:usize, msb:bool)
    {
        let shift = if msb {1} else {0};
        let clear_mask = !(1<<shift);

        // Pixels are stored such that the leftmost pixel is the MSB and rightmost pixel is the LSB
        // of the data byte.
        // This loop starts with the rightmost pixel, and works its way left.
        for x in (0..8).rev(){
            self.pixel[y][x] &= clear_mask;
            self.pixel[y][x] |= (data & 0x01) << shift;
            data >>= 1;
        }
    }

    fn new() -> Tile{
        Tile{
            pixel: [[0;TILE_DIMENSION];TILE_DIMENSION]
        }
    }
}

#[derive(Clone, Copy)]
struct OamSprite{
    ypos: u8,
    xpos: u8,
    tile: u8,

    //These fields are all packed into a single byte.
    behind_background: bool,
    xflip: bool,
    yflip: bool,
    palette: bool,
}

impl OamSprite {
    fn new() -> OamSprite{
        OamSprite{
            ypos: 0,
            xpos: 0,
            tile: 0,
            behind_background: false,
            xflip: false,
            yflip: false,
            palette: false
        }
    }
}

pub struct PPU {
    /// Raw tile data stored in the origial gameboy format.
    tile_data: [u8;TILESET_RAM],
    /// Nicely broken out pixel verions of the raw tile data.
    tiles: [Tile;TILESET_COUNT],

    /// tilemap data.
    tilemaps: [u8;TILEMAPS_SIZE],

    /// Decoded sprite object data
    sprites: [OamSprite; OAM_SPRITE_COUNT],
    /// Raw OAM data.
    sprite_data: [u8;OAM_RAM_SIZE],

    // LCD registers
    lcdc: u8,
    lcds: u8,
}

impl PPU {
    // LCDC bit masks.
    const LCDC_ENABLE_MASK: u8                      = 0b1000_0000;
    const LCDC_WINDOW_TILE_MAP_MASK: u8             = 0b0100_0000;
    const LCDC_WINDOW_DISPLAY_ENABLE_MASK: u8       = 0b0010_0000;
    const LCDC_BG_WINDOW_TILE_MAP_SELECT_MASK: u8   = 0b0001_0000;
    const LCDC_BG_TILE_MAP_SELECT_MASK: u8          = 0b0000_1000;
    const LCDC_OBJ_SIZE_MASK: u8                    = 0b0000_0100;
    const LCDC_OBJ_DISPLAY_ENABLE_MASK: u8          = 0b0000_0010;
    const LCDC_BG_WINDOW_PRIORITY_MASK: u8          = 0b0000_0001;

    ///#Executes the specified number of clock ticks.
    pub fn execute_ticks(&mut self, ticks:u16){

    }

    pub fn new() -> PPU {
        let blank_tile = Tile::new();
        let default_sprite = OamSprite::new();
        PPU {
            tile_data: [0;TILESET_RAM],
            tiles: [blank_tile;TILESET_COUNT],
            tilemaps:[0;TILEMAPS_SIZE],
            sprites: [default_sprite;OAM_SPRITE_COUNT],
            sprite_data: [0;OAM_RAM_SIZE],
            lcdc: 0,
            lcds: 0,
        }
    }

    fn tile_write(&mut self, data:u8, addr:usize)
    {
        let index = (addr - TILESET_START_ADDRESS) / TILE_SIZE;
        let y = (addr>>1) & 0x7;
        let msb =  (addr & 0x01) != 0;

        // Update the raw copy of the data
        self.tile_data[addr - TILESET_START_ADDRESS] = data;
        // Update the tile data.
        self.tiles[index].update_row(data, y, msb);
    }
}

impl BusRW for PPU{
    fn bus_read8(&mut self, addr: usize)-> u8{
        match addr {
            // Tile data read
            TILESET_START_ADDRESS..=TILESET_END_ADDRESS => {
                self.tile_data[addr-TILESET_START_ADDRESS]
            },

            // Tile map read
            TILEMAP_START_ADDRESS..=TILEMAP_END_ADDRESS => {
                self.tile_data[addr-TILEMAP_START_ADDRESS]
            },

            // Object attribute memory read
            OAM_START_ADDRESS..=OAM_END_ADDRESS => {
                self.sprite_data[addr - OAM_START_ADDRESS]
            },
            
            // Unknown read address.
            _ => {
                panic!("Unknown PPU read at address: 0x{:4X}", addr)
            }
        }
    }

    fn bus_write8(&mut self, addr: usize, value: u8){
        // TODO
        match addr {
            // Tile data write
            TILESET_START_ADDRESS..=TILESET_END_ADDRESS => {
                self.tile_write(value, addr);
            }

            // Unknown address.
            _ => {
                panic!("Unknown PPU write at address 0x{:4X}", addr);
            }
        }
    }

    fn bus_read16(&mut self, addr: usize) -> u16 {
        // TODO
        panic!("not implemented");
    }

    fn bus_write16(&mut self, addr: usize, value: u16){
        // TODO
        panic!("not implemented");
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_tile_write()
    {
        // Test data
        let tile_data = [0x7C, 0x7C, 0x00, 0xC6, 0xC6, 0x00, 0x00, 0xFE, 0xC6, 0xC6, 0x00, 0xC6, 0xC6, 0x00, 0x00, 0x00];
        let expected_tile = Tile{
            pixel: [
                [0, 3, 3, 3, 3, 3, 0, 0],
                [2, 2, 0, 0, 0, 2, 2, 0],
                [1, 1, 0, 0, 0, 1, 1, 0],
                [2, 2, 2, 2, 2, 2, 2, 0],
                [3, 3, 0, 0, 0, 3, 3, 0],
                [2, 2, 0, 0, 0, 2, 2, 0],
                [1, 1, 0, 0, 0, 1, 1, 0],
                [0, 0, 0, 0, 0, 0, 0, 0],
            ]
        };

        let mut ppu = PPU::new();

        // Write the tile data to first tile in the PPU
        let mut address = TILESET_START_ADDRESS;
        for x in tile_data {
            ppu.bus_write8(address, x);
            address += 1;
        }
        // Write the tile data to the last tile in the PPU
        address = TILESET_END_ADDRESS - TILE_SIZE + 1;
        for x in tile_data {
            ppu.bus_write8(address, x);
            address += 1;
        }

        // Make sure the tile was written in the first and last locations.
        for y in 0..TILE_DIMENSION{
            for x in 0..TILE_DIMENSION{
                assert_eq!(expected_tile.read_pixel(x as u8, y as u8), ppu.tiles[0].read_pixel(x as u8, y as u8));
                assert_eq!(expected_tile.read_pixel(x as u8, y as u8), ppu.tiles[ppu.tiles.len()-1].read_pixel(x as u8, y as u8));
            }
        }

        // Make sure the raw tile data can be accessed correctly.
        address = TILESET_START_ADDRESS;
        for x in tile_data {
            println!("address: {}", address);
            assert_eq!(x, ppu.bus_read8(address));
            address += 1;
        }
        address = TILESET_END_ADDRESS - TILE_SIZE + 1;
        for x in tile_data {
            assert_eq!(x, ppu.bus_read8(address));
            address += 1;
        }
    }
}
