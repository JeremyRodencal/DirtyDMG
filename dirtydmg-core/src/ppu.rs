use std::io::{Read, Write};
use byteorder::{ReadBytesExt, WriteBytesExt, LittleEndian};
use crate::bus::{BusRW};
use crate::interrupt::InterruptStatus;
use crate::interface::ScanlineBuffer;

/// Overall size of the ram block used for tile sets.
const TILESET_RAM:usize = 0x1800;
/// The total number of tiles in the tile sets.
const TILESET_COUNT:usize = 0x180;
// The base tile index to use for "high" addressing mode.
const TILESET_HIGH_BASE_INDEX:isize = 0x100;
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
const OAM_END_ADDRESS:usize = OAM_START_ADDRESS + (OAM_RAM_SIZE-1);

// LCD Registers
const LCDC_ADDRESS:usize = 0xFF40;
const LCDS_ADDRESS:usize = 0xFF41;

// Sroll registers
const SCY_ADDRESS:usize = 0xFF42;
const SCX_ADDRESS:usize = 0xFF43;
const LY_ADDRESS:usize = 0xFF44;
const LYC_ADDRES:usize = 0xFF45;
const WY_ADDRESS:usize = 0xFF4A;
const WX_ADDRESS:usize = 0xFF4B;

const OAM_DMA_REGISTER_ADDRESS:usize = 0xFF46;

// Palette registers
const BG_PALETTE_ADDRESS:usize = 0xFF47;
const OBJ_PALETTE1_ADDRESS:usize = 0xFF48;
const OBJ_PALETTE2_ADDRESS:usize = 0xFF49;

#[derive(Clone, Copy, PartialEq, Debug)]
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

#[derive(PartialEq, Debug)]
struct Palette{
    pub table: [u8;4],
    pub raw: u8,
}

impl Palette {
    /// Constructs a new blank palette.
    fn new() -> Palette{
        Palette{
            table: [0;4],
            raw: 0
        }
    }

    /// Updates the palette with new encoded data.
    /// 
    /// This will update both the raw and table values.
    fn update(&mut self, mut raw:u8){
        // Update the raw value
        self.raw = raw;
        for i in 0..4{
            self.table[i] = raw & 0b11;
            raw >>= 2;
        }
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
struct OamSprite{
    pub ypos: u8,
    pub xpos: u8,
    pub tile: u8,

    //These fields are all packed into a single byte.
    pub behind_background: bool,
    pub xflip: bool,
    pub yflip: bool,
    pub palette: bool,
}

impl OamSprite {
    const PALLET_ATTRIB_MASK:u8 = 0b0001_0000;
    const XFLIP_ATTRIB_MASK:u8 =  0b0010_0000;
    const YFLIP_ATTRIB_MASK:u8 =  0b0100_0000;
    const BG_PRIORITY_ATTRIB_MASK:u8 = 0b1000_0000;

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

    fn set_attrib_byte(&mut self, data:u8)
    {
        self.behind_background = data & OamSprite::BG_PRIORITY_ATTRIB_MASK != 0;
        self.xflip = data & OamSprite::XFLIP_ATTRIB_MASK != 0;
        self.yflip = data & OamSprite::YFLIP_ATTRIB_MASK != 0;
        self.palette = data & OamSprite::PALLET_ATTRIB_MASK != 0;
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
enum Mode{
    HBlank = 0,
    VBlank = 1,
    SpriteSearch = 2,
    LcdTransfer = 3
}

impl From<u8> for Mode {
    fn from(value: u8) -> Self {
        match value {
            0 => Mode::HBlank,
            1 => Mode::VBlank,
            2 => Mode::SpriteSearch,
            3 => Mode::LcdTransfer,
            _ => {panic!("Invalid LCD mode");}
        }
    }
}

#[derive(PartialEq, Debug)]
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

    // LCDC register
    lcdc: u8,
    lcd_enabled: bool,
    window_tiles_high: bool,    //True if window tiles are in the upper bank.
    window_enabled: bool,
    bg_window_signed_addressing: bool, // True if bg and window tiles are using the signed addressing method
    bg_tiles_high: bool,        // True if the window tiles are in the upper bank.
    obj_double_sprites: bool,   // True if sprites are 8x16, false if 8x8.
    obj_enabled: bool,
    bg_window_enable: bool,     // True if the background/window are enabled.

    // LCD status register
    lcds: u8,
    // interrupt sources
    line_compare_is: bool,
    mode2_is: bool,
    mode1_is: bool,
    mode0_is: bool, 
    line_compare: bool,
    mode: Mode,

    // scroll registers
    scroll_y: u8,
    scroll_x: u8,
    pub line_y: u8,
    line_compare_value: u8,
    window_y: u8,
    window_x: u8,

    // Pallet 
    bg_palette: Palette,
    obj_palette1: Palette,
    obj_palette2: Palette,

    // OAM DMA
    oam_dma_ticks: u8,
    oam_dma_src: u16,

    // Misc State tracking.
    tick_counter: u16,
    pub line_buffer: ScanlineBuffer,
    pub line_pending: bool
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

    const LCDS_LINE_CMP_IS_MASK: u8 = 1<<6;
    const LCDS_MODE2_IS_MASK: u8 =    1<<5;
    const LCDS_MODE1_IS_MASK: u8 =    1<<4;
    const LCDS_MODE0_IS_MASK: u8 =    1<<3;
    const LCDS_LINE_CMP_STATUS_MASK: u8 = 1 << 2;
    const LCDS_MODE_STATUS_MASK: u8 = 0b11;

    const OAM_DMA_TRANSFER_TICKS: u8 = 160; // In cpu ticks or "T" cycles.
    
    const LCD_TICKS_PER_LINE: u16 = 456;
    const LCD_LINE_VBLANK_START: u8 = 144;
    const LCD_LINE_VBLANK_END: u8 = 153;
    const LCD_WIDTH: u8 = 160;

    /// Checks if a DMA transfer is currently executing.
    fn dma_active(&self) -> bool{
        self.oam_dma_ticks != 0
    }

    fn update_dma(&mut self, ticks:u16, bus:&mut impl BusRW){
        // If there is a DMA transfer in progress
        if self.oam_dma_ticks > 0 {
            // If the transfer was just initiated.
            if self.oam_dma_ticks == PPU::OAM_DMA_TRANSFER_TICKS{
                self.dma_transfer(bus);
            }

            // Update the number of remaining DMA ticks.
            if self.oam_dma_ticks as u16 > ticks{
                self.oam_dma_ticks -= ticks as u8;
            } else {
                self.oam_dma_ticks = 0;
            }
        }
    }

    /// Computes the correct tileset index for a given map value.
    fn calc_tileset_index(&self, tiledata: u8) -> usize{
        if self.bg_window_signed_addressing {
            (TILESET_HIGH_BASE_INDEX + (tiledata as i8 as isize)) as usize
        } else {
            tiledata as usize
        }
    }

    /// Populates an array with sprite indicies that overlap the current line.
    /// Returns the number of sprites found.
    fn get_line_sprites(&self, sprites:&mut[u8;10]) -> usize{
        let mut found = 0;
        let height = if self.obj_double_sprites {TILE_DIMENSION*2} 
                     else {TILE_DIMENSION}
                     as i16;

        for (index, sprite) in self.sprites.iter().enumerate(){
            // Check for a Y collision.
            let diff = self.line_y as i16 - (sprite.ypos as i16 - 16);
            if diff >= 0 && diff < height{
                sprites[found] = index as u8;
                found += 1;
            }

            // Abort if we have found our limit of sprites
            if found >= 10{
                break;
            }
            
        }
        // indicate how many items were found.
        found
    }

    fn check_collision_sprite(&self, xpos:u8, sprite: &OamSprite) -> bool{
        if xpos >= 168{
            return false;
        }

        if sprite.xpos > xpos && sprite.xpos <= (xpos + TILE_DIMENSION as u8){
            return true;
        }

        false 
    }

    fn draw_line(&mut self) {
        // Used to hold pixel data.
        let mut pixel_block:u8 = 0;
        let mut sprite_pixel:u8;
        let mut bg_pixel = 0u8;
        let mut sprite_behind:bool;
        let mut bg_trans = false;

        ////BG rendering data////
        // Find the row of tiles the current line falls on.
        let tile_row = (((self.line_y as u16 + self.scroll_y as u16) as u8) / 8) as usize;
        let tile_pixel_y = ((self.line_y as u16 + self.scroll_y as u16) % 8) as u8;
        // Find the column of tiles the current line starts on.
        let tile_col = (self.scroll_x / 8) as usize;
        // The pixel within the tile.
        let mut tile_pixel_x = self.scroll_x % 8;
        // Find the tile index.
        let mut tile_index = tile_row * TILEMAP_DIMENSION + tile_col;
        // Use upper tile map if dictated by current configuration.
        if self.bg_tiles_high {
            tile_index += TILEMAP_ITEM_COUNT;
        }

        //// Window rendering data ////
        let window_tile_row = (self.line_y.wrapping_sub(self.window_y) / 8) as usize;
        let window_tile_pixel_y = self.line_y.wrapping_sub(self.window_y) % 8;

        ////Sprite data////
        let mut line_sprites = [0u8;10];
        let sprite_count = self.get_line_sprites(&mut line_sprites);
        let line_sprites = line_sprites;

        // For each pixel in the scanline
        for scanline_index in 0..PPU::LCD_WIDTH {
            if self.bg_window_enable{
                // If this is going to be a window pixel
                if self.window_enabled && 
                   self.window_y <= self.line_y && 
                   self.window_x <= scanline_index + 7 {
                    
                    // Calculate what map block we are in
                    let window_tile_col = ((scanline_index + 7 - self.window_x) / 8) as usize;
                    let window_tile_pixel_x = (scanline_index + 7 - self.window_x) % 8;
                    let mut window_tilemap_index = window_tile_row * TILEMAP_DIMENSION + window_tile_col;
                    if self.window_tiles_high{
                        window_tilemap_index += TILEMAP_ITEM_COUNT;
                    } 

                    // Translate the tilemap block into an index in the tile set.
                    let tileset_index = self.calc_tileset_index(self.tilemaps[window_tilemap_index]);
                    bg_pixel = self.tiles[tileset_index].read_pixel(window_tile_pixel_x, window_tile_pixel_y);

                    // Decide if the pixel is transparent, then get the pixel value from the palette.
                    bg_trans = bg_pixel == 0;
                    bg_pixel = self.bg_palette.table[bg_pixel as usize];
                }
                // Draw the background for this pixel.
                else {
                    // get the tileset index from the map
                    let tileset_index = self.calc_tileset_index(self.tilemaps[tile_index]);
                    bg_pixel = self.tiles[tileset_index].read_pixel(tile_pixel_x, tile_pixel_y);
                    bg_trans = bg_pixel == 0;
                    bg_pixel = self.bg_palette.table[bg_pixel as usize];

                    // If we have a tile pixel overflow
                    tile_pixel_x += 1;
                    if tile_pixel_x & 0b111 == 0 {
                        // Reset tile pixel to zero
                        tile_pixel_x = 0;

                        // Advance to the next tile, and check for overflow
                        tile_index += 1;
                        if tile_index & 0b11111 == 0 {
                            // println!("tile_index {}", tile_index); // DEBUG!
                            tile_index -= TILEMAP_DIMENSION;
                        }
                    }
                }
            }

            // If objects are enabled.
            sprite_behind = true;
            sprite_pixel = 4;
            if self.obj_enabled{
                for line_sprite in line_sprites.iter().take(sprite_count){
                    let sprt = &self.sprites[*line_sprite as usize];
                    if self.check_collision_sprite(scanline_index, sprt){
                        sprite_behind = sprt.behind_background;

                        let mut tile_x = (scanline_index + TILE_DIMENSION as u8) - sprt.xpos;
                        if sprt.xflip  {
                            tile_x = (TILE_DIMENSION-1) as u8 - tile_x;
                        }
                        let tile_x = tile_x; 

                        let mut tile = sprt.tile;
                        if self.obj_double_sprites{
                            tile &= 0xFE; // ignore lowest bit in double mode.
                        }
                        let mut tile_y = (self.line_y as i16 - (sprt.ypos as i16 - 16)) as u8;
                        if tile_y >= TILE_DIMENSION as u8 {
                            tile_y -= TILE_DIMENSION as u8;
                            if sprt.yflip == false {
                                tile += 1;
                            }
                        }
                        else if sprt.yflip && self.obj_double_sprites{
                            tile += 1;
                        }
                        if sprt.yflip{
                            tile_y = (TILE_DIMENSION-1) as u8 - tile_y;
                        }

                        let tile_y = tile_y;
                        let tile = tile;

                        // Get the sprite pixel
                        let pallet = if sprt.palette { &self.obj_palette2 } 
                                    else {&self.obj_palette1};
                        sprite_pixel = self.tiles[tile as usize].read_pixel(tile_x, tile_y);
                        // Zero is transparrent
                        if sprite_pixel == 0 {
                            sprite_pixel = 4;
                        } else {
                            sprite_pixel = pallet.table[sprite_pixel as usize];
                            break;
                        }

                    }
                    
                }
            }
            pixel_block >>= 2;
            let pixel =
                if sprite_behind && !bg_trans{
                    bg_pixel
                } 
                else if sprite_pixel != 4{
                    sprite_pixel
                }
                else {
                    bg_pixel
                };
            pixel_block |= pixel << 6;

            // If we have completed a pixel block
            if (scanline_index + 1) & 0b11 == 0{
                // update the scanline buffer.
                self.line_buffer.pixeldata[(scanline_index/4) as usize] = pixel_block;
            }
        }
        self.line_pending = true;
    }

    /// # Executes the specified number of clock ticks.
    pub fn execute_ticks(&mut self, ticks:u16, bus:&mut impl BusRW, is: &mut InterruptStatus){
        self.update_dma(ticks, bus);

        // TODO this is really, Really, REALLY wildly inacurate.
        if self.lcd_enabled{
            self.tick_counter += ticks;

            // If the line has expired
            if self.tick_counter >= PPU::LCD_TICKS_PER_LINE {

                // Draw the line if this is not VBLANK
                if self.line_y < PPU::LCD_LINE_VBLANK_START{
                    self.draw_line();
                }

                // correct the tick count and increment the line count.
                self.tick_counter -= PPU::LCD_TICKS_PER_LINE;
                self.line_y += 1;
                self.line_compare = self.line_compare_value == self.line_y;
                if self.line_compare && self.line_compare_is {
                    is.request_lcdstat();
                }

                // if start of vblank
                if self.line_y == PPU::LCD_LINE_VBLANK_START {
                    // Set the mode
                    self.mode = Mode::VBlank;

                    // Trigger interrupts
                    is.request_vblank();
                    if self.mode1_is {
                        is.request_lcdstat();
                    }
                }

                // start of new frame.
                if self.line_y > PPU::LCD_LINE_VBLANK_END {
                    self.line_y = 0;
                    self.line_compare = self.line_compare_value == self.line_y;
                    if self.line_compare && self.line_compare_is {
                        is.request_lcdstat();
                    }
                    self.mode = Mode::SpriteSearch;
                }
            }

            // If we are not in vblank
            if self.line_y < PPU::LCD_LINE_VBLANK_START {
                
                let new_mode = match self.tick_counter {
                    // Mode 2 - OAM_SCAN
                    0..=79 => {
                        Mode::SpriteSearch
                    }
                    // Mode 3 - Drawing Pixels
                    80..=251 => {
                        Mode::LcdTransfer
                    }
                    // Mode 0 - HBLANK
                    _ => {
                        Mode::HBlank
                    }
                };

                // If there was a mode change, set any interrupts.
                if new_mode != self.mode {
                    self.mode = new_mode;
                    match new_mode {
                        Mode::SpriteSearch => {
                            if self.mode2_is{
                                is.request_lcdstat();
                            }
                        }
                        Mode::HBlank => {
                            if self.mode0_is {
                                is.request_lcdstat();
                            }
                        }
                        _ => {}
                    }
                }
            }
        }
    }

    pub fn new() -> PPU {
        let blank_tile = Tile::new();
        let default_sprite = OamSprite::new();
        let mut ppu = PPU {
            tile_data: [0;TILESET_RAM],
            tiles: [blank_tile;TILESET_COUNT],
            tilemaps:[0;TILEMAPS_SIZE],
            sprites: [default_sprite;OAM_SPRITE_COUNT],
            sprite_data: [0;OAM_RAM_SIZE],
            lcdc: 0,
            lcd_enabled: false,
            obj_double_sprites: false,
            obj_enabled: false,
            bg_tiles_high: false,
            bg_window_enable: false,
            bg_window_signed_addressing: false,
            window_enabled: false,
            window_tiles_high: false,
            lcds: 0,
            scroll_y: 0,
            scroll_x: 0,
            line_y: 0,
            line_compare_value: 0,
            window_y: 0,
            window_x: 0,
            line_compare_is: false,
            mode2_is: false,
            mode1_is: false,
            mode0_is: false, 
            line_compare: false,
            mode: Mode::HBlank,
            bg_palette: Palette::new(),
            obj_palette1: Palette::new(),
            obj_palette2: Palette::new(),
            oam_dma_src: 0,
            oam_dma_ticks: 0,
            tick_counter: 0,
            line_buffer: ScanlineBuffer::new(),
            line_pending: false,
        };
        // Setup the screen into a post bootrom state.
        ppu.lcdc_write(
            PPU::LCDC_ENABLE_MASK | 
            PPU::LCDC_OBJ_SIZE_MASK | 
            PPU::LCDC_BG_WINDOW_PRIORITY_MASK
        );
        ppu
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

        // println!("tilewrite to {:#4X}: {:#2X}", addr, data);
    }

    fn sprite_write(&mut self, data:u8, addr:usize) {
        let index = (addr - OAM_START_ADDRESS) / OAM_SPRITE_SIZE;
        let field = addr & 0b11;

        // Update the sprite attributes
        match field {
            0 => {self.sprites[index].ypos = data;},
            1 => {self.sprites[index].xpos = data;},
            2 => {self.sprites[index].tile = data;},
            3 => {self.sprites[index].set_attrib_byte(data);},
            _ => {panic!("THIS IS A BUG! invalid sprite field write");}
        }

        // Save the raw sprite data.
        self.sprite_data[addr - OAM_START_ADDRESS] = data;
    }

    fn lcdc_write(&mut self, data:u8) {
        // Save the new LCDC value
        self.lcdc = data;
        
        self.lcd_enabled = data & PPU::LCDC_ENABLE_MASK != 0;
        self.window_tiles_high = data & PPU::LCDC_WINDOW_TILE_MAP_MASK != 0;
        self.window_enabled = data & PPU::LCDC_WINDOW_DISPLAY_ENABLE_MASK != 0;
        self.bg_window_signed_addressing = data & PPU::LCDC_BG_WINDOW_TILE_MAP_SELECT_MASK == 0;
        self.bg_tiles_high = data & PPU::LCDC_BG_TILE_MAP_SELECT_MASK != 0;
        self.obj_double_sprites = data & PPU::LCDC_OBJ_SIZE_MASK != 0;
        self.obj_enabled = data & PPU::LCDC_OBJ_DISPLAY_ENABLE_MASK != 0;
        self.bg_window_enable = data & PPU::LCDC_BG_WINDOW_PRIORITY_MASK != 0;

        // println!("lcd_enabled {}", self.lcd_enabled);
        // println!("window_tiles_high {}", self.window_tiles_high);
        // println!("window_enabled {}", self.window_enabled);
        // println!("bg_window_signed_addressing {}", self.bg_window_signed_addressing);
        // println!("bg_tiles_high {}", self.bg_tiles_high);
        // println!("obj_double_sprites {}", self.obj_double_sprites);
        // println!("obj_enabled {}", self.obj_enabled);
        // println!("bg_window_enable {}", self.bg_window_enable);
    }

    fn lcds_write(&mut self, data:u8) {
        // Only these bits are writable
        self.line_compare_is = data & PPU::LCDS_LINE_CMP_IS_MASK != 0;
        self.mode2_is = data & PPU::LCDS_MODE2_IS_MASK != 0;
        self.mode1_is = data & PPU::LCDS_MODE1_IS_MASK != 0;
        self.mode0_is = data & PPU::LCDS_MODE0_IS_MASK != 0;
        self.lcds = data & (
            PPU::LCDS_LINE_CMP_IS_MASK |
            PPU::LCDS_MODE2_IS_MASK | 
            PPU::LCDS_MODE1_IS_MASK |
            PPU::LCDS_MODE0_IS_MASK)
    }

    /// # Unpack an lcds byte into all fields.
    /// ## details
    /// 
    /// This allows a previously stored LCDS value to be unpacked. It is not
    /// accessable to the system, and was written to assist with serialization.
    fn lcds_unpack_raw(&mut self, data: u8)
    {
        self.line_compare_is = data & PPU::LCDS_LINE_CMP_IS_MASK != 0;
        self.mode2_is = data & PPU::LCDS_MODE2_IS_MASK != 0;
        self.mode1_is = data & PPU::LCDS_MODE1_IS_MASK != 0;
        self.mode0_is = data & PPU::LCDS_MODE0_IS_MASK != 0;
        self.line_compare = data & PPU::LCDS_LINE_CMP_STATUS_MASK != 0;
        self.mode = Mode::from(data & PPU::LCDS_MODE_STATUS_MASK);
    }

    fn lcds_read(&self) -> u8 {
        // Reassemble the LCDS value one bit at a time, starting with the msb.
        let mut value = 0;
        value |= self.line_compare_is as u8;
        value <<= 1;
        value |= self.mode2_is as u8;
        value <<= 1;
        value |= self.mode1_is as u8;
        value <<= 1;
        value |= self.mode0_is as u8;
        value <<= 1;
        value |= self.line_compare as u8;
        value <<= 2;
        value |= self.mode as u8;
        value
    }

    /// # Stage a DMA transfer
    /// 
    /// The actual transfer will not be executed until the next set of PPU
    /// updates.
    fn dma_start(&mut self, target: u8) {
        self.oam_dma_src = (target as u16) << 8;
        self.oam_dma_ticks = PPU::OAM_DMA_TRANSFER_TICKS;
    }

    /// #Executes the DMA memory transfer.
    /// 
    /// This is not done tick by tick, but in one large operation. It should 
    /// not have any negative effects, since the source area and target area
    /// will be blocked during the transfer.
    fn dma_transfer(&mut self, bus:&mut impl BusRW){
        let address = self.oam_dma_src as usize;
        for x in 0..OAM_RAM_SIZE{
            self.sprite_write(
                bus.bus_read8(address + x), 
                OAM_START_ADDRESS + x);
        }
    }

    /// # Serialize PPU state into a writer
    pub fn serialize<T>(& self, writer: &mut T) 
        where T : Write + ?Sized
    {
        // Raw tile data stored in the origial gameboy format.
        writer.write_all(&self.tile_data).unwrap();
        writer.write_all(&self.tilemaps).unwrap();
        writer.write_all(&self.sprite_data).unwrap();
        writer.write_u8(self.lcdc).unwrap();
        writer.write_u8(self.lcds_read()).unwrap();
        writer.write_u8(self.scroll_y).unwrap();
        writer.write_u8(self.scroll_x).unwrap();
        writer.write_u8(self.line_y).unwrap();
        writer.write_u8(self.line_compare_value).unwrap();
        writer.write_u8(self.window_y).unwrap();
        writer.write_u8(self.window_x).unwrap();
        writer.write_u8(self.bg_palette.raw).unwrap();
        writer.write_u8(self.obj_palette1.raw).unwrap();
        writer.write_u8(self.obj_palette2.raw).unwrap();

        // // OAM DMA
        // oam_dma_ticks: u8,
        writer.write_u8(self.oam_dma_ticks).unwrap();
        // oam_dma_src: u16,
        writer.write_u16::<LittleEndian>(self.oam_dma_src).unwrap();

        // // Misc State tracking.
        // tick_counter: u16,
        writer.write_u16::<LittleEndian>(self.tick_counter).unwrap();
        // pub line_buffer: ScanlineBuffer,
        writer.write_all(&self.line_buffer.pixeldata).unwrap();
        // pub line_pending: bool
        writer.write_u8(self.line_pending as u8).unwrap();
    }

    /// # Load ppu state from a reader
    pub fn deserialize<T>(&mut self, reader: &mut T)
        where T: Read + ?Sized
    {
        // /// Raw tile data stored in the origial gameboy format.
        // tile_data: [u8;TILESET_RAM],
        // /// Nicely broken out pixel verions of the raw tile data.
        // tiles: [Tile;TILESET_COUNT],
        reader.read_exact(&mut self.tile_data).unwrap();
        {
            let mut address_counter = TILESET_START_ADDRESS;
            for x in self.tile_data {
                self.tile_write(x, address_counter);
                address_counter += 1;
            }
        }

        // /// tilemap data.
        // tilemaps: [u8;TILEMAPS_SIZE],
        reader.read_exact(&mut self.tilemaps).unwrap();

        // /// Decoded sprite object data
        // sprites: [OamSprite; OAM_SPRITE_COUNT],
        // /// Raw OAM data.
        // sprite_data: [u8;OAM_RAM_SIZE],
        reader.read_exact(&mut self.sprite_data).unwrap();
        {
            let mut address_counter = OAM_START_ADDRESS;
            for x in self.sprite_data {
                self.sprite_write(x, address_counter);
                address_counter += 1;
            }
        }

        // // LCDC register
        // lcdc: u8,
        // lcd_enabled: bool,
        // window_tiles_high: bool,    //True if window tiles are in the upper bank.
        // window_enabled: bool,
        // bg_window_signed_addressing: bool, // True if bg and window tiles are using the signed addressing method
        // bg_tiles_high: bool,        // True if the window tiles are in the upper bank.
        // obj_double_sprites: bool,   // True if sprites are 8x16, false if 8x8.
        // obj_enabled: bool,
        // bg_window_enable: bool,     // True if the background/window are enabled.
        self.lcdc_write(
            reader.read_u8().unwrap()
        );
        
        // // LCD status register
        // lcds: u8,
        // // interrupt sources
        // line_compare_is: bool,
        // mode2_is: bool,
        // mode1_is: bool,
        // mode0_is: bool, 
        // line_compare: bool,
        // mode: Mode,
        {
            let lcds = reader.read_u8().unwrap();
            self.lcds_unpack_raw(lcds);
        }

        // // scroll registers
        // scroll_y: u8,
        self.scroll_y = reader.read_u8().unwrap();
        // scroll_x: u8,
        self.scroll_x = reader.read_u8().unwrap();
        // pub line_y: u8,
        self.line_y = reader.read_u8().unwrap();
        // line_compare_value: u8,
        self.line_compare_value = reader.read_u8().unwrap();
        // window_y: u8,
        self.window_y = reader.read_u8().unwrap();
        // window_x: u8,
        self.window_x = reader.read_u8().unwrap();

        // // Pallet 
        // bg_palette: Palette,
        self.bg_palette.update(
            reader.read_u8().unwrap()
        );
        // obj_palette1: Palette,
        self.obj_palette1.update(
            reader.read_u8().unwrap()
        );
        // obj_palette2: Palette,
        self.obj_palette2.update(
            reader.read_u8().unwrap()
        );

        // // OAM DMA
        // oam_dma_ticks: u8,
        self.oam_dma_ticks = reader.read_u8().unwrap();
        // oam_dma_src: u16,
        self.oam_dma_src = reader.read_u16::<LittleEndian>().unwrap();

        // // Misc State tracking.
        // tick_counter: u16,
        self.tick_counter = reader.read_u16::<LittleEndian>().unwrap();
        // pub line_buffer: ScanlineBuffer,
        reader.read_exact(&mut self.line_buffer.pixeldata).unwrap();
        // pub line_pending: bool
        self.line_pending = reader.read_u8().unwrap() != 0;
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
                self.tilemaps[addr-TILEMAP_START_ADDRESS]
            },

            // Object attribute memory read
            OAM_START_ADDRESS..=OAM_END_ADDRESS => {
                self.sprite_data[addr - OAM_START_ADDRESS]
            },

            // Individual registers
            LCDC_ADDRESS => {self.lcdc}
            LCDS_ADDRESS => {self.lcds_read()}
            SCY_ADDRESS => {self.scroll_y}
            SCX_ADDRESS => {self.scroll_x}
            LY_ADDRESS => {self.line_y}
            LYC_ADDRES => {self.line_compare_value}
            WY_ADDRESS => {self.window_y}
            WX_ADDRESS => {self.window_x}
            BG_PALETTE_ADDRESS => {self.bg_palette.raw}
            OBJ_PALETTE1_ADDRESS => {self.obj_palette1.raw}
            OBJ_PALETTE2_ADDRESS => {self.obj_palette2.raw}
            OAM_DMA_REGISTER_ADDRESS => {(self.oam_dma_src>>8) as u8}

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

            // Tile map write
            TILEMAP_START_ADDRESS..=TILEMAP_END_ADDRESS => {
                self.tilemaps[addr-TILEMAP_START_ADDRESS] = value;
            },

            // OAM memory write.
            OAM_START_ADDRESS..=OAM_END_ADDRESS => {
                self.sprite_write(value, addr);
            }

            // LCD control register
            LCDC_ADDRESS => {
                self.lcdc_write(value);
            }

            // LCD status register
            LCDS_ADDRESS => {
                self.lcds_write(value);
            }

            // Scroll and compare registers.
            SCY_ADDRESS => {self.scroll_y = value;}
            SCX_ADDRESS => {self.scroll_x = value;}
            LY_ADDRESS => {/*Dead Write*/}
            LYC_ADDRES => {self.line_compare_value = value;}
            WY_ADDRESS => {self.window_y = value;}
            WX_ADDRESS => {self.window_x = value;}
            BG_PALETTE_ADDRESS => {self.bg_palette.update(value);}
            OBJ_PALETTE1_ADDRESS => {self.obj_palette1.update(value);}
            OBJ_PALETTE2_ADDRESS => {self.obj_palette2.update(value);}
            OAM_DMA_REGISTER_ADDRESS => {self.dma_start(value);}

            // Unknown address.
            _ => {
                panic!("Unknown PPU write at address 0x{:4X}", addr);
            }
        }
    }
}

impl Default for PPU {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::ram::Ram;

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

    fn test_pack() -> (PPU, Ram, InterruptStatus){
        let ppu = PPU::new();
        let is = InterruptStatus::new();
        let ram = Ram::new(0x10000, 0);
        (ppu, ram, is)
    }

    impl PPU{
        fn run(&mut self, ticks:u16, bus: &mut impl BusRW, is: &mut InterruptStatus) {
            assert_eq!(ticks % 4, 0, "");
            for _ in 0..ticks/4 {
                self.execute_ticks(4, bus, is);
            }
        }
    }

    #[test]
    fn test_sprite_write() {
        let ref_sprite = OamSprite{
            ypos:1,
            xpos:2,
            tile:127,
            behind_background: true,
            yflip: false,
            xflip: true,
            palette: false
        };
        let ref_sprite_data = [1 as u8, 2, 127, 0xA0];

        // construct a ppu to test against
        let mut ppu = PPU::new();

        // Write the sprite data to the first and last sprite
        for (i, value) in ref_sprite_data.iter().enumerate() {
            ppu.bus_write8(OAM_START_ADDRESS + i, *value);
            ppu.bus_write8(OAM_END_ADDRESS - (OAM_SPRITE_SIZE-1) + i, *value);
        }

        assert_eq!(ref_sprite, ppu.sprites[0]);
        assert_eq!(ref_sprite, ppu.sprites[OAM_SPRITE_COUNT-1]);
        assert_eq!(ppu.sprite_data[0..OAM_SPRITE_SIZE], ref_sprite_data[..]);
        assert_eq!(ppu.sprite_data[(OAM_SPRITE_COUNT-1) * OAM_SPRITE_SIZE..], ref_sprite_data[..]);
    }

    #[test]
    fn test_lcdc_write() {
        let mut ppu = PPU::new();
        ppu.bus_write8(0xFF40, 0xAA);

        assert_eq!(ppu.bus_read8(LCDC_ADDRESS), 0xAA);
        assert_eq!(ppu.lcd_enabled, true);
        assert_eq!(ppu.window_tiles_high, false);
        assert_eq!(ppu.window_enabled, true);
        assert_eq!(ppu.bg_window_signed_addressing, true); // inverted.
        assert_eq!(ppu.bg_tiles_high, true);
        assert_eq!(ppu.obj_double_sprites, false);
        assert_eq!(ppu.obj_enabled, true);
        assert_eq!(ppu.bg_window_enable, false)
    }

    #[test]
    fn test_scy_rw() {
        let mut ppu = PPU::new();
        let value = 39;
        let address = 0xFF42;
        ppu.bus_write8(address, value);
        assert_eq!(ppu.scroll_y, value);
        assert_eq!(ppu.bus_read8(address), value);
    }

    #[test]
    fn test_scx_rw() {
        let mut ppu = PPU::new();
        let value = 84;
        let address = 0xFF43;
        ppu.bus_write8(address, value);
        assert_eq!(ppu.scroll_x, value);
        assert_eq!(ppu.bus_read8(address), value);
    }

    #[test]
    fn test_ly_write_dead() {
        // The line y register is read only, and should not change due to a write.
        let mut ppu = PPU::new();
        let value = 84;
        let address = 0xFF44;
        ppu.bus_write8(address, value);
        assert_eq!(ppu.line_y, 0);
        assert_eq!(ppu.bus_read8(address), 0);
    }

    #[test]
    #[ignore]
    fn test_ly_read(){
        // TODO - this must return the current line once rendering functionality is in place.
        assert_eq!(1,2);
    }

    #[test]
    fn test_lyc_rw() {
        let mut ppu = PPU::new();
        let value = 255;
        let address = 0xFF45;
        ppu.bus_write8(address, value);
        assert_eq!(ppu.line_compare_value, value);
        assert_eq!(ppu.bus_read8(address), value);
    }

    #[test]
    fn test_wy_rw() {
        let mut ppu = PPU::new();
        let value = 43;
        let address = 0xFF4A;
        ppu.bus_write8(address, value);
        assert_eq!(ppu.window_y, value);
        assert_eq!(ppu.bus_read8(address), value);
    }

    #[test]
    fn test_wx_rw() {
        let mut ppu = PPU::new();
        let value = 43;
        let address = 0xFF4B;
        ppu.bus_write8(address, value);
        assert_eq!(ppu.window_x, value);
        assert_eq!(ppu.bus_read8(address), value);
    }

    #[test]
    fn test_lcds_write() {
        let mut ppu = PPU::new();
        let value = 0xFF;
        let address = 0xFF41;
        ppu.bus_write8(address, value);
        assert_eq!(ppu.bus_read8(address), 0x78);
        assert_eq!(ppu.line_compare_is, true);
        assert_eq!(ppu.mode2_is, true);
        assert_eq!(ppu.mode1_is, true);
        assert_eq!(ppu.mode0_is, true);

        ppu.bus_write8(address, 0b0010_1000);
        assert_eq!(ppu.mode0_is, true);
        assert_eq!(ppu.mode1_is, false);
        assert_eq!(ppu.mode2_is, true);
        assert_eq!(ppu.line_compare_is, false);
    }

    #[test]
    fn test_bg_palette_rw(){
        let address = 0xFF47;
        let mut ppu = PPU::new();
        let raw_value = 0b_11_10_01_00;
        let expected_table = [0,1,2,3];

        // Code under test
        ppu.bus_write8(address, raw_value);

        assert_eq!(ppu.bus_read8(address), raw_value);
        assert_eq!(ppu.bg_palette.raw, raw_value);
        assert_eq!(ppu.bg_palette.table, expected_table);
    }

    #[test]
    fn test_obj_palette1_rw(){
        let address = 0xFF48;
        let mut ppu = PPU::new();
        let raw_value = 0b_11_10_01_00;
        let expected_table = [0,1,2,3];

        // Code under test
        ppu.bus_write8(address, raw_value);

        // Postconditions
        assert_eq!(ppu.bus_read8(address), raw_value);
        assert_eq!(ppu.obj_palette1.raw, raw_value);
        assert_eq!(ppu.obj_palette1.table, expected_table);
    }

    #[test]
    fn test_obj_palette2_rw(){
        let address = 0xFF49;
        let mut ppu = PPU::new();
        let raw_value = 0b_11_10_01_00;
        let expected_table = [0,1,2,3];

        // Code under test
        ppu.bus_write8(address, raw_value);

        // Postconditions
        assert_eq!(ppu.bus_read8(address), raw_value);
        assert_eq!(ppu.obj_palette2.raw, raw_value);
        assert_eq!(ppu.obj_palette2.table, expected_table);
    }

    #[test]
    fn test_dma_transfer_start_ticks_and_addr() {
        let mut ppu = PPU::new();
        let address = 0xFF46;
        let value = 45;
        let transfer_address = 45 * 0x100;

        ppu.bus_write8(address, 45);

        assert_eq!(ppu.bus_read8(address), value);
        assert_eq!(ppu.oam_dma_src, transfer_address);
        assert_eq!(ppu.oam_dma_ticks, 160);
    }

    #[test]
    fn test_dma_transfer_ticks_down() {
        // Given a PPU with a staged DMA transfer
        let (mut ppu, mut ram, mut is) = test_pack();

        // let mut ppu = PPU::new();
        // let mut ram = Ram::new(160, 0);
        ppu.bus_write8(OAM_DMA_REGISTER_ADDRESS, 0);

        // When 7 ticks are executed
        ppu.execute_ticks(7, &mut ram, &mut is);

        // Then the ticks count down by the number of executed ticks
        assert_eq!(ppu.oam_dma_ticks, 153);

        // When more ticks are executed than remain on the dma transfer
        ppu.execute_ticks(154, &mut ram, &mut is);

        // Then the ticks will not underflow.
        assert_eq!(ppu.oam_dma_ticks, 0);
    }

    #[test]
    fn test_dma_transfer_moves_data()
    {
        // Given a PPU with a staged DMA transfer, and some initialized ram
        let mut ppu = PPU::new();
        let mut ram = Ram::new(1024, 0);
        let mut is = InterruptStatus::new();
        for x in 256..(256+OAM_RAM_SIZE){
            ram.bus_write8(x, x as u8);
        }
        ppu.bus_write8(OAM_DMA_REGISTER_ADDRESS, 1);

        // When the ppu executes ticks
        ppu.execute_ticks(1, &mut ram, &mut is);

        // Then the OAM memory must contain the new data from the transfer source.
        for x in 0..OAM_RAM_SIZE {
            assert_eq!(ppu.sprite_data[x], x as u8);
        }
    }

    #[test]
    #[ignore]
    fn test_dma_memory_lock() {
        panic!("test_dma_memory_lock is not implemented");
    }

    #[test]
    fn test_tilemap_read_write() {
        let mut ppu = PPU::new();
        let tile_start = 0x9800;
        let tile_end = 0x9FFF;
        let start_value = 0xFC;
        let end_value = 0xFD;

        ppu.bus_write8(tile_start, start_value);
        ppu.bus_write8(tile_end, end_value);
        assert_eq!(ppu.bus_read8(tile_start), start_value);
        assert_eq!(ppu.bus_read8(tile_end), end_value);
        assert_eq!(ppu.tilemaps[0], start_value);
        assert_eq!(ppu.tilemaps[2047], end_value);
    }

    #[test]
    fn test_cycles_through_drawing_modes() {
        // Currently very flawed. Does not account for different timing within a line.
        let (mut ppu, mut ram, mut is) = test_pack();
        ppu.lcd_enabled = true;

        ppu.run(4, &mut ram, &mut is);
        assert_eq!(ppu.mode, Mode::SpriteSearch);
        ppu.run(72, &mut ram, &mut is);
        assert_eq!(ppu.mode, Mode::SpriteSearch);

        ppu.run(4, &mut ram, &mut is);
        assert_eq!(ppu.mode, Mode::LcdTransfer);
        ppu.run(168, &mut ram, &mut is);
        assert_eq!(ppu.mode, Mode::LcdTransfer);

        ppu.run(4, &mut ram, &mut is);
        assert_eq!(ppu.mode, Mode::HBlank);
        ppu.run(200, &mut ram, &mut is);
        assert_eq!(ppu.mode, Mode::HBlank);

        ppu.run(4, &mut ram, &mut is);
        assert_eq!(ppu.mode, Mode::SpriteSearch);
    }

    #[test]
    #[ignore]
    fn test_cycles_more_accurate_modes_for_real(){
        assert!(false);
    }

    #[test]
    fn test_ycomp_stat_interrupt() {
        let (mut ppu, mut ram, mut is) = test_pack();
        is.isrmask = 0xFF;
        ppu.lcd_enabled = true;
        ppu.line_compare_is = true;
        ppu.line_compare_value = 1;

        // Line zero, no interrupt.
        ppu.run(452, &mut ram, &mut is);
        assert_eq!(ppu.line_y, 0);
        assert_eq!(is.is_lcdstat_active(), false);

        // Line 1, interrupt on first tick.
        ppu.run(4, &mut ram, &mut is);
        assert_eq!(ppu.line_y, 1);
        assert_eq!(is.is_lcdstat_active(), true);

        // Line 1, No interrupt on subsequent tick.
        is.clear_lcdstat();
        ppu.run(4, &mut ram, &mut is);
        assert_eq!(is.is_lcdstat_active(), false);
        assert_eq!(ppu.line_y, 1);
    }

    #[test]
    fn test_ycomp_stat_interrupt_zero_case() {
        let (mut ppu, mut ram, mut is) = test_pack();
        is.isrmask = 0xFF;
        ppu.lcd_enabled = true;
        ppu.line_compare_is = false;
        ppu.line_compare_value = 0;

        // Execute 153 lines of drawing
        for _ in 0..153{
            ppu.run(456, &mut ram, &mut is);
        }

        // Line 153, no interrupt
        ppu.line_compare_is = true;
        assert_eq!(ppu.line_y, 153);
        assert_eq!(is.is_lcdstat_active(), false);

        // Line 0, interrupt
        ppu.run(456, &mut ram, &mut is);
        assert_eq!(ppu.line_y, 0);
        assert_eq!(is.is_lcdstat_active(), true);
    }

    #[test]
    fn test_hblank_stat_interrupt() {
        // This is not accurate, since it does not account for variable line timing.
        let (mut ppu, mut ram, mut is) = test_pack();
        is.isrmask = 0xFF;
        ppu.lcd_enabled = true;
        ppu.mode0_is = true;

        // No interrupt yet.
        ppu.run(80 + 172 - 4, &mut ram, &mut is);
        assert_eq!(is.is_lcdstat_active(), false);

        // Transition in to HBLANK, interrupt asserts
        ppu.run(4, &mut ram, &mut is);
        assert_eq!(is.is_lcdstat_active(), true);
        assert_eq!(ppu.mode, Mode::HBlank);

        // Continue in HBLANK, interrupt is not asserted anymore.
        is.clear_lcdstat();
        ppu.run(4, &mut ram, &mut is);
        assert_eq!(is.is_lcdstat_active(), false);
    }

    #[test]
    fn test_vblank_interrupts() {
        let (mut ppu, mut ram, mut is) = test_pack();

        is.isrmask = 0xFF;
        ppu.lcd_enabled = true;
        ppu.mode1_is = true;

        // Get right to the edge of vblank
        for _ in 0..143 {
            ppu.run(456, &mut ram, &mut is);
        }
        assert_eq!(ppu.line_y, 143);
        assert_eq!(is.is_vblank_active(), false);
        assert_eq!(is.is_lcdstat_active(), false);

        // Draw the line to trigger vblank.
        ppu.run(456, &mut ram, &mut is);
        assert_eq!(is.is_vblank_active(), true);
        assert_eq!(is.is_lcdstat_active(), true);

        // Draw another line. No more interrupts should activate.
        is.clear_vblank();
        is.clear_lcdstat();
        ppu.run(456, &mut ram, &mut is);
        assert_eq!(is.is_vblank_active(), false);
        assert_eq!(is.is_lcdstat_active(), false);
    }

    #[test]
    fn test_basic_background_render(){
        let (mut ppu, mut ram, mut is) = test_pack();
        let tile_data = [0x7C, 0x7C, 0x00, 0xC6, 0xC6, 0x00, 0x00, 0xFE, 0xC6, 0xC6, 0x00, 0xC6, 0xC6, 0x00, 0x00, 0x00];
        // What the given tile looks like
        // let expected_tile = Tile{
        //     pixel: [
        //         [0, 3, 3, 3, 3, 3, 0, 0],
        //         [2, 2, 0, 0, 0, 2, 2, 0],
        //         [1, 1, 0, 0, 0, 1, 1, 0],
        //         [2, 2, 2, 2, 2, 2, 2, 0],
        //         [3, 3, 0, 0, 0, 3, 3, 0],
        //         [2, 2, 0, 0, 0, 2, 2, 0],
        //         [1, 1, 0, 0, 0, 1, 1, 0],
        //         [0, 0, 0, 0, 0, 0, 0, 0],
        //     ]
        // };
        ppu.bg_window_enable = true;
        ppu.obj_enabled = false;
        ppu.window_enabled = false;
        ppu.bg_window_signed_addressing = false;
        ppu.lcd_enabled = true;

        // write the pallet data. Should invert colors
        ppu.bus_write8(BG_PALETTE_ADDRESS, 0b0001_1011);

        // Copy in the tile data into the first tile.
        for (i, x) in tile_data.iter().enumerate() {
            ppu.bus_write8(TILESET_START_ADDRESS + i, *x);
        }

        // Render a line.
        ppu.run(460, &mut ram, &mut is);
        assert_eq!(ppu.line_y, 1);

        assert_eq!(ppu.line_pending, true);
        assert_eq!(ppu.line_buffer.pixeldata[0], 0b00_00_00_11);
        assert_eq!(ppu.line_buffer.pixeldata[1], 0b11_11_00_00);
    }

    #[test]
    fn test_basic_background_scroll_render(){
        let (mut ppu, mut ram, mut is) = test_pack();
        let tile_data = [0x7C, 0x7C, 0x00, 0xC6, 0xC6, 0x00, 0x00, 0xFE, 0xC6, 0xC6, 0x00, 0xC6, 0xC6, 0x00, 0x00, 0x00];
        // What the given tile looks like
        // let expected_tile = Tile{
        //     pixel: [
        //         0, 3, 3, 3,   3, 3, 0, 0,
        //         2, 2, 0, 0,   0, 2, 2, 0,
        //         1, 1, 0, 0,   0, 1, 1, 0,
        //         2, 2, 2, 2,   2, 2, 2, 0,
        //         3, 3, 0, 0,   0, 3, 3, 0,
        //         2, 2, 0, 0,   0, 2, 2, 0,
        //         1, 1, 0, 0,   0, 1, 1, 0,
        //         0, 0, 0, 0,   0, 0, 0, 0,
        //     ]
        // };
        ppu.lcd_enabled = true;
        ppu.bg_window_enable = true;
        ppu.obj_enabled = false;
        ppu.bg_window_signed_addressing = false;
        ppu.window_enabled = false;
        
        // write the pallet data. Should invert colors
        ppu.bus_write8(BG_PALETTE_ADDRESS, 0b1110_0100);

        // Scroll 1 pixel down and 4 pixels right
        ppu.bus_write8(SCY_ADDRESS, 1);
        ppu.bus_write8(SCX_ADDRESS, 4);

        // Copy in the tile data into the first tile.
        for (i, x) in tile_data.iter().enumerate() {
            ppu.bus_write8(TILESET_START_ADDRESS + i, *x);
        }

        // Render a line.
        ppu.run(460, &mut ram, &mut is);

        assert_eq!(ppu.line_pending, true);
        //         0, 2, 2, 0,   2, 2, 0, 0,   
        // println!("Pixel 0 to 3: {:#2X}", ppu.line_buffer.pixeldata[0]);
        // println!("Pixel 4 to 7: {:#2X}", ppu.line_buffer.pixeldata[1]);
        assert_eq!(ppu.line_buffer.pixeldata[0], 0b00_10_10_00);
        assert_eq!(ppu.line_buffer.pixeldata[1], 0b00_00_10_10);
    }

    #[test]
    fn test_background_signed_addressing(){
        let (mut ppu, mut ram, mut is) = test_pack();
        let tile_data = [0x7C, 0x7C, 0x00, 0xC6, 0xC6, 0x00, 0x00, 0xFE, 0xC6, 0xC6, 0x00, 0xC6, 0xC6, 0x00, 0x00, 0x00];
        // What the given tile looks like
        // let expected_tile = Tile{
        //     pixel: [
        //         0, 3, 3, 3,   3, 3, 0, 0,
        //         2, 2, 0, 0,   0, 2, 2, 0,
        //         1, 1, 0, 0,   0, 1, 1, 0,
        //         2, 2, 2, 2,   2, 2, 2, 0,
        //         3, 3, 0, 0,   0, 3, 3, 0,
        //         2, 2, 0, 0,   0, 2, 2, 0,
        //         1, 1, 0, 0,   0, 1, 1, 0,
        //         0, 0, 0, 0,   0, 0, 0, 0,
        //     ]
        // };
        ppu.bg_window_enable = true;
        ppu.lcd_enabled = true;
        ppu.bg_window_signed_addressing = true;
        
        // Set BG pallet.
        ppu.bus_write8(BG_PALETTE_ADDRESS, 0b1110_0100);

        // Copy in the tile data into the tile just under 0x8800
        for (i, x) in tile_data.iter().enumerate() {
            ppu.bus_write8(TILESET_START_ADDRESS + (TILE_SIZE * (256 + 99)) + i, *x);
        }

        // Set the tile map to use the correct tile
        for x in 0..TILEMAP_ITEM_COUNT {
            ppu.bus_write8(TILEMAP_A_START_ADDRESS + x, 99);
        }

        // Render a line.
        for _ in 0..154 {
            ppu.run(456, &mut ram, &mut is);
        }
        ppu.line_pending = false;
        for _ in 0..128 { 
            ppu.run(456, &mut ram, &mut is);
        }
        ppu.run(456, &mut ram, &mut is);
        ppu.run(456, &mut ram, &mut is);
        ppu.run(456, &mut ram, &mut is);

        assert_eq!(ppu.line_pending, true);
        assert_eq!(ppu.line_y, 131);
        //         0, 3, 3, 3,   3, 3, 0, 0,
        // println!("Pixel 0 to 3: {:#2X}", ppu.line_buffer.pixeldata[0]);
        // println!("Pixel 4 to 7: {:#2X}", ppu.line_buffer.pixeldata[1]);
        assert_eq!(ppu.line_buffer.pixeldata[0], 0x05);
        assert_eq!(ppu.line_buffer.pixeldata[1], 0x14);
        assert_eq!(ppu.line_buffer.pixeldata[2], 0x05);
        assert_eq!(ppu.line_buffer.pixeldata[3], 0x14);
    }

    #[test]
    fn test_sprite_draw_basecase(){
        let (mut ppu, mut ram, mut is) = test_pack();
        let tile_data = [0x7C, 0x7C, 0x00, 0xC6, 0xC6, 0x00, 0x00, 0xFE, 0xC6, 0xC6, 0x00, 0xC6, 0xC6, 0x00, 0x00, 0x00];
        // What the given tile looks like
        // let expected_tile = Tile{
        //     pixel: [
        //         0, 3, 3, 3,   3, 3, 0, 0,
        //         2, 2, 0, 0,   0, 2, 2, 0,
        //         1, 1, 0, 0,   0, 1, 1, 0,
        //         2, 2, 2, 2,   2, 2, 2, 0,
        //         3, 3, 0, 0,   0, 3, 3, 0,
        //         2, 2, 0, 0,   0, 2, 2, 0,
        //         1, 1, 0, 0,   0, 1, 1, 0,
        //         0, 0, 0, 0,   0, 0, 0, 0,
        //     ]
        // };
        ppu.lcd_enabled = true;
        ppu.obj_enabled = true;
        ppu.bg_window_enable = false;

        // Setup a palette
        ppu.bus_write8(OBJ_PALETTE1_ADDRESS, 0b1110_0100);
        // Copy in the tile data into the tile just under 0x8800
        for (i, x) in tile_data.iter().enumerate() {
            ppu.bus_write8(TILESET_START_ADDRESS + i , *x);
        }
        assert_eq!(ppu.tiles[0].read_pixel(1,0), 3);

        let sprite_data = [
            16, // Y
            8,  // X
            0,  // Tile
            0   // Attributes
        ];

        for (i, x) in sprite_data.iter().enumerate() {
            ppu.bus_write8(OAM_START_ADDRESS + i, *x);
        }
        println!("Sprite 0: {:?}", ppu.sprites[0]);
        assert_eq!(ppu.sprites[0].ypos, 16);

        // Render a line.
        ppu.run(456, &mut ram, &mut is);
        println!("line buffer: {:?}", ppu.line_buffer.pixeldata);
        assert_eq!(ppu.line_pending, true);
        assert_eq!(ppu.line_y, 1);
        //         0, 3, 3, 3,   3, 3, 0, 0,
        assert_eq!(ppu.line_buffer.pixeldata[0], 0b1111_1100);
        assert_eq!(ppu.line_buffer.pixeldata[1], 0b0000_1111);

        ppu.run(456, &mut ram, &mut is);
        println!("line buffer: {:?}", ppu.line_buffer.pixeldata);
        //         2, 2, 0, 0,   0, 2, 2, 0,
        assert_eq!(ppu.line_buffer.pixeldata[0], 0x0A);
        assert_eq!(ppu.line_buffer.pixeldata[1], 0x28);
    }

    #[test]
    fn test_oam_yscan_8x16(){
        let (mut ppu, _ram, _is) = test_pack();
        ppu.obj_double_sprites = true;
        ppu.sprites[10].ypos = 1;
        ppu.sprites[12].ypos = 16;
        let mut sprites_list = [0u8;10];

        let count = ppu.get_line_sprites(&mut sprites_list);

        assert_eq!(count, 2);
        assert_eq!(sprites_list[0], 10);
        assert_eq!(sprites_list[1], 12);
    }

    #[test]
    fn test_oam_yscan_8x8(){
        let (mut ppu, mut _ram, mut _is) = test_pack();
        ppu.obj_double_sprites = false;
        ppu.sprites[10].ypos = 1;
        ppu.sprites[12].ypos = 8;
        ppu.sprites[13].ypos = 9;
        let mut sprites_list = [0u8;10];

        let count = ppu.get_line_sprites(&mut sprites_list);

        assert_eq!(count, 1);
        assert_eq!(sprites_list[0], 13);
    }

    #[test]
    fn test_serialize_deserialize_loop()
    {
        // // // Setup a ppu to serialize // // //
        let mut ppu_src = PPU::new();

        // Populate tileset data with something.
        for (offset, address) in (TILESET_START_ADDRESS..=TILESET_END_ADDRESS).enumerate() {
            ppu_src.bus_write8(address, offset as u8);
        }

        // Populate tilemap data with something.
        for (offset, address) in (TILEMAP_START_ADDRESS..=TILEMAP_END_ADDRESS).enumerate() {
            ppu_src.bus_write8(address, offset as u8);
        }

        // Write LCDC data
        ppu_src.lcdc_write(0xAA);

        // Write LCDS data
        ppu_src.lcds_unpack_raw(0x39);

        // Palettes
        ppu_src.bg_palette.update(0x81);
        ppu_src.obj_palette1.update(0x7E);
        ppu_src.obj_palette1.update(0x55);

        // Sprites
        for (offset, address) in (OAM_START_ADDRESS..=OAM_END_ADDRESS).enumerate() {
            ppu_src.bus_write8(address, offset as u8);
        }

        // scroll registers
        ppu_src.scroll_y = 23;
        ppu_src.scroll_x = 185;
        ppu_src.line_y = 3;
        ppu_src.line_compare_value = 92;
        ppu_src.window_y = 193;
        ppu_src.window_x = 7;

        // OAM DMA
        ppu_src.oam_dma_ticks = 231;
        ppu_src.oam_dma_src = 0xABCD;

        // Misc State tracking.
        ppu_src.tick_counter = 0xDEAF;
        ppu_src.line_pending = true;
        
        // Populate line buffer
        for (offset, value) in ppu_src.line_buffer.pixeldata.iter_mut().enumerate()
        {
            *value = offset as u8;
        }

        // // // Serialize and Deserialize // // //
        let mut ppu_dst = PPU::new();
        let mut buffer = [0u8; 1024*16];
        {
            let mut writer = &mut buffer[..];
            ppu_src.serialize(&mut writer);
        }
        {
            let mut reader = &buffer[..];
            ppu_dst.deserialize(&mut reader);
        }


        // // // Check that both PPUs are equal // // // 
        // assert_eq!(ppu_src.tile_data, ppu_dst.tile_data);
        // assert_eq!(ppu_src.tiles, ppu_dst.tiles);

        // assert_eq!(ppu_src.tilemaps, ppu_dst.tilemaps);

        // assert_eq!(ppu_src.lcdc, ppu_dst.lcdc);
        // assert_eq!(ppu_src.lcd_enabled, ppu_dst.lcd_enabled);
        // assert_eq!(ppu_src.window_tiles_high, ppu_dst.window_tiles_high);
        // assert_eq!(ppu_src.window_enabled, ppu_dst.window_enabled);
        // assert_eq!(ppu_src.bg_window_signed_addressing, ppu_dst.bg_window_signed_addressing);
        // assert_eq!(ppu_src.bg_tiles_high, ppu_dst.bg_tiles_high);
        // assert_eq!(ppu_src.obj_double_sprites, ppu_dst.obj_double_sprites);
        // assert_eq!(ppu_src.obj_enabled, ppu_dst.obj_enabled);
        // assert_eq!(ppu_src.bg_window_enable, ppu_dst.bg_window_enable);

        assert_eq!(ppu_src, ppu_dst);

    }
}
