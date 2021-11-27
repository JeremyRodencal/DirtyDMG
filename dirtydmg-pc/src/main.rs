use std::io::{ErrorKind};
use std::io::Read;
use std::fs;
use std::time::{Instant};

extern crate sdl2;

use sdl2::pixels::Color;
use sdl2::render::{Canvas, RenderTarget};
use sdl2::event::Event;
use sdl2::rect::Point;
use sdl2::keyboard::Keycode;


use dirtydmg_core::dmg::Dmg;
use dirtydmg_core::interface::ScanlineBuffer;

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

fn draw_line<T>(canvas: &mut Canvas<T>, y: u8, buffer: &ScanlineBuffer)
    where T: RenderTarget {

    let colors: [Color;4] = [
        Color{r: 255, g:255, b:255, a: 0},
        Color{r: 170, g:170, b:170, a: 0},
        Color{r: 85,  g:85,  b: 85, a: 0},
        Color{r: 0,   g:0,   b: 0,  a: 0},
    ];

    let mut x = 0;
    for pixelpack in buffer.pixeldata{
        for sub_pixel in 0..4{
            canvas.set_draw_color(colors[((pixelpack >> (2 * sub_pixel))&0b11) as usize]);
            canvas.draw_point(Point::new(x, y as i32)).unwrap();
            x += 1
        }
    }
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

    // SDL stuff
    let context = sdl2::init().unwrap();
    let video = context.video().unwrap();

    let window = video.window("Dirty DMG", 160, 144)
        .position_centered()
        .build()
        .unwrap();
    let mut event_pump = context.event_pump().unwrap();
    let mut canvas = window.into_canvas().build().unwrap();
    canvas.clear();
    canvas.present();

    let mut framecount = 0;
    let mut timer = Instant::now();
    loop {
        dmg.update();

        // if there is a pending line to draw
        if dmg.ppu.as_ref().borrow().line_pending {
            let y = dmg.ppu.as_ref().borrow().line_y;
            dmg.ppu.as_ref().borrow_mut().line_pending = false;
            draw_line(&mut canvas, y, &dmg.ppu.as_ref().borrow().line_buffer);
            if y == 143 {
                canvas.present();
                framecount += 1;
                if framecount == 100 {
                    println!("100 frames in {} seconds for {} fps", timer.elapsed().as_secs_f32(), 100f64/timer.elapsed().as_secs_f64());
                    timer = Instant::now();
                    framecount = 0;
                }
            }
        }

        for event in event_pump.poll_iter() {
            match event {
                Event::Quit {..} |
                Event::KeyDown { keycode: Some(Keycode::Escape), .. } => { },
                _ => {}
            }
        }
    }
}
