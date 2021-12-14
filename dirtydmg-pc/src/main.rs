use std::io::{ErrorKind};
use std::io::Read;
use std::fs;
use std::time::{Instant, Duration};

extern crate sdl2;

use sdl2::pixels::Color;
use sdl2::render::{Canvas, RenderTarget};
use sdl2::event::Event;
use sdl2::rect::{Point, Rect};
use sdl2::keyboard::Keycode;
use sdl2::audio::{AudioSpecDesired, AudioQueue};
use sdl2::joystick::*;

use dirtydmg_core::dmg::Dmg;
use dirtydmg_core::input::Button;
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

fn draw_line<T>(canvas: &mut Canvas<T>, y: u8, buffer: &ScanlineBuffer, scale: u32)
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
            // canvas.draw_point(Point::new(x, y as i32)).unwrap();
            canvas.draw_rect(Rect::new(
                x * scale as i32, 
                y as i32 * scale as i32, 
                scale, scale)).unwrap();
            x += 1
        }
    }
}

fn terrible_input_proc(dmg: &mut Dmg, key:Keycode, pressed:bool){
    let btn = match key {
        Keycode::Z => {Some(Button::B)}
        Keycode::X => {Some(Button::A)}
        Keycode::Return | Keycode::Return2 => {Some(Button::Start)}
        Keycode::C => {Some(Button::Select)}
        Keycode::Up => {Some(Button::Up)}
        Keycode::Down => {Some(Button::Down)}
        Keycode::Left => {Some(Button::Left)}
        Keycode::Right => {Some(Button::Right)}
        _ => None
    };
    if let Some(b) = btn {
        dmg.input(b, pressed);
    }
}

fn terrible_joypad_btn_proc(dmg: &mut Dmg, _which:u32, btn_idx:u8, down:bool){
    println!("btn {}", btn_idx);
    let btn = match btn_idx {
        0 => {Some(Button::A)},
        2 => {Some(Button::B)},
        6 => {Some(Button::Start)},
        4 => {Some(Button::Select)},
        11 => {Some(Button::Up)}
        12 => {Some(Button::Down)}
        13 => {Some(Button::Left)}
        14 => {Some(Button::Right)}
        _ => None
    };
    if let Some(b) = btn {
        dmg.input(b, down);
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

    let scale = 2;

    // SDL stuff
    let context = sdl2::init().unwrap();
    let video = context.video().unwrap();
    let audio = context.audio().unwrap();
    let joystick = context.joystick().unwrap();

    let mut joy:Option<Joystick> = None;
    if let Ok(x) = joystick.num_joysticks(){
        println!("Found {} joysticks", x);
        if x > 0 {
            if let Ok(j) = joystick.open(0){
                println!("Openned joystick {}", 0);
                joy = Some(j);
            }
        }
    }

    let desired_spec = AudioSpecDesired {
        freq: Some(44100),
        channels: Some(1),  // mono
        samples: Some(512)       // default sample size
    };
    let audio_queue:AudioQueue<i8> = audio.open_queue(
        None,
        &desired_spec).unwrap();
    println!("Frequency = {}", audio_queue.spec().freq);
    println!("Buffer size = {}", audio_queue.spec().samples);
    audio_queue.resume();

    let window = video.window("Dirty DMG", 160 * scale, 144 * scale)
        .position_centered()
        .build()
        .unwrap();
    let mut event_pump = context.event_pump().unwrap();
    let mut canvas = window.into_canvas().build().unwrap();
    canvas.clear();
    canvas.present();

    let mut framecount = 0;
    let mut timer = Instant::now();
    let mut quit = false;
    let mut start_instant = Instant::now();
    let frametime = Duration::new(0, 15466666);
    // let frametime = Duration::new(0, 14166666);
    let mut buf = Vec::<i8>::new();
    loop {
        while audio_queue.size() > 512 {
            std::thread::sleep_ms(1);
        }
        dmg.update();

        if let Some((left, _right)) = dmg.apu.as_ref().borrow_mut().get_sample() {
            buf.push(left);
            if buf.len() >= 128{
                audio_queue.queue(&buf);
                buf.clear();
            }
            
            // println!("queued audio");
        }

        // if there is a pending line to draw
        if dmg.ppu.as_ref().borrow().line_pending {
            let y = dmg.ppu.as_ref().borrow().line_y;
            dmg.ppu.as_ref().borrow_mut().line_pending = false;
            draw_line(&mut canvas, y, &dmg.ppu.as_ref().borrow().line_buffer, scale);
            if y == 143 {
                canvas.present();
                framecount += 1;
                if framecount == 100 {
                    println!("100 frames in {} seconds for {} fps", timer.elapsed().as_secs_f32(), 100f64/timer.elapsed().as_secs_f64());
                    timer = Instant::now();
                    framecount = 0;
                }
                for event in event_pump.poll_iter() {
                    match event {
                        Event::Quit {..} |
                        Event::KeyDown { keycode: Some(Keycode::Escape), .. } => { 
                            quit = true;
                        },
                        Event::KeyUp   { keycode: Some(key), .. } => { 
                            terrible_input_proc(&mut dmg, key, false);
                        },
                        Event::KeyDown   { keycode: Some(key), .. } => { 
                            terrible_input_proc(&mut dmg, key, true);
                        },
                        Event::JoyButtonDown{timestamp:_, which, button_idx} => {
                            terrible_joypad_btn_proc(&mut dmg, which, button_idx, true);
                        },
                        Event::JoyButtonUp{timestamp:_, which, button_idx} => {
                            terrible_joypad_btn_proc(&mut dmg, which, button_idx, false);
                        },
                        _ => {}
                    }
                }
                // if start_instant.elapsed() < frametime{
                //     ::std::thread::sleep(frametime - start_instant.elapsed());
                // }
                // start_instant = Instant::now();
            }
        }
        
        if quit{
            return;
        }
    }
}
