use std::borrow::BorrowMut;
use std::io::{ErrorKind, Write};
use std::io::Read;
use std::fs;
use std::time::{Instant, Duration};

extern crate sdl2;

use sdl2::libc::open;
use sdl2::pixels::{Color, PixelFormatEnum};
use sdl2::event::Event;
use sdl2::rect::{Rect};
use sdl2::surface::Surface;
use sdl2::keyboard::Keycode;
use sdl2::audio::{AudioSpecDesired, AudioQueue};
use sdl2::joystick::*;

use dirtydmg_core::dmg::Dmg;
use dirtydmg_core::input::Button;
use dirtydmg_core::interface::ScanlineBuffer;
use dirtydmg_core::sound::AudioChannel;

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

fn load_sram(filepath: &str) -> Result<Vec<u8>, std::io::Error>{
    let mut ram_file = fs::File::open(filepath)?;
    let meta = ram_file.metadata()?;

    if meta.len() >  (1 * 1024 * 1024) {
        return Err(std::io::Error::new(ErrorKind::Other, "ram file is too large."));
    }

    let mut ram_data: Vec<u8> = Vec::new();
    let size = ram_file.read_to_end(&mut ram_data).unwrap_or(0);

    if size == meta.len() as usize{
        Ok(ram_data)
    }
    else {
        Err(std::io::Error::new(ErrorKind::Other, "Failed to load sram data."))
    }
}

fn save_sram(filepath: &str, data: &[u8]){
    let mut sram_file = fs::OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(filepath);
    if let Ok(mut sram_file) = sram_file{
        sram_file.write_all(data).unwrap_or_default();
    }
}

struct DmgSurfaceRenderer {
    surface: Surface<'static>,
    colors: [Color;4]
}

impl DmgSurfaceRenderer{
    fn new(format:PixelFormatEnum) -> DmgSurfaceRenderer{
        let surface = Surface::new(160, 144, format).unwrap();
        let colors: [Color;4] = [
            Color{r: 255, g:255, b:255, a: 0},
            Color{r: 170, g:170, b:170, a: 0},
            Color{r: 85,  g:85,  b: 85, a: 0},
            Color{r: 0,   g:0,   b: 0,  a: 0},
        ];
        DmgSurfaceRenderer{
            surface,
            colors
        }
    }

    fn draw_line(&mut self, buffer: &ScanlineBuffer, y: i32) { 
        let mut x = 0;
        for pixelpack in buffer.pixeldata{
            for sub_pixel in 0..4{
                self.surface.fill_rect(
                    Rect::new(x, y, 1, 1),
                    self.colors[((pixelpack >> (2 * sub_pixel))&0b11) as usize]
                ).unwrap();
                x += 1
            }
        }
    }
}

fn terrible_input_proc(dmg: &mut Dmg, key:Keycode, pressed:bool){
    // Emulator button processing
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

    if pressed == true{
        // UI processing
        let channel_toggle = match key {
            Keycode::Num1 => {Some(AudioChannel::Channel1)}
            Keycode::Num2 => {Some(AudioChannel::Channel2)}
            Keycode::Num3 => {Some(AudioChannel::Channel3)}
            Keycode::Num4 => {Some(AudioChannel::Channel4)}
            _ => None
        };
        if let Some(x) = channel_toggle{
            dmg.apu.as_ref().borrow_mut().user_set_channel_enable_toggle(x);
        }
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

    let ram_filepath = path.to_owned() + ".sram";
    let ram_data = load_sram(&ram_filepath);
    if let Ok(sram) = ram_data {
        dmg.load_sram(&sram);
    }

    let scale = 4;

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
        channels: Some(2),  // stereo
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
    let mut canvas = window.into_canvas()
        .accelerated()
        .build()
        .unwrap();
    canvas.clear();
    canvas.present();

    let mut surfaceBuffer = DmgSurfaceRenderer::new(canvas.default_pixel_format());
    let tc = canvas.texture_creator();
    let mut framecount = 0;
    let mut timer = Instant::now();
    let mut quit = false;
    let mut buf = Vec::<i8>::new();
    loop {
        while audio_queue.size() > 512 {
           std::thread::sleep_ms(1);
        }
        dmg.update();

        if let Some((left, right)) = dmg.apu.as_ref().borrow_mut().get_sample() {
            buf.push(left);
            buf.push(right);
            if buf.len() >= 512 {
                audio_queue.queue(&buf);
                buf.clear();
            }
        }

        // if there is a pending line to draw
        if dmg.ppu.as_ref().borrow().line_pending {
            let mut ppu = dmg.ppu.as_ref().borrow_mut();
            let y = ppu.line_y - 1;
            ppu.line_pending = false;
            surfaceBuffer.draw_line(&ppu.line_buffer, y as i32);
            drop(ppu);
            if y == 143 {
                let tex = tc.create_texture_from_surface(&surfaceBuffer.surface).unwrap();
                canvas.copy(&tex, None, None).unwrap();
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
            }
        }
        
        if quit {
            let mut sram = Vec::new();
            dmg.get_sram(&mut sram);
            if sram.len() > 0 {
                save_sram(&ram_filepath, &sram);
            }
            return;
        }
    }
}
