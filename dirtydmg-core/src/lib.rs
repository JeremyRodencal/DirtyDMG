#![allow(
    clippy::bool_comparison,
)]

pub mod bus;
#[allow(dead_code)]
pub mod cartrige;
pub mod cpu;
#[allow(dead_code)]
pub mod dmg;
pub mod interrupt;
#[allow(dead_code)]
pub mod ppu;
pub mod ram;
#[allow(dead_code)]
pub mod serial;
pub mod timer;
pub mod interface;
pub mod input;
pub mod sound;