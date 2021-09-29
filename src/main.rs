mod dmg;
mod cpu;
mod bus;
mod ram;
mod interrupt;
mod cartrige;

use dmg::Dmg;

fn main() {
    let mut dmg = Dmg::new();
    dmg.do_test();
}
