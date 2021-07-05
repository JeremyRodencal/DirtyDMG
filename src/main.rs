mod dmg;
mod cpu;
mod bus;
mod ram;

use dmg::Dmg;

fn main() {
    let mut dmg = Dmg::new();
    dmg.do_test();
}
