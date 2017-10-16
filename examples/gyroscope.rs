//! Interact with on-board Gyroscope
#![deny(unsafe_code)]
#![deny(warnings)]
#![feature(proc_macro)]
#![no_std]

extern crate cortex_m_rtfm as rtfm;
extern crate f3;

use f3::{spi, l3gd20};
use rtfm::app;

app! {
    device: f3::stm32f30x,
}

fn init(p: init::Peripherals) {
    // Initialize SPI to use with Gyroscope
    let spi1 = spi::Spi(p.SPI1);
    spi1.init(p.GPIOA, p.RCC);
    // Enable use of SPI1
    spi1.enable();
    // Initialize Gyroscope
    let gyro = l3gd20::L3GD20(&spi1, p.GPIOE);
    gyro.init(l3gd20::Config::default()).unwrap();
    rtfm::bkpt();
}

fn idle() -> ! {
    loop {
        rtfm::wfi();
    }
}
