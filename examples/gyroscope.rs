//! Interact with on-board Gyroscope
#![deny(warnings)]
#![feature(proc_macro)]
#![no_std]

extern crate cortex_m_rtfm as rtfm;
extern crate f3;

use f3::{spi, l3gd20};
use rtfm::{app, Threshold};

app! {
    device: f3::stm32f30x,
    tasks: {
        EXTI1: {
            path: gyro_intr,
            resources: [SPI1, GPIOE],
        }
    },
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
    // Need to change which pin is connected to EXTI so enable `SYSCFG`
    p.RCC.apb2enr.write(|w| w.syscfgen().enabled());
    // Set `EXTI1` to be connected to `GPIOE`
    p.SYSCFG.exticr1.write(|w| unsafe {
        w.exti1().bits(0x04)
    });
    // Enable interrupt from Gyroscope on `EXTI1`
    p.EXTI.imr1.write(|w| w.mr1().set_bit());
    // Select rising trigger for `EXTI1`
    p.EXTI.rtsr1.write(|w| w.tr1().set_bit());
}

fn idle() -> ! {
    loop {
        rtfm::wfi();
    }
}

fn gyro_intr(_t: &mut Threshold, r: EXTI1::Resources) {
    let spi1 = spi::Spi(& **r.SPI1);
    let gyro = l3gd20::L3GD20(&spi1, & **r.GPIOE);
    let _status = gyro.status().unwrap();
    // Reading measurements clears interrupt on `DRDY` pin
    let _read = gyro.measure(l3gd20::ScaleSelection::Dps2000).unwrap();
    rtfm::bkpt();
}
