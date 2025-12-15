#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{gpio::{Input, Level, Output, Pull, Speed}, time::Hertz};
use embassy_stm32::spi::{Config as SpiConfig, Spi};
use embassy_stm32::usart::{Config as UartConfig, Uart};
use embassy_stm32::{bind_interrupts, peripherals, usart};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

use core::fmt::Write;
use heapless::String;

use lr1120::Lr1120;

bind_interrupts!(struct Irqs {
    USART2 => usart::InterruptHandler<peripherals::USART2>;
});

/// Task to blink up to two leds
#[embassy_executor::task(pool_size = 2)]
async fn blink(mut led: Output<'static>, delay: u64) {

    loop {
        led.set_high();
        Timer::after_millis(delay).await;
        led.set_low();
        Timer::after_millis(delay).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Starting get_temp (v4)");

    // Blink both TX/RX LEDs on the radio module to confirm PIN mapping
    let led_tx = Output::new(p.PC1, Level::High, Speed::Low);
    spawner.spawn(blink(led_tx, 500)).unwrap();

    let led_rx = Output::new(p.PC0, Level::High, Speed::Low);
    spawner.spawn(blink(led_rx, 133)).unwrap();

    // Control pins
    let busy = Input::new(p.PB3, Pull::Up);
    let nreset = Output::new(p.PA0, Level::High, Speed::Low);

    // SPI
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = Hertz(4_000_000);
    let spi = Spi::new(p.SPI1, p.PA5, p.PA7, p.PA6, p.DMA1_CH3, p.DMA1_CH2, spi_config);
    let nss = Output::new(p.PA8, Level::High, Speed::VeryHigh);

    // UART on Virtual Com: 115200bauds, 1 stop bit, no parity, no flow control
    let uart_config = UartConfig::default();
    let mut uart = Uart::new(p.USART2, p.PA3, p.PA2, Irqs, p.DMA1_CH7, p.DMA1_CH6, uart_config).unwrap();

    let mut lr1120 = Lr1120::new_blocking(nreset, busy, spi, nss);
    lr1120.reset().await
        .unwrap_or_else(|_| error!("Unable to reset chip !"));

    // String buffer
    let mut s: String<128> = String::new();

    // Check version
    let fw_version = lr1120.get_version().await;
    match fw_version {
        Ok(v) => {
            info!("FW Version {:02x}.{:02x}", v.major(), v.minor());
            core::write!(&mut s, "FW Version {}.{}!\r\n", v.major(), v.minor()).unwrap();
            uart.write(s.as_bytes()).await.ok();
        }
        Err(e) => error!("{}", e),
    }

    // Report status
    match lr1120.get_status().await {
        Ok((status,intr)) => info!("{} | Intr={:08x}", status, intr.value()),
        Err(e) => error!("{}", e),
    }

    // Get periodic temperature measurement
    loop {
        Timer::after_secs(10).await;
        match lr1120.get_temperature().await {
            Ok(t) => {
                let dt_scl = 794*(t as u32);
                let t_deg = 25 + 429 - ((dt_scl + 1024) >> 11);
                info!("{} =>  {}", t, t_deg);
                s.clear();
                match core::write!(&mut s, "T = {} => {}\r\n", t, t_deg) {
                    Ok(_) => {uart.write(s.as_bytes()).await.ok();}
                    Err(_) => {error!("Unable to write string");}
                }

            }
            Err(e) => error!("{}", e),
        }
    }

}
