#![no_std]
#![no_main]

//! # LoRa TX/RX demo application
//!
//! Blinking led green is for RX, red is for TX
//! Long press on user button switch the board role between TX and RX
//! Short press either send a packet of incrementing byte or display RX stats in RX

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};

use lr1120_apps::board::{BoardNucleoL476Rg, BoardRole, ButtonPressKind, LedMode, Lr1120Stm32};
use lr1120::{
    lora::{LoraBw, LoraModulationParams, LoraPacketParams, Sf},
    radio::{PacketType, RampTime},
    status::Intr,
    system::{ChipMode, DioNum, DioRfSwitchCfg}
};

const PLD_SIZE : u8 = 10;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Starting lora_txrx");

    let board = BoardNucleoL476Rg::init(&spawner).await;
    let mut lr1120 = board.lr1120;
    let mut irq1 = board.irq1;
    BoardNucleoL476Rg::led_green_set(LedMode::BlinkSlow);

    // Packet ID: correspond to first byte sent
    let mut pkt_id = 0_u8;

    // Initialize transceiver for LoRa communication
    // 901MHz, 0dbM, SF5 BW1000, CR 4/5
    lr1120.set_rf(901_000_000).await.expect("Setting RF to 901MHz");
    lr1120.calibrate(false, false, true, true, true, true).await.expect("Front-End calibration");

    match lr1120.get_status().await {
        Ok((status, intr)) => info!("Calibration Done: {} | {}", status, intr),
        Err(e) => warn!("Calibration Failed: {}", e),
    }

    let modulation = LoraModulationParams::basic(Sf::Sf5, LoraBw::Bw500);
    let packet_params = LoraPacketParams::basic(PLD_SIZE, &modulation);

    lr1120.set_packet_type(PacketType::Lora).await.expect("Setting packet type");
    lr1120.set_lora_modulation(&modulation).await.expect("SetLoraModulation");
    // Packet Preamble 8 Symbols, 10 Byte payload, Explicit header with CRC and up-chirp
    lr1120.set_lora_packet(&packet_params).await.expect("Setting packet parameters");
    lr1120.set_tx_params(0, RampTime::Ramp16u).await.expect("Setting TX parameters");
    lr1120.set_dio_irq(Intr::new_txrx(), Intr::new(0)).await.expect("Setting DIO7 as IRQ");
    let rf_sw_cfg = DioRfSwitchCfg::new_lf_hf(DioNum::Dio6, DioNum::Dio5, DioNum::Dio8, DioNum::Dio8)
        .with_gnss(DioNum::Dio7);
    lr1120.set_dio_rf_switch(rf_sw_cfg, false).await.expect("SetDioRfSw");

    // Start RX continuous
    match lr1120.set_rx(0xFFFFFFFF, true).await {
        Ok(_) => info!("[RX] Searching Preamble"),
        Err(e) => error!("Fail while set_rx() : {}", e),
    }

    // Set DIO9 as IRQ for RX Done
    // lr1120.set_dio_irq(DioNum::Dio7, Intr::new(IRQ_MASK_RX_DONE)).await.expect("Setting DIO7 as IRQ");

    // Wait for a button press for actions
    let mut button_press = BoardNucleoL476Rg::get_button_evt();

    let mut role = BoardRole::Rx;
    loop {
        match select(button_press.changed(), irq1.wait_for_rising_edge()).await {
            Either::First(press) => {
                match (press, role) {
                    // Short press in RX => clear stats
                    (ButtonPressKind::Short, BoardRole::Rx) => show_and_clear_rx_stats(&mut lr1120).await,
                    // Short press in TX => send a packet
                    (ButtonPressKind::Short, BoardRole::Tx) => {
                        send_pkt(&mut lr1120, &mut pkt_id).await;
                        BoardNucleoL476Rg::led_red_set(LedMode::Flash);
                    }
                    // Long press: switch role TX/RX
                    (ButtonPressKind::Long, _) => {
                        role.toggle();
                        switch_mode(&mut lr1120, role.is_rx()).await;
                    }
                    (n, r) => warn!("{} in role {} not implemented !", n, r),
                }
            }
            // RX Interrupt
            Either::Second(_) => {
                BoardNucleoL476Rg::led_green_set(LedMode::Flash);
                show_rx_pkt(&mut lr1120).await;
            }
        }
    }
}

async fn show_and_clear_rx_stats(lr1120: &mut Lr1120Stm32) {
    let stats = lr1120.get_rx_stats().await.expect("RX stats");
    info!("[RX] Clearing stats | RX={}, CRC Err={}, HdrErr={}, FalseSync={}",
        stats.pkt_rx(),
        stats.crc_error(),
        stats.header_error(),
        stats.false_sync(),
    );
    lr1120.clear_rx_stats().await.expect("Clear stats");
}

async fn send_pkt(lr1120: &mut Lr1120Stm32, pkt_id: &mut u8) {
    info!("[TX] Sending packet {}", *pkt_id);
    let len = PLD_SIZE as usize;
    // Create payload and send it to the TX FIFO
    for (i,d) in lr1120.buffer_mut().iter_mut().take(len).enumerate() {
        *d = pkt_id.wrapping_add(i as u8);
    }
    lr1120.wr_tx_buffer(len).await.expect("FIFO write");
    lr1120.set_tx(0).await.expect("SetTx");
    *pkt_id += 1;
}

async fn switch_mode(lr1120: &mut Lr1120Stm32, is_rx: bool) {
    lr1120.set_chip_mode(ChipMode::Fs).await.expect("SetFs");
    if is_rx {
        // lr1120.get_rx_stats()
        lr1120.set_rx(0xFFFFFFFF, true).await.expect("SetRx");
        BoardNucleoL476Rg::led_red_set(LedMode::Off);
        BoardNucleoL476Rg::led_green_set(LedMode::BlinkSlow);
        info!(" -> Switched to RX");
    } else {
        BoardNucleoL476Rg::led_red_set(LedMode::BlinkSlow);
        BoardNucleoL476Rg::led_green_set(LedMode::Off);
        info!(" -> Switching to FS: ready for TX");
    }
}

async fn show_rx_pkt(lr1120: &mut Lr1120Stm32) {
    let (_, intr) = lr1120.get_status().await.expect("Getting intr");
    if intr.rx_done() {
        let buffer_status = lr1120.get_rx_buffer_status().await.expect("RX Fifo level");
        if intr.rx_error() {
            warn!("[RX] intr={} | {} bytes @ {}",
                intr,
                buffer_status.pld_len(),
                buffer_status.offset());
        } else {
            let nb_byte = buffer_status.pld_len();

            // match lr1120.rd_rx_buffer(buffer_status.offset(), nb_byte).await {
            //     Ok(_) => info!("Read successfull"),
            //     Err(e) => {
            //         warn!("Read fail: {}", e);
            //         let err = lr1120.get_errors().await.expect("GetError");
            //         info!("Error = {}", err);
            //     }
            // }
            lr1120.rd_rx_buffer(buffer_status.offset(), nb_byte+2).await.expect("RdRxBuffer");
            let pkt_status = lr1120.get_lora_packet_status().await.expect("RX status");
            let snr = pkt_status.snr_pkt();
            let snr_frac = (snr&3) * 25;
            info!("[RX] Payload = {:02x} | intr={:08x} | RSSI=-{}dBm, SNR={}.{:02}",
                // pld,
                lr1120.buffer()[..(nb_byte+2) as usize],
                intr.value(),
                pkt_status.rssi_pkt()>>1,
                snr>>2, snr_frac,
            );
        }
        lr1120.clear_rx_buffer().await.expect("Clear Rx Buffer");
    } else if intr.tx_done() {
        info!("[TX] Packet sent");
    }
    lr1120.clear_irqs(None).await.expect("Clear IRQs");
}
