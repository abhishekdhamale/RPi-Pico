//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::fixed_point::FixedPoint;
use panic_probe as _;

use rp2040_hal::rom_data;

pub const BLOCK_SIZE: u32 = 65536;
pub const SECTOR_SIZE: usize = 4096;
// pub const BLOCK64_ERASE: u8 = 0xD8;
pub const SECTOR_ERASE: u8 = 0x20;
pub const CHIP_ERASE: u8 = 0x60; // OR 0x60;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

// use bsp::hal::{
//     clocks::{init_clocks_and_plls, Clock},
//     pac,
//     sio::Sio,
//     watchdog::Watchdog,
// };


const FLASH_XIP_BASE: u32 = 0x1000_0000;
const FLASH_XIP_FIRMWARE_BASE: u32 = 0x20000;
const FLASH_XIP_APPLISTART_ADDR: u32 = FLASH_XIP_BASE + FLASH_XIP_FIRMWARE_BASE;

// fn hal_flash_erase(addr: usize, _len: usize) {
//     //let addr = FLASH_END - FLASH_USER_SIZE;
//     unsafe {
//         cortex_m::interrupt::free(|_cs| {
//             rom_data::connect_internal_flash();
//             rom_data::flash_exit_xip();
//             rom_data::flash_range_erase(
//                 addr.try_into().unwrap(),
//                 SECTOR_SIZE,
//                 BLOCK_SIZE,
//                 CHIP_ERASE,
//             );
//             rom_data::flash_flush_cache(); // Get the XIP working again
//             rom_data::flash_enter_cmd_xip(); // Start XIP back up
//         });
//     }
// }

#[rustfmt::skip]
pub fn boot_from(fw_base_address: usize) {
    static mut USER_RESET: Option<extern "C" fn()> = None;
    let scb = bsp::pac::SCB::ptr();
    let address = fw_base_address as u32;
    unsafe {
        let sp = *(address as *const u32);
        let rv = *((address + 4) as *const u32);
        USER_RESET = Some(core::mem::transmute(rv));
        (*scb).vtor.write(address);
        cortex_m::register::msp::write(sp);
        (USER_RESET.unwrap())();
    }
}

#[entry]
fn main() -> ! {
    // info!("Program start");

    // let mut pac = pac::Peripherals::take().unwrap();
    // let core = pac::CorePeripherals::take().unwrap();
    // let mut watchdog = Watchdog::new(pac.WATCHDOG);
    // let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    // let external_xtal_freq_hz = 12_000_000u32;
    // let clocks = init_clocks_and_plls(
    //     external_xtal_freq_hz,
    //     pac.XOSC,
    //     pac.CLOCKS,
    //     pac.PLL_SYS,
    //     pac.PLL_USB,
    //     &mut pac.RESETS,
    //     &mut watchdog,
    // )
    // .ok()
    // .unwrap();

    //let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    // let pins = bsp::Pins::new(
    //     pac.IO_BANK0,
    //     pac.PADS_BANK0,
    //     sio.gpio_bank0,
    //     &mut pac.RESETS,
    // );

    // let mut led_pin = pins.led.into_push_pull_output();

    // for _v  in 0..4 {
    //     info!("on!");
    //     led_pin.set_high().unwrap();
    //     delay.delay_ms(50);
    //     info!("off!");
    //     led_pin.set_low().unwrap();
    //     delay.delay_ms(50);
    // }

    // for _v  in 0..6 {
    //     hal_flash_erase(0x1000_0000, 1);
    // }

    boot_from(FLASH_XIP_APPLISTART_ADDR as usize);

    loop {
        
    }

}

// End of file
