//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::entry;
// use defmt::*;
use defmt_rtt as _;
// use embedded_hal::digital::v2::OutputPin;
// use embedded_time::fixed_point::FixedPoint;
use panic_probe as _;

use rp2040_hal::rom_data;

// use core::slice;
// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

// use bsp::hal::{
//     clocks::{init_clocks_and_plls, Clock},
//     pac,
//     //ssio::Sio,
//     watchdog::Watchdog,
// };

const FLASH_XIP_BASE: u32 = 0x1000_0000;
const FLASH_XIP_FIRMWARE_BASE: u32 = 0x20000;
const FLASH_XIP_APPLISTART_ADDR: u32 = FLASH_XIP_BASE + FLASH_XIP_FIRMWARE_BASE;

pub const BLOCK_SIZE: u32 = 65536;
pub const SECTOR_SIZE: usize = 4096;
pub const PAGE_SIZE: u32 = 256;
// These _ERASE commands are highly dependent on the flash chip you're using
pub const SECTOR_ERASE: u8 = 0x20; // Tested and works with W25Q16JV flash chip
pub const BLOCK32_ERASE: u8 = 0x52;
pub const BLOCK64_ERASE: u8 = 0xD8;
/* IMPORTANT NOTE ABOUT RP2040 FLASH SPACE ADDRESSES:
When you pass an `addr` to a `rp2040-hal::rom_data` function it wants
addresses that start at `0x0000_0000`. However, when you want to read
that data back using something like `slice::from_raw_parts()` you
need the address space to start at `0x1000_0000` (aka `FLASH_XIP_BASE`).
*/

pub const FLASH_START: u32 = 0x0000_0000;
pub const FLASH_END: u32 = 0x0020_0000;
pub const FLASH_USER_SIZE: u32 = 4096; // Amount dedicated to user prefs/stuff

pub const FLASH_ADDRESS_OFFSET : u32 = 0x20000;

#[inline(never)]
#[link_section = ".data.ram_func"]
fn write_flash(addr: usize, data: &[u8], len: usize) {

    let address = addr as u32;
    // let len = len as u32;
    // let mut idx = 0u32;
    // let mut src = data;
    // let mut dst = address as *mut u8;

    unsafe {
        cortex_m::interrupt::free(|_cs| {
            rom_data::connect_internal_flash();
            rom_data::flash_exit_xip();
            rom_data::flash_range_erase(address, SECTOR_SIZE, BLOCK_SIZE, SECTOR_ERASE);
            rom_data::flash_range_program(address, data.as_ptr(), data.len());
            rom_data::flash_flush_cache(); // Get the XIP working again
            rom_data::flash_enter_cmd_xip(); // Start XIP back up
        });
    }
    defmt::println!("write_flash() Complete"); // TEMP
}

#[inline(never)]
#[link_section = ".data.ram_func"]
fn chip_erase(){
    // let mut total_blocks = 0;
    let mut temp_sector_buf = [0;4096];
    for x in 0..4096{
        temp_sector_buf[x] = 0x55;
    }
    for block_num in 1..32{
        for sector_num in 0..16{
            let sector_start_addr= (block_num*65536) + (sector_num*4096);
            unsafe {
                cortex_m::interrupt::free(|_cs| {
                    rom_data::connect_internal_flash();
                    rom_data::flash_exit_xip();
                    rom_data::flash_range_erase(sector_start_addr as u32, SECTOR_SIZE, BLOCK_SIZE, SECTOR_ERASE);
                    rom_data::flash_range_program(sector_start_addr as u32, temp_sector_buf.as_ptr(), temp_sector_buf.len());
                    rom_data::flash_flush_cache(); // Get the XIP working again
                    rom_data::flash_enter_cmd_xip(); // Start XIP back up
                });
            }
        }
        // defmt::println!("{}: Block Start Address: {:x}", total_blocks, (block_num*65536));
        // total_blocks += 1;
    }
    // info!("Chip erase complete.");
    // defmt::println!("Chip erase complete.");
}

// fn erase_flash(addr: u32) {
//     unsafe {
//         cortex_m::interrupt::free(|_cs| {
//             rom_data::connect_internal_flash();
//             rom_data::flash_exit_xip();
//             rom_data::flash_range_erase(addr, SECTOR_SIZE, BLOCK_SIZE, SECTOR_ERASE);
//             rom_data::flash_flush_cache(); // Get the XIP working again
//             rom_data::flash_enter_cmd_xip(); // Start XIP back up
//         });
//     }
// }

// fn read_flash(addr: u32) -> &'static mut [u8] {
//     let addr = addr as *mut u8;
//     let my_slice = unsafe { slice::from_raw_parts_mut(addr, FLASH_USER_SIZE as usize) };
//     my_slice
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

    //erase_flash(FLASH_ADDRESS_OFFSET);

    //let data = crate::read_flash(FLASH_ADDRESS_OFFSET);

    //defmt::println!("Flash data[0]: {:?}", data[0]);

    let mut buf = [0; 16];


    // for x in 0..256{
    //     buf[x] = 0xff;
    // }
    // buf[4] = 0x55;

    // // buf[1] = 0x11;
    // // buf[2] = 0x33;
    // // buf[3] = 0x22;
    // // buf[4] = 0x11;

    // let mut buf = [0; 256];
    // for x in 0..256{
    //     buf[x] = 0x55;
    // }

    buf[0] = 0x77;
    buf[1] = 0x55;
    buf[2] = 0x77;
    buf[3] = 0x55;
    buf[4] = 0x77;
    buf[5] = 0x55;
    buf[6] = 0x77;
    buf[7] = 0x55;
    buf[8] = 0x77;
    buf[9] = 0x55;
    buf[10] = 0x77;
    buf[11] = 0x55;
    buf[12] = 0x77;
    buf[13] = 0x55;
    buf[14] = 0x77;
    buf[15] = 0x55;
    crate::write_flash((FLASH_ADDRESS_OFFSET) as usize, &buf, buf.len());

    // let data2 = crate::read_flash(FLASH_ADDRESS_OFFSET);

    // defmt::println!("Flash data[0]: {:?}", data2[0]);

    // chip_erase();

    // boot_sfrom(FLASH_XIP_APPLISTART_ADDR as usize);

    // defmt::println!("Flash Erase Complete.");

    loop {
        
    }

}

// End of file
