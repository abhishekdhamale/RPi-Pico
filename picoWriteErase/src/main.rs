//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::entry;
// use defmt::*;
use defmt_rtt as _;
// use embedded_hal::digital::v2::OutputPin;
use embedded_time::fixed_point::FixedPoint;
use panic_probe as _;

use rp2040_hal::rom_data;

// use core::slice;
// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    //ssio::Sio,
    watchdog::Watchdog,
};

const FLASH_XIP_BASE: u32 = 0x1000_0000;
const FLASH_XIP_FIRMWARE_BASE: u32 = 0x20000;
const FLASH_XIP_APPLISTART_ADDR: u32 = FLASH_XIP_BASE + FLASH_XIP_FIRMWARE_BASE;

pub const BLOCK_SIZE: u32 = 65536;
pub const SECTOR_SIZE: usize = 4096;
pub const PAGE_SIZE: usize = 256;
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

use cortex_m::asm;

#[inline(never)]
#[link_section = ".data.ram_func"]
fn write_flash(address: usize, data: *const u8, len: usize) {

    let mut addr = address - 0x10000000;

    // let mut pac = pac::Peripherals::take().unwrap();
    // let core = pac::CorePeripherals::take().unwrap();
    // let mut watchdog = Watchdog::new(pac.WATCHDOG);
    // // let sio = Sio::new(pac.SIO);

    // // External high-speed crystal on the pico board is 12Mhz
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

    // let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    // let address = addr as u32;
    // let len = len as u32;
    // let mut idx = 0u32;
    // let mut src = data;
    // let mut dst = address as *mut u8;

    // if ((len == 1) || (len == 4)){

        // let offset = address - 0x10000000;
        // let block: usize = offset / 65536;
        // let sector: usize = (offset - (65536*block))/ 4096;
        // let page = (offset - (65536*block) - (4096*sector))/ 256;
        // let byte = offset - (65536*block) - (4096*sector) - (256*page);
        // defmt::println!("Offset: {:x} Block:{} Sector:{} Page:{} Byte:{}", offset, block, sector, page, byte);
        // let mut page_start_addr: usize = (block*65536) + (sector*4096) + (page*256);
        // let mut src = data as *mut u8;
        // // let mut val: u8 = 0u8;
        // // let mut idx: usize = 0;
        // let mut temp_page_buf: [u8; 256] = [0; 256];
        // let mut dst = page_start_addr as *mut u8;
        // for x in 0..256{
        //     unsafe{
        //         temp_page_buf[x] = *dst;
        //     }
        //     dst = ((dst as u32) + x as u32) as *mut u8;
        // }
        // for x in 0..len{
        //     unsafe{
        //         // val = *src;
        //         // idx = x+byte;
        //         // temp_page_buf[idx] = 0x55;
        //         temp_page_buf[x+byte] = *src;
        //     }
        //     defmt::println!("page_start_addr: {:x} >> MagicBytes::{}>>{:x} ",page_start_addr, idx,val);
        //     src = ((src as u32) + 1) as *mut u8;
        // }
        // unsafe {
        //     cortex_m::interrupt::free(|_cs| {
        //         rom_data::connect_internal_flash();
        //         rom_data::flash_exit_xip();
        //         // rom_data::flash_range_erase(page_start_addr as u32, SECTOR_SIZE, BLOCK_SIZE, SECTOR_ERASE);
        //         rom_data::flash_range_program(page_start_addr as u32, temp_page_buf.as_ptr(), temp_page_buf.len());
        //         rom_data::flash_flush_cache(); // Get the XIP working again
        //         rom_data::flash_enter_cmd_xip(); // Start XIP back up
        //     });
        // }
        // let starting_page = addr as u32;
        // let ending_page = (addr + len) as u32;
        // defmt::println!("starting_page={:x}, ending_page={:x}, len={}", starting_page, ending_page, len);
    
        // let mut src = data as *mut u8;

        // // for addr in (starting_page..ending_page).step_by(SECTOR_SIZE as usize) {

        // //     defmt::println!("Data Address: {:x} ",data.as_ptr());


        // //     src = ((src as u32) + SECTOR_SIZE as u32) as *mut u8;
        // // }


    if len <= 4{

        let offset = address - 0x10000000;
        let block: usize = offset / 65536;
        let sector: usize = (offset - (65536*block))/ 4096;
        let page = (offset - (65536*block) - (4096*sector))/ 256;
        let byte = offset - (65536*block) - (4096*sector) - (256*page);
        defmt::println!("Offset: {:x} Block:{} Sector:{} Page:{} Byte:{}", offset, block, sector, page, byte);
        let mut page_start_addr: usize = (block*65536) + (sector*4096) + (page*256);
        let mut src = data as *mut u8;
        let mut temp_page_buf: [u8; 256] = [0; 256];
        let mut dst = (page_start_addr + 0x10000000) as *mut u8;
        defmt::println!("page_start_addr: {:x}",page_start_addr);
        for x in 0..256{
            unsafe{
                temp_page_buf[x] = *dst;
            }
            dst = ((dst as u32) + 1 as u32) as *mut u8;
        }
        for y in 0..len{
            unsafe{
                temp_page_buf[y+byte] = *src;
            }
            defmt::println!("page_start_addr: {:x} >> MagicBytes:{:x} ",page_start_addr, temp_page_buf[y+byte]);
            // defmt::println!("Data {:x} ",temp_page_buf[y+byte]);
            src = ((src as u32) + 1) as *mut u8;
        }
        defmt::println!("Data {} ",temp_page_buf);


        // let mut temp_page_buf: [u8; PAGE_SIZE] = [0xff; PAGE_SIZE];
        // let starting_page = addr as u32;
        // let ending_page = (addr + len) as u32;
        // // defmt::println!("starting_page={:x}, ending_page={:x}, len={}", starting_page, ending_page, len);
        // let mut src = address as u32; //as u32;
        // src = (src & 0xffffff00); // as *mut u8;
        // src = src as *mut u8;
        // defmt::println!("src address: {:x} ",src);
        // for x in 0..PAGE_SIZE{
        //     unsafe{
        //         temp_page_buf[x] = *src;
        //     }
        //     src = ((src as u32) + 1 as u32) as *mut u8;
        // }
        // defmt::println!("Data {} ",temp_page_buf);
        unsafe {
            cortex_m::interrupt::free(|_cs| {
                rom_data::connect_internal_flash();
                rom_data::flash_exit_xip();
                // rom_data::flash_range_erase(page_start_addr as u32, SECTOR_SIZE, BLOCK_SIZE, SECTOR_ERASE);
                rom_data::flash_range_program(page_start_addr as u32, temp_page_buf.as_ptr(), temp_page_buf.len());
                rom_data::flash_flush_cache(); // Get the XIP working again
                rom_data::flash_enter_cmd_xip(); // Start XIP back up
            });
        }
        // temp_page_buf = [0xff; PAGE_SIZE];
    }

    else
    {

        let mut temp_page_buf: [u8; PAGE_SIZE] = [0xff; PAGE_SIZE];
        let starting_page = addr as u32;
        let ending_page = (addr + len) as u32;
        // defmt::println!("starting_page={:x}, ending_page={:x}, len={}", starting_page, ending_page, len);
        let mut src = data; //as u32;
        for addr in (starting_page..ending_page).step_by(PAGE_SIZE as usize) {
            // defmt::println!("DataWrite Address: {:x} ",addr);
            for x in 0..PAGE_SIZE{
                unsafe{
                    temp_page_buf[x] = *src;
                }
                src = ((src as u32) + 1 as u32) as *mut u8;
                //defmt::println!("Data {:x} ",temp_page_buf[x]);
            }
            unsafe {
                cortex_m::interrupt::free(|_cs| {
                    rom_data::connect_internal_flash();
                    rom_data::flash_exit_xip();
                    // rom_data::flash_range_erase(page_start_addr as u32, SECTOR_SIZE, BLOCK_SIZE, SECTOR_ERASE);
                    rom_data::flash_range_program(addr as u32, temp_page_buf.as_ptr(), temp_page_buf.len());
                    rom_data::flash_flush_cache(); // Get the XIP working again
                    rom_data::flash_enter_cmd_xip(); // Start XIP back up
                });
            }
            temp_page_buf = [0xff; PAGE_SIZE];
        }

    }

    // unsafe {
    //     cortex_m::interrupt::free(|_cs| {
    //         rom_data::connect_internal_flash();
    //         delay.delay_ms(100);
    //         rom_data::flash_exit_xip();
    //         delay.delay_ms(100);
    //         // rom_data::flash_range_erase(0x3fffc, SECTOR_SIZE, BLOCK_SIZE, SECTOR_ERASE);
    //         // delay.delay_ms(100);
    //         rom_data::flash_range_program(0x3fffc, data.as_ptr(), data.len());
    //         delay.delay_ms(100);
    //         rom_data::flash_flush_cache(); // Get the XIP working again
    //         delay.delay_ms(100);
    //         rom_data::flash_enter_cmd_xip(); // Start XIP back up
    //     });
    // }
    // defmt::println!("Data Length: {}", data.len());
    // unsafe {
    //     cortex_m::interrupt::free(|_cs| {
    //         rom_data::connect_internal_flash();
    //         delay.delay_ms(100);
    //         rom_data::flash_exit_xip();
    //         delay.delay_ms(100);
    //         rom_data::flash_range_erase(0x21000, SECTOR_SIZE, BLOCK_SIZE, SECTOR_ERASE);
    //         delay.delay_ms(100);
    //         rom_data::flash_range_program(0x21000, data.as_ptr(), data.len());
    //         delay.delay_ms(100);
    //         rom_data::flash_flush_cache(); // Get the XIP working again
    //         delay.delay_ms(100);
    //         rom_data::flash_enter_cmd_xip(); // Start XIP back up
    //     });
    // }

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


// fn write_flash(addr: usize, data: &[u8], len: usize) {

//     let offset = address - 0x10000000;
//     let block: usize = offset / 65536;
//     let sector: usize = (offset - (65536*block))/ 4096;
//     let page = (offset - (65536*block) - (4096*sector))/ 256;
//     let byte = offset - (65536*block) - (4096*sector) - (256*page);
//     defmt::println!("Offset: {:x} Block:{} Sector:{} Page:{} Byte:{}", offset, block, sector, page, byte);

//     let mut sector_start_addr: usize = (block*65536);
//     defmt::println!("Sector-Start-Addr: {:x}", sector_start_addr);

//     let mut page_start_addr: usize = (block*65536) + (sector*4096) + (page*256);
//     defmt::println!("Page-Start-Addr: {:x}", page_start_addr);

//     let mut tempSectorBuf = [0; 4096];
//     let mut dst = sector_start_addr as *mut u8;
//     for x in 0..4096{
//         unsafe{
//             tempSectorBuf[x] = *dst;
//         }
//         dst = ((dst as u32) + x as u32) as *mut u8;
//     }

//     let mut val = 0u8;
//     let mut idx: usize = 0;
//     let byte_offset = byte + (page*256);
//     let mut src = data as *mut u8;
    
//     for x in 0..len{
//         unsafe{
//             val = *src;
//             idx = x + byte_offset;
//             tempSectorBuf[idx] = val;
//             // tempSectorBuf[x + byte_offset] = *src;
//         }
//         defmt::println!("MagicBytes::{}>>{:x} ",idx,val);
//         src = ((src as u32) + 1) as *mut u8;
//     }


//     let mut page_start_addr: usize = (block*65536) + (sector*4096) + (page*256);
//     let mut src = data as *mut u8;
//     let mut val: u8 = 0u8;
//     let mut idx: usize = 0;
//     let mut temp_page_buf: [u8; 256] = [0; 256];
//     for x in 0..256{
//         temp_page_buf[x] = 0xff;
//     }
//     for x in 0..len{
//         unsafe{
//             val = *src;
//             idx = x+byte;
//             temp_page_buf[idx] = 0x55;
//             // temp_page_buf[x+byte] = *src;
//         }
//         defmt::println!("page_start_addr: {:x} >> MagicBytes::{}>>{:x} ",page_start_addr, idx,val);
//         src = ((src as u32) + 1) as *mut u8;
//     }
//     unsafe {
//         cortex_m::interrupt::free(|_cs| {
//             rom_data::connect_internal_flash();
//             rom_data::flash_exit_xip();
//             rom_data::flash_range_erase(page_start_addr as u32, SECTOR_SIZE, BLOCK_SIZE, SECTOR_ERASE);
//             rom_data::flash_range_program(page_start_addr as u32, temp_page_buf.as_ptr(), temp_page_buf.len());
//             rom_data::flash_flush_cache(); // Get the XIP working again
//             rom_data::flash_enter_cmd_xip(); // Start XIP back up
//         });
//     }

// }



fn hal_flash_write(address: usize, data: *const u8, len: usize) {

    let offset = address - 0x10000000;
    let block: usize = offset / 65536;
    let sector: usize = (offset - (65536*block))/ 4096;
    let page = (offset - (65536*block) - (4096*sector))/ 256;
    let byte = offset - (65536*block) - (4096*sector) - (256*page);
    defmt::println!("Offset: {:x} Block:{} Sector:{} Page:{} Byte:{}", offset, block, sector, page, byte);
    let mut sector_start_addr: usize = block*65536;
    defmt::println!("Sector-Start-Addr: {:x}", sector_start_addr);
    let mut page_start_addr: usize = (block*65536) + (sector*4096) + (page*256);
    defmt::println!("Page-Start-Addr: {:x}", page_start_addr);
    let mut tempSectorBuf = [0; 4096];
    let mut dst = sector_start_addr as *mut u8;
    for x in 0..4096{
        unsafe{
            tempSectorBuf[x] = *dst;
        }
        dst = ((dst as u32) + x as u32) as *mut u8;
    }
    let mut val = 0u8;
    let mut idx: usize = 0;
    let byte_offset = byte + (page*256);
    let mut src = data as *mut u8;
    for x in 0..len{
        unsafe{
            val = *src;
            idx = x + byte_offset;
            tempSectorBuf[idx] = val;
            // tempSectorBuf[x + byte_offset] = *src;
        }
        defmt::println!("MagicBytes::{}>>{:x} ",idx,val);
        src = ((src as u32) + 1) as *mut u8;
    }
}




#[inline(never)]
#[link_section = ".data.ram_func"] // IMP
fn hal_flash_erase(addr: usize, len: usize) {

    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    // let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();
    
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let starting_page = addr as u32;
    let ending_page = (addr + len) as u32;
    defmt::println!("starting_page={:x}, ending_page={:x}, len={}", starting_page, ending_page, len);

    for addr in (starting_page..ending_page).step_by(SECTOR_SIZE as usize) {
        // Enable erasing
        defmt::println!("Address: {:x}", addr);
        unsafe {
            cortex_m::interrupt::free(|_cs| {
                rom_data::connect_internal_flash();
                // asm::delay(8000000);
                delay.delay_ms(100);
                rom_data::flash_exit_xip();
                // asm::delay(8000000);
                delay.delay_ms(100);
                rom_data::flash_range_erase(addr, SECTOR_SIZE, BLOCK_SIZE, SECTOR_ERASE);
                // asm::delay(8000000);
                delay.delay_ms(100);
                rom_data::flash_flush_cache(); // Get the XIP working again
                // asm::delay(8000000);
                delay.delay_ms(100);
                rom_data::flash_enter_cmd_xip(); // Start XIP back up
                // asm::delay(8000000);
                delay.delay_ms(100);
            });
        }
        defmt::println!("Sector Erase complete.");
    }
}

#[entry]
fn main() -> ! {



    //erase_flash(FLASH_ADDRESS_OFFSET);

    //let data = crate::read_flash(FLASH_ADDRESS_OFFSET);

    //defmt::println!("Flash data[0]: {:?}", data[0]);

    // let mut buf: [u8; 4096] = [0x57; 4096];


    // for x in 0..4096{
    //     buf[x] = 0x07;
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

    // buf[0] = 0xff;
    // buf[1] = 0x77;
    // buf[2] = 0x55;
    // buf[3] = 0x33;
    // buf[4] = 0x77;
    // buf[5] = 0x55;
    // buf[6] = 0x77;
    // buf[7] = 0x55;
    // buf[8] = 0x77;
    // buf[9] = 0x55;
    // buf[10] = 0x77;
    // buf[11] = 0x55;
    // buf[12] = 0x77;
    // buf[13] = 0x55;
    // buf[14] = 0x77;
    // buf[15] = 0x55;

    // write_flash(0x20000, &buf, buf.len());

    // crate::write_flash((FLASH_ADDRESS_OFFSET) as usize, &buf, buf.len());

    // hal_flash_write(0x3fffc, &buf, buf.len());

    // hal_flash_erase(0x20000, 16384); //WORKS

    let mut buf: [u8; 1] = [0x70; 1];
    buf[0] = 0x70;
    // buf[1] = 0x11;
    // buf[2] = 0x22;
    // buf[3] = 0x33;

    let buf_ref = &buf; // a shared reference to this data...
    let buf_ptr = buf_ref as *const u8;


    write_flash(0x10023ffb, buf_ptr, 1);

    // write_flash(0x10023000, 0x10020000 as *const u8, 4096);

    // buf = [0x35; 4096];

    // write_flash(0x21000, &buf, buf.len());

    // let data2 = crate::read_flash(FLASH_ADDRESS_OFFSET);

    // defmt::println!("Flash data[0]: {:?}", data2[0]);

    // chip_erase();

    // boot_sfrom(FLASH_XIP_APPLISTART_ADDR as usize);

    // defmt::println!("Flash Erase Complete.");

    loop {
        
    }

}

// End of file
