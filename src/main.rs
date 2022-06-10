/*!
# Pico USB Serial Example

Creates a USB Serial device on a Pico board, with the USB driver running in
the main thread.

This will create a USB Serial device echoing anything it receives. Incoming
ASCII characters are converted to upercase, so you can tell it is working
and not just local-echo!

See the `Cargo.toml` file for Copyright and license details.
*/

#![no_std]
#![no_main]

// The macro for our start-up function
use cortex_m_rt::entry;

use core::*;

// GPIO traits
use embedded_hal::digital::v2::OutputPin;

// Time handling traits
use embedded_time::rate::*;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Pull in any important traits
use rp_pico::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

// USB Communications Class Device support
use usbd_serial::SerialPort;

use rp2040_hal::rom_data;

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
pub const FLASH_XIP_BASE: u32 = 0x1000_0000;
pub const FLASH_START: u32 = 0x0000_0000;
pub const FLASH_END: u32 = 0x0020_0000;
pub const FLASH_USER_SIZE: u32 = 16384; // Amount dedicated to user prefs/stuff

#[inline(never)]
#[link_section = ".data.ram_func"]
fn write_flash(data: &[u8]) {
    let addr = FLASH_END - FLASH_USER_SIZE;
    unsafe {
        cortex_m::interrupt::free(|_cs| {
            rom_data::connect_internal_flash();
            rom_data::flash_exit_xip();
            rom_data::flash_range_erase(addr, SECTOR_SIZE, BLOCK_SIZE, SECTOR_ERASE);
            rom_data::flash_range_program(addr, data.as_ptr(), data.len());
            rom_data::flash_flush_cache(); // Get the XIP working again
            rom_data::flash_enter_cmd_xip(); // Start XIP back up
        });
    }
    //defmt::println!("write_flash() Complete"); // TEMP
}

fn read_flash() -> &'static mut [u8] {
    let addr = (FLASH_XIP_BASE + FLASH_END - FLASH_USER_SIZE) as *mut u8;
    let my_slice = unsafe { slice::from_raw_parts_mut(addr, FLASH_USER_SIZE as usize) };
    my_slice
}

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then echoes any characters
/// received over USB Serial.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Set the LED to be an output
    let mut led_pin = pins.led.into_push_pull_output();

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Set up the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);

    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut said_hello = false;

    //let data = crate::read_flash();
    //defmt::println!("Flash data[0]: {:?}", data[0]);

    let mut buffer = [0; 1024];
    //buffer[0] = 0;

    let mut flash_data = crate::read_flash();

    loop {
        // A welcome message at the beginning
        if !said_hello && timer.get_counter() >= 2_000_000 {
            said_hello = true;

            let _ = serial.write(b"Flash Read:\r\n");
            let _ = serial.write(flash_data);

            //let _ = serial.write(b"Flash Write:\r\n");
            //crate::write_flash(&buf);

            //let mod_data = crate::read_flash();
            //let _ = serial.write(b"Flash Read:\r\n");
            //let _ = serial.write(mod_data);
        }

        // Check for new data
        if usb_dev.poll(&mut [&mut serial]) {
            let mut rcvd = [0u8; 256];
            match serial.read(&mut rcvd) {
                Err(_e) => {
                    // Do nothing
                }
                Ok(0) => {
                    // Do nothing
                }
                Ok(count) => {
                    // Convert to upper case
                    rcvd.iter_mut().take(count).for_each(|b| {
                        b.make_ascii_uppercase();
                    });

                    // Send back to the host

                    // let mut wr_ptr = &rcvd[..count];
                    // while !wr_ptr.is_empty() {
                    //     led_pin.set_high().unwrap();
                    //     delay.delay_ms(50);
                    //     match serial.write(wr_ptr) {
                    //         Ok(len) => wr_ptr = &wr_ptr[len..],
                    //         // On error, just drop unwritten data.
                    //         // One possible error is Err(WouldBlock), meaning the USB
                    //         // write buffer is full.
                    //         Err(_) => break,
                    //     };
                    //     delay.delay_ms(50);
                    //     led_pin.set_low().unwrap();
                    // }

                    led_pin.set_high().unwrap();

                    // buffer[0] = buffer[0] + 1;

                    //for i in slice::range(0, buffer.len()) {
                    //    buffer[i] = rcvd[0];
                    //}

                    for item in IntoIterator::into_iter(buffer).enumerate() {
                        let (i, _x): (usize, u8) = item;
                        //println!("array[{i}] = {x}");
                        buffer[i] = rcvd[0];
                    }

                    crate::write_flash(&buffer);
                    flash_data = crate::read_flash();
                    delay.delay_ms(50);
                    let _ = serial.write(b"Flash Read:\r\n");
                    let _ = serial.write(flash_data);
                    led_pin.set_low().unwrap();
                }
            }
        }

        //delay.delay_ms(50);
    }
}

// End of file
