#![no_std]
#![no_main]

extern crate defmt_rtt;
extern crate panic_probe;

use stm32h7xx_hal::gpio::*;
use stm32h7xx_hal::hal::digital::v2::*;
use stm32h7xx_hal::usb_hs::{UsbBus, USB1_ULPI};
use stm32h7xx_hal::{prelude::*, rcc, stm32};

use usb_device::prelude::*;

mod us_timer;
mod winusb;

static mut EP_MEMORY: [u32; 1024] = [0; 1024];

#[cortex_m_rt::entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().unwrap();
    let _cp = stm32::CorePeripherals::take().unwrap();

    // Initialize low-level hardware
    let ccdr = {
        // Allow debugging while sleeping or stopped
        // dp.DBGMCU.cr.modify(|_, w| {
        //     w.dbgsleep_d1()
        //         .set_bit()
        //         .dbgstby_d1()
        //         .set_bit()
        //         .dbgstop_d1()
        //         .set_bit()
        //         .dbgsleep_d1()
        //         .set_bit()
        //         .d1dbgcken()
        //         .set_bit()
        //         .d3dbgcken()
        //         .set_bit()
        // });

        // Enable CPU caches
        // cp.SCB.enable_icache();
        // cp.SCB.enable_dcache(&mut cp.CPUID);

        let pwr = dp.PWR.constrain();
        // Enable Voltage Offset 0 - 480MHz Overdrive
        let pwrcfg = pwr.vos0(&dp.SYSCFG).freeze();

        // Enable SRAMs
        // dp.RCC.ahb2enr.modify(|_, w| {
        //     w.sram1en()
        //         .enabled()
        //         .sram2en()
        //         .enabled()
        //         .sram3en()
        //         .enabled()
        // });

        // Configure clocks
        let rcc = dp.RCC.constrain();
        rcc.use_hse(24.mhz())
            .bypass_hse()
            .sysclk(480.mhz())
            .hclk(240.mhz())
            .pll1_strategy(rcc::PllConfigStrategy::Iterative)
            .freeze(pwrcfg, &dp.SYSCFG)
    };

    let clocks = ccdr.clocks;
    let prec = ccdr.peripheral;

    us_timer::init(dp.TIM2, prec.TIM2, &clocks);

    // Initialize GPIO

    let gpioa = dp.GPIOA.split(prec.GPIOA);
    let gpiob = dp.GPIOB.split(prec.GPIOB);
    let gpioc = dp.GPIOC.split(prec.GPIOC);
    let _gpiod = dp.GPIOD.split(prec.GPIOD);
    let _gpioe = dp.GPIOE.split(prec.GPIOE);
    let _gpiof = dp.GPIOF.split(prec.GPIOF);
    let gpiog = dp.GPIOG.split(prec.GPIOG);
    let _gpioh = dp.GPIOH.split(prec.GPIOH);
    let _gpioi = dp.GPIOI.split(prec.GPIOI);
    let gpioj = dp.GPIOJ.split(prec.GPIOJ);
    let _gpiok = dp.GPIOK.split(prec.GPIOK);

    let debug_btn = gpiog.pg8.into_pull_up_input();
    us_timer::wait(10.ms()); // Wait for pull-up to charge debounce cap
    if debug_btn.is_low().unwrap() {
        defmt::info!("Pausing startup due to debug button press");

        let mut released = false;
        let mut pressed = false;
        loop {
            match (released, pressed, debug_btn.is_low().unwrap()) {
                (true, true, true) => {
                    defmt::info!("Resuming startup due to debug button press");
                    break;
                }
                (_, _, false) => {
                    released = true;
                    pressed = false;
                }
                (_, _, true) => {
                    pressed = true;
                }
            };
            us_timer::wait(10.ms());
        }
    }

    defmt::info!("Initializing USB");

    let mut ulpi_reset = gpiob.pb2.into_push_pull_output();
    ulpi_reset.set_low().unwrap();
    us_timer::wait(10.ms());

    let mut usb_led = gpioj.pj2.into_open_drain_output();
    usb_led.set_high().unwrap();

    // Verify that you've configured the correct dir/nxt pins
    let usb_peripheral = USB1_ULPI {
        usb_global: dp.OTG1_HS_GLOBAL,
        usb_device: dp.OTG1_HS_DEVICE,
        usb_pwrclk: dp.OTG1_HS_PWRCLK,
        ulpi_dir: gpioc
            .pc2
            .into_alternate_af10()
            .set_speed(Speed::High)
            .into(),
        ulpi_stp: gpioc.pc0.into_alternate_af10().set_speed(Speed::High),
        ulpi_nxt: gpioc
            .pc3
            .into_alternate_af10()
            .set_speed(Speed::High)
            .into(),
        ulpi_d0: gpioa.pa3.into_alternate_af10().set_speed(Speed::High),
        ulpi_d1: gpiob.pb0.into_alternate_af10().set_speed(Speed::High),
        ulpi_d2: gpiob.pb1.into_alternate_af10().set_speed(Speed::High),
        ulpi_d3: gpiob.pb10.into_alternate_af10().set_speed(Speed::High),
        ulpi_d4: gpiob.pb11.into_alternate_af10().set_speed(Speed::High),
        ulpi_d5: gpiob.pb12.into_alternate_af10().set_speed(Speed::High),
        ulpi_d6: gpiob.pb13.into_alternate_af10().set_speed(Speed::High),
        ulpi_d7: gpiob.pb5.into_alternate_af10().set_speed(Speed::High),
        ulpi_clk: gpioa.pa5.into_alternate_af10().set_speed(Speed::High),
        prec: prec.USB1OTG,
        hclk: clocks.hclk(),
    };

    let usb_bus = UsbBus::new(usb_peripheral, unsafe { &mut EP_MEMORY });
    let mut cdc = usbd_serial::SerialPort::new_with_store(&usb_bus, [0u8; 1024], [0u8; 1024]);
    let mut winusb = winusb::MicrosoftDescriptors;

    let mut device = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16D0, 0x0FE9))
        .manufacturer("Test")
        .product("USB Test")
        .device_class(usbd_serial::USB_CLASS_CDC)
        .device_sub_class(2)
        .self_powered(true)
        .max_power(0)
        .max_packet_size_0(64)
        .build();

    let bus = device.bus();
    let mut vid: u16 = bus.ulpi_read(0x00).unwrap_or(0) as u16;
    vid |= (bus.ulpi_read(0x01).unwrap_or(0) as u16) << 8;
    let mut pid: u16 = bus.ulpi_read(0x02).unwrap_or(0) as u16;
    pid |= (bus.ulpi_read(0x03).unwrap_or(0) as u16) << 8;
    if vid == 0 || pid == 0 {
        defmt::panic!("USB PHY Not Responding");
    } else {
        defmt::info!(
            "USB Initialized: PHY VID={=u16:x} PHY PID={=u16:x}",
            vid,
            pid
        );
    }

    let mut old_state = device.state();
    let mut last_print = us_timer::timestamp();

    loop {
        let new_state = device.state();
        if new_state != old_state {
            match new_state {
                UsbDeviceState::Default => {
                    usb_led.set_high().unwrap();
                    defmt::info!("USB: Default");
                }
                UsbDeviceState::Addressed => {
                    usb_led.toggle().unwrap();
                    defmt::info!("USB: Addressed");
                }
                UsbDeviceState::Configured => {
                    usb_led.toggle().unwrap();
                    defmt::info!("USB: Configured");
                }
                UsbDeviceState::Suspend => {
                    usb_led.set_high().unwrap();
                    defmt::info!("USB: Suspended");
                }
            }
        }
        old_state = new_state;

        if device.poll(&mut [&mut winusb, &mut cdc]) {
            usb_led.toggle().unwrap();
        }

        let mut buf = [0u8; 64];
        match cdc.read(&mut buf) {
            Ok(n) if n > 0 => {
                defmt::debug!("read() -> {}", n);
                match cdc.write(&buf[..n]) {
                    Ok(n) => defmt::debug!("echo write() -> {}", n),
                    Err(e) => defmt::error!("echo write() -> {:?}", defmt::Debug2Format(&e)),
                }
            }
            Ok(0) => defmt::debug!("read() -> 0"),
            Ok(_) => defmt::unreachable!(),
            Err(UsbError::WouldBlock) => defmt::trace!("read() -> WouldBlock"),
            Err(e) => defmt::error!("read() -> {:?}", defmt::Debug2Format(&e)),
        }

        let now = us_timer::timestamp();
        if now - last_print >= 1000_000 && new_state == UsbDeviceState::Configured {
            last_print = now;
            defmt::info!("writing {}", now);
            match cdc.write(&now.to_le_bytes()) {
                Ok(n) => defmt::debug!("write(8) -> {}", n),
                Err(e) => defmt::error!("write(8) -> {:?}", defmt::Debug2Format(&e)),
            }
        }
    }
}
