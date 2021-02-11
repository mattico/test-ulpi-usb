#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

extern crate alloc;
extern crate defmt_rtt;
extern crate panic_probe;

#[global_allocator]
static ALLOCATOR: alloc_cortex_m::CortexMHeap = alloc_cortex_m::CortexMHeap::empty();

use alloc::string::ToString;

use stm32h7xx_hal::gpio::*;
use stm32h7xx_hal::hal::digital::v2::*;
use stm32h7xx_hal::usb_hs::{UsbBus, USB1_ULPI};
use stm32h7xx_hal::{prelude::*, stm32};

use usb_device::prelude::*;

static mut EP_MEMORY: [u32; 1024] = [0; 1024];

#[cortex_m_rt::entry]
fn main() -> ! {
    let start = cortex_m_rt::heap_start() as usize;
    unsafe { ALLOCATOR.init(start, 1024 * 10) }

    let dp = stm32::Peripherals::take().unwrap();
    let cp = stm32::CorePeripherals::take().unwrap();

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
            .freeze(pwrcfg, &dp.SYSCFG)
    };

    let clocks = ccdr.clocks;
    let prec = ccdr.peripheral;

    let mut delay = stm32h7xx_hal::delay::Delay::new(cp.SYST, clocks);

    // Initialize GPIO

    let gpioa = dp.GPIOA.split(prec.GPIOA);
    let gpiob = dp.GPIOB.split(prec.GPIOB);
    let gpioc = dp.GPIOC.split(prec.GPIOC);
    let _gpiod = dp.GPIOD.split(prec.GPIOD);
    let _gpioe = dp.GPIOE.split(prec.GPIOE);
    let _gpiof = dp.GPIOF.split(prec.GPIOF);
    let _gpiog = dp.GPIOG.split(prec.GPIOG);
    let _gpioh = dp.GPIOH.split(prec.GPIOH);
    let _gpioi = dp.GPIOI.split(prec.GPIOI);
    let gpioj = dp.GPIOJ.split(prec.GPIOJ);
    let _gpiok = dp.GPIOK.split(prec.GPIOK);

    defmt::info!("Initializing USB");

    let mut ulpi_reset = gpiob.pb2.into_push_pull_output();
    ulpi_reset.set_low().unwrap();
    delay.delay_ms(10u8);

    let mut usb_led = gpioj.pj2.into_open_drain_output();
    usb_led.set_high().unwrap();

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
    let mut cdc = usbd_serial::SerialPort::new(&usb_bus);
    let mut winusb = winusb::MicrosoftDescriptors;

    let mut device = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16D0, 0x0FE9))
        .manufacturer("Test")
        .product("USB Test")
        .device_class(usbd_serial::USB_CLASS_CDC)
        .self_powered(true)
        .max_power(0)
        .max_packet_size_0(64)
        .build();

    let bus = device.bus();
    let mut vid: u16 = bus.ulpi_read(0x00).unwrap_or(0) as u16;
    vid |= (bus.ulpi_read(0x01).unwrap_or(0) as u16) << 8;
    let mut pid: u16 = bus.ulpi_read(0x02).unwrap_or(0) as u16;
    pid |= (bus.ulpi_read(0x03).unwrap_or(0) as u16) << 8;
    defmt::info!(
        "USB Initialized: PHY VID={=u16:x} PHY PID={=u16:x}",
        vid,
        pid
    );

    let mut old_state = device.state();
    let mut iters = 0u32;

    loop {
        let new_data = device.poll(&mut [&mut winusb, &mut cdc]);

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

        if new_data {
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
            Err(e) => defmt::error!("read() -> {:?}", defmt::Debug2Format(&e)),
        }

        if iters % 1000 == 0 {
            defmt::info!("writing {}", iters);
            let data = iters.to_string();
            match cdc.write(data.as_bytes()) {
                Ok(n) => defmt::debug!("write({}) -> {}", data.len(), n),
                Err(e) => defmt::error!("write({}) -> {:?}", data.len(), defmt::Debug2Format(&e)),
            }
        }

        iters = iters.wrapping_add(1);
    }
}

/// We need to respond a certain way to a vendor request in order to be able to use MS's USB CDC driver
/// without an .inf file.
mod winusb {
    use core::convert::TryFrom;
    use usb_device::class_prelude::*;
    use usb_device::control::RequestType;

    const GET_OS_FEATURE: u8 = b'A';

    #[allow(non_snake_case)]
    #[repr(u16)]
    #[derive(num_enum::TryFromPrimitive)]
    pub enum OSFeatureDescriptorType {
        CompatibleID = 4,
        Properties = 5,
    }

    const MS_COMPATIBLE_ID_DESCRIPTOR: [u8; 40] = [
        0x28, 0x00, 0x00, 0x00, // Length 40 bytes
        0x00, 0x01, // Version
        0x04, 0x00, // Compatibility ID Descriptor index
        0x01, // Number of sections
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Reserved: 7 bytes
        0x01, // Interface Number
        0x01, // Reserved
        b'W', b'I', b'N', b'U', b'S', b'B', 0x00, 0x00, // Compatible ID: 8 bytes ASCII
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Sub-Compatible ID: 8 bytes ASCII
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Reserved: 6 bytes
    ];

    const IF2_MS_PROPERTIES_OS_DESCRIPTOR: [u8; 142] = [
        0x8e, 0x00, 0x00, 0x00, // Length, 40 bytes
        0x00, 0x01, // Version
        0x05, 0x00, // wIndex: properties
        0x01, 0x00, // wCount: a single property
        0x84, 0x00, 0x00, 0x00, // Property length
        0x01, 0x00, 0x00, 0x00, // dwPropertyDataType: REG_SZ
        0x28, 0x00, // name length
        b'D', 0, b'e', 0, b'v', 0, b'i', 0, b'c', 0, b'e', 0, b'I', 0, b'n', 0, b't', 0, b'e', 0,
        b'r', 0, b'f', 0, b'a', 0, b'c', 0, b'e', 0, b'G', 0, b'U', 0, b'I', 0, b'D', 0, 0, 0,
        0x4e, 0x00, 0x00, 0x00, // data length
        b'{', 0, b'C', 0, b'D', 0, b'B', 0, b'3', 0, b'B', 0, b'5', 0, b'A', 0, b'D', 0, b'-', 0,
        b'2', 0, b'9', 0, b'3', 0, b'B', 0, b'-', 0, b'4', 0, b'6', 0, b'6', 0, b'3', 0, b'-', 0,
        b'A', 0, b'A', 0, b'3', 0, b'6', 0, b'-', 0, b'1', 0, b'A', 0, b'A', 0, b'E', 0, b'4', 0,
        b'6', 0, b'4', 0, b'6', 0, b'3', 0, b'7', 0, b'7', 0, b'6', 0, b'}', 0, 0, 0,
    ];

    pub struct MicrosoftDescriptors;

    impl<B: UsbBus> UsbClass<B> for MicrosoftDescriptors {
        fn control_in(&mut self, xfer: ControlIn<B>) {
            let req = xfer.request();
            if req.request_type != RequestType::Vendor {
                return;
            }

            if req.request == GET_OS_FEATURE {
                match OSFeatureDescriptorType::try_from(req.index) {
                    Ok(OSFeatureDescriptorType::CompatibleID) => {
                        // Handle request for an Extended Compatible ID Descriptor.
                        // Interface number is ignored as there is only one device-wide
                        // Compatible ID Descriptor.
                        xfer.accept_with_static(&MS_COMPATIBLE_ID_DESCRIPTOR).ok();
                    }
                    Ok(OSFeatureDescriptorType::Properties) => {
                        // Handle request for an Extended Properties OS Descriptor.
                        match req.value as u8 {
                            2 => {
                                xfer.accept_with_static(&IF2_MS_PROPERTIES_OS_DESCRIPTOR)
                                    .ok();
                            }
                            _ => {
                                xfer.reject().ok();
                            }
                        }
                    }
                    _ => {
                        xfer.reject().ok();
                    }
                }
            }
        }
    }
}

#[defmt::panic_handler]
fn defmt_panic() -> ! {
    cortex_m::asm::udf();
}

#[cortex_m_rt::exception]
fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    defmt::panic!("Hard Fault: {=str}", alloc::format!("{:#?}", ef));
}

#[cortex_m_rt::exception]
fn DefaultHandler(irqn: i16) {
    defmt::panic!("DefaultHandler IRQn: {=i16}", irqn);
}

#[alloc_error_handler]
fn alloc_error_handler(layout: core::alloc::Layout) -> ! {
    panic!("Unable to allocate {} bytes", layout.size());
}
