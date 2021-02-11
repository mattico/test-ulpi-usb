#![allow(unused)]

use core::convert::TryInto;
use core::sync::atomic::{AtomicU32, Ordering};

use stm32h7xx_hal::{
    pac,
    prelude::*,
    rcc::{self, rec},
    time::{MicroSeconds, U32Ext},
    timer,
};

///! A 64-bit timer which ticks every 1 microsecond.
///! You must call `us_timer::handle_irq()` from the TIM2 interrupt
///! for this to work past 32-bits.

pub const CORE_CLOCK: u32 = 480_000_000;

pub static OVERFLOWS: AtomicU32 = AtomicU32::new(0);

/// Initializes the microsecond timer. Must call only once in init().
pub fn init(timer: pac::TIM2, timer_rec: rec::Tim2, clocks: &rcc::CoreClocks) {
    let mut timer = timer.tick_timer(1.mhz(), timer_rec, clocks);
    timer.listen(timer::Event::TimeOut);
    defmt::timestamp!("{=u64}", timestamp());
}

/// Call from the TIM2 interrupt to handle timer overflow and count past 32-bits.
/// The interrupt should be configured at maximum priority, it won't take very long.
pub fn handle_irq() {
    OVERFLOWS.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    let timer = unsafe { &*pac::TIM2::ptr() };
    timer.sr.modify(|_, w| w.uif().clear());
}

/// Returns the 64-bit number of microseconds after init() completed.
pub fn timestamp() -> u64 {
    let overflows = OVERFLOWS.load(Ordering::Relaxed) as u64;
    let timer = unsafe { &*pac::TIM2::ptr() };
    let ctr = timer.cnt.read().bits() as u64;
    (overflows << 32) + ctr
}

/// Pauses execution for the specified amount of time. Microsecond resolution.
/// May take longer than `duration` if an interrupt executes past the deadline.
pub fn wait(duration: impl Into<MicroSeconds>) {
    let duration = duration.into().0 as u64;
    let start = timestamp();
    while timestamp() - start < duration {
        cortex_m::asm::nop();
    }
}

#[derive(Copy, Clone)]
pub struct TimeoutError {
    msg: &'static str,
}

impl defmt::Format for TimeoutError {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "Timeout: {=str}", self.msg)
    }
}

pub fn wait_timeout(
    mut f: impl FnMut() -> bool,
    timeout: impl Into<MicroSeconds>,
    msg: &'static str,
) -> Result<(), TimeoutError> {
    let timeout = timeout.into().0 as u64;
    let start = timestamp();
    loop {
        if f() {
            return Ok(());
        }
        if timestamp() - start > timeout {
            return Err(TimeoutError { msg });
        }
    }
}
