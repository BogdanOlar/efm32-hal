//! Analog Comparator

#[cfg(feature = "efemb")]
pub mod efemb;
pub mod irq;

use crate::{
    acmp::irq::default_handler,
    pac::acmp::vals::{Csressel, HysteresisHyst, Vasel},
    peripherals, Sealed, SingleCycleRMW,
};
use cortex_m::peripheral::NVIC;
use embassy_hal_internal::Peri;

/// Acmp peripheral ID
#[derive(Debug, Clone, Copy, Eq, PartialEq, Ord, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum AcmpId {
    /// Acmp0 peripheral
    Acmp0,
}

impl AcmpId {
    /// Acmp peripheral count
    pub const COUNT: usize = 1;
}

/// Handler function for an ACMP interrupt
pub type AcmpIrqHandler = fn(flags: Interrupts, regs: crate::pac::acmp::Acmp);

/// A timer peripheral instance usable by the HAL timer driver.
pub trait AcmpInstance: Sealed + embassy_hal_internal::PeripheralType + 'static {
    /// Returns the chiptool PAC register-block handle for this timer instance.
    fn regs(&self) -> crate::pac::acmp::Acmp;
    /// Acmp peripheral ID
    fn id(&self) -> AcmpId;
    /// Peripheral Interrupt
    fn interrupt(&self) -> crate::pac::Interrupt;
}

impl Sealed for peripherals::Acmp0 {}
impl AcmpInstance for peripherals::Acmp0 {
    fn regs(&self) -> crate::pac::acmp::Acmp {
        crate::pac::ACMP0
    }

    fn id(&self) -> AcmpId {
        AcmpId::Acmp0
    }

    fn interrupt(&self) -> crate::pac::Interrupt {
        crate::pac::Interrupt::ACMP0
    }
}

/// Analog Comparator
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Acmp<'d, T: AcmpInstance> {
    peri: Peri<'d, T>,
}

impl<'d, T: AcmpInstance> Acmp<'d, T> {
    /// New Acmp driver
    ///
    /// **NOTE**: This will enable the NVIC interrupt
    pub fn new(peri: Peri<'d, T>, config: &Config) -> Self {
        let mut instance = Self { peri };
        let p = instance.peri.regs();

        NVIC::mask(instance.peri.interrupt());

        // enable clock
        crate::pac::CMU
            .hfperclken0()
            .modify(|w| w.set_acmp(instance.peri.id() as usize, true));
        instance.disable_interrupts(Interrupts::ALL);
        instance.clear_interrupts(Interrupts::ALL);

        // Using `.write` to ensure rest of fields are set to `0`
        p.ctrl().write(|w| {
            w.set_fullbias(config.full_bias);
            w.set_biasprog(config.bias_prog.as_inner());
            w.set_accuracy(config.accuracy.into());
        });
        instance.set_hysteresis_0(config.hist_0);
        instance.set_hysteresis_1(config.hist_1);
        instance.set_input_cs_res(config.input_sel.cs_res_sel);
        instance.set_input_vlp(config.input_sel.vlp_sel);
        instance.set_input_vb(config.input_sel.vb_sel);
        instance.set_input_va(config.input_sel.va_sel);
        instance.set_input_neg(config.input_sel.neg_sel);
        instance.set_input_pos(config.input_sel.pos_sel);

        // initialize interrupts
        if let Some(handler) = config.interrupts.handler {
            instance.set_irq_handler(handler);
        } else {
            instance.clear_irq_handler();
        }
        if let Some(edge) = config.interrupts.edge {
            instance.enable_interrupt(Interruptsel::Edge(edge));
        }
        if config.interrupts.warmup {
            instance.enable_interrupt(Interruptsel::Warmup);
        }
        if config.interrupts.conflict {
            instance.enable_interrupt(Interruptsel::Conflict);
        }
        unsafe {
            NVIC::unmask(instance.peri.interrupt());
        };

        // enable/disable acmp
        instance.set_enabled(config.enabled);

        instance
    }

    /// Set Acmp enabled/disabled
    pub fn set_enabled(&mut self, enabled: bool) {
        self.peri.regs().ctrl().modify(|w| w.set_en(enabled));
    }

    /// Configure Capacitive Sense internal resistor
    pub fn set_input_cs_res(&mut self, cs_resistor: Option<CsResSel>) {
        self.peri.regs().inputsel().modify(|w| match cs_resistor {
            Some(res) => {
                w.set_csressel(Csressel::from_bits(res as u8));
                w.set_csresen(true);
            }
            None => w.set_csresen(false),
        });
    }

    /// Low-Power Sampled Voltage Selection
    pub fn set_input_vlp(&mut self, vlp: VlpSel) {
        self.peri
            .regs()
            .inputsel()
            .modify(|w| w.set_vlpsel(vlp.into()));
    }

    /// Select the input for the VB Divider
    pub fn set_input_vb(&mut self, vb: VbSel) {
        self.peri
            .regs()
            .inputsel()
            .modify(|w| w.set_vbsel(vb.into()));
    }

    /// Select the input for the VA Divider
    pub fn set_input_va(&mut self, va: VaSel) {
        self.peri
            .regs()
            .inputsel()
            .modify(|w| w.set_vasel(Vasel::from_bits(va as u8)));
    }

    /// Select negative input.
    pub fn set_input_neg(&mut self, neg: InputSelect) {
        self.peri
            .regs()
            .inputsel()
            .modify(|w| w.set_negsel(neg as u8));
    }

    /// Select positive input.
    pub fn set_input_pos(&mut self, pos: InputSelect) {
        self.peri
            .regs()
            .inputsel()
            .modify(|w| w.set_possel(pos as u8));
    }

    /// Set Hysteresis 0
    pub fn set_hysteresis_0(&mut self, hysteresis: Hysteresis) {
        self.peri.regs().hysteresis0().write(|w| {
            w.set_divvb(hysteresis.div_vb.as_inner());
            w.set_divva(hysteresis.div_va.as_inner());
            w.set_hyst(HysteresisHyst::from_bits(hysteresis.hyst as u8));
        });
    }

    /// Set Hysteresis 1
    pub fn set_hysteresis_1(&mut self, hysteresis: Hysteresis) {
        self.peri.regs().hysteresis1().write(|w| {
            w.set_divvb(hysteresis.div_vb.as_inner());
            w.set_divva(hysteresis.div_va.as_inner());
            w.set_hyst(HysteresisHyst::from_bits(hysteresis.hyst as u8));
        });
    }

    /// Set single interrupt flag
    pub fn get_interrupt(&self, flag: Interrupt) -> bool {
        (self.peri.regs().if_().read().0 & (1u32 << flag as u8)) != 0
    }

    /// Enable the given Acmp peripheral interrupt
    pub fn enable_interrupt(&mut self, interrupt: Interruptsel) {
        let p = self.peri.regs();

        match interrupt {
            Interruptsel::Edge(edge) => {
                match edge {
                    EdgeSelect::Rising => p.ctrl().modify(|w| {
                        w.set_irise(true);
                        w.set_ifall(false);
                    }),
                    EdgeSelect::Falling => p.ctrl().modify(|w| {
                        w.set_irise(false);
                        w.set_ifall(true);
                    }),
                    EdgeSelect::Both => p.ctrl().modify(|w| {
                        w.set_irise(true);
                        w.set_ifall(true);
                    }),
                }
                p.ien().modify(|w| w.set_edge(true));
            }
            Interruptsel::Warmup => p.ien().modify(|w| w.set_warmup(true)),
            Interruptsel::Conflict => p.ien().modify(|w| w.set_aportconflict(true)),
        }
    }

    /// Enable multiple interrupts
    pub fn enable_interrupts(&mut self, flags: Interrupts) {
        self.peri.regs().ien().sc_set(flags.0);
    }

    /// Disable multiple interrupts
    pub fn disable_interrupts(&mut self, flags: Interrupts) {
        self.peri.regs().ien().sc_clear(flags.0);
    }

    /// Get all interrupt flags
    pub fn get_interrupts(&mut self) -> Interrupts {
        Interrupts::new_unchecked(self.peri.regs().if_().read().0)
    }

    /// Set multiple interrupt flags
    pub fn set_interrupts(&mut self, flags: Interrupts) {
        self.peri.regs().ifs().sc_set(flags.0);
    }

    /// Clear multiple interrupt flags
    pub fn clear_interrupts(&mut self, flags: Interrupts) {
        self.peri.regs().ifc().sc_set(flags.0);
    }

    /// Async await for any enabled interrupt(s)
    #[cfg(feature = "efemb")]
    pub async fn interrupts(&self) -> Interrupts {
        efemb::InterruptsFuture::new(unsafe { self.peri.clone_unchecked() }).await
    }

    pub(crate) fn set_irq_handler(&mut self, handler: AcmpIrqHandler) {
        critical_section::with(|cs| {
            irq::ACMP_HANDLER[self.peri.id() as usize]
                .borrow(cs)
                .replace(handler)
        });
    }

    pub(crate) fn clear_irq_handler(&mut self) {
        critical_section::with(|cs| {
            irq::ACMP_HANDLER[self.peri.id() as usize]
                .borrow(cs)
                .replace(default_handler)
        });
    }
}

/// Acmp interrupt
#[derive(Debug, Clone, Copy, Eq, PartialEq, Ord, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Interruptsel {
    /// Rising or falling edge on the analog comparator output
    Edge(EdgeSelect),
    /// Warm-up
    Warmup,
    /// Set if any of the APORT BUSes being requested by the ACMPn are also being requested by another peripheral
    Conflict,
}

/// Acmp edge interrupt options
#[derive(Debug, Clone, Copy, Eq, PartialEq, Ord, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum EdgeSelect {
    /// Edge interrupt on Rising edge
    Rising,
    /// Edge interrupt on Falling edge
    Falling,
    /// Edge interrupt on both Rising and Falling edges
    Both,
}

impl From<Interruptsel> for Interrupt {
    fn from(value: Interruptsel) -> Self {
        match value {
            Interruptsel::Edge(_) => Self::Edge,
            Interruptsel::Warmup => Self::Warmup,
            Interruptsel::Conflict => Self::Conflict,
        }
    }
}

/// Acmp interrupt flags
#[derive(Debug, Default, Clone, Copy, Eq, PartialEq, Ord, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Interrupt {
    /// Rising or falling edge on the analog comparator output
    #[default]
    Edge = 0,
    /// Warm-up
    Warmup,
    /// Set if any of the APORT BUSes being requested by the ACMPn are also being requested by another peripheral
    Conflict,
}

/// Interrupt flags bitfield
#[derive(Debug, Default, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(transparent)]
pub struct Interrupts(u32);

impl Interrupts {
    /// All interrupt flags
    pub const ALL: Self = Self::new(Interrupt::Edge)
        .with_flag(Interrupt::Warmup)
        .with_flag(Interrupt::Conflict);

    /// Create interrupts bitfield starting from given flag
    pub const fn new(flag: Interrupt) -> Self {
        Self(1u32 << flag as u8)
    }

    /// Bitfield builder with given interrupt flag
    pub const fn with_flag(self, flag: Interrupt) -> Self {
        Self(self.0 | (1u32 << flag as u8))
    }

    /// Get state of given flag
    pub const fn is_set(&self, flag: Interrupt) -> bool {
        (self.0 & (1u32 << flag as u8)) != 0
    }

    /// Create a new bitfield from unchecked `u32` raw bitfield.
    ///
    /// This is meant to be used to convert the values of the IF* registers into [`Interrupts`]
    fn new_unchecked(raw_flags: u32) -> Self {
        Self(raw_flags)
    }
}

impl From<Interrupts> for u32 {
    fn from(value: Interrupts) -> Self {
        value.0
    }
}

/// Acmp interrupts configuration
#[derive(Debug, Default, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InteruptsConfig {
    /// Enable edge interrupts
    pub edge: Option<EdgeSelect>,
    /// Enable warmup interrupt
    pub warmup: bool,
    /// Enable conflict interrupt
    pub conflict: bool,
    /// Optionally provide an IRQ handler function
    pub handler: Option<AcmpIrqHandler>,
}

/// Acmp config
#[derive(Debug, Default, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// Full Bias Current
    pub full_bias: bool,
    ///  Bias current level
    pub bias_prog: BiasProg,
    /// Accuracy Mode
    pub accuracy: Accuracy,
    /// Hysteresis values used when the comparator output is 0
    pub hist_0: Hysteresis,
    /// Hysteresis values used when the comparator output is 1
    pub hist_1: Hysteresis,
    /// Input connections to the positive and negative inputs of the comparator
    pub input_sel: InputSelection,
    /// Interrupts configuration
    pub interrupts: InteruptsConfig,
    /// Acmp enabled
    pub enabled: bool,
}

/// Bias current level
#[derive(Debug, Default, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BiasProg {
    bias: u8,
}

impl BiasProg {
    /// Maximum value for [`BiasProg`]
    pub const MAX: Self = Self { bias: 63 };

    /// Try to construct [`BiasProg`] from a literal value.
    ///
    /// `0 <= val <= 63`
    pub const fn try_from_u8(val: u8) -> Option<Self> {
        match val {
            0..64 => Some(Self { bias: val }),
            _ => None,
        }
    }

    /// Get the raw value
    pub fn as_inner(&self) -> u8 {
        self.bias
    }
}

impl From<BiasProg> for u8 {
    fn from(value: BiasProg) -> Self {
        value.bias
    }
}

/// ACMP Accuracy Mode
///
/// **NOTE** high frequency changes can cause the ACMP performance to degrade. For such uses, such as quickly scanning
/// through multiple channels or setting the ACMP to oscillate for capacitive sense, this bit should be set to `High`
#[derive(Debug, Default, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Accuracy {
    /// ACMP operates in low-accuracy mode but consumes less current.
    #[default]
    Low,
    /// ACMP operates in high-accuracy mode but consumes more current.
    High,
}

impl From<Accuracy> for bool {
    fn from(value: Accuracy) -> Self {
        match value {
            Accuracy::Low => false,
            Accuracy::High => true,
        }
    }
}

/// Hysteresis
#[derive(Debug, Default, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Hysteresis {
    /// Divider for VB Voltage
    pub div_vb: DivV,
    /// Divider for VA Voltage
    pub div_va: DivV,
    /// Hysteresis Select
    pub hyst: Hyst,
}

/// Hysteresis Select
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Ord, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Hyst {
    #[doc = "No hysteresis."]
    #[default]
    Hyst0 = 0x0,
    #[doc = "14 mV hysteresis."]
    Hyst1 = 0x01,
    #[doc = "25 mV hysteresis."]
    Hyst2 = 0x02,
    #[doc = "30 mV hysteresis."]
    Hyst3 = 0x03,
    #[doc = "35 mV hysteresis."]
    Hyst4 = 0x04,
    #[doc = "39 mV hysteresis."]
    Hyst5 = 0x05,
    #[doc = "42 mV hysteresis."]
    Hyst6 = 0x06,
    #[doc = "45 mV hysteresis."]
    Hyst7 = 0x07,
    #[doc = "No hysteresis."]
    Hyst8 = 0x08,
    #[doc = "-14 mV hysteresis."]
    Hyst9 = 0x09,
    #[doc = "-25 mV hysteresis."]
    Hyst10 = 0x0a,
    #[doc = "-30 mV hysteresis."]
    Hyst11 = 0x0b,
    #[doc = "-35 mV hysteresis."]
    Hyst12 = 0x0c,
    #[doc = "-39 mV hysteresis."]
    Hyst13 = 0x0d,
    #[doc = "-42 mV hysteresis."]
    Hyst14 = 0x0e,
    #[doc = "-45 mV hysteresis."]
    Hyst15 = 0x0f,
}

/// Voltage divider
#[derive(Debug, Default, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DivV {
    div: u8,
}

impl DivV {
    /// Maximum value for [`DivV`]
    pub const MAX: Self = Self { div: 63 };

    /// Try to construct [`DivV`] from a literal value.
    ///
    /// `0 <= val <= 63`
    pub const fn try_from_u8(val: u8) -> Option<Self> {
        match val {
            0..64 => Some(Self { div: val }),
            _ => None,
        }
    }

    /// Get the raw value
    pub const fn as_inner(&self) -> u8 {
        self.div
    }
}

impl From<DivV> for u8 {
    fn from(value: DivV) -> Self {
        value.div
    }
}

/// Acmp Input Selection
#[derive(Debug, Default, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InputSelection {
    /// Enable internal capacitive sense resistor
    pub cs_res_sel: Option<CsResSel>,
    /// Low-Power Sampled Voltage Selection
    pub vlp_sel: VlpSel,
    /// VB Selection
    pub vb_sel: VbSel,
    /// VA Selection
    pub va_sel: VaSel,
    /// Negative Input Select
    pub neg_sel: InputSelect,
    /// Positive Input Select
    pub pos_sel: InputSelect,
}

/// Capacitive Sense Mode Internal Resistor Select
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Ord, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum CsResSel {
    #[doc = "Internal capacitive sense resistor value 0."]
    #[default]
    Res0 = 0x0,
    #[doc = "Internal capacitive sense resistor value 1."]
    Res1 = 0x01,
    #[doc = "Internal capacitive sense resistor value 2."]
    Res2 = 0x02,
    #[doc = "Internal capacitive sense resistor value 3."]
    Res3 = 0x03,
    #[doc = "Internal capacitive sense resistor value 4."]
    Res4 = 0x04,
    #[doc = "Internal capacitive sense resistor value 5."]
    Res5 = 0x05,
    #[doc = "Internal capacitive sense resistor value 6."]
    Res6 = 0x06,
    #[doc = "Internal capacitive sense resistor value 7."]
    Res7 = 0x07,
}

/// Low-Power Sampled Voltage Selection
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Ord, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum VlpSel {
    /// Low-Power Sampled Voltage is VADIV
    #[default]
    VaDiv,
    /// Low-Power Sampled Voltage is VBDIV
    VbDiv,
}

impl From<VlpSel> for bool {
    fn from(value: VlpSel) -> Self {
        match value {
            VlpSel::VaDiv => false,
            VlpSel::VbDiv => true,
        }
    }
}

/// Input for the VB Divider
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Ord, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum VbSel {
    /// 1.25V
    #[default]
    V1p25,
    /// 2.50V
    V2p50,
}

impl From<VbSel> for bool {
    fn from(value: VbSel) -> Self {
        match value {
            VbSel::V1p25 => false,
            VbSel::V2p50 => true,
        }
    }
}

/// Input for the VA Divider
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Ord, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum VaSel {
    /// VDD for the VA divider
    #[default]
    Vdd = 0x00,
    /// APORT2Y CHannel 0 for the VA divider
    Aport2YCh0 = 0x01,
    /// APORT2Y CHannel 2 for the VA divider
    Aport2YCh2 = 0x03,
    /// APORT2Y CHannel 4 for the VA divider
    Aport2YCh4 = 0x05,
    /// APORT2Y CHannel 6 for the VA divider
    Aport2YCh6 = 0x07,
    /// APORT2Y CHannel 8 for the VA divider
    Aport2YCh8 = 0x09,
    /// APORT2Y CHannel 10 for the VA divider
    Aport2YCh10 = 0x0B,
    /// APORT2Y CHannel 12 for the VA divider
    Aport2YCh12 = 0x0D,
    /// APORT2Y CHannel 14 for the VA divider
    Aport2YCh14 = 0x0F,
    /// APORT2Y CHannel 16 for the VA divider
    Aport2YCh16 = 0x11,
    /// APORT2Y CHannel 18 for the VA divider
    Aport2YCh18 = 0x13,
    /// APORT2Y CHannel 20 for the VA divider
    Aport2YCh20 = 0x15,
    /// APORT2Y CHannel 22 for the VA divider
    Aport2YCh22 = 0x17,
    /// APORT2Y CHannel 24 for the VA divider
    Aport2YCh24 = 0x19,
    /// APORT2Y CHannel 26 for the VA divider
    Aport2YCh26 = 0x1B,
    /// APORT2Y CHannel 28 for the VA divider
    Aport2YCh28 = 0x1D,
    /// APORT2Y CHannel 30 for the VA divider
    Aport2YCh30 = 0x1F,
    /// APORT1X CHannel 0 for the VA divider
    Aport1XCh0 = 0x20,
    /// APORT1Y CHannel 1 for the VA divider
    Aport1YCh1 = 0x21,
    /// APORT1X CHannel 2 for the VA divider
    Aport1XCh2 = 0x22,
    /// APORT1Y CHannel 3 for the VA divider
    Aport1YCh3 = 0x23,
    /// APORT1X CHannel 4 for the VA divider
    Aport1XCh4 = 0x24,
    /// APORT1Y CHannel 5 for the VA divider
    Aport1YCh5 = 0x25,
    /// APORT1X CHannel 6 for the VA divider
    Aport1XCh6 = 0x26,
    /// APORT1Y CHannel 7 for the VA divider
    Aport1YCh7 = 0x27,
    /// APORT1X CHannel 8 for the VA divider
    Aport1XCh8 = 0x28,
    /// APORT1Y CHannel 9 for the VA divider
    Aport1YCh9 = 0x29,
    /// APORT1X CHannel 10 for the VA divider
    Aport1XCh10 = 0x2A,
    /// APORT1Y CHannel 11 for the VA divider
    Aport1YCh11 = 0x2B,
    /// APORT1X CHannel 12 for the VA divider
    Aport1XCh12 = 0x2C,
    /// APORT1Y CHannel 13 for the VA divider
    Aport1YCh13 = 0x2D,
    /// APORT1X CHannel 14 for the VA divider
    Aport1XCh14 = 0x2E,
    /// APORT1Y CHannel 15 for the VA divider
    Aport1YCh15 = 0x2F,
    /// APORT1X CHannel 16 for the VA divider
    Aport1XCh16 = 0x30,
    /// APORT1Y CHannel 17 for the VA divider
    Aport1YCh17 = 0x31,
    /// APORT1X CHannel 18 for the VA divider
    Aport1XCh18 = 0x32,
    /// APORT1Y CHannel 19 for the VA divider
    Aport1YCh19 = 0x33,
    /// APORT1X CHannel 20 for the VA divider
    Aport1XCh20 = 0x34,
    /// APORT1Y CHannel 21 for the VA divider
    Aport1YCh21 = 0x35,
    /// APORT1X CHannel 22 for the VA divider
    Aport1XCh22 = 0x36,
    /// APORT1Y CHannel 23 for the VA divider
    Aport1YCh23 = 0x37,
    /// APORT1X CHannel 24 for the VA divider
    Aport1XCh24 = 0x38,
    /// APORT1Y CHannel 25 for the VA divider
    Aport1YCh25 = 0x39,
    /// APORT1X CHannel 26 for the VA divider
    Aport1XCh26 = 0x3A,
    /// APORT1Y CHannel 27 for the VA divider
    Aport1YCh27 = 0x3B,
    /// APORT1X CHannel 28 for the VA divider
    Aport1XCh28 = 0x3C,
    /// APORT1Y CHannel 29 for the VA divider
    Aport1YCh29 = 0x3D,
    /// APORT1X CHannel 30 for the VA divider
    Aport1XCh30 = 0x3E,
    /// APORT1Y CHannel 31 for the VA divider
    Aport1YCh31 = 0x3F,
}

/// Input select for POS and NEG inputs
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Ord, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum InputSelect {
    #[default]
    /// Dedicated APORT0X CHannel 0
    Aport0XCh0 = 0x00,
    /// Dedicated APORT0X CHannel 1
    Aport0XCh1,
    /// Dedicated APORT0X CHannel 2
    Aport0XCh2,
    /// Dedicated APORT0X CHannel 3
    Aport0XCh3,
    /// Dedicated APORT0X CHannel 4
    Aport0XCh4,
    /// Dedicated APORT0X CHannel 5
    Aport0XCh5,
    /// Dedicated APORT0X CHannel 6
    Aport0XCh6,
    /// Dedicated APORT0X CHannel 7
    Aport0XCh7,
    /// Dedicated APORT0X CHannel 8
    Aport0XCh8,
    /// Dedicated APORT0X CHannel 9
    Aport0XCh9,
    /// Dedicated APORT0X CHannel 10
    Aport0XCh10,
    /// Dedicated APORT0X CHannel 11
    Aport0XCh11,
    /// Dedicated APORT0X CHannel 12
    Aport0XCh12,
    /// Dedicated APORT0X CHannel 13
    Aport0XCh13,
    /// Dedicated APORT0X CHannel 14
    Aport0XCh14,
    /// Dedicated APORT0X CHannel 15
    Aport0XCh15,
    /// Dedicated APORT0Y CHannel 0
    Aport0YCh0,
    /// Dedicated APORT0Y CHannel 1
    Aport0YCh1,
    /// Dedicated APORT0Y CHannel 2
    Aport0YCh2,
    /// Dedicated APORT0Y CHannel 3
    Aport0YCh3,
    /// Dedicated APORT0Y CHannel 4
    Aport0YCh4,
    /// Dedicated APORT0Y CHannel 5
    Aport0YCh5,
    /// Dedicated APORT0Y CHannel 6
    Aport0YCh6,
    /// Dedicated APORT0Y CHannel 7
    Aport0YCh7,
    /// Dedicated APORT0Y CHannel 8
    Aport0YCh8,
    /// Dedicated APORT0Y CHannel 9
    Aport0YCh9,
    /// Dedicated APORT0Y CHannel 10
    Aport0YCh10,
    /// Dedicated APORT0Y CHannel 11
    Aport0YCh11,
    /// Dedicated APORT0Y CHannel 12
    Aport0YCh12,
    /// Dedicated APORT0Y CHannel 13
    Aport0YCh13,
    /// Dedicated APORT0Y CHannel 14
    Aport0YCh14,
    /// Dedicated APORT0Y CHannel 15
    Aport0YCh15,
    /// Dedicated APORT1X CHannel 0
    Aport1XCh0,
    /// Dedicated APORT1Y CHannel 1
    Aport1YCh1,
    /// Dedicated APORT1X CHannel 2
    Aport1XCh2,
    /// Dedicated APORT1Y CHannel 3
    Aport1YCh3,
    /// Dedicated APORT1X CHannel 4
    Aport1XCh4,
    /// Dedicated APORT1Y CHannel 5
    Aport1YCh5,
    /// Dedicated APORT1X CHannel 6
    Aport1XCh6,
    /// Dedicated APORT1Y CHannel 7
    Aport1YCh7,
    /// Dedicated APORT1X CHannel 8
    Aport1XCh8,
    /// Dedicated APORT1Y CHannel 9
    Aport1YCh9,
    /// Dedicated APORT1X CHannel 10
    Aport1XCh10,
    /// Dedicated APORT1Y CHannel 11
    Aport1YCh11,
    /// Dedicated APORT1X CHannel 12
    Aport1XCh12,
    /// Dedicated APORT1Y CHannel 13
    Aport1YCh13,
    /// Dedicated APORT1X CHannel 14
    Aport1XCh14,
    /// Dedicated APORT1Y CHannel 15
    Aport1YCh15,
    /// Dedicated APORT1X CHannel 16
    Aport1XCh16,
    /// Dedicated APORT1Y CHannel 17
    Aport1YCh17,
    /// Dedicated APORT1X CHannel 18
    Aport1XCh18,
    /// Dedicated APORT1Y CHannel 19
    Aport1YCh19,
    /// Dedicated APORT1X CHannel 20
    Aport1XCh20,
    /// Dedicated APORT1Y CHannel 21
    Aport1YCh21,
    /// Dedicated APORT1X CHannel 22
    Aport1XCh22,
    /// Dedicated APORT1Y CHannel 23
    Aport1YCh23,
    /// Dedicated APORT1X CHannel 24
    Aport1XCh24,
    /// Dedicated APORT1Y CHannel 25
    Aport1YCh25,
    /// Dedicated APORT1X CHannel 26
    Aport1XCh26,
    /// Dedicated APORT1Y CHannel 27
    Aport1YCh27,
    /// Dedicated APORT1X CHannel 28
    Aport1XCh28,
    /// Dedicated APORT1Y CHannel 29
    Aport1YCh29,
    /// Dedicated APORT1X CHannel 30
    Aport1XCh30,
    /// Dedicated APORT1Y CHannel 31
    Aport1YCh31,
    /// Dedicated APORT2Y CHannel 0
    Aport2YCh0,
    /// Dedicated APORT2X CHannel 1
    Aport2XCh1,
    /// Dedicated APORT2Y CHannel 2
    Aport2YCh2,
    /// Dedicated APORT2X CHannel 3
    Aport2XCh3,
    /// Dedicated APORT2Y CHannel 4
    Aport2YCh4,
    /// Dedicated APORT2X CHannel 5
    Aport2XCh5,
    /// Dedicated APORT2Y CHannel 6
    Aport2YCh6,
    /// Dedicated APORT2X CHannel 7
    Aport2XCh7,
    /// Dedicated APORT2Y CHannel 8
    Aport2YCh8,
    /// Dedicated APORT2X CHannel 9
    Aport2XCh9,
    /// Dedicated APORT2Y CHannel 10
    Aport2YCh10,
    /// Dedicated APORT2X CHannel 11
    Aport2XCh11,
    /// Dedicated APORT2Y CHannel 12
    Aport2YCh12,
    /// Dedicated APORT2X CHannel 13
    Aport2XCh13,
    /// Dedicated APORT2Y CHannel 14
    Aport2YCh14,
    /// Dedicated APORT2X CHannel 15
    Aport2XCh15,
    /// Dedicated APORT2Y CHannel 16
    Aport2YCh16,
    /// Dedicated APORT2X CHannel 17
    Aport2XCh17,
    /// Dedicated APORT2Y CHannel 18
    Aport2YCh18,
    /// Dedicated APORT2X CHannel 19
    Aport2XCh19,
    /// Dedicated APORT2Y CHannel 20
    Aport2YCh20,
    /// Dedicated APORT2X CHannel 21
    Aport2XCh21,
    /// Dedicated APORT2Y CHannel 22
    Aport2YCh22,
    /// Dedicated APORT2X CHannel 23
    Aport2XCh23,
    /// Dedicated APORT2Y CHannel 24
    Aport2YCh24,
    /// Dedicated APORT2X CHannel 25
    Aport2XCh25,
    /// Dedicated APORT2Y CHannel 26
    Aport2YCh26,
    /// Dedicated APORT2X CHannel 27
    Aport2XCh27,
    /// Dedicated APORT2Y CHannel 28
    Aport2YCh28,
    /// Dedicated APORT2X CHannel 29
    Aport2XCh29,
    /// Dedicated APORT2Y CHannel 30
    Aport2YCh30,
    /// Dedicated APORT2X CHannel 31
    Aport2XCh31,
    /// Dedicated APORT3X CHannel 0
    Aport3XCh0,
    /// Dedicated APORT3Y CHannel 1
    Aport3YCh1,
    /// Dedicated APORT3X CHannel 2
    Aport3XCh2,
    /// Dedicated APORT3Y CHannel 3
    Aport3YCh3,
    /// Dedicated APORT3X CHannel 4
    Aport3XCh4,
    /// Dedicated APORT3Y CHannel 5
    Aport3YCh5,
    /// Dedicated APORT3X CHannel 6
    Aport3XCh6,
    /// Dedicated APORT3Y CHannel 7
    Aport3YCh7,
    /// Dedicated APORT3X CHannel 8
    Aport3XCh8,
    /// Dedicated APORT3Y CHannel 9
    Aport3YCh9,
    /// Dedicated APORT3X CHannel 10
    Aport3XCh10,
    /// Dedicated APORT3Y CHannel 11
    Aport3YCh11,
    /// Dedicated APORT3X CHannel 12
    Aport3XCh12,
    /// Dedicated APORT3Y CHannel 13
    Aport3YCh13,
    /// Dedicated APORT3X CHannel 14
    Aport3XCh14,
    /// Dedicated APORT3Y CHannel 15
    Aport3YCh15,
    /// Dedicated APORT3X CHannel 16
    Aport3XCh16,
    /// Dedicated APORT3Y CHannel 17
    Aport3YCh17,
    /// Dedicated APORT3X CHannel 18
    Aport3XCh18,
    /// Dedicated APORT3Y CHannel 19
    Aport3YCh19,
    /// Dedicated APORT3X CHannel 20
    Aport3XCh20,
    /// Dedicated APORT3Y CHannel 21
    Aport3YCh21,
    /// Dedicated APORT3X CHannel 22
    Aport3XCh22,
    /// Dedicated APORT3Y CHannel 23
    Aport3YCh23,
    /// Dedicated APORT3X CHannel 24
    Aport3XCh24,
    /// Dedicated APORT3Y CHannel 25
    Aport3YCh25,
    /// Dedicated APORT3X CHannel 26
    Aport3XCh26,
    /// Dedicated APORT3Y CHannel 27
    Aport3YCh27,
    /// Dedicated APORT3X CHannel 28
    Aport3XCh28,
    /// Dedicated APORT3Y CHannel 29
    Aport3YCh29,
    /// Dedicated APORT3X CHannel 30
    Aport3XCh30,
    /// Dedicated APORT3Y CHannel 31
    Aport3YCh31,
    /// Dedicated APORT4Y CHannel 0
    Aport4YCh0,
    /// Dedicated APORT4X CHannel 1
    Aport4XCh1,
    /// Dedicated APORT4Y CHannel 2
    Aport4YCh2,
    /// Dedicated APORT4X CHannel 3
    Aport4XCh3,
    /// Dedicated APORT4Y CHannel 4
    Aport4YCh4,
    /// Dedicated APORT4X CHannel 5
    Aport4XCh5,
    /// Dedicated APORT4Y CHannel 6
    Aport4YCh6,
    /// Dedicated APORT4X CHannel 7
    Aport4XCh7,
    /// Dedicated APORT4Y CHannel 8
    Aport4YCh8,
    /// Dedicated APORT4X CHannel 9
    Aport4XCh9,
    /// Dedicated APORT4Y CHannel 10
    Aport4YCh10,
    /// Dedicated APORT4X CHannel 11
    Aport4XCh11,
    /// Dedicated APORT4Y CHannel 12
    Aport4YCh12,
    /// Dedicated APORT4X CHannel 13
    Aport4XCh13,
    /// Dedicated APORT4Y CHannel 14
    Aport4YCh14,
    /// Dedicated APORT4X CHannel 15
    Aport4XCh15,
    /// Dedicated APORT4Y CHannel 16
    Aport4YCh16,
    /// Dedicated APORT4X CHannel 17
    Aport4XCh17,
    /// Dedicated APORT4Y CHannel 18
    Aport4YCh18,
    /// Dedicated APORT4X CHannel 19
    Aport4XCh19,
    /// Dedicated APORT4Y CHannel 20
    Aport4YCh20,
    /// Dedicated APORT4X CHannel 21
    Aport4XCh21,
    /// Dedicated APORT4Y CHannel 22
    Aport4YCh22,
    /// Dedicated APORT4X CHannel 23
    Aport4XCh23,
    /// Dedicated APORT4Y CHannel 24
    Aport4YCh24,
    /// Dedicated APORT4X CHannel 25
    Aport4XCh25,
    /// Dedicated APORT4Y CHannel 26
    Aport4YCh26,
    /// Dedicated APORT4X CHannel 27
    Aport4XCh27,
    /// Dedicated APORT4Y CHannel 28
    Aport4YCh28,
    /// Dedicated APORT4X CHannel 29
    Aport4YCh29,
    /// Dedicated APORT4Y CHannel 30
    Aport4XCh30,
    /// Dedicated APORT4X CHannel 31
    Aport4XCh31,
    /// Low-Power Sampled Voltage
    Vlp = 0xFB,
    /// Divided VB Voltage
    VbDiv = 0xFC,
    /// Divided VA Voltage
    VaDiv = 0xFD,
    /// ACMPVDD as selected via PWRSEL
    Vdd = 0xFE,
    /// VSS
    Vss = 0xFF,
}
