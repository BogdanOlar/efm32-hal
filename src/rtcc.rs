//! Real Time Counter and Calendar

/// Rtcc Capture/Compare channel IDs
#[derive(Debug, Default, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum ChannelId {
    /// Rtcc Capture/Compare channel Id 0
    #[default]
    Ch0,
    /// Rtcc Capture/Compare channel Id 1
    Ch1,
    /// Rtcc Capture/Compare channel Id 2
    Ch2,
}
