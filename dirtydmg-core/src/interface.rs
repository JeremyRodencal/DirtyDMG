/// Structure used to hold gameboy graphics data.
/// 
/// Pixels are bitpacked within bytes, with each pixel being 2 bits long.
/// Pixels are aranged in the bits such that the more significant the bits,
/// the further right they are on the screen. The same applies to the byte
/// Index, where higher values are farther right.
#[derive(Debug, PartialEq, Eq)]
pub struct ScanlineBuffer {
    pub pixeldata: [u8;40]
}

impl ScanlineBuffer{
    pub fn new() -> ScanlineBuffer{
        ScanlineBuffer {
            pixeldata: [0;40]
        }
    }
}

impl Default for ScanlineBuffer {
    fn default() -> Self {
        Self::new()
    }
}
