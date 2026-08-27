//!
//! Driver for the Sitronix ST7796S, over its 4-wire serial (SPI) interface.
//!
//! Datasheet: ST7796S V1.0, Sitronix, 2014/11 (279 pages; command tables
//! start at "9.1 Command Table List").
//!
//! ## Why a packet union instead of a byte-slice `write_command`
//!
//! `ST77xx_Generic` in `st77xx.zig` sends a command as an opcode plus a
//! caller-assembled `[]const u8` of parameter bytes -- correct, but the
//! correctness lives entirely in the caller's head: nothing stops you from
//! passing three bytes to a two-byte command, or writing a gamma curve's
//! bytes in the wrong order. This driver instead gives every command its
//! own payload type, sized and bit-ordered to match its datasheet row
//! exactly, joined into one tagged union (`Packet`) keyed by opcode. `send`
//! is the single place a `Packet` becomes SPI bytes -- get the payload type
//! wrong and it's a compile error, not a bad frame on the wire days later.
//!
//! ## Scope
//!
//! Covers Command Table 1 (the base set) and the Command Table 2 registers
//! that apply to the SPI interface (power, VCOM, gamma, frame timing,
//! extended addressing). Left out on purpose:
//!
//!   - Commands specific to the parallel RGB interface -- this driver only
//!     speaks 4-wire SPI.
//!   - `NVMADW`/`NVMBPROG`/`NVMSTRD` -- one-time-programmable factory trim
//!     commands. They can permanently alter the panel's calibration; no
//!     end-user driver should be able to reach them by accident.
//!
//! A handful of Table 2 registers (`PWR3`, `RDID4`, `DOCA`) have datasheet
//! rows that don't extract cleanly into named bitfields (garbled OCR,
//! apparent typos in the source PDF). Rather than guess at bit positions in
//! code that tunes analog panel voltages, those are exposed as raw bytes
//! with a doc comment saying so -- see each type below.
//!
//! ## Byte order
//!
//! Every payload type here is either a `packed struct` whose fields are
//! declared in the same order the datasheet lists its bytes (byte 0's bits
//! first, low-to-high within each byte -- D0 before D7, matching this
//! package's existing convention in `st77xx.zig`'s `MemoryDataAccessControl`),
//! or a plain `[N]u8`. `@bitCast`/`std.mem.asBytes` on a little-endian
//! target then reproduces the datasheet's byte order directly. Every
//! microzig target today is little-endian, so this isn't qualified further
//! in the code below. The handful of registers with a genuine multi-byte
//! *value* (`CASET`'s X range, and similar) can't be expressed as a plain
//! bitfield struct this way -- the controller wants those big-endian -- so
//! those payload types store their wire bytes directly and offer an
//! `.init(...)` constructor that does the big-endian encoding once, so
//! callers still pass plain integers.
//!
//! ## Usage
//!
//! This driver does no chip I/O on its own beyond what you ask for --
//! `init` only wires up the bus and pins. Bringing the panel up (hardware
//! reset, `SWRESET`, `SLPOUT`, pixel format, orientation, and -- for
//! anything beyond factory defaults -- power/VCOM/gamma tuning) is
//! sequenced entirely by the caller through `send`, `rset`, and friends.
//! That's deliberate: which registers matter and what to set them to is a
//! property of the panel module, not of the ST7796S command set, and baking
//! one panel's tuning values into this file would make it useful for one
//! board and no others.
//!
//! Typical bring-up:
//!
//!     var lcd = ST7796S.init(bus, reset_pin, dc_pin);
//!     try lcd.rset(delay_ms);
//!     try lcd.send(.{ .swreset = {} });
//!     delay_ms(150);
//!     try lcd.send(.{ .slpout = {} });
//!     delay_ms(120);
//!     try lcd.send(.{ .colmod = .{ .mcu_format = .bpp16, .rgb_format = .bpp16 } });
//!     try lcd.send(.{ .madctl = .{ .addr = .deg0, .rgb = .rgb } });
//!     try lcd.send(.{ .dispon = {} });
//!
//!     try lcd.rect(0, 239, 0, 319);
//!     try lcd.fill(.{ .r = 31, .g = 0, .b = 0 }, 240 * 320); // red screen
//!

const std = @import("std");
const mdf = @import("../root.zig");

const DatagramDevice = mdf.base.DatagramDevice;
const Digital_IO = mdf.base.Digital_IO;

/// Every command this driver knows how to send, keyed by its datasheet hex
/// opcode. This *is* `Packet`'s tag type -- see `Packet` below.
pub const Opcode = enum(u8) {
    // -- Command Table 1 --
    nop = 0x00,
    swreset = 0x01,
    rddid = 0x04,
    rddst = 0x09,
    rddpm = 0x0A,
    rddmadctl = 0x0B,
    rddcolmod = 0x0C,
    rddim = 0x0D,
    rddsm = 0x0E,
    rddsdr = 0x0F,
    slpin = 0x10,
    slpout = 0x11,
    ptlon = 0x12,
    noron = 0x13,
    invoff = 0x20,
    invon = 0x21,
    dispoff = 0x28,
    dispon = 0x29,
    caset = 0x2A,
    raset = 0x2B,
    ramwr = 0x2C,
    ramrd = 0x2E,
    ptlar = 0x30,
    vscrdef = 0x33,
    teoff = 0x34,
    teon = 0x35,
    madctl = 0x36,
    vscrsadd = 0x37,
    idmoff = 0x38,
    idmon = 0x39,
    colmod = 0x3A,
    ramwrc = 0x3C,
    ramrdc = 0x3E,
    tescan = 0x44,
    rdtescan = 0x45,
    wrdisbv = 0x51,
    rddisbv = 0x52,
    wrctrld = 0x53,
    rdctrld = 0x54,
    wrcabc = 0x55,
    rdcabc = 0x56,
    wrcabcmb = 0x5E,
    rdcabcmb = 0x5F,
    rdfchksum = 0xAA,
    rdcchksum = 0xAF,
    rdid1 = 0xDA,
    rdid2 = 0xDB,
    rdid3 = 0xDC,

    // -- Command Table 2 (extended; SPI-relevant subset) --
    ifmode = 0xB0,
    frmctr1 = 0xB1,
    frmctr2 = 0xB2,
    frmctr3 = 0xB3,
    dic = 0xB4,
    bpc = 0xB5,
    dfc = 0xB6,
    em = 0xB7,
    pwr1 = 0xC0,
    pwr2 = 0xC1,
    pwr3 = 0xC2,
    vcmpctl = 0xC5,
    vmctl = 0xC6,
    rdid4 = 0xD3,
    pgc = 0xE0,
    ngc = 0xE1,
    dgc1 = 0xE2,
    dgc2 = 0xE3,
    doca = 0xE8,
    cscon = 0xF0,
    spirc = 0xFB,
};

/// One command plus its exact-width parameter payload -- the tag is the
/// opcode byte, the payload is what follows it on the wire. Build one with
/// e.g. `.{ .madctl = .{ .rgb = .bgr, .addr = .deg90 } }` or, for a
/// parameter-less command, `.{ .dispon = {} }`.
///
/// A command with no parameters (or one this driver reads back rather than
/// writes -- `rddid`, `rddst`, and the like) carries `void`: `send` skips
/// the data phase entirely for those. Commands read back through `read`
/// (see below) are listed here the same way; `read` supplies the opcode
/// itself and doesn't need a payload to send.
pub const Packet = union(Opcode) {
    nop: void,
    swreset: void,
    rddid: void,
    rddst: void,
    rddpm: void,
    rddmadctl: void,
    rddcolmod: void,
    rddim: void,
    rddsm: void,
    rddsdr: void,
    slpin: void,
    slpout: void,
    ptlon: void,
    noron: void,
    invoff: void,
    invon: void,
    dispoff: void,
    dispon: void,
    caset: ColumnAddressSet,
    raset: RowAddressSet,
    ramwr: void,
    ramrd: void,
    ptlar: PartialArea,
    vscrdef: VerticalScrollDefinition,
    teoff: void,
    teon: TearingEffectLine,
    madctl: MemoryAccessControl,
    vscrsadd: VerticalScrollStartAddress,
    idmoff: void,
    idmon: void,
    colmod: PixelFormat,
    ramwrc: void,
    ramrdc: void,
    tescan: TearScanline,
    rdtescan: void,
    wrdisbv: Brightness,
    rddisbv: void,
    wrctrld: DisplayControl,
    rdctrld: void,
    wrcabc: AdaptiveBrightnessControl,
    rdcabc: void,
    wrcabcmb: Brightness,
    rdcabcmb: void,
    rdfchksum: void,
    rdcchksum: void,
    rdid1: void,
    rdid2: void,
    rdid3: void,

    ifmode: InterfaceMode,
    frmctr1: FrameRateControl,
    frmctr2: FrameRateControlIdle,
    frmctr3: FrameRateControlIdle,
    dic: DisplayInversionControl,
    bpc: BlankingPorchControl,
    dfc: DisplayFunctionControl,
    em: EntryModeSet,
    pwr1: PowerControl1,
    pwr2: PowerControl2,
    /// Power Control 3. The datasheet row for this register (§9.3.11,
    /// opcode 0xC2) didn't extract into unambiguous bit positions -- pass
    /// the raw byte and check §9.3.11 directly before relying on
    /// particular bits. Common ST7796S reference init sequences use 0xA4
    /// here for LCD-glass source driving, but that's a panel-tuning value,
    /// not something this driver should assume for you.
    pwr3: u8,
    vcmpctl: VcomControl,
    vmctl: VcomOffset,
    /// Read ID4 (3 bytes). The datasheet's own field names for this row
    /// repeat ("ID41" appears twice where "ID41/ID42/ID43" is clearly
    /// intended) -- treated as opaque bytes rather than guessed at.
    rdid4: void,
    /// Positive Gamma Control (14 bytes). Left as a raw curve rather than
    /// 14 named nibble-pairs: the datasheet's own names (`V63P`, `V0P`, ...)
    /// are curve-point indices, not independently meaningful bits, and
    /// wrongly-declared bitfields here would be *harder* to get right than
    /// an opaque array. See §9.3.18 for what each byte controls.
    pgc: [14]u8,
    /// Negative Gamma Control (14 bytes) -- mirrors `pgc`. See §9.3.19.
    ngc: [14]u8,
    /// Digital Gamma Control 1 (64 bytes: 64 indices, each byte packing an
    /// R correction nibble in its high bits and a B correction nibble in
    /// its low bits). Only takes effect when digital gamma is enabled
    /// elsewhere (`dfc`). See §9.3.20.
    dgc1: [64]u8,
    /// Digital Gamma Control 2 (64 bytes) -- mirrors `dgc1`. See §9.3.21.
    dgc2: [64]u8,
    /// Display Output Ctrl Adjust (8 bytes). Mostly vendor-fixed constants
    /// per the datasheet rather than independently documented bits --
    /// treated as an opaque tuning blob. See §9.3.22 before changing it.
    doca: [8]u8,
    /// Command Set Control -- unlocks (`unlock_1`/`unlock_2`) or relocks
    /// (`lock_1`/`lock_2`) access to the rest of Command Table 2. The
    /// datasheet requires *two* separate `cscon` writes in sequence for
    /// either direction; there's no error signalled if you send only one
    /// or send them out of order, which is exactly the kind of mistake
    /// worth documenting rather than discovering on a bench. See the
    /// `unlock_1`/`unlock_2`/`lock_1`/`lock_2` constants below.
    cscon: u8,
    spirc: SpiReadControl,

    /// First of the two `cscon` values that unlock Command Table 2.
    pub const unlock_1: u8 = 0xC3;
    /// Second of the two `cscon` values that unlock Command Table 2. Send
    /// only after `unlock_1` has already gone out.
    pub const unlock_2: u8 = 0x96;
    /// First of the two `cscon` values that relock Command Table 2.
    pub const lock_1: u8 = 0x3C;
    /// Second of the two `cscon` values that relock Command Table 2. Send
    /// only after `lock_1` has already gone out.
    pub const lock_2: u8 = 0x69;
};

// ---------------------------------------------------------------------
// Command Table 1 payloads
// ---------------------------------------------------------------------

/// `CASET` (0x2A) -- the column (X) range that `RAMWR`/`RAMRD` will
/// address, inclusive on both ends. Big-endian on the wire, hence the
/// constructor rather than plain integer fields (see the file header's
/// "Byte order" note).
pub const ColumnAddressSet = extern struct {
    xs: [2]u8,
    xe: [2]u8,

    pub fn init(xs: u16, xe: u16) ColumnAddressSet {
        var self: ColumnAddressSet = undefined;
        std.mem.writeInt(u16, &self.xs, xs, .big);
        std.mem.writeInt(u16, &self.xe, xe, .big);
        return self;
    }
};

/// `RASET` (0x2B) -- the row (Y) range, mirrors `ColumnAddressSet`.
pub const RowAddressSet = extern struct {
    ys: [2]u8,
    ye: [2]u8,

    pub fn init(ys: u16, ye: u16) RowAddressSet {
        var self: RowAddressSet = undefined;
        std.mem.writeInt(u16, &self.ys, ys, .big);
        std.mem.writeInt(u16, &self.ye, ye, .big);
        return self;
    }
};

/// `PTLAR` (0x30) -- the row range partial mode displays, once `PTLON` is
/// active; everything outside it stays whatever it last held.
pub const PartialArea = extern struct {
    start: [2]u8,
    end: [2]u8,

    pub fn init(start: u16, end: u16) PartialArea {
        var self: PartialArea = undefined;
        std.mem.writeInt(u16, &self.start, start, .big);
        std.mem.writeInt(u16, &self.end, end, .big);
        return self;
    }
};

/// `VSCRDEF` (0x33) -- splits the panel into a fixed top area, a scrolling
/// middle area, and a fixed bottom area, in rows. `VSCRSADD` then picks
/// which row of GRAM appears at the top of the scrolling area.
pub const VerticalScrollDefinition = extern struct {
    top_fixed: [2]u8,
    scroll: [2]u8,
    bottom_fixed: [2]u8,

    pub fn init(top_fixed: u16, scroll: u16, bottom_fixed: u16) VerticalScrollDefinition {
        var self: VerticalScrollDefinition = undefined;
        std.mem.writeInt(u16, &self.top_fixed, top_fixed, .big);
        std.mem.writeInt(u16, &self.scroll, scroll, .big);
        std.mem.writeInt(u16, &self.bottom_fixed, bottom_fixed, .big);
        return self;
    }
};

pub const TearingEffectMode = enum(u1) {
    /// Tearing-effect pulse on V-blanking only.
    v_blank_only = 0,
    /// Tearing-effect pulse on both V-blanking and H-blanking.
    v_and_h_blank = 1,
};

/// `TEON` (0x35) -- enables the tearing-effect output pin and picks which
/// blanking edges it pulses on. Only useful if that pin is wired up.
pub const TearingEffectLine = packed struct(u8) {
    mode: TearingEffectMode,
    _reserved: u7 = 0,
};

pub const RefreshOrder = enum(u1) { forward = 0, reverse = 1 };
pub const ColorOrder = enum(u1) { rgb = 0, bgr = 1 };

/// The three orientation bits (`MV`/`MX`/`MY`) `MADCTL` uses together to
/// pick one of four rotations. Matches `st77xx.zig`'s
/// `MemoryDataAccessControl.AddressOrder` in spirit -- kept as a separate
/// type here since this driver's `MADCTL` also carries `MH`, which that
/// one doesn't expose.
pub const AddressOrder = packed struct(u3) {
    mv: u1,
    mx: u1,
    my: u1,

    pub const deg0: AddressOrder = .{ .mv = 0, .mx = 0, .my = 0 };
    pub const deg90: AddressOrder = .{ .mv = 1, .mx = 1, .my = 0 };
    pub const deg180: AddressOrder = .{ .mv = 0, .mx = 1, .my = 1 };
    pub const deg270: AddressOrder = .{ .mv = 1, .mx = 0, .my = 1 };
};

/// `MADCTL` (0x36) -- orientation and color order for everything `RAMWR`
/// writes afterwards. `addr` picks the rotation (see `AddressOrder`); `mh`
/// and `ml` flip the panel's own internal scan direction rather than the
/// logical addressing, and are rarely needed outside of unusual module
/// wiring.
pub const MemoryAccessControl = packed struct(u8) {
    _reserved: u2 = 0,
    mh: RefreshOrder = .forward,
    rgb: ColorOrder = .rgb,
    ml: RefreshOrder = .forward,
    addr: AddressOrder = .deg0,
};

/// `VSCRSADD` (0x37) -- see `VerticalScrollDefinition`.
pub const VerticalScrollStartAddress = extern struct {
    address: [2]u8,

    pub fn init(address: u16) VerticalScrollStartAddress {
        var self: VerticalScrollStartAddress = undefined;
        std.mem.writeInt(u16, &self.address, address, .big);
        return self;
    }
};

pub const InterfaceBits = enum(u3) {
    bpp12 = 0b011,
    bpp16 = 0b101,
    bpp18 = 0b110,
};

/// `COLMOD` (0x3A) -- pixel format, set independently for the MCU (SPI)
/// interface and the RGB interface (this driver only drives the former;
/// `rgb_format` matters only if the panel is *also* wired for the parallel
/// RGB interface at the same time). `draw`/`fill`/`read` all assume
/// `mcu_format = .bpp16` (RGB565) -- see the note on `read` in the driver
/// section below before using another format with them.
pub const PixelFormat = packed struct(u8) {
    mcu_format: InterfaceBits,
    _reserved0: u1 = 0,
    rgb_format: InterfaceBits,
    _reserved1: u1 = 0,
};

/// `TESCAN` (0x44) -- the scanline that a tearing-effect pulse should fire
/// on, when `TEON`'s mode calls for both edges. In panel rows, 0-indexed.
pub const TearScanline = extern struct {
    line: [2]u8,

    pub fn init(line: u16) TearScanline {
        var self: TearScanline = undefined;
        std.mem.writeInt(u16, &self.line, line, .big);
        return self;
    }
};

/// A plain 0-255 brightness value -- used as-is by `WRDISBV` (backlight
/// brightness) and `WRCABCMB` (the floor CABC won't dim below).
pub const Brightness = packed struct(u8) {
    value: u8,
};

/// `WRCTRLD` (0x53) -- turns the backlight and adaptive-brightness paths on
/// or off. `bctrl` gates whether `dd`/`bl` (and CABC, if enabled via
/// `WRCABC`) have any effect at all versus the backlight running at a
/// fixed level.
pub const DisplayControl = packed struct(u8) {
    _reserved0: u2 = 0,
    backlight_on: bool = false,
    dimming_on: bool = false,
    _reserved1: u1 = 0,
    brightness_control_on: bool = false,
    _reserved2: u2 = 0,
};

pub const CabcMode = enum(u2) {
    off = 0b00,
    user_interface = 0b01,
    still_picture = 0b10,
    moving_image = 0b11,
};

/// The third defined encoding (`0b10`) is absent from the datasheet's own
/// table for this field -- treated as reserved rather than guessed at.
pub const ColorEnhancementLevel = enum(u2) {
    low = 0b00,
    medium = 0b01,
    high = 0b11,
};

/// `WRCABC` (0x55) -- content-adaptive backlight control. `mode` picks
/// which of the panel's built-in dimming curves to use; has no effect
/// unless `WRCTRLD.brightness_control_on` is also set. `color_enhancement`
/// and `enhancement_level` are a separate knob (image color boosting),
/// independent of `mode`.
pub const AdaptiveBrightnessControl = packed struct(u8) {
    mode: CabcMode = .off,
    _reserved0: u2 = 0,
    enhancement_level: ColorEnhancementLevel = .low,
    _reserved1: u1 = 0,
    color_enhancement: bool = false,
};

// ---------------------------------------------------------------------
// Command Table 2 payloads (SPI-relevant extended registers)
// ---------------------------------------------------------------------

pub const SignalPolarity = enum(u1) { active_low = 0, active_high = 1 };
pub const PixelClockEdge = enum(u1) { rising = 0, falling = 1 };
pub const DataEnablePolarity = enum(u1) { active_high = 0, active_low = 1 };

/// `IFMODE` (0xB0) -- RGB-interface timing polarities, plus `spi_select`,
/// the one field of this register that matters to a pure-SPI driver: it
/// picks whether the panel's `DOUT` pin is used for 3/4-wire SPI reads
/// (`shared`) or left unused with all I/O on the single data line
/// (`three_or_four_wire`). Everything else here is inert unless the RGB
/// interface is also wired up.
pub const InterfaceMode = packed struct(u8) {
    data_enable: DataEnablePolarity = .active_high,
    pixel_clock: PixelClockEdge = .rising,
    hsync: SignalPolarity = .active_low,
    vsync: SignalPolarity = .active_low,
    _reserved: u3 = 0,
    spi_select: enum(u1) { shared = 0, three_or_four_wire = 1 } = .shared,
};

/// `FRMCTR1` (0xB1) -- frame rate in normal (full-color) mode: an internal
/// clock divider (`clock_div`), a frame-frequency select (`rate_select`),
/// and a per-line timing count (`line_period`). See §9.3.2 for the exact
/// frequency formula; defaults match the datasheet's own power-on value
/// (0xA0, 0x10).
pub const FrameRateControl = packed struct(u16) {
    clock_div: u2 = 0b00,
    _reserved0: u2 = 0,
    rate_select: u4 = 0xA,
    line_period: u5 = 0x10,
    _reserved1: u3 = 0,
};

/// `FRMCTR2` (0xB2, idle-mode) / `FRMCTR3` (0xB3, partial-mode) -- the
/// per-line timing count for those display modes. Structurally simpler
/// than `FRMCTR1`: idle and partial mode don't have an independent clock
/// divider or rate-select field.
pub const FrameRateControlIdle = packed struct(u16) {
    _reserved0: u8 = 0,
    line_period: u5 = 0x10,
    _reserved1: u3 = 0,
};

/// `DIC`/`INVTR` (0xB4) -- forces one of three inversion modes regardless
/// of what `INVON`/`INVOFF` selected; the fourth encoding is reserved.
/// Defaults to `.one_dot`, the datasheet's own power-on value.
pub const InversionMode = enum(u2) {
    column = 0b00,
    one_dot = 0b01,
    two_dot = 0b10,
};

pub const DisplayInversionControl = packed struct(u8) {
    dinv: InversionMode = .one_dot,
    _reserved: u6 = 0,
};

/// `BPC` (0xB5) -- vertical/horizontal blanking porch widths, in lines
/// (`front_porch`/`back_porch`) and pixel clocks (`h_back_porch`). Only
/// meaningful with the RGB interface active. Defaults match the
/// datasheet's own power-on values.
pub const BlankingPorchControl = packed struct(u32) {
    front_porch: u8 = 0x02,
    back_porch: u8 = 0x02,
    _reserved: u8 = 0,
    h_back_porch: u8 = 0x04,
};

pub const ScanDirection = enum(u1) { forward = 0, reverse = 1 };

/// `DFC` (0xB6) -- gate/source driver scan configuration and the panel's
/// line count (`line_count`, in units of 8 lines: physical rows =
/// `(line_count + 1) * 8`, defaulting to 0x3B -> 480, the ST7796S's
/// maximum). `line_count` is the one field of this register this driver's
/// callers are likely to actually need to set (it must match the panel's
/// real resolution for the address window to behave); the rest are
/// wiring-dependent scan-order knobs -- `sm`/`gs` in particular only mean
/// something as a pair, looked up against the gate-output-sequence table
/// in §9.3.7, not independently. See §9.3.7 before changing any of them
/// from their power-on defaults.
pub const DisplayFunctionControl = packed struct(u24) {
    /// Source/VCOM output level in the non-display area (`PT[1:0]`).
    pt: u2 = 0,
    /// Gate/source scan mode in the non-display area (`PTG[1:0]`).
    ptg: u2 = 0,
    _reserved0: u1 = 0,
    /// `RM`: which interface writes GRAM (`false` = system/SPI, `true` =
    /// RGB). This driver only speaks SPI, so this should stay `false`.
    rgb_ram_access: bool = false,
    /// `RCM`: RGB interface transfer mode (DE vs. SYNC). Irrelevant unless
    /// the RGB interface is also wired up.
    rcm_sync_mode: bool = false,
    bypass: bool = true,

    /// `ISC[3:0]`: scan cycle length when `ptg` selects interval scan.
    scan_cycle: u4 = 0b0010,
    /// `SM`: gate driver pin arrangement, paired with `gs` -- see the
    /// type doc comment.
    sm: bool = false,
    /// `SS`: source driver output shift direction.
    source_output: ScanDirection = .forward,
    /// `GS`: gate driver output shift direction, paired with `sm`.
    gate_output: ScanDirection = .forward,
    _reserved1: u1 = 0,

    line_count: u6 = 0x3B,
    _reserved2: u2 = 0,
};

/// How a 16bpp (R,G,B) pixel gets expanded to the panel's internal 18bpp
/// (r,g,b) GRAM storage, for `EntryModeSet.color_expansion`.
pub const ColorExpansion = enum(u2) {
    /// r(0) = b(0) = 0.
    zero = 0b00,
    /// r(0) = b(0) = 1.
    one = 0b01,
    /// The 5/6-bit value's MSB is copied down into the new LSB.
    msb_to_lsb = 0b10,
    /// r(0) = b(0) = the pixel's green LSB.
    match_green = 0b11,
};

/// `EM` (0xB7) -- `gon`/`dte` together select the gate driver's output
/// level (see the table below); every combination other than the
/// power-on default parks the gate driver at a fixed rail rather than
/// driving the panel, so leave both `true` unless you're deliberately
/// sequencing power-up/down. `deep_standby` cuts internal logic and SRAM
/// power entirely -- exiting it requires either six CS low pulses or a
/// hardware reset, not just clearing this bit again.
///
///     gon    dte    G1..G480
///     false  false  VGH
///     false  true   VGH
///     true   false  VGL
///     true   true   normal display (the default)
pub const EntryModeSet = packed struct(u8) {
    _reserved0: u1 = 0,
    dte: bool = true,
    gon: bool = true,
    deep_standby: bool = false,
    _reserved1: u2 = 0,
    color_expansion: ColorExpansion = .zero,
};

/// `PWR1` (0xC0) -- the two analog rails everything else on the panel is
/// derived from: `avdd` (positive analog supply) and `avcl` (negative
/// analog supply), plus the gate-driver high/low rail selects (`vghs`/
/// `vgls`). These are panel-specific tuning values from the module's own
/// datasheet or reference design, not something this driver should guess
/// at -- get them wrong and you can exceed the glass's rated voltage.
/// Defaults match the datasheet's own power-on values (§9.3.9): AVDD
/// 6.60V, AVCL -4.4V, VGH 13.257V, VGL -10.429V.
pub const PowerControl1 = packed struct(u16) {
    _reserved0: u4 = 0,
    avcl: u2 = 0b00,
    avdd: u2 = 0b10,

    vgls: u3 = 0b101,
    _reserved1: u1 = 0,
    vghs: u3 = 0b010,
    _reserved2: u1 = 0,
};

/// `PWR2` (0xC1) -- `vrh` sets the GVDD reference level that `PWR1`'s rails
/// (and so the whole gamma curve) are scaled against. Same caveat as
/// `PowerControl1`: a panel-tuning value, not a driver default -- kept at
/// the datasheet's own power-on value (0x13) here only because *some*
/// default has to compile.
pub const PowerControl2 = packed struct(u8) {
    vrh: u7 = 0x13,
    _reserved: u1 = 0,
};

/// `VCMPCTL` (0xC5) -- VCOM voltage. The single most commonly hand-tuned
/// register on this whole chip (it directly controls flicker/crosstalk);
/// panel-specific. Defaults to the datasheet's own power-on value, 0x1C
/// (VCOM = 1.0V).
pub const VcomControl = packed struct(u8) {
    vcom: u6 = 0x1C,
    _reserved: u2 = 0,
};

/// `VMCTL`/"VCOM Offset" (0xC6) -- an additional trim on top of
/// `VcomControl`, selected between two internal reference sources by
/// `use_register`.
pub const VcomOffset = packed struct(u8) {
    offset: u6 = 0,
    _reserved: u1 = 0,
    use_register: bool = false,
};

/// `SPIRC` (0xFB, read-only) -- how many dummy clocks the controller
/// expects before read data on the SPI bus, and whether that count is
/// currently enabled. Mostly useful for diagnosing a bring-up where
/// `read`'s single dummy byte (see below) doesn't match what the panel
/// was actually configured for at reset.
pub const SpiReadControl = packed struct(u8) {
    dummy_clocks: u4,
    enabled: bool,
    _reserved: u3 = 0,
};

/// The panel's native pixel format: RGB565, matching `PixelFormat.bpp16`.
/// `draw`, `fill`, and `read` all operate in this format.
pub const Color = packed struct(u16) {
    b: u5 = 0,
    g: u6 = 0,
    r: u5 = 0,
};

comptime {
    assertPayloadLayouts();
}

/// Every `Packet` payload, checked against the byte width its datasheet
/// row declares. Catches a mis-sized or misaligned struct at every
/// compile, the same guarantee `graphics/types.zig`'s `assertLayout`
/// gives `controller`'s own types (that file lives in a different repo;
/// this is the same idea, applied here).
fn assertPayloadLayouts() void {
    const checks = .{
        .{ ColumnAddressSet, 4 },
        .{ RowAddressSet, 4 },
        .{ PartialArea, 4 },
        .{ VerticalScrollDefinition, 6 },
        .{ TearingEffectLine, 1 },
        .{ MemoryAccessControl, 1 },
        .{ VerticalScrollStartAddress, 2 },
        .{ PixelFormat, 1 },
        .{ TearScanline, 2 },
        .{ Brightness, 1 },
        .{ DisplayControl, 1 },
        .{ AdaptiveBrightnessControl, 1 },
        .{ InterfaceMode, 1 },
        .{ FrameRateControl, 2 },
        .{ FrameRateControlIdle, 2 },
        .{ DisplayInversionControl, 1 },
        .{ BlankingPorchControl, 4 },
        .{ DisplayFunctionControl, 3 },
        .{ EntryModeSet, 1 },
        .{ PowerControl1, 2 },
        .{ PowerControl2, 1 },
        .{ VcomControl, 1 },
        .{ VcomOffset, 1 },
        .{ SpiReadControl, 1 },
        .{ Color, 2 },
    };

    for (checks) |check| {
        const T = check[0];
        const expected_size = check[1];
        if (@sizeOf(T) != expected_size) @compileError(std.fmt.comptimePrint(
            "{s} must be {d} byte(s) to match its datasheet row, but is {d}",
            .{ @typeName(T), expected_size, @sizeOf(T) },
        ));
    }
}

/// Everything `init`/`rset`/`send`/`rect`/`draw`/`fill`/`read` can fail
/// with -- one set for all seven, rather than a different error union per
/// function, so every call site (and every fault-injection test) handles
/// failure the same way.
pub const Error = DatagramDevice.AnyError || Digital_IO.WriteError;

const Self = @This();

bus: DatagramDevice,
reset: Digital_IO,
data_cmd: Digital_IO,

/// Wires up the driver against a bus and two GPIOs. No chip I/O happens
/// here -- see the file header for why bring-up is entirely the caller's
/// job via `rset`/`send`.
pub fn init(bus: DatagramDevice, reset: Digital_IO, data_cmd: Digital_IO) Self {
    return .{ .bus = bus, .reset = reset, .data_cmd = data_cmd };
}

/// The *electrical* reset: pulses the `reset` pin per the datasheet's
/// power-on-sequence timing (RESX held high, then low for >=10us, then
/// high again, with >=120ms before the panel will accept `SLPOUT`).
/// Distinct from `send(.{.swreset = {}})` (the software-reset opcode) --
/// the datasheet requires a hardware reset at least once after power-on
/// before software reset is guaranteed to work.
pub fn rset(self: Self, delay_ms: *const fn (ms: u32) void) Error!void {
    try self.reset.write(.high);
    delay_ms(5);
    try self.reset.write(.low);
    delay_ms(15);
    try self.reset.write(.high);
    delay_ms(120);
}

/// Turns a `Packet` into SPI bytes: the opcode with `data_cmd` low, then
/// -- if the packet carries a parameter payload -- that payload with
/// `data_cmd` high. Both phases happen inside one bus transaction (`CS`
/// stays asserted throughout), matching how `st77xx.zig`'s
/// `write_command` treats a command and its parameters as one transfer.
///
/// This is the only place a `Packet` is serialized; every other function
/// below is built from it.
pub fn send(self: Self, packet: Packet) Error!void {
    const opcode: Opcode = packet;

    try self.data_cmd.write(.low);
    try self.bus.connect();
    defer self.bus.disconnect();
    try self.bus.write(&.{@intFromEnum(opcode)});

    switch (packet) {
        inline else => |payload| {
            if (@TypeOf(payload) != void) {
                try self.data_cmd.write(.high);
                try self.bus.write(std.mem.asBytes(&payload));
            }
        },
    }
}

/// Sets the column/row window that `draw`/`fill`/`read` (or a manual
/// `RAMWR`/`RAMRD`) address next -- `CASET` then `RASET`, both inclusive
/// ranges. Deliberately doesn't touch `RAMWR`/`RAMRD` itself: addressing
/// and memory access are separate operations on this controller (and in
/// this driver), so setting a window doesn't commit you to reading or
/// writing it.
pub fn rect(self: Self, xs: u16, xe: u16, ys: u16, ye: u16) Error!void {
    try self.send(.{ .caset = .init(xs, xe) });
    try self.send(.{ .raset = .init(ys, ye) });
}

/// Writes a run of distinct pixel values into the current window (set via
/// `rect`) -- a blit. Issues `RAMWR`, then streams `pixels` as one
/// transfer with `CS` held low throughout: a `CS` edge mid-stream ends the
/// transfer on this controller, so the whole run has to be one bus
/// transaction, not one per pixel.
pub fn draw(self: Self, pixels: []const Color) Error!void {
    try self.send(.{ .ramwr = {} });
    try self.data_cmd.write(.high);
    try self.bus.connect();
    defer self.bus.disconnect();
    try self.bus.write(std.mem.sliceAsBytes(pixels));
}

/// Writes `count` copies of one color into the current window -- the
/// common case (clearing or filling a rectangle) without needing a
/// `count`-sized buffer of repeated pixels. Same held-`CS` reasoning as
/// `draw`.
pub fn fill(self: Self, color: Color, count: u32) Error!void {
    try self.send(.{ .ramwr = {} });
    try self.data_cmd.write(.high);
    try self.bus.connect();
    defer self.bus.disconnect();

    var remaining = count;
    while (remaining > 0) : (remaining -= 1) {
        try self.bus.write(std.mem.asBytes(&color));
    }
}

/// Issues a read-type command and fills `buf` with its response.
///
/// Every read command on this controller -- `RAMRD` included -- sends
/// exactly one dummy byte before the real response starts; `read`
/// discards that byte itself, so `buf` should be sized for the response
/// alone (e.g. `buf.len == 2` for one `RAMRD` pixel in the panel's default
/// RGB565 format, more for a multi-pixel read, 4 bytes for `RDDST`, and so
/// on -- see each command's `Packet` payload comment above for its
/// response width).
///
/// One thing this driver does *not* pin down, because it varies across
/// the ST77xx family and needs checking against this exact panel's
/// configuration: whether `RAMRD` reads back in the same 16-bit format
/// `draw`/`fill` write in, or in the controller's wider native format
/// regardless of `COLMOD`. Confirm against the datasheet's `RAMRD`
/// section (and `RDDCOLMOD`, if in doubt at runtime) before assuming
/// `read`'s bytes line up with `Color` for pixel data specifically --
/// they do for every other read command here, which all return fixed,
/// unambiguous byte widths.
pub fn read(self: Self, packet: Packet, buf: []u8) Error!void {
    const opcode: Opcode = packet;

    try self.data_cmd.write(.low);
    try self.bus.connect();
    defer self.bus.disconnect();
    try self.bus.write(&.{@intFromEnum(opcode)});

    try self.data_cmd.write(.high);
    var dummy: [1]u8 = undefined;
    _ = try self.bus.read(&dummy);
    _ = try self.bus.read(buf);
}

// =======================================================================
// Tests
// =======================================================================
//
// Two layers, per the project plan: happy-path tests covering every
// `Packet` variant plus each of the seven driver functions, and a
// fault-injection suite covering the failure modes actually reachable at
// this API's level (a bus call failing mid-operation). Deliberately not a
// full deterministic-simulation harness (à la TigerBeetle): this driver is
// synchronous, single-threaded, and every operation is a short, fixed bus
// call sequence with no retry or recovery logic to explore -- the fault
// surface is small enough to enumerate by hand, which is what the fake
// below does.

const testing = std.testing;

fn noopDelay(_: u32) void {}

test "every Packet variant sends its documented opcode and payload width" {
    inline for (@typeInfo(Packet).@"union".fields) |field| {
        var dc = Digital_IO.TestDevice.init(.output, .low);
        var rst = Digital_IO.TestDevice.init(.output, .low);
        var td = DatagramDevice.TestDevice.init_receiver_only();
        defer td.deinit();

        const lcd = init(td.datagram_device(), rst.digital_io(), dc.digital_io());

        const payload: field.type = if (field.type == void) {} else std.mem.zeroes(field.type);
        const packet = @unionInit(Packet, field.name, payload);

        try lcd.send(packet);

        const opcode: Opcode = packet;
        try testing.expectEqual(@intFromEnum(opcode), td.packets.items[0][0]);

        var total_len: usize = 0;
        for (td.packets.items) |sent| total_len += sent.len;
        try testing.expectEqual(1 + @sizeOf(field.type), total_len);
    }
}

test "madctl encodes rotation and color order" {
    var dc = Digital_IO.TestDevice.init(.output, .low);
    var rst = Digital_IO.TestDevice.init(.output, .low);
    var td = DatagramDevice.TestDevice.init_receiver_only();
    defer td.deinit();

    const lcd = init(td.datagram_device(), rst.digital_io(), dc.digital_io());
    try lcd.send(.{ .madctl = .{ .rgb = .bgr, .addr = .deg90 } });

    // D7..D0 = my(0) mx(1) mv(1) ml(0) rgb(1) mh(0) 0 0 = 0b0110_1000.
    try td.expect_sent(&.{ &.{0x36}, &.{0b0110_1000} });
}

test "colmod packs mcu and rgb interface formats into one byte" {
    var dc = Digital_IO.TestDevice.init(.output, .low);
    var rst = Digital_IO.TestDevice.init(.output, .low);
    var td = DatagramDevice.TestDevice.init_receiver_only();
    defer td.deinit();

    const lcd = init(td.datagram_device(), rst.digital_io(), dc.digital_io());
    try lcd.send(.{ .colmod = .{ .mcu_format = .bpp16, .rgb_format = .bpp16 } });

    // D7..D0 = 0 101 0 101 0 = 0b0101_0101 = 0x55, the standard "16bpp
    // both interfaces" value used throughout the ST77xx family.
    try td.expect_sent(&.{ &.{0x3A}, &.{0x55} });
}

test "extended-command unlock and relock are each two separate cscon writes" {
    var dc = Digital_IO.TestDevice.init(.output, .low);
    var rst = Digital_IO.TestDevice.init(.output, .low);
    var td = DatagramDevice.TestDevice.init_receiver_only();
    defer td.deinit();

    const lcd = init(td.datagram_device(), rst.digital_io(), dc.digital_io());
    try lcd.send(.{ .cscon = Packet.unlock_1 });
    try lcd.send(.{ .cscon = Packet.unlock_2 });
    try lcd.send(.{ .cscon = Packet.lock_1 });
    try lcd.send(.{ .cscon = Packet.lock_2 });

    try td.expect_sent(&.{
        &.{0xF0}, &.{0xC3},
        &.{0xF0}, &.{0x96},
        &.{0xF0}, &.{0x3C},
        &.{0xF0}, &.{0x69},
    });
}

test "rset pulses reset high, low, then high again" {
    var pin = LoggingPin.init();
    defer pin.deinit();
    var dc = Digital_IO.TestDevice.init(.output, .low);
    var td = DatagramDevice.TestDevice.init_receiver_only();
    defer td.deinit();

    const lcd = init(td.datagram_device(), pin.digital_io(), dc.digital_io());
    try lcd.rset(noopDelay);

    try testing.expectEqualSlices(Digital_IO.State, &.{ .high, .low, .high }, pin.history.items);
}

test "rect sends CASET then RASET, both big-endian" {
    var dc = Digital_IO.TestDevice.init(.output, .low);
    var rst = Digital_IO.TestDevice.init(.output, .low);
    var td = DatagramDevice.TestDevice.init_receiver_only();
    defer td.deinit();

    const lcd = init(td.datagram_device(), rst.digital_io(), dc.digital_io());
    try lcd.rect(0x0010, 0x00EF, 0x0000, 0x013F);

    try td.expect_sent(&.{
        &.{0x2A}, &.{ 0x00, 0x10, 0x00, 0xEF },
        &.{0x2B}, &.{ 0x00, 0x00, 0x01, 0x3F },
    });
}

test "draw sends RAMWR then streams a distinct pixel run in one transfer" {
    var dc = Digital_IO.TestDevice.init(.output, .low);
    var rst = Digital_IO.TestDevice.init(.output, .low);
    var td = DatagramDevice.TestDevice.init_receiver_only();
    defer td.deinit();

    const lcd = init(td.datagram_device(), rst.digital_io(), dc.digital_io());
    const pixels = [_]Color{
        .{ .r = 31, .g = 0, .b = 0 },
        .{ .r = 0, .g = 63, .b = 0 },
        .{ .r = 0, .g = 0, .b = 31 },
    };
    try lcd.draw(&pixels);

    try td.expect_sent(&.{ &.{0x2C}, std.mem.sliceAsBytes(&pixels) });
}

test "draw with an empty slice still issues RAMWR and no data phase" {
    var dc = Digital_IO.TestDevice.init(.output, .low);
    var rst = Digital_IO.TestDevice.init(.output, .low);
    var td = DatagramDevice.TestDevice.init_receiver_only();
    defer td.deinit();

    const lcd = init(td.datagram_device(), rst.digital_io(), dc.digital_io());
    try lcd.draw(&.{});

    try td.expect_sent(&.{ &.{0x2C}, &.{} });
}

test "fill sends RAMWR then streams count copies of one color" {
    var dc = Digital_IO.TestDevice.init(.output, .low);
    var rst = Digital_IO.TestDevice.init(.output, .low);
    var td = DatagramDevice.TestDevice.init_receiver_only();
    defer td.deinit();

    const lcd = init(td.datagram_device(), rst.digital_io(), dc.digital_io());
    const red: Color = .{ .r = 31, .g = 0, .b = 0 };
    try lcd.fill(red, 3);

    try td.expect_sent(&.{
        &.{0x2C},
        std.mem.asBytes(&red),
        std.mem.asBytes(&red),
        std.mem.asBytes(&red),
    });
}

test "fill with count 1 sends exactly one pixel" {
    var dc = Digital_IO.TestDevice.init(.output, .low);
    var rst = Digital_IO.TestDevice.init(.output, .low);
    var td = DatagramDevice.TestDevice.init_receiver_only();
    defer td.deinit();

    const lcd = init(td.datagram_device(), rst.digital_io(), dc.digital_io());
    const blue: Color = .{ .r = 0, .g = 0, .b = 31 };
    try lcd.fill(blue, 1);

    try td.expect_sent(&.{ &.{0x2C}, std.mem.asBytes(&blue) });
}

test "read discards the mandatory dummy byte before filling the buffer" {
    var dc = Digital_IO.TestDevice.init(.output, .low);
    var rst = Digital_IO.TestDevice.init(.output, .low);
    var td = DatagramDevice.TestDevice.init(&.{
        &.{0x00},
        &.{ 0x12, 0x34 },
    }, true);
    defer td.deinit();

    const lcd = init(td.datagram_device(), rst.digital_io(), dc.digital_io());
    var buf: [2]u8 = undefined;
    try lcd.read(.{ .ramrd = {} }, &buf);

    try testing.expectEqualSlices(u8, &.{ 0x12, 0x34 }, &buf);
    try td.expect_sent(&.{&.{0x2E}});
}

/// A `Digital_IO` fake that records every state it's ever written, in
/// order. `mdf.base.Digital_IO.TestDevice` only tracks current state, not
/// history, so `rset`'s edge sequence needs this instead.
const LoggingPin = struct {
    dir: Digital_IO.Direction = .output,
    history: std.array_list.Managed(Digital_IO.State),

    fn init() LoggingPin {
        return .{ .history = std.array_list.Managed(Digital_IO.State).init(testing.allocator) };
    }

    fn deinit(self: *LoggingPin) void {
        self.history.deinit();
    }

    fn digital_io(self: *LoggingPin) Digital_IO {
        return .{ .ptr = self, .vtable = &vtable };
    }

    fn set_direction(ctx: *anyopaque, dir: Digital_IO.Direction) Digital_IO.SetDirError!void {
        const self: *LoggingPin = @ptrCast(@alignCast(ctx));
        self.dir = dir;
    }

    fn set_bias(ctx: *anyopaque, bias: ?Digital_IO.State) Digital_IO.SetBiasError!void {
        _ = ctx;
        _ = bias;
    }

    fn write(ctx: *anyopaque, state: Digital_IO.State) Digital_IO.WriteError!void {
        const self: *LoggingPin = @ptrCast(@alignCast(ctx));
        self.history.append(state) catch return error.IoError;
    }

    fn read(ctx: *anyopaque) Digital_IO.ReadError!Digital_IO.State {
        const self: *LoggingPin = @ptrCast(@alignCast(ctx));
        return self.history.items[self.history.items.len - 1];
    }

    const vtable = Digital_IO.VTable{
        .set_direction_fn = set_direction,
        .set_bias_fn = set_bias,
        .write_fn = write,
        .read_fn = read,
    };
};

/// A `DatagramDevice` fake for fault injection: like
/// `mdf.base.DatagramDevice.TestDevice`, but `connect`/`write`/`read` can
/// each be told to fail on a specific call number, so a test can check
/// that a failure *partway* through a multi-step operation (`rect`) or a
/// multi-byte burst (`draw`/`fill`) is reported rather than swallowed, and
/// that `disconnect` still runs. Kept local to this file's tests rather
/// than added to the shared `DatagramDevice.TestDevice` -- promoting fault
/// injection to that shared type is a reasonable follow-up if other
/// drivers want the same thing, but isn't needed for this driver alone.
const FaultyBus = struct {
    arena: std.heap.ArenaAllocator,
    sent: std.array_list.Managed([]u8),

    connected: bool = false,
    write_count: u32 = 0,
    read_count: u32 = 0,
    disconnect_count: u32 = 0,

    fail_connect: bool = false,
    /// 1-indexed `write`/`writev` call to fail on. `null` never fails.
    fail_write_at: ?u32 = null,
    /// 1-indexed `read`/`readv` call to fail on. `null` never fails.
    fail_read_at: ?u32 = null,
    /// Bytes handed back on each successive `read`/`readv` call.
    read_data: []const []const u8 = &.{},

    fn init() FaultyBus {
        return .{
            .arena = std.heap.ArenaAllocator.init(testing.allocator),
            .sent = std.array_list.Managed([]u8).init(testing.allocator),
        };
    }

    fn deinit(self: *FaultyBus) void {
        self.arena.deinit();
        self.sent.deinit();
    }

    fn datagram_device(self: *FaultyBus) DatagramDevice {
        return .{ .ptr = self, .vtable = &vtable };
    }

    fn connect(ctx: *anyopaque) DatagramDevice.ConnectError!void {
        const self: *FaultyBus = @ptrCast(@alignCast(ctx));
        if (self.fail_connect) return error.IoError;
        self.connected = true;
    }

    fn disconnect(ctx: *anyopaque) void {
        const self: *FaultyBus = @ptrCast(@alignCast(ctx));
        self.disconnect_count += 1;
        self.connected = false;
    }

    fn writev(ctx: *anyopaque, datagrams: []const []const u8) DatagramDevice.WriteError!void {
        const self: *FaultyBus = @ptrCast(@alignCast(ctx));
        if (!self.connected) return error.NotConnected;
        self.write_count += 1;
        if (self.fail_write_at) |n| {
            if (self.write_count == n) return error.IoError;
        }

        var total: usize = 0;
        for (datagrams) |d| total += d.len;
        const buf = self.arena.allocator().alloc(u8, total) catch return error.IoError;
        var offset: usize = 0;
        for (datagrams) |d| {
            @memcpy(buf[offset..][0..d.len], d);
            offset += d.len;
        }
        self.sent.append(buf) catch return error.IoError;
    }

    fn readv(ctx: *anyopaque, datagrams: []const []u8) DatagramDevice.ReadError!usize {
        const self: *FaultyBus = @ptrCast(@alignCast(ctx));
        if (!self.connected) return error.NotConnected;
        self.read_count += 1;
        if (self.fail_read_at) |n| {
            if (self.read_count == n) return error.IoError;
        }
        if (self.read_count - 1 >= self.read_data.len) return error.IoError;

        const src = self.read_data[self.read_count - 1];
        var total: usize = 0;
        for (datagrams) |d| total += d.len;
        const written = @min(src.len, total);

        var offset: usize = 0;
        for (datagrams) |d| {
            const amount = @min(d.len, written -| offset);
            @memcpy(d[0..amount], src[offset..][0..amount]);
            offset += amount;
            if (amount < d.len) break;
        }

        if (src.len > total) return error.BufferOverrun;
        return written;
    }

    const vtable = DatagramDevice.VTable{
        .connect_fn = connect,
        .disconnect_fn = disconnect,
        .writev_fn = writev,
        .readv_fn = readv,
    };
};

test "rect never sends RASET if CASET's write fails" {
    var bus = FaultyBus.init();
    defer bus.deinit();
    bus.fail_write_at = 1;

    var dc = Digital_IO.TestDevice.init(.output, .low);
    var rst = Digital_IO.TestDevice.init(.output, .low);
    const lcd = init(bus.datagram_device(), rst.digital_io(), dc.digital_io());

    try testing.expectError(error.IoError, lcd.rect(0, 1, 0, 1));
    try testing.expectEqual(@as(u32, 1), bus.write_count);
    try testing.expectEqual(@as(u32, 1), bus.disconnect_count);
}

test "connect failing means no write is attempted" {
    var bus = FaultyBus.init();
    defer bus.deinit();
    bus.fail_connect = true;

    var dc = Digital_IO.TestDevice.init(.output, .low);
    var rst = Digital_IO.TestDevice.init(.output, .low);
    const lcd = init(bus.datagram_device(), rst.digital_io(), dc.digital_io());

    try testing.expectError(error.IoError, lcd.send(.{ .dispon = {} }));
    try testing.expectEqual(@as(u32, 0), bus.write_count);
}

test "fill stops and still disconnects if a write fails partway through the burst" {
    var bus = FaultyBus.init();
    defer bus.deinit();
    // Call 1 is RAMWR's opcode; calls 2-3 are the first two pixels; call 4 fails.
    bus.fail_write_at = 4;

    var dc = Digital_IO.TestDevice.init(.output, .low);
    var rst = Digital_IO.TestDevice.init(.output, .low);
    const lcd = init(bus.datagram_device(), rst.digital_io(), dc.digital_io());

    const red: Color = .{ .r = 31, .g = 0, .b = 0 };
    try testing.expectError(error.IoError, lcd.fill(red, 5));

    try testing.expectEqual(@as(u32, 4), bus.write_count);
    // One disconnect for `send(.ramwr)`'s own transaction, one for the
    // burst's early exit -- never left connected after the error.
    try testing.expectEqual(@as(u32, 2), bus.disconnect_count);
}

test "draw propagates a failure on its single burst write and still disconnects" {
    var bus = FaultyBus.init();
    defer bus.deinit();
    bus.fail_write_at = 2; // call 1 = RAMWR opcode (ok), call 2 = the pixel burst

    var dc = Digital_IO.TestDevice.init(.output, .low);
    var rst = Digital_IO.TestDevice.init(.output, .low);
    const lcd = init(bus.datagram_device(), rst.digital_io(), dc.digital_io());

    const pixels = [_]Color{ .{ .r = 1, .g = 2, .b = 3 }, .{ .r = 4, .g = 5, .b = 6 } };
    try testing.expectError(error.IoError, lcd.draw(&pixels));
    try testing.expectEqual(@as(u32, 2), bus.disconnect_count);
}

test "read propagates a failure on the dummy byte and still disconnects" {
    var bus = FaultyBus.init();
    defer bus.deinit();
    bus.fail_read_at = 1;

    var dc = Digital_IO.TestDevice.init(.output, .low);
    var rst = Digital_IO.TestDevice.init(.output, .low);
    const lcd = init(bus.datagram_device(), rst.digital_io(), dc.digital_io());

    var buf: [2]u8 = undefined;
    try testing.expectError(error.IoError, lcd.read(.{ .ramrd = {} }, &buf));
    try testing.expectEqual(@as(u32, 1), bus.disconnect_count);
}

test "read propagates a failure on the real data, not just the dummy byte" {
    var bus = FaultyBus.init();
    defer bus.deinit();
    bus.fail_read_at = 2;
    bus.read_data = &.{&.{0x00}};

    var dc = Digital_IO.TestDevice.init(.output, .low);
    var rst = Digital_IO.TestDevice.init(.output, .low);
    const lcd = init(bus.datagram_device(), rst.digital_io(), dc.digital_io());

    var buf: [2]u8 = undefined;
    try testing.expectError(error.IoError, lcd.read(.{ .ramrd = {} }, &buf));
}

test "a short read reports BufferOverrun rather than partial success" {
    var bus = FaultyBus.init();
    defer bus.deinit();
    bus.read_data = &.{ &.{0x00}, &.{ 0x12, 0x34, 0x56 } };

    var dc = Digital_IO.TestDevice.init(.output, .low);
    var rst = Digital_IO.TestDevice.init(.output, .low);
    const lcd = init(bus.datagram_device(), rst.digital_io(), dc.digital_io());

    var buf: [2]u8 = undefined;
    try testing.expectError(error.BufferOverrun, lcd.read(.{ .ramrd = {} }, &buf));
}

test "a data_cmd pin that can't be written surfaces as an error, not a panic" {
    // Direction .input makes Digital_IO.TestDevice.write fail with
    // `error.Unsupported` -- a cheap way to exercise the GPIO failure path
    // without a bespoke fault-injecting pin.
    var dc = Digital_IO.TestDevice.init(.input, .low);
    var rst = Digital_IO.TestDevice.init(.output, .low);
    var td = DatagramDevice.TestDevice.init_receiver_only();
    defer td.deinit();

    const lcd = init(td.datagram_device(), rst.digital_io(), dc.digital_io());
    try testing.expectError(error.Unsupported, lcd.send(.{ .dispon = {} }));
}
