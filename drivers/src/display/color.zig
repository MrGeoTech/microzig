//!
//! This file provides common color types found on the supported displays.
//!

const std = @import("std");

/// A color type encoding only black and white.
pub const BlackWhite = enum(u1) {
    black = 0,
    white = 1,
};

/// A packed RGB color, `r_bits`/`g_bits`/`b_bits` wide per channel,
/// tightly packed into the smallest whole number of bytes that fits
/// (`r_bits + g_bits + b_bits`, rounded up to a byte), most significant
/// channel (red) first, and written to the wire in `endianness` byte
/// order regardless of the host's own.
///
/// This one factory covers every tightly-packed RGB pixel format --
/// `Color(5, 6, 5, .big)` is RGB565, `Color(3, 3, 2, .big)` is RGB332,
/// and so on -- rather than a separate named function per format. Stored
/// as a plain `[N]u8`, not a backing integer: an integer whose bit width
/// isn't already a power-of-two byte count (`u24` for a hypothetical
/// tightly-packed 18-bit color, say) has a `@sizeOf` larger than its bit
/// width once ABI alignment padding is counted, which would silently put
/// extra padding bytes on the wire through a driver's ordinary
/// `std.mem.asBytes`/`sliceAsBytes` calls. A byte array has no such
/// padding to trip over -- `@sizeOf` of a single-`[N]u8`-field struct is
/// always exactly `N`.
///
/// One real limit, not papered over: this packs `r_bits`/`g_bits`/`b_bits`
/// as one contiguous bitfield, which is the right wire format for RGB565
/// and similar tightly-packed formats but is *not* how common 18bpp SPI
/// TFT modes work -- those typically send one byte per channel
/// (left-justified, low bits don't-care) rather than 18 contiguous
/// packed bits. `Color(6, 6, 6, .big)` as written here would not produce
/// that layout; an 18bpp format needs a different packing mode of its
/// own, not this one reused verbatim.
pub fn Color(
    comptime r_bits: comptime_int,
    comptime g_bits: comptime_int,
    comptime b_bits: comptime_int,
    comptime endianness: std.builtin.Endian,
) type {
    const total_bits = r_bits + g_bits + b_bits;
    const byte_count = (total_bits + 7) / 8;
    /// Wide enough to hold all three channels packed together; used only
    /// to do the packing arithmetic in, not for storage (see the type
    /// doc comment on why storage is a byte array instead).
    const Value = std.meta.Int(.unsigned, byte_count * 8);

    return extern struct {
        const Self = @This();

        bytes: [byte_count]u8,

        pub const black: Self = .rgb(0x00, 0x00, 0x00);
        pub const white: Self = .rgb(0xFF, 0xFF, 0xFF);
        pub const red: Self = .rgb(0xFF, 0x00, 0x00);
        pub const green: Self = .rgb(0x00, 0xFF, 0x00);
        pub const blue: Self = .rgb(0x00, 0x00, 0xFF);
        pub const cyan: Self = .rgb(0x00, 0xFF, 0xFF);
        pub const magenta: Self = .rgb(0xFF, 0x00, 0xFF);
        pub const yellow: Self = .rgb(0xFF, 0xFF, 0x00);

        /// Builds a color from full 8-bit-per-channel components, scaled
        /// down to this format's actual bit depth per channel (dropping
        /// the low, least-significant bits -- the common, cheap way to
        /// narrow an 8-bit channel, not a rounding conversion).
        pub fn rgb(r: u8, g: u8, b: u8) Self {
            const rv: Value = @as(Value, r) >> (8 - r_bits);
            const gv: Value = @as(Value, g) >> (8 - g_bits);
            const bv: Value = @as(Value, b) >> (8 - b_bits);

            const packed_value: Value = (rv << (g_bits + b_bits)) | (gv << b_bits) | bv;

            var self: Self = undefined;
            std.mem.writeInt(Value, &self.bytes, packed_value, endianness);
            return self;
        }

        /// Builds a color from per-channel percentages, `0.0` to `1.0`;
        /// values outside that range are clamped rather than wrapping.
        pub fn pct(r: f32, g: f32, b: f32) Self {
            return .rgb(
                @intFromFloat(std.math.clamp(r, 0.0, 1.0) * 255.0),
                @intFromFloat(std.math.clamp(g, 0.0, 1.0) * 255.0),
                @intFromFloat(std.math.clamp(b, 0.0, 1.0) * 255.0),
            );
        }
    };
}

test "RGB565 packs and byte-orders like the well-known 0xF8 0x00 red" {
    const RGB565 = Color(5, 6, 5, .big);
    try std.testing.expectEqual(2, @sizeOf(RGB565));
    try std.testing.expectEqualSlices(u8, &.{ 0xF8, 0x00 }, &RGB565.rgb(0xFF, 0x00, 0x00).bytes);
    try std.testing.expectEqualSlices(u8, &.{ 0x00, 0x00 }, &RGB565.black.bytes);
    try std.testing.expectEqualSlices(u8, &.{ 0xFF, 0xFF }, &RGB565.white.bytes);
}

test "the same RGB565 color little-endian byte-swaps the big-endian one" {
    const big = Color(5, 6, 5, .big).rgb(0x12, 0x34, 0x56);
    const little = Color(5, 6, 5, .little).rgb(0x12, 0x34, 0x56);
    try std.testing.expectEqual(big.bytes[0], little.bytes[1]);
    try std.testing.expectEqual(big.bytes[1], little.bytes[0]);
}

test "pct scales like rgb with float components" {
    const RGB565 = Color(5, 6, 5, .big);
    try std.testing.expectEqualSlices(u8, &RGB565.rgb(0xFF, 0x00, 0x00).bytes, &RGB565.pct(1.0, 0.0, 0.0).bytes);
    try std.testing.expectEqualSlices(u8, &RGB565.black.bytes, &RGB565.pct(0.0, 0.0, 0.0).bytes);
}
