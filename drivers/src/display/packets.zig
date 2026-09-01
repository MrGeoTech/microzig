//!
//! Shared shapes for the small multi-byte, big-endian payloads that show
//! up repeatedly in the ST77xx family's command protocol -- a column or
//! row address range (`CASET`/`RASET`), a single scanline number
//! (`TESCAN`), and similar. Not a place for a chip's own `Opcode`/
//! `Packet` union, which is inherently specific to one command set --
//! just the payload *shapes* underneath it that multiple chips in the
//! family happen to share.
//!

const std = @import("std");

/// A single big-endian value of `T`, wire-encoded -- the shape of
/// commands like `TESCAN`/`VSCRSADD` that take one multi-byte number.
pub fn BigEndianValue(comptime T: type) type {
    return extern struct {
        const Self = @This();

        bytes: [@sizeOf(T)]u8,

        pub fn init(value: T) Self {
            var self: Self = undefined;
            std.mem.writeInt(T, &self.bytes, value, .big);
            return self;
        }
    };
}

/// A start/end pair of big-endian values of `T`, wire-encoded -- the
/// shape of commands like `CASET`/`RASET`/`PTLAR` that set an inclusive
/// address range.
pub fn BigEndianRange(comptime T: type) type {
    return extern struct {
        const Self = @This();

        start: [@sizeOf(T)]u8,
        end: [@sizeOf(T)]u8,

        pub fn init(start: T, end: T) Self {
            var self: Self = undefined;
            std.mem.writeInt(T, &self.start, start, .big);
            std.mem.writeInt(T, &self.end, end, .big);
            return self;
        }
    };
}

test "BigEndianValue writes big-endian bytes regardless of host endianness" {
    const V = BigEndianValue(u16);
    try std.testing.expectEqualSlices(u8, &.{ 0x01, 0x2C }, &V.init(0x012C).bytes);
}

test "BigEndianRange writes both bounds, both big-endian" {
    const R = BigEndianRange(u16);
    const r = R.init(0x0010, 0x00EF);
    try std.testing.expectEqualSlices(u8, &.{ 0x00, 0x10 }, &r.start);
    try std.testing.expectEqualSlices(u8, &.{ 0x00, 0xEF }, &r.end);
}
