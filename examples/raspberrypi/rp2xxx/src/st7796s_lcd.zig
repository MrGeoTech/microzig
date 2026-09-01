//! ST7796S Driver Demo
//!
//! Brings up an ST7796S SPI panel (e.g. the 52Pi EP-0172, a 320x480 4"
//! TFT) using `microzig.drivers.display.ST7796S` and drives it directly
//! -- unlike the `st7789` example, no off-screen framebuffer is needed
//! here: `fill`/`draw` stream straight to the panel, so a full-screen
//! fill is one `rect` + one `fill` call, not a buffer the size of the
//! display.
//!
//! Pinout (placeholders -- update to match your wiring). MISO is wired
//! because this driver, unlike the `ST77xx` family driver the `st7789`
//! example uses, can read the panel back -- if your board's breakout
//! doesn't expose a separate MISO/SDO pin (common on cheap 4-wire-only
//! ST7796S modules), leave it unconnected and skip the read-back check
//! below; everything else here is write-only.
//!
//! - SCK:  GPIO 10
//! - MOSI: GPIO 11
//! - MISO: GPIO 8
//! - CS:   GPIO 9
//! - DC:   GPIO 14
//! - RST:  GPIO 15
//!
//! Bring-up here is deliberately minimal: hardware reset, `SWRESET`,
//! `SLPOUT`, `COLMOD`, `MADCTL`, `DISPON` -- the sequence known to work
//! broadly across ST7796S modules (it's what a hand-rolled driver for
//! this same chip family already runs successfully). Extended tuning
//! (`PWR1`-`PWR3`, `VCMPCTL`, gamma) is deliberately left at the chip's
//! own power-on defaults rather than guessed at here -- if colors look
//! washed out or unstable on your specific panel, dial those in via
//! `lcd.send(.{ .pwr1 = ... })` and friends; see the doc comments on
//! `st7796s.PowerControl1` and its neighbors for what each field does.

const std = @import("std");
const microzig = @import("microzig");
const st7796s = microzig.drivers.display.ST7796S;

const rp2xxx = microzig.hal;
const time = rp2xxx.time;
const gpio = rp2xxx.gpio;
const drivers = rp2xxx.drivers;

const DISPLAY_CS_PIN = 9;
const DISPLAY_SCK_PIN = 10;
const DISPLAY_MOSI_PIN = 11;
const DISPLAY_MISO_PIN = 8;
const DISPLAY_DC_PIN = 14;
const DISPLAY_RST_PIN = 15;

const PANEL_WIDTH = 320;
const PANEL_HEIGHT = 480;

const uart = rp2xxx.uart.instance.num(0);
const uart_tx_pin = gpio.num(0);
const uart_rx_pin = gpio.num(1);

pub const panic = microzig.panic;

pub const std_options = microzig.std_options(.{
    .log_level = .debug,
    .logFn = rp2xxx.uart.log,
});

comptime {
    _ = microzig.export_startup();
}

const spi = rp2xxx.spi.instance.SPI1;

const black: st7796s.Color = .{};
const white: st7796s.Color = .{ .r = 31, .g = 63, .b = 31 };
const red: st7796s.Color = .{ .r = 31, .g = 0, .b = 0 };
const green: st7796s.Color = .{ .r = 0, .g = 63, .b = 0 };
const blue: st7796s.Color = .{ .r = 0, .g = 0, .b = 31 };

/// Fills the whole panel with one color -- `rect` sets the address
/// window to everything, `fill` streams one `RAMWR` burst of repeated
/// pixels, no buffer sized to the display needed.
fn fill_screen(lcd: st7796s, color: st7796s.Color) !void {
    try lcd.rect(0, PANEL_WIDTH - 1, 0, PANEL_HEIGHT - 1);
    try lcd.fill(color, @as(u32, PANEL_WIDTH) * PANEL_HEIGHT);
}

/// Fills one quadrant -- demonstrates addressing a sub-rectangle rather
/// than the whole panel.
fn fill_quadrant(lcd: st7796s, left: bool, top: bool, color: st7796s.Color) !void {
    const half_w = PANEL_WIDTH / 2;
    const half_h = PANEL_HEIGHT / 2;
    const xs: u16 = if (left) 0 else half_w;
    const xe: u16 = if (left) half_w - 1 else PANEL_WIDTH - 1;
    const ys: u16 = if (top) 0 else half_h;
    const ye: u16 = if (top) half_h - 1 else PANEL_HEIGHT - 1;

    try lcd.rect(xs, xe, ys, ye);
    try lcd.fill(color, @as(u32, half_w) * half_h);
}

/// Draws a horizontal red-to-blue gradient across every row -- exercises
/// `draw` (a run of *distinct* pixel values) rather than `fill`'s single
/// repeated one.
fn draw_gradient(lcd: st7796s) !void {
    var row: [PANEL_WIDTH]st7796s.Color = undefined;
    for (&row, 0..) |*pixel, x| {
        const level: u5 = @intCast((x * 31) / (PANEL_WIDTH - 1));
        pixel.* = .{ .r = level, .g = 0, .b = 31 - level };
    }

    for (0..PANEL_HEIGHT) |y| {
        try lcd.rect(0, PANEL_WIDTH - 1, @intCast(y), @intCast(y));
        try lcd.draw(&row);
    }
}

pub fn main() !void {
    inline for (&.{ uart_tx_pin, uart_rx_pin }) |pin| {
        pin.set_function(.uart);
    }

    uart.apply(.{
        .clock_config = rp2xxx.clock_config,
    });

    const cs_pin = gpio.num(DISPLAY_CS_PIN);
    const sck_pin = gpio.num(DISPLAY_SCK_PIN);
    const mosi_pin = gpio.num(DISPLAY_MOSI_PIN);
    const miso_pin = gpio.num(DISPLAY_MISO_PIN);
    const dc_pin = gpio.num(DISPLAY_DC_PIN);
    const rst_pin = gpio.num(DISPLAY_RST_PIN);

    inline for (&.{ cs_pin, sck_pin, mosi_pin, miso_pin }) |pin| {
        pin.set_function(.spi);
    }

    try spi.apply(.{
        .clock_config = rp2xxx.clock_config,
        .baud_rate = 10_000_000, // 10 MHz is a safe starting point; check your panel's datasheet for its max.
    });

    var spi_dev = drivers.SPI_Device.init(spi, .{
        .chip_select = .{ .pin = cs_pin, .active_level = .low },
    });

    dc_pin.set_function(.sio);
    dc_pin.set_direction(.out);
    rst_pin.set_function(.sio);
    rst_pin.set_direction(.out);

    var dc_gpio = drivers.GPIO_Device.init(dc_pin);
    var rst_gpio = drivers.GPIO_Device.init(rst_pin);

    const lcd = st7796s.init(spi_dev.datagram_device(), rst_gpio.digital_io(), dc_gpio.digital_io());

    try lcd.rset(time.sleep_ms);

    try lcd.send(.{ .swreset = {} });
    time.sleep_ms(150);
    try lcd.send(.{ .slpout = {} });
    time.sleep_ms(120);
    try lcd.send(.{ .colmod = .{ .mcu_format = .bpp16, .rgb_format = .bpp16 } });
    try lcd.send(.{ .madctl = .{ .rgb = .rgb, .addr = .deg0 } });
    try lcd.send(.{ .dispon = {} });
    time.sleep_ms(50);

    // Sanity check that MISO is actually wired and the panel is
    // answering, not just accepting writes blind: RDDID's response is
    // the manufacturer/module-version/module-driver ID, which is fixed
    // and non-zero on real hardware. Comment this out if your board has
    // no MISO connection (see the pinout note above).
    const id = try lcd.read(.rddid, &.{});
    std.log.info("ST7796S RDDID: {any}", .{id.rddid});

    while (true) {
        inline for (.{ red, green, blue, white, black }) |color| {
            try fill_screen(lcd, color);
            time.sleep_ms(1000);
        }

        try fill_quadrant(lcd, true, true, red);
        try fill_quadrant(lcd, false, true, green);
        try fill_quadrant(lcd, true, false, blue);
        try fill_quadrant(lcd, false, false, white);
        time.sleep_ms(2000);

        try draw_gradient(lcd);
        time.sleep_ms(2000);
    }
}
