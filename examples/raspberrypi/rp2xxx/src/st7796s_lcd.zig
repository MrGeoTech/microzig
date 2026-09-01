//! ST7796S Driver Demo -- 52Pi EP-0172 "Pico Breadboard Kit Plus"
//!
//! Brings up the EP-0172's 3.5" 320x480 ST7796SU1 panel using
//! `microzig.drivers.display.ST7796S` and drives it directly -- unlike
//! the `st7789` example, no off-screen framebuffer is needed here:
//! `fill`/`draw` stream straight to the panel, so a full-screen fill is
//! one `rect` + one `fill` call, not a buffer the size of the display.
//!
//! Pinout, from the board's own wiki (wiki.52pi.com/index.php?title=EP-0172):
//!
//!     Pico    TFT
//!     GP2  -> CLK
//!     GP3  -> DIN (MOSI)
//!     GP5  -> CS
//!     GP6  -> DC
//!     GP7  -> RST
//!
//! That's SPI0's pin group, and it's the same wiring an already-working
//! hand-rolled driver for this exact board uses. There's no MISO/SDO in
//! that table -- this board's display header is 4-wire, write-only, so
//! `read` (this driver's one capability the `st7789` example's `ST77xx`
//! driver doesn't have) isn't reachable from here. GP8/GP9 are already
//! spoken for elsewhere on this board (the capacitive touch controller's
//! I2C0 SDA/SCL) -- don't repurpose them as SPI RX.
//!
//! Bring-up here is deliberately minimal: hardware reset, `SWRESET`,
//! `SLPOUT`, `COLMOD`, `MADCTL`, `DISPON` -- the sequence known to work
//! broadly across ST7796S modules (it's what the hand-rolled driver for
//! this same board already runs successfully). Extended tuning
//! (`PWR1`-`PWR3`, `VCMPCTL`, gamma) is deliberately left at the chip's
//! own power-on defaults rather than guessed at here -- if colors look
//! washed out or unstable, dial those in via `lcd.send(.{ .pwr1 = ... })`
//! and friends; see the doc comments on `st7796s.PowerControl1` and its
//! neighbors for what each field does.
//!
//! ## If nothing seems to happen after flashing
//!
//! This example logs over the Pico's *hardware* UART (GP0 TX / GP1 RX),
//! not USB CDC -- the Pico's USB port on this firmware doesn't enumerate
//! as a serial device at all, so no `/dev/ttyACM0` (or `COMx`) ever
//! appears, on any board, whether or not the example is working. Seeing
//! `std.log` output means wiring a USB-to-serial adapter to GP0/GP1/GND;
//! without one, watch the onboard LED (GP25) instead -- it blinks once a
//! second the whole time `main` is running, independent of the display
//! or logging, so it tells you the firmware booted and is alive even
//! with nothing else connected.

const std = @import("std");
const microzig = @import("microzig");
const st7796s = microzig.drivers.display.ST7796S;

const rp2xxx = microzig.hal;
const time = rp2xxx.time;
const gpio = rp2xxx.gpio;
const drivers = rp2xxx.drivers;

const DISPLAY_SCK_PIN = 2;
const DISPLAY_MOSI_PIN = 3;
const DISPLAY_CS_PIN = 5;
const DISPLAY_DC_PIN = 6;
const DISPLAY_RST_PIN = 7;

const LED_PIN = 25;

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

const spi = rp2xxx.spi.instance.SPI0;

const black = st7796s.Color.black;
const white = st7796s.Color.white;
const red = st7796s.Color.red;
const green = st7796s.Color.green;
const blue = st7796s.Color.blue;

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
        const level: u8 = @intCast((x * 255) / (PANEL_WIDTH - 1));
        pixel.* = .rgb(level, 0, 255 - level);
    }

    for (0..PANEL_HEIGHT) |y| {
        try lcd.rect(0, PANEL_WIDTH - 1, @intCast(y), @intCast(y));
        try lcd.draw(&row);
    }
}

pub fn main() !void {
    const led_pin = gpio.num(LED_PIN);
    led_pin.set_function(.sio);
    led_pin.set_direction(.out);

    inline for (&.{ uart_tx_pin, uart_rx_pin }) |pin| {
        pin.set_function(.uart);
    }

    uart.apply(.{
        .clock_config = rp2xxx.clock_config,
    });

    const cs_pin = gpio.num(DISPLAY_CS_PIN);
    const sck_pin = gpio.num(DISPLAY_SCK_PIN);
    const mosi_pin = gpio.num(DISPLAY_MOSI_PIN);
    const dc_pin = gpio.num(DISPLAY_DC_PIN);
    const rst_pin = gpio.num(DISPLAY_RST_PIN);

    inline for (&.{ cs_pin, sck_pin, mosi_pin }) |pin| {
        pin.set_function(.spi);
    }

    try spi.apply(.{
        .clock_config = rp2xxx.clock_config,
        .baud_rate = 10_000_000, // 10 MHz is a safe starting point; the ST7796SU1 datasheet allows more.
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

    std.log.info("ST7796S bring-up done, starting color demo", .{});

    var last_blink = time.get_time_since_boot().to_us();

    while (true) {
        inline for (.{ red, green, blue, white, black }) |color| {
            try fill_screen(lcd, color);
            blink_while_waiting(led_pin, &last_blink, 1000);
        }

        try fill_quadrant(lcd, true, true, red);
        try fill_quadrant(lcd, false, true, green);
        try fill_quadrant(lcd, true, false, blue);
        try fill_quadrant(lcd, false, false, white);
        blink_while_waiting(led_pin, &last_blink, 2000);

        try draw_gradient(lcd);
        blink_while_waiting(led_pin, &last_blink, 2000);
    }
}

/// Waits `duration_ms`, toggling `led_pin` once a second while it does --
/// so the "is the firmware alive" heartbeat keeps blinking through every
/// pause in the color demo, not just once at startup.
fn blink_while_waiting(led_pin: gpio.Pin, last_blink: *u64, duration_ms: u32) void {
    const start = time.get_time_since_boot().to_us();
    const duration_us: u64 = @as(u64, duration_ms) * 1000;

    while (time.get_time_since_boot().to_us() - start < duration_us) {
        const now = time.get_time_since_boot().to_us();
        if (now - last_blink.* >= 500_000) {
            last_blink.* = now;
            led_pin.toggle();
        }
        time.sleep_ms(10);
    }
}
