//! Reads a DHT11 sensor on GPIO14 and serves the latest reading as JSON over
//! HTTP, using the Pico W's built-in Wi-Fi (CYW43439).

#![no_std]
#![no_main]

use core::cell::Cell;
use core::fmt::{self, Write as _};

use blinky::{DecodeError, Dhl11Decoder, Reading};
use cortex_m::interrupt::{self, Mutex};
use cyw43::aligned_bytes;
use cyw43_pio::{DEFAULT_CLOCK_DIVIDER, PioSpi};
use defmt::{error, info, unwrap};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_net::tcp::TcpSocket;
use embassy_net::{Config, DhcpConfig, Stack, StackResources};
use embassy_rp::gpio::{Flex, Level, Output, Pull};
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::pio::{self, Pio};
use embassy_rp::{bind_interrupts, dma};
use embassy_time::{Duration, Instant, Timer};
use panic_probe as _;
use static_cell::StaticCell;

/// Wi-Fi credentials, provided at compile time, e.g.
/// `WIFI_SSID=MyNet WIFI_PASSWORD=secret cargo r --release`.
const WIFI_SSID: &str = env!("WIFI_SSID");
const WIFI_PASSWORD: &str = env!("WIFI_PASSWORD");

/// TCP port of the HTTP server.
const HTTP_PORT: u16 = 80;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>;
});

/// Latest sensor reading, shared between the DHT11 loop and the HTTP server.
static LAST_READING: Mutex<Cell<Option<Reading>>> = Mutex::new(Cell::new(None));

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let firmware = aligned_bytes!("../cyw43-firmware/43439A0.bin");
    let clm = aligned_bytes!("../cyw43-firmware/43439A0_clm.bin");
    let nvram = aligned_bytes!("../cyw43-firmware/nvram_rp2040.bin");

    let power = Output::new(p.PIN_23, Level::Low);
    let chip_select = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        DEFAULT_CLOCK_DIVIDER,
        pio.irq0,
        chip_select,
        p.PIN_24,
        p.PIN_29,
        dma::Channel::new(p.DMA_CH0, Irqs),
    );

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, power, spi, firmware, nvram).await;
    spawner.spawn(unwrap!(cyw43_task(runner)));

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    // TCP/IP stack on the Wi-Fi device; the address comes from DHCP.
    static RESOURCES: StaticCell<StackResources<8>> = StaticCell::new();
    let (stack, net_runner) = embassy_net::new(
        net_device,
        Config::dhcpv4(DhcpConfig::default()),
        RESOURCES.init(StackResources::new()),
        Instant::now().as_micros() as u64,
    );
    static STACK: StaticCell<Stack<'static>> = StaticCell::new();
    let stack = STACK.init(stack);
    spawner.spawn(unwrap!(net_task(net_runner)));
    for _ in 0..HTTP_WORKERS {
        spawner.spawn(unwrap!(http_server_task(*stack)));
    }

    loop {
        // The CYW43439 is 2.4 GHz-only and does not support 802.11w (PMF), so
        // try WPA2/WPA3 first and fall back to plain WPA2.
        let mut wpa2_only = cyw43::JoinOptions::new(WIFI_PASSWORD.as_bytes());
        wpa2_only.auth = cyw43::JoinAuth::Wpa2;
        let attempts = [
            cyw43::JoinOptions::new(WIFI_PASSWORD.as_bytes()),
            wpa2_only,
        ];
        let mut joined = false;
        for options in attempts {
            match control.join(WIFI_SSID, options).await {
                Ok(()) => {
                    joined = true;
                    break;
                }
                Err(e) => error!("Wi-Fi join to '{}' failed: {:?}", WIFI_SSID, e),
            }
        }
        if joined {
            break;
        }
        Timer::after_secs(5).await;
    }
    stack.wait_link_up().await;
    stack.wait_config_up().await;
    if let Some(config) = stack.config_v4() {
        info!("HTTP server listening at http://{}/", config.address);
    }

    let mut dht_pin = Flex::new(p.PIN_14);

    loop {
        match read_dht11(&mut dht_pin).await {
            Ok(reading) => {
                interrupt::free(|cs| LAST_READING.borrow(cs).set(Some(reading)));
                info!(
                    "relative humidity: {}.{} %",
                    reading.humidity_pp, reading.humidity_dp
                );
                info!(
                    "temperature: {}.{} C",
                    reading.temperature_c, reading.temperature_dp
                );
            }
            Err(DecodeError::TooManySamples(count)) => {
                error!("Decoding error: Too many samples; got {}", count);
            }
            Err(DecodeError::NotEnoughSamples(count)) => {
                error!("Decoding error: Not enough samples; got {}", count);
            }
            Err(DecodeError::BadChecksum) => {
                error!("Decoding error: Bad checksum");
            }
            Err(DecodeError::BitsOutOfTolerance(value, index)) => {
                error!(
                    "Decoding error: Bits out of tolerance, index={}, value={}",
                    index, value
                );
            }
        }
        Timer::after_secs(2).await;
    }
}

async fn read_dht11(dht_pin: &mut Flex<'_>) -> Result<Reading, DecodeError> {
    dht_pin.set_as_output();
    dht_pin.set_low();
    Timer::after_millis(20).await;

    dht_pin.set_as_input();
    dht_pin.set_pull(Pull::Up);

    let start = Instant::now();
    capture_next_high_pulse(dht_pin).await;
    capture_next_high_pulse(dht_pin).await;
    let mut decoder = Dhl11Decoder::new();
    while start.elapsed().as_millis() < 200 {
        let Some(width) = capture_next_high_pulse(dht_pin).await else {
            break;
        };
        decoder.push_pulse(width);
    }
    decoder.finish()
}

async fn capture_next_high_pulse(dht_pin: &mut Flex<'_>) -> Option<u32> {
    let deadline = Instant::now() + Duration::from_millis(5);
    while dht_pin.is_low() && Instant::now() < deadline {}
    let t0 = Instant::now();
    while dht_pin.is_high() && Instant::now() < deadline {}
    if Instant::now() < deadline {
        Some(t0.elapsed().as_micros() as u32)
    } else {
        None
    }
}

/// Runs the Wi-Fi driver.
#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, cyw43::SpiBus<Output<'static>, PioSpi<'static, PIO0, 0>>>,
) -> ! {
    runner.run().await
}

/// Runs the TCP/IP stack.
#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, cyw43::NetDriver<'static>>) -> ! {
    runner.run().await
}

/// Embedded frontend page, served at `/`.
const INDEX_HTML: &str = include_str!("../frontend/index.html");

/// Number of concurrent HTTP listeners. Browsers open several connections in
/// parallel; a listener that is busy serving is not in LISTEN state, so a
/// single one would answer the others with RST (connection refused).
const HTTP_WORKERS: usize = 4;

/// Serves the frontend at `/` and the latest reading as JSON at `/data`.
#[embassy_executor::task(pool_size = HTTP_WORKERS)]
async fn http_server_task(stack: Stack<'static>) {
    loop {
        // A fresh socket per connection: after close() the old one is not in
        // Closed state yet, so re-listening on it would fail with InvalidState.
        let mut rx_buffer = [0u8; 1024];
        let mut tx_buffer = [0u8; 1024];
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(2)));

        info!("HTTP: waiting for a connection on port {}", HTTP_PORT);
        if let Err(e) = socket.accept(HTTP_PORT).await {
            error!("HTTP: accept error: {:?}", e);
            continue;
        }
        info!("HTTP: connection accepted");

        // Read the request line once to route the request. Real clients send
        // it immediately; the short timeout keeps an idle connection from
        // stalling the single-threaded server.
        let mut req = [0u8; 256];
        let req_len = socket
            .read_with(|buf| {
                let n = core::cmp::min(buf.len(), req.len());
                req[..n].copy_from_slice(&buf[..n]);
                (n, n)
            })
            .await
            .unwrap_or(0);
        let request = core::str::from_utf8(&req[..req_len]).unwrap_or("");

        let mut json = [0u8; 64];
        let (status, content_type, body): (&str, &str, &[u8]) =
            if request.starts_with("GET /data") {
                let reading = interrupt::free(|cs| LAST_READING.borrow(cs).get());
                let body_len = match reading {
                    Some(r) => {
                        let mut w = Writer { buf: &mut json, pos: 0 };
                        let _ = write!(
                            w,
                            "{{\"humidity_percent\":{}.{},\"temperature_celsius\":{}.{}}}",
                            r.humidity_pp, r.humidity_dp, r.temperature_c, r.temperature_dp
                        );
                        w.pos
                    }
                    None => {
                        json[..4].copy_from_slice(b"null");
                        4
                    }
                };
                ("200 OK", "application/json", &json[..body_len])
            } else if request.starts_with("GET / ")
                || request.starts_with("GET /?")
                || request.starts_with("GET /index.html")
            {
                ("200 OK", "text/html; charset=utf-8", INDEX_HTML.as_bytes())
            } else {
                // e.g. /favicon.ico, or a client that sent nothing in time.
                ("404 Not Found", "text/plain", b"not found")
            };

        let mut header = [0u8; 256];
        let mut w = Writer {
            buf: &mut header,
            pos: 0,
        };
        let _ = write!(
            w,
            "HTTP/1.1 {}\r\nContent-Type: {}\r\nAccess-Control-Allow-Origin: *\r\nContent-Length: {}\r\nConnection: close\r\n\r\n",
            status,
            content_type,
            body.len()
        );
        let header_len = w.pos;
        let total = header_len + body.len();

        // Stream header + body through a small window buffer.
        let mut window = [0u8; 512];
        let mut sent = 0;
        while sent < total {
            let chunk = core::cmp::min(window.len(), total - sent);
            let from_header = core::cmp::min(chunk, header_len.saturating_sub(sent));
            if from_header > 0 {
                window[..from_header].copy_from_slice(&header[sent..sent + from_header]);
            }
            if chunk > from_header {
                let off = sent + from_header - header_len;
                window[from_header..chunk].copy_from_slice(&body[off..off + (chunk - from_header)]);
            }
            match socket
                .write_with(|buf| {
                    let n = core::cmp::min(buf.len(), chunk);
                    buf[..n].copy_from_slice(&window[..n]);
                    (n, n)
                })
                .await
            {
                Ok(n) => sent += n,
                Err(e) => {
                    error!("HTTP: write error: {:?}", e);
                    break;
                }
            }
        }
        // Wait until the response is actually on the wire, then close.
        let _ = socket.flush().await;
        socket.close();
    }
}

/// Stack-allocated string builder for the fixed-size response buffers.
struct Writer<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl fmt::Write for Writer<'_> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let n = core::cmp::min(s.len(), self.buf.len() - self.pos);
        self.buf[self.pos..self.pos + n].copy_from_slice(&s.as_bytes()[..n]);
        self.pos += n;
        Ok(())
    }
}
