use common::drivers::gnss::ublox_m10::{Event, UbloxM10};
use common::messages::sensor::{GpsFixSample, GpsFixSampleStamped};
use common::utilities::time::MeasurementTimestamp;
use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_rp::Peri;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::PIO1;
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_time::{Duration, Instant, Timer};
use heapless::String;
use nmea::Nmea;
use nmea::sentences::FixType;
use rp235x_base::pio_uart::new_duplex_pio_uart;

use crate::buses::GpsPioUartBus;
use crate::channels::GPS_CHANNEL;
use crate::pinmap;
use crate::resources::GpsPioUartPins;

const GPS_BOOT_DELAY: Duration = Duration::from_millis(750);
const GPS_NMEA_SENTENCE_MAX_BYTES: usize = 128;
const KNOTS_TO_MPS: f32 = 0.514_444;

bind_interrupts!(struct GpsIrqs {
    PIO1_IRQ_0 => PioInterruptHandler<PIO1>;
});

pub fn spawn(spawner: &Spawner, bus: GpsPioUartBus, pins: GpsPioUartPins) {
    spawner.spawn(gps_pio_uart_task(bus.pio, pins)).unwrap();
}

#[embassy_executor::task]
async fn gps_pio_uart_task(pio_peripheral: Peri<'static, PIO1>, pins: GpsPioUartPins) -> ! {
    Timer::after(GPS_BOOT_DELAY).await;

    let mut pio = Pio::new(pio_peripheral, GpsIrqs);
    let (_gps_tx, mut gps_rx) = new_duplex_pio_uart(
        &mut pio.common,
        pio.sm0,
        pio.sm1,
        pins,
        pinmap::GPS_PIO_UART_BAUD,
    );
    let mut gps = UbloxM10::new();
    let mut nmea_capture = GpsNmeaCapture::new();
    let mut nmea_parser = Nmea::default();

    info!(
        "sam-m10q gps pio uart ready: mcu_tx=GP{=u8} mcu_rx=GP{=u8} baud={=u32}",
        pinmap::GPS_PIO_UART_TX,
        pinmap::GPS_PIO_UART_RX,
        pinmap::GPS_PIO_UART_BAUD,
    );

    loop {
        let byte = gps_rx.read_u8().await;
        if let Some(sentence) = nmea_capture.push(byte) {
            parse_nmea_sentence(&mut nmea_parser, sentence);
        }

        let event = match gps.push_byte(byte) {
            Ok(event) => event,
            Err(_) => {
                warn!("sam-m10q pio uart ubx parser error; resetting parser");
                gps.reset_parser();
                continue;
            }
        };

        match event {
            Some(Event::Ack(ack)) if ack.acknowledged => {
                info!(
                    "sam-m10q pio uart ack class=0x{=u8:02x} id=0x{=u8:02x}",
                    ack.class_id, ack.message_id,
                );
            }
            Some(Event::Ack(ack)) => {
                warn!(
                    "sam-m10q pio uart nack class=0x{=u8:02x} id=0x{=u8:02x}",
                    ack.class_id, ack.message_id,
                );
            }
            Some(Event::NavPvt(nav_pvt)) => {
                let now = Instant::now();
                let timestamp = MeasurementTimestamp::from_micros(now.as_micros());
                GPS_CHANNEL
                    .immediate_publisher()
                    .publish_immediate(nav_pvt.gps_fix_sample_stamped(timestamp));
            }
            Some(Event::OtherPacket) | None => {}
        }
    }
}

fn parse_nmea_sentence(nmea_parser: &mut Nmea, sentence: &str) {
    if !is_supported_nmea_sentence(sentence) {
        return;
    }

    let sentence_type = match nmea_parser.parse(sentence) {
        Ok(sentence_type) => sentence_type,
        Err(_) => {
            warn!("sam-m10q gps tx nmea parse failed");
            return;
        }
    };

    if matches!(
        sentence_type,
        nmea::SentenceType::GGA | nmea::SentenceType::GNS
    ) {
        publish_nmea_fix(nmea_parser);
    }
}

fn is_supported_nmea_sentence(sentence: &str) -> bool {
    let bytes = sentence.as_bytes();
    matches!(bytes.get(3..6), Some(b"GGA" | b"GNS" | b"RMC" | b"VTG"))
}

fn publish_nmea_fix(nmea_parser: &Nmea) {
    let Some(sample) = nmea_fix_sample(nmea_parser) else {
        return;
    };

    GPS_CHANNEL
        .immediate_publisher()
        .publish_immediate(GpsFixSampleStamped {
            timestamp: MeasurementTimestamp::from_micros(Instant::now().as_micros()),
            sample,
        });
}

fn nmea_fix_sample(nmea_parser: &Nmea) -> Option<GpsFixSample> {
    let fix_type = gps_fix_type_code(nmea_parser.fix_type()?);
    if fix_type == 0 {
        return None;
    }

    Some(GpsFixSample {
        lat_deg: nmea_parser.latitude()?,
        lon_deg: nmea_parser.longitude()?,
        alt_m: nmea_parser.altitude()?,
        vel_ned_mps: nmea_velocity_ned_mps(nmea_parser),
        sats: nmea_parser
            .fix_satellites()
            .unwrap_or_default()
            .min(u8::MAX as u32) as u8,
        fix_type,
    })
}

fn gps_fix_type_code(fix_type: FixType) -> u8 {
    match fix_type {
        FixType::Invalid | FixType::Estimated | FixType::Manual | FixType::Simulation => 0,
        FixType::Gps => 3,
        FixType::DGps | FixType::Pps => 4,
        FixType::Rtk | FixType::FloatRtk => 5,
    }
}

fn nmea_velocity_ned_mps(nmea_parser: &Nmea) -> [f32; 3] {
    let (Some(speed_knots), Some(course_deg)) =
        (nmea_parser.speed_over_ground, nmea_parser.true_course)
    else {
        return [0.0, 0.0, 0.0];
    };
    let speed_mps = speed_knots * KNOTS_TO_MPS;
    let course_rad = course_deg * core::f32::consts::PI / 180.0;

    [
        speed_mps * libm::cosf(course_rad),
        speed_mps * libm::sinf(course_rad),
        0.0,
    ]
}

struct GpsNmeaCapture {
    sentence: String<GPS_NMEA_SENTENCE_MAX_BYTES>,
    capturing: bool,
    overflowed: bool,
}

impl GpsNmeaCapture {
    fn new() -> Self {
        Self {
            sentence: String::new(),
            capturing: false,
            overflowed: false,
        }
    }

    fn push(&mut self, byte: u8) -> Option<&str> {
        match byte {
            b'$' => {
                self.start_sentence();
                None
            }
            b'\r' | b'\n' => self.finish_sentence(),
            0x20..=0x7e if self.capturing => {
                self.push_ascii(byte);
                None
            }
            _ if self.capturing => {
                self.drop_sentence();
                None
            }
            _ => None,
        }
    }

    fn start_sentence(&mut self) {
        self.sentence.clear();
        self.capturing = true;
        self.overflowed = false;
        let _ = self.sentence.push('$');
    }

    fn push_ascii(&mut self, byte: u8) {
        if !self.overflowed && self.sentence.push(byte as char).is_err() {
            self.overflowed = true;
        }
    }

    fn finish_sentence(&mut self) -> Option<&str> {
        if !self.capturing {
            return None;
        }

        if self.overflowed {
            warn!(
                "sam-m10q gps tx nmea sentence exceeded {=usize} bytes",
                GPS_NMEA_SENTENCE_MAX_BYTES,
            );
            self.drop_sentence();
            return None;
        }

        self.capturing = false;
        Some(self.sentence.as_str())
    }

    fn drop_sentence(&mut self) {
        self.sentence.clear();
        self.capturing = false;
        self.overflowed = false;
    }
}
