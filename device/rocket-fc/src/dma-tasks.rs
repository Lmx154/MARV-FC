use super::telemetry_presets::{
    DEBUG_PRESET, RATE_PRESET, SensorRates, log_debug_sample, log_rates_sample,
};
use super::*;

const LOG_FLUSH_INTERVAL_MS: u64 = 50;
const LOG_BUFFER_CAP: usize = 128;

struct LogBuffer<const N: usize> {
    buf: Deque<SensorsState, N>,
    drops: u32,
}

impl<const N: usize> LogBuffer<N> {
    fn new() -> Self {
        Self {
            buf: Deque::new(),
            drops: 0,
        }
    }

    fn push(&mut self, sample: SensorsState) {
        if self.buf.push_back(sample).is_err() {
            let _ = self.buf.pop_front();
            let _ = self.buf.push_back(sample);
            self.drops = self.drops.wrapping_add(1);
        }
    }

    fn pop(&mut self) -> Option<SensorsState> {
        self.buf.pop_front()
    }

    fn len(&self) -> usize {
        self.buf.len()
    }

    fn take_drops(&mut self) -> u32 {
        let drops = self.drops;
        self.drops = 0;
        drops
    }
}

#[embassy_executor::task]
pub(crate) async fn logger_task(
    mut led: PioWs2812<'static, embassy_rp::peripherals::PIO0, 0, 1>,
    base_color: RGB8,
) {
    let mut n: u32 = 0;
    let mut colors = [RGB8 { r: 0, g: 0, b: 0 }; 1];
    let mut led_on = false;
    // Track previous counters and time to compute real per-second rates
    let mut prev_imu_seq: u32 = 0;
    let mut prev_imu2_seq: u32 = 0;
    let mut prev_mag_seq: u32 = 0;
    let mut prev_baro_seq: u32 = 0;
    let mut prev_gps_seq: u32 = 0;
    let mut prev_t = Instant::now();
    let mut log_buf: LogBuffer<LOG_BUFFER_CAP> = LogBuffer::new();
    let mut last_flush = Instant::now();
    loop {
        // Fetch a snapshot and enqueue it for batched debug logging.
        let s = STATE.lock().await.clone();
        log_buf.push(s);

        let flush_due = last_flush.elapsed().as_millis() >= LOG_FLUSH_INTERVAL_MS;
        let backlog_high = log_buf.len() >= LOG_BUFFER_CAP.saturating_sub(1);
        if flush_due || backlog_high {
            let dropped = log_buf.take_drops();
            if dropped != 0 {
                warn!("Logger buffer dropped {} samples", dropped);
            }
            while let Some(sample) = log_buf.pop() {
                log_debug_sample(&sample, DEBUG_PRESET);
            }
            last_flush = Instant::now();
        }

        // Once per ~1s (or slightly more), compute true rates from deltas and print with current values
        let elapsed_ms = prev_t.elapsed().as_millis() as u32;
        if elapsed_ms >= 1000 {
            let rates = SensorRates {
                imu: (s.imu.seq.wrapping_sub(prev_imu_seq) * 1000) / elapsed_ms.max(1),
                imu2: (s.imu2.seq.wrapping_sub(prev_imu2_seq) * 1000) / elapsed_ms.max(1),
                mag: (s.mag.seq.wrapping_sub(prev_mag_seq) * 1000) / elapsed_ms.max(1),
                baro: (s.baro.seq.wrapping_sub(prev_baro_seq) * 1000) / elapsed_ms.max(1),
                gps: (s.gps.seq.wrapping_sub(prev_gps_seq) * 1000) / elapsed_ms.max(1),
            };
            log_rates_sample(&s, rates, RATE_PRESET);
            prev_imu_seq = s.imu.seq;
            prev_imu2_seq = s.imu2.seq;
            prev_mag_seq = s.mag.seq;
            prev_baro_seq = s.baro.seq;
            prev_gps_seq = s.gps.seq;
            prev_t = Instant::now();
        }

        // Simple heartbeat on addressable LED: brief blink at ~2 Hz using configured color
        let pulse = (n & 0xFF) < 50;
        if pulse != led_on {
            led_on = pulse;
            colors[0] = if led_on {
                base_color
            } else {
                RGB8 { r: 0, g: 0, b: 0 }
            };
            led.write(&colors).await;
        }
        Timer::after_micros(1000).await;
        n = n.wrapping_add(1);
    }
}

const UART_TX_LOG_EVERY: u32 = 0;
const UART_TX_LOG_PAYLOAD_MAX: usize = 32;

// UART custom link task (FC -> radio) using custom packet framing + CRC.
#[embassy_executor::task]
pub(crate) async fn uart_link_task(
    uart: RpUart<'static>,
    params: &'static Mutex<RawMutex, ParamRegistry>,
) -> ! {
    let cfg = UartTxConfig {
        log_every: UART_TX_LOG_EVERY,
        log_payload_max: UART_TX_LOG_PAYLOAD_MAX,
        summary_every: 50,
    };
    let rate = ParamUartTelemetryRate { params };
    let source = StateUartTelemetrySource;
    common::tasks::coms::uart_telemetry_task(uart, source, rate, EmbassyDelay, cfg).await
}
