//! The status LED's display (`decision-status-led-20261118`): what the one LED shows, as a pure
//! function of the device rows, a few flags, the brightness level and the time since boot.
//!
//! Boot: steady white until [`ROLL_CALL_START_MS`], then the roll call (nine blinks, one per device
//! in the order IMU1, IMU2, BARO, MAG, GNSS, BATT, RC, SD, ESC: green OK, red FAILED, yellow
//! anything else), then the running display: a one-second pulse whose colour is the highest row
//! that applies (see [`Mode`]). Three quick flashes (green good, red bad) are drawn over it when a
//! settings save or a calibration ends; an event that arrives before the roll call has finished
//! waits for its end, and at most one waits (a newer one replaces an older one).
//!
//! No hardware and no clock: the caller passes the time. The colour type is this module's own (three
//! bytes), so the crate does not depend on the LED driver.

use crate::health::State;

/// The LED's colour, 8 bits per channel (the driver scales it by the brightness).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Colour {
    /// Red.
    pub r: u8,
    /// Green.
    pub g: u8,
    /// Blue.
    pub b: u8,
}

impl Colour {
    /// All channels off.
    pub const BLACK: Self = Self::new(0, 0, 0);
    /// Steady colour of the first seconds after boot.
    pub const WHITE: Self = Self::new(255, 255, 255);
    /// OK; the good flashes; GNSS fix.
    pub const GREEN: Self = Self::new(0, 255, 0);
    /// FAILED; the bad flashes; a vital device failed.
    pub const RED: Self = Self::new(255, 0, 0);
    /// Anything else in the roll call; something degraded.
    pub const YELLOW: Self = Self::new(255, 255, 0);
    /// Nothing wrong, no GNSS fix; half of the calibration alternation.
    pub const BLUE: Self = Self::new(0, 0, 255);
    /// A motor is being commanded.
    pub const PURPLE: Self = Self::new(160, 0, 255);

    /// A colour from its channels.
    pub const fn new(r: u8, g: u8, b: u8) -> Self {
        Self { r, g, b }
    }
}

/// What to show now: the colour and the driver's brightness (0 dark, 255 full).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Output {
    /// The colour before the brightness scaling.
    pub colour: Colour,
    /// 0 to 255.
    pub brightness: u8,
}

impl Output {
    /// Dark (one canonical value, so that a change of "dark colour" is never a change).
    pub const DARK: Self = Self {
        colour: Colour::BLACK,
        brightness: 0,
    };
}

/// Index of IMU1 in [`Inputs::rows`] (the roll call's order).
pub const IMU1: usize = 0;
/// Index of IMU2.
pub const IMU2: usize = 1;
/// Index of the barometer.
pub const BARO: usize = 2;
/// Index of the magnetometer.
pub const MAG: usize = 3;
/// Index of the GNSS receiver.
pub const GNSS: usize = 4;
/// Index of the battery monitor.
pub const BATT: usize = 5;
/// Index of the radio receiver.
pub const RC: usize = 6;
/// Index of the SD card.
pub const SD: usize = 7;
/// Index of the ESCs.
pub const ESC: usize = 8;
/// The number of rows the display reads (the roll call's blinks).
pub const ROWS: usize = 9;

/// What the display reads.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Inputs {
    /// The nine rows' health states in roll-call order (the index constants of this module).
    pub rows: [State; ROWS],
    /// The motor test's spin is active (later: armed).
    pub motor_spin: bool,
    /// A calibration is running.
    pub calibrating: bool,
    /// A log is open on the SD card.
    pub log_open: bool,
    /// The latest GNSS sample is younger than 3 s and has a 3D fix or better.
    pub gnss_fix: bool,
    /// `LED_BRIGHT`: 0 off, 1 dim, 2 medium, 3 bright (see [`level_brightness`]).
    pub level: i32,
    /// The low-battery monitor's level (Amendment BATT): 0 none, 1 LOW, 2 CRITICAL (`fw_core::battery::Level`).
    pub battery: u8,
}

/// The roll call starts this long after boot (ms); the ESCs are stable from about 5.4 s.
pub const ROLL_CALL_START_MS: u32 = 6000;
/// One roll-call blink's lit time (ms).
pub const BLINK_ON_MS: u32 = 200;
/// The dark time after each roll-call blink (ms).
pub const BLINK_OFF_MS: u32 = 150;
/// The roll call's end (ms since boot): nine blinks of 350 ms.
pub const ROLL_CALL_END_MS: u32 = ROLL_CALL_START_MS + ROWS as u32 * (BLINK_ON_MS + BLINK_OFF_MS);

/// The pulse's period (ms).
pub const PULSE_MS: u32 = 1000;
/// The pulse's lowest brightness, in thousandths of the level.
pub const PULSE_MIN_PERMILLE: u32 = 150;
/// The double blink's lit time of each blink and the dark time between them (ms); then dark to the
/// end of the pulse's period.
pub const DOUBLE_BLINK_MS: u32 = 100;
/// The calibration alternation's half period: blue, white, blue, ... at 5 Hz (ms).
pub const ALTERNATE_MS: u32 = 100;
/// A flash's lit time and the dark time after it (ms).
pub const FLASH_MS: u32 = 100;
/// The number of flashes of an event.
pub const FLASHES: u32 = 3;
/// The time an event's flashes take (ms).
pub const EVENT_MS: u32 = FLASHES * 2 * FLASH_MS;

/// The driver brightness of a level: 0 off, 1 -> 20, 2 -> 80, 3 and above -> 255 (a level below 0 is off).
pub const fn level_brightness(level: i32) -> u8 {
    match level {
        i32::MIN..=0 => 0,
        1 => 20,
        2 => 80,
        _ => 255,
    }
}

/// The running display's row that applies, highest first.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    /// Red, double blink: IMU1, IMU2, BARO, MAG or BATT is FAILED.
    VitalFailed,
    /// Red at 5 Hz: the battery monitor reached CRITICAL (Amendment BATT).
    BatteryCritical,
    /// Yellow at 5 Hz: the battery monitor reached LOW.
    BatteryLow,
    /// Purple, solid: a motor is being commanded.
    MotorSpin,
    /// Blue / white at 5 Hz: a calibration is running.
    Calibrating,
    /// Yellow pulse: something is degraded, recovering or failed, or no log is open.
    Attention,
    /// Green pulse: a GNSS fix.
    GnssFix,
    /// Blue pulse: otherwise.
    Normal,
}

/// The row that applies. RC or ESC merely ABSENT or STARTING (radio off, no battery) is `Normal`.
pub fn mode(inputs: &Inputs) -> Mode {
    let row = |i: usize| inputs.rows.get(i).copied().unwrap_or(State::Absent);
    if [IMU1, IMU2, BARO, MAG, BATT]
        .into_iter()
        .any(|i| row(i) == State::Failed)
    {
        Mode::VitalFailed
    } else if inputs.battery >= 2 {
        Mode::BatteryCritical
    } else if inputs.battery == 1 {
        Mode::BatteryLow
    } else if inputs.motor_spin {
        Mode::MotorSpin
    } else if inputs.calibrating {
        Mode::Calibrating
    } else if inputs
        .rows
        .iter()
        .any(|s| matches!(s, State::Degraded | State::Recovering))
        || [GNSS, RC, SD, ESC]
            .into_iter()
            .any(|i| row(i) == State::Failed)
        || !inputs.log_open
    {
        Mode::Attention
    } else if inputs.gnss_fix {
        Mode::GnssFix
    } else {
        Mode::Normal
    }
}

/// The roll call's colour for a row: green OK, red FAILED, yellow anything else.
pub const fn roll_call_colour(state: State) -> Colour {
    match state {
        State::Ok => Colour::GREEN,
        State::Failed => Colour::RED,
        State::Starting | State::Degraded | State::Recovering | State::Absent => Colour::YELLOW,
    }
}

/// The pulse's brightness in thousandths of the level at `now_ms`: a smoothstep up over the first half of each
/// period and down over the second, between [`PULSE_MIN_PERMILLE`] and 1000.
pub const fn pulse_permille(now_ms: u32) -> u32 {
    let phase = now_ms % PULSE_MS;
    let half = PULSE_MS / 2;
    let x = if phase < half {
        phase
    } else {
        PULSE_MS - phase
    };
    // smoothstep(x / half) in thousandths: 3t^2 - 2t^3
    let t = x * 1000 / half;
    let s = (3 * t * t * 1000 - 2 * t * t * t) / 1_000_000;
    PULSE_MIN_PERMILLE + (1000 - PULSE_MIN_PERMILLE) * s / 1000
}

fn scaled(base: u8, permille: u32) -> u8 {
    ((u32::from(base) * permille + 500) / 1000).min(255) as u8
}

fn lit(colour: Colour, brightness: u8) -> Output {
    if brightness == 0 {
        Output::DARK
    } else {
        Output { colour, brightness }
    }
}

/// The display without the event flashes: white, the roll call, then the running row, at `now_ms`
/// since boot.
pub fn display(inputs: &Inputs, now_ms: u32) -> Output {
    let base = level_brightness(inputs.level);
    if now_ms < ROLL_CALL_START_MS {
        return lit(Colour::WHITE, base);
    }
    if now_ms < ROLL_CALL_END_MS {
        let t = now_ms - ROLL_CALL_START_MS;
        let slot = BLINK_ON_MS + BLINK_OFF_MS;
        if t % slot >= BLINK_ON_MS {
            return Output::DARK;
        }
        let state = inputs
            .rows
            .get((t / slot) as usize)
            .copied()
            .unwrap_or(State::Absent);
        return lit(roll_call_colour(state), base);
    }
    match mode(inputs) {
        Mode::VitalFailed => {
            let phase = now_ms % PULSE_MS;
            let on = phase < DOUBLE_BLINK_MS
                || (2 * DOUBLE_BLINK_MS..3 * DOUBLE_BLINK_MS).contains(&phase);
            lit(Colour::RED, if on { base } else { 0 })
        }
        Mode::BatteryCritical => lit(Colour::RED, if (now_ms / ALTERNATE_MS).is_multiple_of(2) { base } else { 0 }),
        Mode::BatteryLow => lit(Colour::YELLOW, if (now_ms / ALTERNATE_MS).is_multiple_of(2) { base } else { 0 }),
        Mode::MotorSpin => lit(Colour::PURPLE, base),
        Mode::Calibrating => {
            let colour = if (now_ms / ALTERNATE_MS).is_multiple_of(2) {
                Colour::BLUE
            } else {
                Colour::WHITE
            };
            lit(colour, base)
        }
        Mode::Attention => lit(Colour::YELLOW, scaled(base, pulse_permille(now_ms))),
        Mode::GnssFix => lit(Colour::GREEN, scaled(base, pulse_permille(now_ms))),
        Mode::Normal => lit(Colour::BLUE, scaled(base, pulse_permille(now_ms))),
    }
}

/// The LED's state: the event flashes and the one waiting event. Everything else is [`display`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Led {
    /// The event waiting (true good, false bad); a newer one replaces it.
    pending: Option<bool>,
    /// The flashes in progress: (good, start in ms since boot).
    flashing: Option<(bool, u32)>,
}

impl Led {
    /// An LED with nothing pending.
    pub const fn new() -> Self {
        Self {
            pending: None,
            flashing: None,
        }
    }

    /// A save or a calibration ended: good (accepted) or bad (failed). It is shown when the roll call
    /// is over and no other flashes are running; a newer event replaces one still waiting.
    pub fn event(&mut self, good: bool) {
        self.pending = Some(good);
    }

    /// What to show at `now_ms` since boot (not going back in time between calls).
    pub fn show(&mut self, inputs: &Inputs, now_ms: u32) -> Output {
        if let Some((_, start)) = self.flashing
            && now_ms.saturating_sub(start) >= EVENT_MS
        {
            self.flashing = None;
        }
        if self.flashing.is_none()
            && now_ms >= ROLL_CALL_END_MS
            && let Some(good) = self.pending.take()
        {
            self.flashing = Some((good, now_ms));
        }
        match self.flashing {
            Some((good, start)) => {
                let on = now_ms.saturating_sub(start) % (2 * FLASH_MS) < FLASH_MS;
                let colour = if good { Colour::GREEN } else { Colour::RED };
                lit(
                    colour,
                    if on {
                        level_brightness(inputs.level)
                    } else {
                        0
                    },
                )
            }
            None => display(inputs, now_ms),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const OK: State = State::Ok;

    fn all(state: State) -> [State; ROWS] {
        [state; ROWS]
    }

    fn healthy() -> Inputs {
        Inputs {
            rows: all(OK),
            motor_spin: false,
            calibrating: false,
            log_open: true,
            gnss_fix: false,
            level: 3,
            battery: 0,
        }
    }

    /// A time well after the roll call, at the start of a pulse period.
    const RUN: u32 = 20_000;

    #[test]
    fn the_levels_map_to_the_driver_brightness() {
        assert_eq!(level_brightness(0), 0);
        assert_eq!(level_brightness(1), 20);
        assert_eq!(level_brightness(2), 80);
        assert_eq!(level_brightness(3), 255);
        assert_eq!(level_brightness(-1), 0);
        assert_eq!(level_brightness(9), 255);
    }

    #[test]
    fn white_until_the_roll_call() {
        let mut inputs = healthy();
        inputs.level = 1;
        for t in [0, 1, 3000, ROLL_CALL_START_MS - 1] {
            assert_eq!(
                display(&inputs, t),
                Output {
                    colour: Colour::WHITE,
                    brightness: 20
                },
                "{t}"
            );
        }
    }

    #[test]
    fn the_roll_call_blinks_the_nine_rows_in_order_with_their_colours() {
        let mut inputs = healthy();
        inputs.rows = [
            State::Ok,
            State::Failed,
            State::Degraded,
            State::Recovering,
            State::Starting,
            State::Absent,
            State::Absent,
            State::Ok,
            State::Failed,
        ];
        let expected = [
            Colour::GREEN,
            Colour::RED,
            Colour::YELLOW,
            Colour::YELLOW,
            Colour::YELLOW,
            Colour::YELLOW,
            Colour::YELLOW,
            Colour::GREEN,
            Colour::RED,
        ];
        for (k, colour) in expected.into_iter().enumerate() {
            let start = ROLL_CALL_START_MS + k as u32 * 350;
            for t in [start, start + 100, start + 199] {
                assert_eq!(
                    display(&inputs, t),
                    Output {
                        colour,
                        brightness: 255
                    },
                    "blink {k} at {t}"
                );
            }
            for t in [start + 200, start + 349] {
                assert_eq!(display(&inputs, t), Output::DARK, "gap {k} at {t}");
            }
        }
    }

    #[test]
    fn the_roll_call_reads_the_row_at_the_moment_of_its_blink() {
        let mut inputs = healthy();
        inputs.rows = all(State::Starting);
        let blink_7 = ROLL_CALL_START_MS + 7 * 350;
        assert_eq!(display(&inputs, blink_7).colour, Colour::YELLOW);
        inputs.rows[SD] = State::Ok;
        assert_eq!(display(&inputs, blink_7).colour, Colour::GREEN);
        assert_eq!(display(&inputs, ROLL_CALL_START_MS).colour, Colour::YELLOW);
    }

    #[test]
    fn the_roll_call_ends_after_nine_blinks() {
        assert_eq!(ROLL_CALL_END_MS, 9150);
        let inputs = healthy();
        assert_ne!(
            display(&inputs, ROLL_CALL_END_MS - 1),
            display(&inputs, ROLL_CALL_END_MS - 1 - 200)
        );
        // From the end, the running display: a blue pulse for a healthy vehicle without a fix.
        assert_eq!(display(&inputs, ROLL_CALL_END_MS).colour, Colour::BLUE);
    }

    #[test]
    fn a_failed_vital_row_beats_everything_with_a_red_double_blink() {
        for vital in [IMU1, IMU2, BARO, MAG, BATT] {
            let mut inputs = healthy();
            inputs.motor_spin = true;
            inputs.calibrating = true;
            inputs.gnss_fix = true;
            inputs.rows[vital] = State::Failed;
            assert_eq!(mode(&inputs), Mode::VitalFailed, "row {vital}");
        }
        let mut inputs = healthy();
        inputs.rows[BATT] = State::Failed;
        let base = RUN;
        let shape: std::vec::Vec<bool> = (0..10)
            .map(|k| display(&inputs, base + k * 100 + 50) != Output::DARK)
            .collect();
        assert_eq!(
            shape,
            [
                true, false, true, false, false, false, false, false, false, false
            ]
        );
        assert_eq!(display(&inputs, base).colour, Colour::RED);
        assert_eq!(display(&inputs, base).brightness, 255);
    }

    #[test]
    fn each_running_row_beats_the_ones_below_it() {
        let mut inputs = healthy();
        inputs.gnss_fix = true;
        inputs.motor_spin = true;
        inputs.calibrating = true;
        inputs.log_open = false;
        inputs.rows[SD] = State::Degraded;
        assert_eq!(mode(&inputs), Mode::MotorSpin);
        assert_eq!(display(&inputs, RUN).colour, Colour::PURPLE);
        assert_eq!(display(&inputs, RUN + 333).brightness, 255);
        inputs.motor_spin = false;
        assert_eq!(mode(&inputs), Mode::Calibrating);
        inputs.calibrating = false;
        assert_eq!(mode(&inputs), Mode::Attention);
        inputs.log_open = true;
        assert_eq!(mode(&inputs), Mode::Attention);
        inputs.rows[SD] = State::Ok;
        assert_eq!(mode(&inputs), Mode::GnssFix);
        inputs.gnss_fix = false;
        assert_eq!(mode(&inputs), Mode::Normal);
    }

    #[test]
    fn the_calibration_alternates_blue_and_white_at_5_hz() {
        let mut inputs = healthy();
        inputs.calibrating = true;
        let colours: std::vec::Vec<Colour> = (0..6)
            .map(|k| display(&inputs, RUN + k * 100 + 10).colour)
            .collect();
        assert_eq!(
            colours,
            [
                Colour::BLUE,
                Colour::WHITE,
                Colour::BLUE,
                Colour::WHITE,
                Colour::BLUE,
                Colour::WHITE
            ]
        );
        assert_eq!(display(&inputs, RUN + 10).brightness, 255);
    }

    #[test]
    fn yellow_for_degraded_recovering_failed_rows_and_a_missing_log() {
        for (row, state) in [
            (IMU1, State::Degraded),
            (BARO, State::Recovering),
            (MAG, State::Degraded),
            (GNSS, State::Failed),
            (RC, State::Failed),
            (SD, State::Failed),
            (ESC, State::Failed),
            (ESC, State::Degraded),
            (RC, State::Recovering),
        ] {
            let mut inputs = healthy();
            inputs.rows[row] = state;
            assert_eq!(mode(&inputs), Mode::Attention, "{row} {state:?}");
            assert_eq!(display(&inputs, RUN).colour, Colour::YELLOW);
        }
        let mut inputs = healthy();
        inputs.log_open = false;
        assert_eq!(mode(&inputs), Mode::Attention);
    }

    #[test]
    fn rc_and_esc_absent_or_starting_are_not_yellow() {
        for state in [State::Absent, State::Starting] {
            let mut inputs = healthy();
            inputs.rows[RC] = state;
            inputs.rows[ESC] = state;
            assert_eq!(mode(&inputs), Mode::Normal, "{state:?}");
            assert_eq!(display(&inputs, RUN).colour, Colour::BLUE);
            inputs.gnss_fix = true;
            assert_eq!(mode(&inputs), Mode::GnssFix);
        }
    }

    #[test]
    fn the_pulse_stays_within_its_bounds_and_is_smooth() {
        let mut lo = u32::MAX;
        let mut hi = 0;
        let mut previous = pulse_permille(0);
        for t in 0..3000 {
            let p = pulse_permille(t);
            assert!((PULSE_MIN_PERMILLE..=1000).contains(&p), "{t}: {p}");
            assert!(p.abs_diff(previous) <= 10, "{t}: {previous} -> {p}");
            previous = p;
            lo = lo.min(p);
            hi = hi.max(p);
        }
        assert_eq!((lo, hi), (PULSE_MIN_PERMILLE, 1000));
        assert_eq!(pulse_permille(0), PULSE_MIN_PERMILLE);
        assert_eq!(pulse_permille(500), 1000);
        assert_eq!(pulse_permille(1000), PULSE_MIN_PERMILLE);
        // Rises over the first half, falls over the second.
        assert!(pulse_permille(250) < pulse_permille(400));
        assert!(pulse_permille(750) > pulse_permille(900));
        // The displayed brightness follows it at each level.
        for level in 1..=3 {
            let mut inputs = healthy();
            inputs.level = level;
            let top = level_brightness(level);
            for t in 0..1000 {
                let b = display(&inputs, RUN + t).brightness;
                assert!(
                    b <= top && u32::from(b) * 1000 + 500 >= u32::from(top) * 150,
                    "{level} {t} {b}"
                );
            }
            assert_eq!(display(&inputs, RUN + 500).brightness, top);
        }
    }

    #[test]
    fn level_0_is_dark_in_every_display() {
        let mut inputs = healthy();
        inputs.level = 0;
        let mut led = Led::new();
        led.event(true);
        for t in (0..30_000).step_by(7) {
            assert_eq!(led.show(&inputs, t), Output::DARK, "{t}");
        }
        inputs.motor_spin = true;
        assert_eq!(display(&inputs, RUN), Output::DARK);
        inputs.rows[IMU1] = State::Failed;
        assert_eq!(display(&inputs, RUN), Output::DARK);
    }

    #[test]
    fn an_event_flashes_three_times_over_the_display_and_goes_back() {
        let inputs = healthy();
        let mut led = Led::new();
        let t0 = RUN;
        assert_eq!(led.show(&inputs, t0 - 20), display(&inputs, t0 - 20));
        led.event(true);
        let shape: std::vec::Vec<Output> = (0..8)
            .map(|k| led.show(&inputs, t0 + k * 100 + 50))
            .collect();
        let green = Output {
            colour: Colour::GREEN,
            brightness: 255,
        };
        assert_eq!(
            shape[..6],
            [
                green,
                Output::DARK,
                green,
                Output::DARK,
                green,
                Output::DARK
            ]
        );
        assert_eq!(shape[6], display(&inputs, t0 + 650));
        assert_eq!(shape[7], display(&inputs, t0 + 750));
        let mut led = Led::new();
        led.event(false);
        assert_eq!(led.show(&inputs, t0).colour, Colour::RED);
    }

    #[test]
    fn a_flash_is_drawn_over_the_red_double_blink_and_the_spin() {
        let mut inputs = healthy();
        inputs.motor_spin = true;
        inputs.rows[IMU2] = State::Failed;
        let mut led = Led::new();
        led.event(false);
        assert_eq!(led.show(&inputs, RUN).colour, Colour::RED);
        assert_eq!(led.show(&inputs, RUN + 150), Output::DARK);
        inputs.rows[IMU2] = State::Ok;
        led.event(true);
        assert_eq!(
            led.show(&inputs, RUN + 600),
            Output {
                colour: Colour::GREEN,
                brightness: 255
            }
        );
    }

    #[test]
    fn an_event_during_the_roll_call_waits_until_it_has_finished() {
        let inputs = healthy();
        let mut led = Led::new();
        led.event(true);
        for t in (ROLL_CALL_START_MS..ROLL_CALL_END_MS).step_by(10) {
            assert_eq!(led.show(&inputs, t), display(&inputs, t), "{t}");
        }
        assert_eq!(led.show(&inputs, 100), display(&inputs, 100));
        // At its end the flashes start, whatever the roll call's last slot showed.
        assert_eq!(
            led.show(&inputs, ROLL_CALL_END_MS),
            Output {
                colour: Colour::GREEN,
                brightness: 255
            }
        );
        assert_eq!(led.show(&inputs, ROLL_CALL_END_MS + 150), Output::DARK);
        assert_eq!(
            led.show(&inputs, ROLL_CALL_END_MS + 600),
            display(&inputs, ROLL_CALL_END_MS + 600)
        );
    }

    #[test]
    fn at_most_one_event_waits_and_the_newer_replaces_the_older() {
        let inputs = healthy();
        let mut led = Led::new();
        led.event(true);
        led.event(false);
        let green = Colour::GREEN;
        assert_eq!(led.show(&inputs, RUN).colour, Colour::RED);
        // While it flashes, two more arrive: only the last one is shown, after it.
        led.event(true);
        led.event(false);
        led.event(true);
        assert_eq!(led.show(&inputs, RUN + 400).colour, Colour::RED);
        assert_eq!(led.show(&inputs, RUN + 600).colour, green);
        assert_eq!(led.show(&inputs, RUN + 700), Output::DARK);
        assert_eq!(led.show(&inputs, RUN + 1200), display(&inputs, RUN + 1200));
        // Nothing is left over.
        assert_eq!(led.show(&inputs, RUN + 1300), display(&inputs, RUN + 1300));
    }

    #[test]
    fn the_roll_call_colour_covers_every_state() {
        assert_eq!(roll_call_colour(State::Ok), Colour::GREEN);
        assert_eq!(roll_call_colour(State::Failed), Colour::RED);
        for s in [
            State::Starting,
            State::Degraded,
            State::Recovering,
            State::Absent,
        ] {
            assert_eq!(roll_call_colour(s), Colour::YELLOW);
        }
        assert_eq!(display(&healthy(), 0).colour, Colour::WHITE);
    }

    /// Amendment BATT: CRITICAL is red and LOW yellow at 5 Hz (100 ms on, 100 ms off), above the motor spin, the calibration
    /// and the yellow pulse, below a failed vital device; level 0 changes nothing.
    #[test]
    fn the_battery_levels_blink_at_5_hz_above_row_2_and_below_row_1() {
        let base = |battery| Inputs { battery, ..healthy() };
        let busy = |battery| Inputs { motor_spin: true, calibrating: true, log_open: false, ..base(battery) };
        for (battery, colour, mode_now) in [(2, Colour::RED, Mode::BatteryCritical), (1, Colour::YELLOW, Mode::BatteryLow)] {
            assert_eq!(mode(&busy(battery)), mode_now);
            for k in 0..10u32 {
                let out = display(&busy(battery), RUN + 100 * k);
                if k % 2 == 0 {
                    assert_eq!(out, Output { colour, brightness: level_brightness(3) }, "k {k}");
                } else {
                    assert_eq!(out, Output::DARK, "k {k}");
                }
            }
        }
        assert_eq!(mode(&base(2)), Mode::BatteryCritical);
        assert_eq!(mode(&Inputs { battery: 2, rows: { let mut r = all(OK); r[BATT] = State::Failed; r }, ..healthy() }), Mode::VitalFailed);
        // Negative control: with no level the same inputs are the rows below.
        assert_eq!(mode(&busy(0)), Mode::MotorSpin);
        assert_eq!(mode(&base(0)), Mode::Normal);
    }
}
