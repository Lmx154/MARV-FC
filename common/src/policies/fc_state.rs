//! Flight computer state machine and policy enforcement (HAL-agnostic).
//! Encodes arming + custom mode into MAVLink-friendly fields.

use mavio::dialects::common::enums::{MavModeFlag, MavResult, MavState};

pub const CUSTOM_MODE_MODE_IDLE: u32 = 0;
pub const CUSTOM_MODE_MODE_CONFIG: u32 = 1;

/// Project-specific operation mode encoded into MAVLink `custom_mode`.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FcCustomMode {
    Idle,
    Config,
}

impl FcCustomMode {
    pub fn from_raw(raw: u32) -> Option<Self> {
        match raw {
            CUSTOM_MODE_MODE_IDLE => Some(Self::Idle),
            CUSTOM_MODE_MODE_CONFIG => Some(Self::Config),
            _ => None,
        }
    }

    pub fn as_u32(self) -> u32 {
        match self {
            Self::Idle => CUSTOM_MODE_MODE_IDLE,
            Self::Config => CUSTOM_MODE_MODE_CONFIG,
        }
    }
}

/// Policy knobs that decide how the state machine behaves.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct FcStatePolicy {
    pub usb_forces_config: bool,
    pub allow_arm_in_config: bool,
}

impl Default for FcStatePolicy {
    fn default() -> Self {
        Self {
            usb_forces_config: true,
            allow_arm_in_config: false,
        }
    }
}

/// Snapshot of the current FC state expressed in MAVLink terms.
#[derive(Clone, Copy, Debug)]
pub struct FcStateSnapshot {
    pub usb_connected: bool,
    pub armed: bool,
    pub mode: FcCustomMode,
    pub custom_mode: u32,
    pub base_mode: MavModeFlag,
    pub system_status: MavState,
}

impl FcStateSnapshot {
    pub fn status_text(&self) -> &'static str {
        match (self.armed, self.mode, self.usb_connected) {
            (true, _, _) => "STATE: MAV_STATE_ACTIVE (ARMED)",
            (false, FcCustomMode::Idle, _) => "STATE: MAV_STATE_STANDBY / MODE_IDLE",
            (false, FcCustomMode::Config, true) => "STATE: MAV_STATE_STANDBY / MODE_CONFIG (USB)",
            (false, FcCustomMode::Config, false) => "STATE: MAV_STATE_STANDBY / MODE_CONFIG",
        }
    }
}

/// Result of applying a command or policy-induced transition.
#[derive(Clone, Copy, Debug)]
pub struct CommandOutcome {
    pub mav_result: MavResult,
    pub snapshot: FcStateSnapshot,
    pub state_changed: bool,
    pub statustext: Option<&'static str>,
}

impl CommandOutcome {
    fn accepted(snapshot: FcStateSnapshot, state_changed: bool) -> Self {
        let statustext = if state_changed {
            Some(snapshot.status_text())
        } else {
            None
        };
        Self {
            mav_result: MavResult::Accepted,
            snapshot,
            state_changed,
            statustext,
        }
    }

    fn denied(snapshot: FcStateSnapshot, msg: &'static str) -> Self {
        Self {
            mav_result: MavResult::Denied,
            snapshot,
            state_changed: false,
            statustext: Some(msg),
        }
    }

    pub fn unsupported(snapshot: FcStateSnapshot) -> Self {
        Self {
            mav_result: MavResult::Unsupported,
            snapshot,
            state_changed: false,
            statustext: Some("Denied: unknown mode"),
        }
    }
}

/// Minimal FC state (HAL-agnostic).
#[derive(Clone, Copy, Debug)]
pub struct FcState {
    usb_connected: bool,
    armed: bool,
    mode: FcCustomMode,
}

impl FcState {
    pub fn new() -> Self {
        Self {
            usb_connected: false,
            armed: false,
            mode: FcCustomMode::Idle,
        }
    }

    pub fn snapshot(&self) -> FcStateSnapshot {
        let mut base_mode = MavModeFlag::CUSTOM_MODE_ENABLED;
        if self.armed {
            base_mode.insert(MavModeFlag::SAFETY_ARMED);
        }

        let system_status = if self.armed {
            MavState::Active
        } else {
            MavState::Standby
        };

        FcStateSnapshot {
            usb_connected: self.usb_connected,
            armed: self.armed,
            mode: self.mode,
            custom_mode: self.mode.as_u32(),
            base_mode,
            system_status,
        }
    }

    /// Update USB connectivity flag and enforce forced CONFIG/disarm policy.
    /// Returns a snapshot when the state changed (useful to emit STATUSTEXT/heartbeat).
    pub fn set_usb_connected(
        &mut self,
        connected: bool,
        policy: FcStatePolicy,
    ) -> Option<FcStateSnapshot> {
        let mut changed = false;
        if self.usb_connected != connected {
            self.usb_connected = connected;
            changed = true;
        }

        if policy.usb_forces_config {
            if self.usb_connected {
                if self.mode != FcCustomMode::Config {
                    self.mode = FcCustomMode::Config;
                    changed = true;
                }
                if self.armed && !policy.allow_arm_in_config {
                    self.armed = false;
                    changed = true;
                }
            } else if self.mode == FcCustomMode::Config && !self.armed {
                self.mode = FcCustomMode::Idle;
                changed = true;
            }
        }

        if changed {
            Some(self.snapshot())
        } else {
            None
        }
    }

    /// Handle a mode request (MAV_CMD_DO_SET_MODE). Mode changes are only allowed while disarmed.
    pub fn request_mode(&mut self, requested: FcCustomMode, policy: FcStatePolicy) -> CommandOutcome {
        if self.armed {
            return CommandOutcome::denied(self.snapshot(), "Denied: armed");
        }

        if self.usb_connected && policy.usb_forces_config && requested == FcCustomMode::Idle {
            return CommandOutcome::denied(self.snapshot(), "Denied: USB connected + CONFIG forced");
        }

        if requested == self.mode {
            return CommandOutcome::accepted(self.snapshot(), false);
        }

        self.mode = requested;
        CommandOutcome::accepted(self.snapshot(), true)
    }

    /// Handle arm/disarm request (MAV_CMD_COMPONENT_ARM_DISARM).
    pub fn request_arm(&mut self, arm: bool, policy: FcStatePolicy) -> CommandOutcome {
        if arm {
            if self.armed {
                return CommandOutcome::accepted(self.snapshot(), false);
            }

            if self.usb_connected && policy.usb_forces_config {
                return CommandOutcome::denied(self.snapshot(), "Denied: CONFIG/USB");
            }

            if self.mode == FcCustomMode::Config && !policy.allow_arm_in_config {
                return CommandOutcome::denied(self.snapshot(), "Denied: MODE_CONFIG");
            }

            self.armed = true;
            return CommandOutcome::accepted(self.snapshot(), true);
        }

        let was_armed = self.armed;
        self.armed = false;

        if policy.usb_forces_config && self.usb_connected {
            self.mode = FcCustomMode::Config;
        }

        CommandOutcome::accepted(self.snapshot(), was_armed)
    }
}
