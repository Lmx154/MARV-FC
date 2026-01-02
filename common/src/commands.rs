//! Single source of truth for USB-CDC CLI commands.
//!
//! The CLI layer should do two things only:
//! 1) parse a line into (command, args[])
//! 2) dispatch to handlers using this registry
//!
//! NOTE: these are *commands*, not parameters. Commands trigger actions.

#![allow(dead_code)]

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum CommandGroup {
    System,
    Param,
    I2c,
    Sensor,
    Sd,
    Telemetry,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum CommandId {
    // --- System ---
    Help,
    Info,
    Reboot,

    // --- Parameters (backed by ParamRegistry) ---
    ParamList,
    ParamGet,
    ParamSet,
    ParamReset,

    // --- I2C ---
    I2cScan,

    // --- Sensors (quick bring-up checks) ---
    SensorStatus,
    SensorRead,

    // --- SD (bring-up) ---
    SdSmokeTest,
    SdWrite,
    SdRead,
    SdEditFirst,

    // --- Telemetry ---
    TelemOnce,
}

#[derive(Copy, Clone, Debug)]
pub struct CommandDef {
    pub id: CommandId,
    pub group: CommandGroup,
    /// Primary token (what user types first).
    pub name: &'static str,
    /// Short usage string (for `help`).
    pub usage: &'static str,
    /// One-liner help text.
    pub help: &'static str,
}

pub const COMMAND_DEFS: &[CommandDef] = &[
    // --- System ---
    CommandDef {
        id: CommandId::Help,
        group: CommandGroup::System,
        name: "help",
        usage: "help [group]",
        help: "List commands (optionally filtered by group).",
    },
    CommandDef {
        id: CommandId::Info,
        group: CommandGroup::System,
        name: "info",
        usage: "info",
        help: "Print firmware build / board / transport info.",
    },
    CommandDef {
        id: CommandId::Reboot,
        group: CommandGroup::System,
        name: "reboot",
        usage: "reboot",
        help: "Reboot the flight computer.",
    },

    // --- Parameters ---
    CommandDef {
        id: CommandId::ParamList,
        group: CommandGroup::Param,
        name: "param.list",
        usage: "param.list",
        help: "List all parameters (name, type, value).",
    },
    CommandDef {
        id: CommandId::ParamGet,
        group: CommandGroup::Param,
        name: "param.get",
        usage: "param.get <NAME>",
        help: "Get a parameter value by name.",
    },
    CommandDef {
        id: CommandId::ParamSet,
        group: CommandGroup::Param,
        name: "param.set",
        usage: "param.set <NAME> <VALUE>",
        help: "Set a parameter by name (type checked).",
    },
    CommandDef {
        id: CommandId::ParamReset,
        group: CommandGroup::Param,
        name: "param.reset",
        usage: "param.reset",
        help: "Reset all parameters to defaults.",
    },

    // --- I2C ---
    CommandDef {
        id: CommandId::I2cScan,
        group: CommandGroup::I2c,
        name: "i2c.scan",
        usage: "i2c.scan [start_hex] [end_hex]",
        help: "Scan I2C bus for responding addresses.",
    },

    // --- Sensors ---
    CommandDef {
        id: CommandId::SensorStatus,
        group: CommandGroup::Sensor,
        name: "sensor.status",
        usage: "sensor.status",
        help: "Print a quick status line for each sensor driver.",
    },
    CommandDef {
        id: CommandId::SensorRead,
        group: CommandGroup::Sensor,
        name: "sensor.read",
        usage: "sensor.read <bmi088|icm45686|bmm350|bmp390|gps>",
        help: "Read one sample from a sensor and print it.",
    },

    // --- SD bring-up (maps to your SdBlackBox helper) ---
    CommandDef {
        id: CommandId::SdSmokeTest,
        group: CommandGroup::Sd,
        name: "sd.smoke",
        usage: "sd.smoke",
        help: "Run SD blackbox smoke-test (create/write/edit/read).",
    },
    CommandDef {
        id: CommandId::SdWrite,
        group: CommandGroup::Sd,
        name: "sd.write",
        usage: "sd.write <name> <ascii_text>",
        help: "Create/truncate a file and write text (bounded).",
    },
    CommandDef {
        id: CommandId::SdRead,
        group: CommandGroup::Sd,
        name: "sd.read",
        usage: "sd.read <name>",
        help: "Read entire file contents (bounded).",
    },
    CommandDef {
        id: CommandId::SdEditFirst,
        group: CommandGroup::Sd,
        name: "sd.edit1",
        usage: "sd.edit1 <name> <from> <to>",
        help: "Replace first occurrence of <from> with <to> in a file.",
    },

    // --- Telemetry ---
    CommandDef {
        id: CommandId::TelemOnce,
        group: CommandGroup::Telemetry,
        name: "telem.once",
        usage: "telem.once",
        help: "Send one telemetry frame immediately (for link testing).",
    },
];

pub fn find_command(name: &str) -> Option<&'static CommandDef> {
    COMMAND_DEFS.iter().find(|c| c.name == name)
}
