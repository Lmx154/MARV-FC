use std::{
    env,
    path::{Path, PathBuf},
    process::{Command, ExitStatus, Stdio},
    time::Instant,
};

const DEFAULT_ENDPOINT: &str = "127.0.0.1:9000";
const DEFAULT_TIMEOUT_S: u64 = 180;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Preset {
    Contracts,
    G0,
    G1,
    G2,
    G2Hardening,
    Mission,
    Quick,
    Full,
}

#[derive(Clone, Debug)]
struct Config {
    preset: Preset,
    endpoint: String,
    timeout_s: u64,
    relaxed_reset: bool,
    keep_going: bool,
    list: bool,
}

#[derive(Clone, Debug)]
struct Step {
    name: &'static str,
    kind: StepKind,
    filter: &'static str,
    ignored: bool,
    envs: Vec<(&'static str, String)>,
}

#[derive(Clone, Copy, Debug)]
enum StepKind {
    Harness,
    TelemetryPackage,
}

#[derive(Clone, Debug)]
struct StepResult {
    name: &'static str,
    status: Option<ExitStatus>,
    elapsed_s: f32,
}

fn main() {
    let config = parse_args();
    let root = repo_root();
    let steps = steps_for(config.preset, &config);

    if config.list {
        print_steps(&steps);
        return;
    }

    println!(
        "gazebo replay preset={:?} endpoint={} timeout={}s relaxed_reset={} keep_going={}",
        config.preset, config.endpoint, config.timeout_s, config.relaxed_reset, config.keep_going
    );
    println!("repo root: {}", root.display());

    let mut results = Vec::new();
    for step in steps {
        let result = run_step(&root, &config, &step);
        let failed = !result.status.is_some_and(|status| status.success());
        results.push(result);
        if failed && !config.keep_going {
            break;
        }
    }

    println!();
    println!("summary:");
    let mut failed = false;
    for result in &results {
        let label = match result.status {
            Some(status) if status.success() => "PASS",
            Some(status) => {
                failed = true;
                if let Some(code) = status.code() {
                    println!(
                        "  FAIL {:<42} {:.1}s exit={}",
                        result.name, result.elapsed_s, code
                    );
                } else {
                    println!("  FAIL {:<42} {:.1}s signal", result.name, result.elapsed_s);
                }
                continue;
            }
            None => {
                failed = true;
                "FAIL"
            }
        };
        println!("  {label} {:<42} {:.1}s", result.name, result.elapsed_s);
    }

    if failed {
        std::process::exit(1);
    }
}

fn parse_args() -> Config {
    let mut config = Config {
        preset: Preset::Quick,
        endpoint: env::var("MARV_GAZEBO_BRIDGE_ADDR")
            .unwrap_or_else(|_| DEFAULT_ENDPOINT.to_string()),
        timeout_s: DEFAULT_TIMEOUT_S,
        relaxed_reset: false,
        keep_going: false,
        list: false,
    };

    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "contracts" => config.preset = Preset::Contracts,
            "g0" => config.preset = Preset::G0,
            "g1" => config.preset = Preset::G1,
            "g2" => config.preset = Preset::G2,
            "g2-hardening" => config.preset = Preset::G2Hardening,
            "mission" => config.preset = Preset::Mission,
            "quick" => config.preset = Preset::Quick,
            "full" => config.preset = Preset::Full,
            "--relaxed-reset" => config.relaxed_reset = true,
            "--strict-reset" => config.relaxed_reset = false,
            "--keep-going" => config.keep_going = true,
            "--list" => config.list = true,
            "--help" | "-h" => {
                print_help();
                std::process::exit(0);
            }
            "--endpoint" => {
                config.endpoint = args
                    .next()
                    .unwrap_or_else(|| die("--endpoint requires host:port"));
            }
            "--timeout-s" => {
                let value = args
                    .next()
                    .unwrap_or_else(|| die("--timeout-s requires seconds"));
                config.timeout_s = value
                    .parse()
                    .unwrap_or_else(|_| die("--timeout-s must be an integer"));
            }
            unknown => die(&format!("unknown argument {unknown:?}; try --help")),
        }
    }

    config
}

fn print_help() {
    println!(
        "Run MARV Gazebo bridge replay tests quickly.

USAGE:
  cargo run --manifest-path testing/deterministic-harness/Cargo.toml --bin gazebo_replay -- [preset] [options]

PRESETS:
  quick         contracts + G0 actuator truth + G1 origin + G2 warmup (default)
  contracts     pure parser/config bridge contracts only
  g0            live actuator truth table
  g1            live truth-control origin and vertical step
  g2            live estimator warmup and closed loop
  g2-hardening  live G2 hardening/disturbance gates
  mission       live takeoff/hover/land and G3 navigation gates
  full          all of the above

OPTIONS:
  --endpoint HOST:PORT   bridge endpoint (default env MARV_GAZEBO_BRIDGE_ADDR or 127.0.0.1:9000)
  --timeout-s SECONDS    timeout wrapper per test command (default 180)
  --relaxed-reset        loosen reset cleanliness gates for diagnostics
  --strict-reset         use normal reset cleanliness gates (default)
  --keep-going           continue after failures
  --list                 print selected steps without running"
    );
}

fn steps_for(preset: Preset, config: &Config) -> Vec<Step> {
    match preset {
        Preset::Contracts => contracts(),
        Preset::G0 => vec![g0_all(config)],
        Preset::G1 => vec![g1_origin(config), g1_vertical(config)],
        Preset::G2 => vec![g2_warmup(config), g2_loop(config)],
        Preset::G2Hardening => vec![
            g2_hardening(config),
            g2_disturbance_and_limits(config),
            g2_lateral_disturbance(config),
        ],
        Preset::Mission => vec![g2_takeoff_hover_land(config), g3_navigation(config)],
        Preset::Quick => {
            let mut steps = contracts();
            steps.push(g0_all(config));
            steps.push(g1_origin(config));
            steps.push(g2_warmup(config));
            steps
        }
        Preset::Full => {
            let mut steps = contracts();
            steps.push(g0_all(config));
            steps.push(g1_origin(config));
            steps.push(g1_vertical(config));
            steps.push(g2_warmup(config));
            steps.push(g2_loop(config));
            steps.push(g2_hardening(config));
            steps.push(g2_disturbance_and_limits(config));
            steps.push(g2_lateral_disturbance(config));
            steps.push(g2_takeoff_hover_land(config));
            steps.push(g3_navigation(config));
            steps
        }
    }
}

fn contracts() -> Vec<Step> {
    vec![
        Step {
            name: "telemetry gazebo_bridge_client",
            kind: StepKind::TelemetryPackage,
            filter: "gazebo_bridge_client",
            ignored: false,
            envs: Vec::new(),
        },
        Step {
            name: "p08 gazebo contracts",
            kind: StepKind::Harness,
            filter: "p08_gazebo_contract",
            ignored: false,
            envs: Vec::new(),
        },
    ]
}

fn g0_all(config: &Config) -> Step {
    Step {
        name: "G0 actuator truth table",
        kind: StepKind::Harness,
        filter: "p11_gazebo_actuator_truth_table_runtime",
        ignored: true,
        envs: common_env(config, "G0")
            .into_iter()
            .chain([
                ("MARV_GAZEBO_G0_AUTO_RESET", "1".to_string()),
                ("MARV_GAZEBO_G0_CASE", "all".to_string()),
            ])
            .collect(),
    }
}

fn g1_origin(config: &Config) -> Step {
    Step {
        name: "G1 truth origin hold",
        kind: StepKind::Harness,
        filter: "p12_gazebo_control_truth_runtime",
        ignored: true,
        envs: live_env(config, "G1"),
    }
}

fn g1_vertical(config: &Config) -> Step {
    Step {
        name: "G1 truth vertical step",
        kind: StepKind::Harness,
        filter: "p12_gazebo_control_truth_vertical_step_runtime",
        ignored: true,
        envs: live_env(config, "G1"),
    }
}

fn g2_warmup(config: &Config) -> Step {
    Step {
        name: "G2 estimator warmup",
        kind: StepKind::Harness,
        filter: "p13_gazebo_estimator_warmup",
        ignored: true,
        envs: live_env(config, "G2"),
    }
}

fn g2_loop(config: &Config) -> Step {
    Step {
        name: "G2 estimator loop",
        kind: StepKind::Harness,
        filter: "p13_gazebo_estimator_loop",
        ignored: true,
        envs: live_env(config, "G2"),
    }
}

fn g2_hardening(config: &Config) -> Step {
    Step {
        name: "G2 hardening",
        kind: StepKind::Harness,
        filter: "p14_gazebo_g2_control_hardening",
        ignored: true,
        envs: live_env(config, "G2"),
    }
}

fn g2_disturbance_and_limits(config: &Config) -> Step {
    Step {
        name: "G2 disturbance/demand limits",
        kind: StepKind::Harness,
        filter: "p14_gazebo_g2_runtime_disturbance_and_demand_limits",
        ignored: true,
        envs: live_env(config, "G2"),
    }
}

fn g2_lateral_disturbance(config: &Config) -> Step {
    Step {
        name: "G2 lateral disturbance",
        kind: StepKind::Harness,
        filter: "p14_gazebo_g2_runtime_lateral_setpoint_disturbance",
        ignored: true,
        envs: live_env(config, "G2"),
    }
}

fn g2_takeoff_hover_land(config: &Config) -> Step {
    Step {
        name: "G2 takeoff hover land",
        kind: StepKind::Harness,
        filter: "p15_gazebo_takeoff_hover_land",
        ignored: true,
        envs: live_env(config, "G2"),
    }
}

fn g3_navigation(config: &Config) -> Step {
    Step {
        name: "G3 navigation",
        kind: StepKind::Harness,
        filter: "p17_gazebo_navigation_validation",
        ignored: true,
        envs: live_env(config, "G3"),
    }
}

fn live_env(config: &Config, generation: &'static str) -> Vec<(&'static str, String)> {
    let mut envs = common_env(config, generation);
    envs.push((auto_reset_key(generation), "1".to_string()));
    if config.relaxed_reset {
        envs.extend(reset_relaxation(generation));
    }
    envs
}

fn common_env(config: &Config, _generation: &'static str) -> Vec<(&'static str, String)> {
    vec![("MARV_GAZEBO_BRIDGE_ADDR", config.endpoint.clone())]
}

fn auto_reset_key(generation: &str) -> &'static str {
    match generation {
        "G0" => "MARV_GAZEBO_G0_AUTO_RESET",
        "G1" => "MARV_GAZEBO_G1_AUTO_RESET",
        "G2" => "MARV_GAZEBO_G2_AUTO_RESET",
        "G3" => "MARV_GAZEBO_G3_AUTO_RESET",
        _ => unreachable!("unknown Gazebo generation"),
    }
}

fn reset_relaxation(generation: &str) -> Vec<(&'static str, String)> {
    match generation {
        "G1" => vec![
            ("MARV_GAZEBO_G1_RESET_MAX_POSITION_M", "1.0".to_string()),
            ("MARV_GAZEBO_G1_RESET_MAX_VELOCITY_MPS", "3.0".to_string()),
        ],
        "G2" => vec![
            ("MARV_GAZEBO_G2_RESET_MAX_POSITION_M", "1.0".to_string()),
            ("MARV_GAZEBO_G2_RESET_MAX_VELOCITY_MPS", "3.0".to_string()),
        ],
        "G3" => vec![
            ("MARV_GAZEBO_G3_RESET_MAX_POSITION_M", "1.0".to_string()),
            ("MARV_GAZEBO_G3_RESET_MAX_VELOCITY_MPS", "3.0".to_string()),
        ],
        _ => Vec::new(),
    }
}

fn run_step(root: &Path, config: &Config, step: &Step) -> StepResult {
    println!();
    println!("==> {}", step.name);
    let start = Instant::now();
    let mut command = timeout_command(config.timeout_s);
    append_cargo_test(root, &mut command, step);
    command.current_dir(root);
    command.stdout(Stdio::inherit());
    command.stderr(Stdio::inherit());
    for (key, value) in &step.envs {
        command.env(key, value);
    }

    let status = command.status().ok();
    let elapsed_s = start.elapsed().as_secs_f32();
    StepResult {
        name: step.name,
        status,
        elapsed_s,
    }
}

fn timeout_command(timeout_s: u64) -> Command {
    let mut command = Command::new("timeout");
    command.arg(format!("{timeout_s}s"));
    command
}

fn append_cargo_test(root: &Path, command: &mut Command, step: &Step) {
    command.arg("cargo").arg("test");
    match step.kind {
        StepKind::Harness => {
            command
                .arg("--manifest-path")
                .arg(root.join("testing/deterministic-harness/Cargo.toml"));
        }
        StepKind::TelemetryPackage => {
            command.arg("-p").arg("telemetry-app");
        }
    }
    command.arg(step.filter);
    if step.ignored {
        command.arg("--").arg("--ignored").arg("--nocapture");
    }
}

fn print_steps(steps: &[Step]) {
    for step in steps {
        let ignored = if step.ignored {
            " -- --ignored --nocapture"
        } else {
            ""
        };
        println!("{:<42} cargo test {}{}", step.name, step.filter, ignored);
        for (key, value) in &step.envs {
            println!("  {key}={value}");
        }
    }
}

fn repo_root() -> PathBuf {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    manifest_dir
        .parent()
        .and_then(Path::parent)
        .map(Path::to_path_buf)
        .unwrap_or_else(|| die("could not find repository root from CARGO_MANIFEST_DIR"))
}

fn die(message: &str) -> ! {
    eprintln!("gazebo_replay: {message}");
    std::process::exit(2);
}
