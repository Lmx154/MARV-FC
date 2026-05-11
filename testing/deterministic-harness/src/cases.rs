pub type HarnessResult<T> = Result<T, HarnessFailure>;

#[derive(Clone, Debug, PartialEq)]
pub struct TestCase {
    pub name: String,
    pub nominal_dt_s: f32,
    pub max_ticks: u64,
    pub random_seed: Option<u64>,
}

impl TestCase {
    pub fn new(name: impl Into<String>, nominal_dt_s: f32, max_ticks: u64) -> Self {
        Self {
            name: name.into(),
            nominal_dt_s,
            max_ticks,
            random_seed: None,
        }
    }

    pub fn with_seed(mut self, seed: u64) -> Self {
        self.random_seed = Some(seed);
        self
    }

    pub fn validate(&self) -> HarnessResult<()> {
        if self.name.trim().is_empty() {
            return Err(HarnessFailure::new(
                0,
                0,
                "test case name must not be empty",
            ));
        }
        if !self.nominal_dt_s.is_finite() || self.nominal_dt_s <= 0.0 {
            return Err(HarnessFailure::new(
                0,
                0,
                "test case nominal_dt_s must be finite and positive",
            ));
        }
        if self.max_ticks == 0 {
            return Err(HarnessFailure::new(
                0,
                0,
                "test case max_ticks must be non-zero",
            ));
        }

        Ok(())
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct StepTrace {
    pub tick: u64,
    pub sim_time_us: u64,
    pub dt_s: f32,
    pub label: String,
    pub motors: Option<[f32; 4]>,
}

impl StepTrace {
    pub fn new(tick: u64, sim_time_us: u64, dt_s: f32, label: impl Into<String>) -> Self {
        Self {
            tick,
            sim_time_us,
            dt_s,
            label: label.into(),
            motors: None,
        }
    }

    pub fn with_motors(mut self, motors: [f32; 4]) -> Self {
        self.motors = Some(motors);
        self
    }
}

pub type HarnessTrace = Vec<StepTrace>;

#[derive(Clone, Debug, PartialEq)]
pub struct HarnessFailure {
    pub tick: u64,
    pub sim_time_us: u64,
    pub message: String,
    pub last_trace: Option<StepTrace>,
}

impl HarnessFailure {
    pub fn new(tick: u64, sim_time_us: u64, message: impl Into<String>) -> Self {
        Self {
            tick,
            sim_time_us,
            message: message.into(),
            last_trace: None,
        }
    }

    pub fn with_last_trace(mut self, last_trace: StepTrace) -> Self {
        self.last_trace = Some(last_trace);
        self
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct HarnessReport {
    pub case: TestCase,
    pub traces: HarnessTrace,
    pub failure: Option<HarnessFailure>,
}

impl HarnessReport {
    pub fn new(case: TestCase) -> Self {
        Self {
            case,
            traces: Vec::new(),
            failure: None,
        }
    }

    pub fn push_trace(&mut self, trace: StepTrace) {
        self.traces.push(trace);
    }

    pub fn fail(&mut self, message: impl Into<String>) -> HarnessFailure {
        let last_trace = self.traces.last().cloned();
        let (tick, sim_time_us) = last_trace
            .as_ref()
            .map(|trace| (trace.tick, trace.sim_time_us))
            .unwrap_or((0, 0));
        let mut failure = HarnessFailure::new(tick, sim_time_us, message);
        failure.last_trace = last_trace;
        self.failure = Some(failure.clone());
        failure
    }

    pub fn passed(&self) -> bool {
        self.failure.is_none()
    }

    pub fn last_trace(&self) -> Option<&StepTrace> {
        self.traces.last()
    }
}

pub fn assert_all_finite(
    tick: u64,
    sim_time_us: u64,
    label: &str,
    values: &[f32],
) -> HarnessResult<()> {
    if values.iter().all(|value| value.is_finite()) {
        Ok(())
    } else {
        Err(HarnessFailure::new(
            tick,
            sim_time_us,
            format!("{label} contains non-finite values: {values:?}"),
        ))
    }
}

pub fn assert_unit_interval(
    tick: u64,
    sim_time_us: u64,
    label: &str,
    values: &[f32],
) -> HarnessResult<()> {
    if values
        .iter()
        .all(|value| value.is_finite() && (0.0..=1.0).contains(value))
    {
        Ok(())
    } else {
        Err(HarnessFailure::new(
            tick,
            sim_time_us,
            format!("{label} must be finite normalized values in [0, 1]: {values:?}"),
        ))
    }
}
