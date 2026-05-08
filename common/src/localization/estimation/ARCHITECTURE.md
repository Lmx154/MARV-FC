# Estimator Architecture

This module is a structural port of the Python estimator into fixed-size, `no_std`-friendly Rust.
The goal is to keep the same separation of responsibilities:

- estimator contracts and reusable Gaussian filter engines
- process models
- measurement models
- external policies such as gating
- a composition layer that wires the models into a usable estimator stack

The current public composition is a **layered estimator**:

- **attitude layer**: quaternion attitude plus gyro bias, implemented as an **ESKF**
- **navigation layer**: position, velocity, and accelerometer bias, implemented as an **EKF**

These two layers are composed in `stacks/layered_navigation.rs`.

## High-Level Layering

The estimator is deliberately split into generic reusable pieces.

```text
raw IMU / GNSS / baro samples
            |
            v
integration boundary outside this crate
            |
            +--> AttitudeInput
            |      |
            |      v
            |   Attitude ESKF
            |   state: [q, b_g]
            |   covariance: 6x6 error covariance
            |
            +--> NavigationInput
                   |
                   v
                Navigation EKF
                state: [p, v, b_a]
                covariance: 9x9 covariance
```

The important coupling is:

1. The attitude ESKF predicts the quaternion from gyroscope input.
2. The resulting quaternion is converted into `R_b_to_i`.
3. That rotation matrix is fed into the navigation EKF predict step.

So the layers are **separate filters**, but they are not independent:

- the navigation predict step depends on the current attitude estimate
- the attitude filter does **not** consume navigation state
- the stack exposes a single combined nominal state, but it keeps **two separate covariance blocks**

## What Lives Where

### `core/`

This contains the generic contracts and reusable filter engines.

- `base.rs`
  - `EuclideanState`
  - `ProcessModel`
  - `EuclideanProcessModel`
  - `ErrorStateProcessModel`
  - `MeasurementModel`
- `ekf.rs`
  - generic additive-state EKF engine
- `eskf.rs`
  - generic error-state EKF engine
- `gaussian.rs`
  - shared covariance math
  - innovation covariance
  - Mahalanobis distance
  - Kalman gain
  - Joseph-form covariance update

### `models/`

This contains the process models.

- `attitude.rs`
  - attitude nominal state and gyro-bias error-state dynamics
- `navigation.rs`
  - position/velocity/accelerometer-bias dynamics
- `strapdown.rs`
  - a generic monolithic inertial propagation model
  - currently not used by the layered stack, but useful as a reusable building block

### `measurements/`

This contains measurement models only.

- `gravity.rs`
  - accelerometer gravity-alignment update for attitude
- `gps_position.rs`
  - position observation for navigation
- `gps_velocity.rs`
  - velocity observation for navigation
- `baro_altitude.rs`
  - scalar altitude observation for navigation

### `policies/`

This contains external decisions that should stay outside process and measurement math.

- `gating.rs`
  - accept/reject/skip policies

### `stacks/`

This is the composition layer.

- `layered_navigation.rs`
  - owns one attitude ESKF and one navigation EKF
  - defines the combined external state view
  - routes predict and measurement calls to the correct layer

## Which Layer Is Which

### Attitude estimator

The attitude estimator is:

- the **ESKF**
- the **manifold-valued** layer
- the filter that estimates:
  - quaternion `q`
  - gyro bias `b_g`

Nominal state:

```text
x_att_nominal = [ q, b_g ]
```

Error state:

```text
delta_x_att = [ delta_theta, delta_b_g ]    (6x1)
```

Why ESKF here:

- the attitude state contains a quaternion
- quaternions live on a manifold, not in a plain Euclidean vector space
- direct additive correction of the quaternion itself is the wrong structure
- the correct small-error update is:

```text
q <- q (*) Exp(delta_theta)
```

This is exactly what the ESKF gives you:

- propagate the nominal quaternion normally
- maintain covariance in a small Euclidean error state
- inject the correction into the nominal quaternion after the update

### Navigation estimator

The navigation estimator is:

- the **EKF**
- the **Euclidean** layer
- the filter that estimates:
  - position `p`
  - velocity `v`
  - accelerometer bias `b_a`

State:

```text
x_nav = [ p, v, b_a ]    (9x1)
```

Why EKF here:

- all current navigation states are ordinary vectors in Euclidean space
- the correction is naturally additive

```text
x_nav <- x_nav + delta_x
```

- there is no manifold-valued state in this layer in the current implementation

If navigation later grows a quaternion or another constrained orientation state, that would be a reason to revisit the design. As implemented now, plain EKF is the correct fit.

## Combined State and Covariance

The composition layer exposes a unified nominal state:

```text
x_layered = [ q, b_g, p, v, b_a ]
```

but it does **not** build one monolithic covariance over that whole state.

Instead it keeps:

- `P_att`: 6x6 covariance for the attitude ESKF error state
- `P_nav`: 9x9 covariance for the navigation EKF state

That means the structure is:

```text
P_layered = { P_att, P_nav }
```

not

```text
P_full = 15x15
```

This is an intentional structural decision.

Benefits:

- preserves the existing Python layering
- keeps each filter generic and reusable
- avoids a large monolithic filter with mixed responsibilities
- makes it easy to swap measurement models per layer

Tradeoff:

- the stack does not explicitly maintain cross-covariance terms between attitude and navigation

That is acceptable for the current architecture because the design priority is modularity and clear separation, not a single tightly coupled inertial navigation filter.

## Predict Step

The layered predict call does two predicts in order.

## 1. Attitude ESKF predict

Input:

```text
u_att = gyroscope_rps
```

The attitude process model:

1. subtracts estimated gyro bias
2. converts angular increment to a small quaternion
3. right-multiplies the nominal quaternion

Nominal propagation:

```text
omega_corr = omega_meas - b_g
q_k+1 = normalize( q_k (*) Exp(omega_corr * dt) )
b_g,k+1 = b_g,k
```

Error-state covariance propagation:

```text
P_k+1 = F P_k F^T + G Q G^T
```

with:

```text
delta_x_att = [ delta_theta, delta_b_g ]
```

continuous-time error dynamics matrix:

```text
A_att =
[ -[omega_corr]_x   -I ]
[      0             0 ]
```

discrete first-order approximation used by the code:

```text
F_att ~= I + A_att * dt
```

process noise Jacobian:

```text
G_att =
[ -I   0 ]
[  0   I ]
```

process noise covariance:

```text
Q_att =
diag(
  sigma_g^2 dt,
  sigma_g^2 dt,
  sigma_g^2 dt,
  sigma_bg^2 dt,
  sigma_bg^2 dt,
  sigma_bg^2 dt
)
```

Interpretation:

- gyro white noise drives attitude error
- gyro bias random walk drives bias error

## 2. Navigation EKF predict

After the attitude predict, the stack computes:

```text
R_b_to_i = R(q_attitude)
```

Then it forms the navigation control input:

```text
u_nav = [ a_meas, R_b_to_i ]
```

Nominal propagation:

```text
a_corr_body = a_meas - b_a
a_i = R_b_to_i * a_corr_body

p_k+1 = p_k + v_k dt + 0.5 a_i dt^2
v_k+1 = v_k + a_i dt
b_a,k+1 = b_a,k
```

Current implementation detail:

- `accelerometer_includes_gravity = true` by default
- so the model assumes the incoming acceleration is already in the form expected by the process model configuration
- if you want the model to explicitly add gravity, the config supports that

Covariance propagation is the standard EKF predict:

```text
P_k+1 = F P_k F^T + G Q G^T
```

with state:

```text
x_nav = [ p, v, b_a ]
```

The code uses the discrete-time transition matrix:

```text
F_nav =
[ I   I dt   -0.5 R_b_to_i dt^2 ]
[ 0    I      -R_b_to_i dt      ]
[ 0    0            I           ]
```

This is the exact matrix currently implemented in `models/navigation.rs`.

The corresponding continuous-time intuition is:

```text
A_nav =
[ 0   I   0  ]
[ 0   0  -R  ]
[ 0   0   0  ]
```

where `R = R_b_to_i`.

The implemented process noise mapping is:

```text
G_nav =
[ R  0  0 ]
[ 0  R  0 ]
[ 0  0  I ]
```

and the discrete noise covariance is:

```text
Q_nav =
[ sigma_a^2 dt^3/3 I   sigma_a^2 dt^2/2 I      0 ]
[ sigma_a^2 dt^2/2 I   sigma_a^2 dt I          0 ]
[        0                    0           sigma_ba^2 dt I ]
```

Interpretation:

- accelerometer noise drives both position and velocity uncertainty
- accelerometer bias random walk drives the bias state

## Measurement Update Step

Each measurement model provides four pieces:

- `h(x)` through `predict_measurement`
- innovation `y`
- Jacobian `H`
- covariance `R`

Both EKF and ESKF use the same measurement-side math:

```text
y = innovation(z, h(x))
S = H P H^T + R
K = P H^T S^-1
```

and both use the Joseph covariance form:

```text
P_new = (I - K H) P (I - K H)^T + K R K^T
```

The difference is what happens to the state after computing the correction.

### EKF measurement step

For the navigation EKF:

```text
delta_x = K y
x <- x + delta_x
```

This is ordinary additive correction in the state coordinates.

### ESKF measurement step

For the attitude ESKF:

```text
delta_x = K y
x_nominal <- inject(x_nominal, delta_x)
P <- J_r P J_r^T
```

That extra reset step is the key difference:

- the correction is interpreted as a small error state
- the nominal quaternion is updated through quaternion injection
- the covariance is mapped through the reset Jacobian `J_r`

For the current attitude model:

```text
J_r =
[ I - 0.5 [delta_theta]_x   0 ]
[          0                I ]
```

## Which Measurements Hit Which Layer

### Attitude measurements

Current attitude measurement model:

- gravity alignment from accelerometer

Measurement:

```text
z_g = measured body-frame gravity direction / acceleration proxy
```

Prediction:

```text
h_att(x) = R_b_to_i(q)^T g_i
```

Jacobian:

```text
H_gravity = [ [h_att(x)]_x   0 ]
```

This update corrects:

- attitude error
- indirectly gyro bias through the state covariance coupling

### Navigation measurements

Current navigation measurements:

- GPS position
- GPS velocity
- barometric altitude

GPS position:

```text
h_pos(x) = p
H_pos = [ I  0  0 ]
```

GPS velocity:

```text
h_vel(x) = v
H_vel = [ 0  I  0 ]
```

Barometric altitude:

```text
h_baro(x) = p[axis]
H_baro = [ 0 ... 1_at_axis ... 0 ]
```

These updates correct:

- position
- velocity
- accelerometer bias, indirectly through covariance coupling

## How Process and Measurement Models Compose

The design is intentionally contract-driven.

The filter engines do not know anything about rockets, drones, IMUs, or GPS directly.

They only require:

### Process model contract

- propagate the nominal state
- supply `F`
- supply `G`
- supply `Q`
- for ESKF, also supply:
  - `inject`
  - `reset_jacobian`

### Measurement model contract

- predict `h(x)`
- compute innovation `y`
- supply `H`
- supply `R`

That means:

- the EKF engine can be reused with any Euclidean state/process/measurement set
- the ESKF engine can be reused with any manifold-valued nominal state and Euclidean error-state model

The layered stack simply picks a set of concrete models and routes calls to them.

## EKF vs ESKF: What Is Actually Different

This is the core conceptual difference.

### EKF

An EKF assumes the nominal state itself can be updated additively:

```text
x <- x + delta_x
```

This is appropriate when the state lives in a Euclidean vector space.

Examples in this code:

- position
- velocity
- accelerometer bias

### ESKF

An ESKF separates:

- the nominal state on the manifold
- a small Euclidean error state around it

The filter covariance lives in the error-state coordinates, not directly in the nominal manifold coordinates.

That is appropriate when the nominal state contains quantities like:

- quaternion attitude
- rotations on SO(3)
- other constrained states where direct addition is not the natural correction operator

Examples in this code:

- quaternion attitude

### Why attitude uses ESKF

Because the quaternion is not a plain 4-vector you should correct with direct addition.

The correct small-angle update is multiplicative:

```text
q <- q (*) Exp(delta_theta)
```

This avoids:

- invalid quaternion updates
- poor linearization behavior from pretending the quaternion lives in flat Euclidean space
- ad hoc renormalization as the only correction mechanism

### Why navigation uses EKF

Because the current navigation state is purely Euclidean:

```text
[p, v, b_a]
```

No manifold-valued orientation state is inside this layer.

So a standard EKF is simpler and correct for the implemented state definition.

## Rocket vs Drone Process Model

This needs to be stated carefully:

**the current Rust estimator does not implement a rocket-specific process model and a drone-specific process model.**

At the estimator-core level, the implemented process models are **vehicle-agnostic**.

That means:

- the current attitude process model is the same for rockets and drones
- the current navigation process model is the same for rockets and drones

Why that is true:

- both rockets and drones obey the same rigid-body inertial kinematics at this abstraction level
- the IMU already measures the specific force and angular rate that drive the estimator
- the estimator currently does not include explicit aerodynamic, thrust, control-surface, rotor, or wind states

So the **current matrix for a rocket and the current matrix for a drone are the same**.

### Current implemented attitude model for both rocket and drone

```text
A_att =
[ -[omega_corr]_x   -I ]
[      0             0 ]
```

```text
F_att ~= I + A_att dt
```

This is appropriate for either vehicle because both use the same gyro-driven attitude kinematics with gyro bias estimation.

### Current implemented navigation model for both rocket and drone

```text
F_nav =
[ I   I dt   -0.5 R_b_to_i dt^2 ]
[ 0    I      -R_b_to_i dt      ]
[ 0    0            I           ]
```

This is also the same for both because the state only captures translational kinematics and accelerometer bias. Vehicle-specific forces are not modeled as explicit states here.

## So When Would Rocket and Drone Matrices Differ?

They would differ only if you choose to move beyond generic inertial kinematics and estimate vehicle-specific dynamics.

Examples:

### Rocket-specific future model

A rocket-oriented process model might augment the state with things like:

- thrust scale or thrust bias
- ballistic coefficient / drag parameter
- angle-of-attack-related aero states
- wind-relative velocity terms
- flight-phase-dependent process noise

Then the translational dynamics would no longer be only:

```text
a_i = R a_body
```

They would become something more like:

```text
a_i = R a_body + f_thrust(x,u) + f_drag(x,u) + f_gravity
```

The Jacobian would then gain extra columns and rows for those new states.

### Drone-specific future model

A drone-oriented process model might augment the state with things like:

- thrust scale / collective bias
- rotor response lag states
- drag coefficients
- wind states
- accelerometer scale factors

That would also change the process Jacobian because acceleration would explicitly depend on these additional states or inputs.

## Current Reality vs Future Specialization

Current reality:

- generic inertial estimator
- no explicit rocket-only or drone-only process state
- same attitude and navigation process matrices for both vehicles

Future specialization:

- keep the `core/` engines unchanged
- add new `models/` implementations
- choose the proper stack composition outside or inside `stacks/`

That is exactly why the architecture was ported structurally instead of collapsed into a single filter.

## Role of `strapdown.rs`

`strapdown.rs` is a reusable inertial propagation model that combines:

- position
- velocity
- quaternion

into a single nominal state with a corresponding error-state model.

Its continuous-time error dynamics are:

```text
A_strapdown =
[ 0   I             0                    ]
[ 0   0   -R_b_to_i [a_corr]_x          ]
[ 0   0          -[omega_corr]_x         ]
```

with error state:

```text
delta_x_strapdown = [ delta_p, delta_v, delta_theta ]
```

This model is not currently the public layered stack. It exists as a generic building block for future estimators that may want:

- a single ESKF over `[p, v, q]`
- a tighter inertial coupling
- a different composition than the current layered design

## Gating and Update Ownership

Measurement gating is deliberately external.

That means:

- measurement models only describe sensor math
- process models only describe propagation math
- the EKF/ESKF engines compute innovations and updates
- gating policies decide whether a measurement is accepted, rejected, or skipped

This keeps the estimator core generic and avoids hard-coding rocket-specific or drone-specific trust logic into the filter engine itself.

## Practical Mental Model

The simplest way to think about the current architecture is:

1. **Attitude ESKF**
   - "Use gyro to propagate quaternion"
   - "Use gravity to trim attitude drift"
2. **Navigation EKF**
   - "Use attitude rotation plus accelerometer to propagate position/velocity"
   - "Use GPS and baro to correct translation and accelerometer bias"

That is the whole layered structure.

The architecture is intentionally generic enough that "rocket" vs "drone" should mostly mean:

- different sensor availability
- different gating and scheduling
- different process-noise tuning
- possibly different future process-model implementations

not a rewrite of the filter engines.
