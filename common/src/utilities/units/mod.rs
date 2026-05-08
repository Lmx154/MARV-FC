//! Unit conversion helpers belong here.

pub const FEET_PER_METER: f32 = 3.280_839_8;
pub const STANDARD_SEA_LEVEL_PRESSURE_PA: f32 = 101_325.0;

const STANDARD_ATMOSPHERE_ALTITUDE_FACTOR_M: f32 = 44_330.0;
const STANDARD_ATMOSPHERE_PRESSURE_EXPONENT: f32 = 0.190_294_95;

pub fn pressure_altitude_m(pressure_pa: f32, sea_level_pressure_pa: f32) -> Option<f32> {
    if pressure_pa <= 0.0 || sea_level_pressure_pa <= 0.0 {
        return None;
    }

    Some(
        STANDARD_ATMOSPHERE_ALTITUDE_FACTOR_M
            * (1.0
                - micromath::F32Ext::powf(
                    pressure_pa / sea_level_pressure_pa,
                    STANDARD_ATMOSPHERE_PRESSURE_EXPONENT,
                )),
    )
}

pub fn pressure_altitude_ft(pressure_pa: f32, sea_level_pressure_pa: f32) -> Option<f32> {
    pressure_altitude_m(pressure_pa, sea_level_pressure_pa)
        .map(|altitude_m| altitude_m * FEET_PER_METER)
}

#[cfg(test)]
mod tests {
    use super::{STANDARD_SEA_LEVEL_PRESSURE_PA, pressure_altitude_ft};

    #[test]
    fn sea_level_pressure_maps_near_zero_altitude() {
        let altitude_ft = pressure_altitude_ft(
            STANDARD_SEA_LEVEL_PRESSURE_PA,
            STANDARD_SEA_LEVEL_PRESSURE_PA,
        )
        .expect("sea-level pressure should produce an altitude");

        assert!(altitude_ft.abs() < 1.0);
    }

    #[test]
    fn lower_pressures_map_to_higher_altitudes() {
        let lower_pressure_altitude_ft =
            pressure_altitude_ft(75_260.0, STANDARD_SEA_LEVEL_PRESSURE_PA)
                .expect("expected altitude");
        let higher_pressure_altitude_ft =
            pressure_altitude_ft(90_000.0, STANDARD_SEA_LEVEL_PRESSURE_PA)
                .expect("expected altitude");

        assert!(lower_pressure_altitude_ft > higher_pressure_altitude_ft);
        assert!(higher_pressure_altitude_ft > 0.0);
    }
}
