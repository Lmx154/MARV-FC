//! Navigation-frame helpers.

const EARTH_RADIUS_M: f32 = 6_378_137.0;
const DEG_TO_RAD: f32 = core::f32::consts::PI / 180.0;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GeodeticPosition {
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub alt_msl_m: f32,
}

impl GeodeticPosition {
    pub const fn new(lat_deg: f64, lon_deg: f64, alt_msl_m: f32) -> Self {
        Self {
            lat_deg,
            lon_deg,
            alt_msl_m,
        }
    }

    pub fn is_finite(self) -> bool {
        self.lat_deg.is_finite() && self.lon_deg.is_finite() && self.alt_msl_m.is_finite()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LocalNedFrame {
    origin: GeodeticPosition,
}

impl LocalNedFrame {
    pub const fn new(origin: GeodeticPosition) -> Self {
        Self { origin }
    }

    pub const fn origin(self) -> GeodeticPosition {
        self.origin
    }

    pub fn from_first_valid(origin: &mut Option<Self>, position: GeodeticPosition) -> Option<Self> {
        if !position.is_finite() {
            return None;
        }

        Some(*origin.get_or_insert(Self::new(position)))
    }

    pub fn position_ned_m(self, position: GeodeticPosition) -> Option<[f32; 3]> {
        if !position.is_finite() {
            return None;
        }

        let origin = self.origin;
        let origin_lat_rad = origin.lat_deg as f32 * DEG_TO_RAD;
        let north_m = ((position.lat_deg - origin.lat_deg) as f32) * DEG_TO_RAD * EARTH_RADIUS_M;
        let east_m = ((position.lon_deg - origin.lon_deg) as f32)
            * DEG_TO_RAD
            * EARTH_RADIUS_M
            * micromath::F32Ext::cos(origin_lat_rad);
        let down_m = origin.alt_msl_m - position.alt_msl_m;

        Some([north_m, east_m, down_m])
    }
}

#[cfg(test)]
mod tests {
    use super::{GeodeticPosition, LocalNedFrame};

    #[test]
    fn first_valid_position_initializes_origin_and_maps_to_zero() {
        let position = GeodeticPosition::new(30.0, -97.0, 120.0);
        let mut origin = None;

        let frame = LocalNedFrame::from_first_valid(&mut origin, position)
            .expect("finite position should initialize frame");

        assert_eq!(frame.origin(), position);
        assert_eq!(frame.position_ned_m(position), Some([0.0, 0.0, 0.0]));
    }

    #[test]
    fn nearby_position_maps_to_local_ned_axes() {
        let frame = LocalNedFrame::new(GeodeticPosition::new(30.0, -97.0, 120.0));

        let position = frame
            .position_ned_m(GeodeticPosition::new(30.000_01, -96.999_99, 125.0))
            .expect("finite position should map into NED");

        assert!(position[0] > 1.0);
        assert!(position[1] > 0.9);
        assert_eq!(position[2], -5.0);
    }

    #[test]
    fn invalid_position_does_not_initialize_frame() {
        let mut origin = None;

        assert!(
            LocalNedFrame::from_first_valid(
                &mut origin,
                GeodeticPosition::new(f64::NAN, -97.0, 120.0)
            )
            .is_none()
        );
        assert!(origin.is_none());
    }
}
