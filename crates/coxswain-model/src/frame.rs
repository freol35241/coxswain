//! Local NED tangent frame anchored at a fixed origin.
//!
//! Small-area approximation: the WGS84 meridian and normal radii are
//! evaluated once at the origin latitude and treated as constant, so a
//! radian of latitude or longitude maps to a fixed number of meters
//! everywhere in the frame. Fine at harbor scale; the error grows with
//! distance from the origin, so revisit before open-ocean missions.

use coxswain_contract::GeoPoint;

const WGS84_A_M: f64 = 6_378_137.0;
const WGS84_E2: f64 = 0.006_694_379_990_14;

#[derive(Clone, Copy, Debug)]
pub struct LocalFrame {
    origin: GeoPoint,
    /// Meridian radius of curvature at the origin, meters per radian of
    /// latitude.
    r_m: f64,
    /// Normal radius times cos(origin latitude), meters per radian of
    /// longitude.
    r_n_cos_lat: f64,
}

impl LocalFrame {
    pub fn new(origin: GeoPoint) -> Self {
        let sin_lat = libm::sin(origin.lat_rad);
        let cos_lat = libm::cos(origin.lat_rad);
        let w2 = 1.0 - WGS84_E2 * sin_lat * sin_lat;
        let r_n = WGS84_A_M / libm::sqrt(w2);
        let r_m = WGS84_A_M * (1.0 - WGS84_E2) / (w2 * libm::sqrt(w2));
        Self {
            origin,
            r_m,
            r_n_cos_lat: r_n * cos_lat,
        }
    }

    pub fn origin(&self) -> GeoPoint {
        self.origin
    }

    /// North/east meters from the origin.
    pub fn to_local(&self, p: GeoPoint) -> (f64, f64) {
        (
            (p.lat_rad - self.origin.lat_rad) * self.r_m,
            (p.lon_rad - self.origin.lon_rad) * self.r_n_cos_lat,
        )
    }

    pub fn to_geo(&self, n_m: f64, e_m: f64) -> GeoPoint {
        GeoPoint {
            lat_rad: self.origin.lat_rad + n_m / self.r_m,
            lon_rad: self.origin.lon_rad + e_m / self.r_n_cos_lat,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip_within_two_km_box() {
        let origin = GeoPoint {
            lat_rad: 57.67_f64.to_radians(),
            lon_rad: 11.85_f64.to_radians(),
        };
        let frame = LocalFrame::new(origin);
        // Points expressed in geodetic coordinates first, so the test goes
        // geo -> local -> geo, the direction the estimator uses.
        for i in -2..=2 {
            for j in -2..=2 {
                let p = frame.to_geo(1_000.0 * f64::from(i), 1_000.0 * f64::from(j));
                let (n, e) = frame.to_local(p);
                let q = frame.to_geo(n, e);
                assert!(libm::fabs(q.lat_rad - p.lat_rad) < 1e-9);
                assert!(libm::fabs(q.lon_rad - p.lon_rad) < 1e-9);
            }
        }
        // Scale sanity: a radian of latitude at 57.67 N is on the order of
        // the meridian radius, so 1e-5 rad is roughly 64 m.
        let (n, _) = frame.to_local(GeoPoint {
            lat_rad: origin.lat_rad + 1e-5,
            lon_rad: origin.lon_rad,
        });
        assert!(n > 63.0 && n < 65.0);
    }
}
