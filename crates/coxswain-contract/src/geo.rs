/// WGS84 position in radians. Radians because the math consumes them; the
/// manifest compiler converts from the degrees humans author.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct GeoPoint {
    pub lat_rad: f64,
    pub lon_rad: f64,
}
