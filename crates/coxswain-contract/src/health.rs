#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum HealthLevel {
    Nominal,
    Degraded,
    Fault,
}

/// Covariance-based estimator health. The stds come straight from the state
/// covariance; the staleness flags say which fused roles have gone past their
/// declared max_age. The supervisor applies thresholds, this only reports.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct EstimatorHealth {
    pub level: HealthLevel,
    pub position_std_m: f64,
    pub heading_std_rad: f64,
    pub gnss_stale: bool,
    pub heading_stale: bool,
    pub yaw_rate_stale: bool,
}
