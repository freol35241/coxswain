/// Integer claimant identity. Names map to ids at the adapter edge; authority
/// logic never parses strings.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ClaimantId(pub u16);

/// The onboard autonomy claimant. Reserved so conn_grant_default = autonomy
/// has a target before any remote claimant registers.
pub const AUTONOMY: ClaimantId = ClaimantId(0);

#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ConnState {
    Unheld,
    Held(ClaimantId),
}

#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ArmingState {
    Disarmed,
    Armed,
}
