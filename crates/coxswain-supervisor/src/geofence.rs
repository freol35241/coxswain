//! Point-in-ring test for the geofence.
//!
//! Planar small-area approximation: a vertex maps to (lon_rad * cos(lat0),
//! lat_rad) with lat0 the ring's first vertex latitude. Good for fences of
//! operations-area scale (a few kilometres), where meridian convergence
//! across the fence is negligible. Not valid near the poles or for rings
//! spanning the antimeridian; the manifest compiler is where such rings
//! should be rejected.

use coxswain_contract::GeoPoint;

/// Ray cast, even-odd rule. Points exactly on a vertex or edge classify
/// either way: the boundary has measure zero, and the breach latch makes the
/// choice irrelevant because the failsafe setpoint drives the vessel back to
/// a decisively inside position anyway.
///
/// A ring that repeats its first vertex (the manifest's closed-ring form) is
/// handled: the degenerate closing edge has equal latitudes and never
/// crosses the cast ray.
pub(crate) fn point_in_ring(ring: &[GeoPoint], p: GeoPoint) -> bool {
    debug_assert!(ring.len() >= 3);
    let scale = libm::cos(ring[0].lat_rad);
    let px = p.lon_rad * scale;
    let py = p.lat_rad;
    let mut inside = false;
    // Pairs (v0,v1) .. (v_last,v0): every edge exactly once.
    for (a, b) in ring.iter().zip(ring.iter().cycle().skip(1)) {
        let (ax, ay) = (a.lon_rad * scale, a.lat_rad);
        let (bx, by) = (b.lon_rad * scale, b.lat_rad);
        // The crossing test guarantees ay != by, so the division is safe.
        if (ay > py) != (by > py) && px < (bx - ax) * (py - ay) / (by - ay) + ax {
            inside = !inside;
        }
    }
    inside
}

#[cfg(test)]
mod tests {
    use super::point_in_ring;
    use coxswain_contract::GeoPoint;

    // Micro-radian coordinates near the equator, so cos(lat0) is ~1 and the
    // planar projection is effectively the identity.
    fn pt(x: f64, y: f64) -> GeoPoint {
        GeoPoint {
            lat_rad: y * 1e-6,
            lon_rad: x * 1e-6,
        }
    }

    // L-shape: the unit square grid cells (0..4, 0..2) plus (2..4, 2..4).
    fn l_shape() -> [GeoPoint; 6] {
        [
            pt(0.0, 0.0),
            pt(4.0, 0.0),
            pt(4.0, 4.0),
            pt(2.0, 4.0),
            pt(2.0, 2.0),
            pt(0.0, 2.0),
        ]
    }

    #[test]
    fn non_convex_ring_inside_and_outside() {
        let ring = l_shape();
        assert!(point_in_ring(&ring, pt(1.0, 1.0)));
        assert!(point_in_ring(&ring, pt(3.0, 1.0)));
        assert!(point_in_ring(&ring, pt(3.0, 3.0)));
        // The notch of the L is outside even though its bounding box is not.
        assert!(!point_in_ring(&ring, pt(1.0, 3.0)));
        assert!(!point_in_ring(&ring, pt(5.0, 1.0)));
        assert!(!point_in_ring(&ring, pt(-1.0, 1.0)));
        assert!(!point_in_ring(&ring, pt(1.0, -1.0)));
    }

    #[test]
    fn closed_ring_duplicate_endpoint_is_harmless() {
        let open = l_shape();
        let closed = [
            open[0], open[1], open[2], open[3], open[4], open[5], open[0],
        ];
        for probe in [
            pt(1.0, 1.0),
            pt(3.0, 3.0),
            pt(1.0, 3.0),
            pt(5.0, 1.0),
            pt(-1.0, -1.0),
        ] {
            assert_eq!(point_in_ring(&open, probe), point_in_ring(&closed, probe));
        }
    }

    #[test]
    fn boundary_points_classify_either_way() {
        // On-vertex and on-edge results are deliberately unspecified (see the
        // function doc): the boundary has measure zero and the breach latch
        // makes the choice irrelevant. Asserted here is only that the
        // classification terminates and returns a value.
        let ring = l_shape();
        let _ = point_in_ring(&ring, pt(0.0, 0.0)); // vertex
        let _ = point_in_ring(&ring, pt(2.0, 0.0)); // edge midpoint
    }
}
