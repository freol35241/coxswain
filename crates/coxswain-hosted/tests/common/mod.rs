//! Shared by the rig tests.

use std::ops::{Deref, DerefMut};
use std::process::Child;

/// A child process that dies with the test. A failing assertion unwinds
/// past the test's own teardown, and an orphaned vessel keeps the test's
/// stderr open, so cargo waits on it indefinitely; zenohd would keep its
/// port. Dropping this kills and reaps the child instead.
pub struct Reaped(pub Child);

impl Drop for Reaped {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

impl Deref for Reaped {
    type Target = Child;
    fn deref(&self) -> &Child {
        &self.0
    }
}

impl DerefMut for Reaped {
    fn deref_mut(&mut self) -> &mut Child {
        &mut self.0
    }
}
