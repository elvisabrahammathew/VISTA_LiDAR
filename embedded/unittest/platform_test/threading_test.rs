//! Unit tests for cross-platform thread priority validation.

use super::*;

/// Confirms that every documented project priority is accepted.
#[test]
fn accepts_priority_levels_one_through_five() {
    for level in HIGHEST_PRIORITY..=LOWEST_PRIORITY {
        assert_eq!(ThreadPriority::new(level).unwrap().level(), level);
    }
}

/// Confirms that values outside the documented range fail before spawning.
#[test]
fn rejects_priority_outside_project_range() {
    assert!(ThreadPriority::new(0).is_err());
    assert!(ThreadPriority::new(6).is_err());
}

/// Confirms that a stop request is visible through every cloned token.
#[test]
fn shares_cooperative_stop_state() {
    let first = StopToken::new();
    let second = first.clone();

    assert!(!second.is_stop_requested());
    first.request_stop();
    assert!(second.is_stop_requested());
}
