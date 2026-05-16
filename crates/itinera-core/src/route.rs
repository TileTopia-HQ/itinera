use serde::{Deserialize, Serialize};

use itinera_graph::Coord;

/// A computed route with geometry and turn-by-turn instructions.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Route {
    /// Total distance in meters.
    pub distance_m: f64,
    /// Total duration in seconds.
    pub duration_s: f64,
    /// Ordered node IDs along the path.
    pub node_ids: Vec<u32>,
    /// Route geometry (all coordinates along the path).
    pub geometry: Vec<Coord>,
    /// Turn-by-turn steps.
    pub steps: Vec<RouteStep>,
}

/// A single step in turn-by-turn navigation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RouteStep {
    /// Distance of this step in meters.
    pub distance_m: f64,
    /// Duration of this step in seconds.
    pub duration_s: f64,
    /// Road name (if available).
    pub name: Option<String>,
    /// Maneuver at the start of this step.
    pub maneuver: StepManeuver,
}

/// Maneuver type for navigation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum StepManeuver {
    Depart,
    Arrive,
    TurnLeft,
    TurnRight,
    TurnSlightLeft,
    TurnSlightRight,
    TurnSharpLeft,
    TurnSharpRight,
    Continue,
    UTurn,
    Roundabout { exit_number: u8 },
    Merge,
    Fork { direction: ForkDirection },
}

/// Direction of a fork.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ForkDirection {
    Left,
    Right,
}
