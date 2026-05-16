use serde::{Deserialize, Serialize};

use crate::NodeId;

/// Opaque edge identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct EdgeId(pub u32);

/// A directed edge in the road network graph.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Edge {
    /// Source node.
    pub from: NodeId,
    /// Target node.
    pub to: NodeId,
    /// Length in meters.
    pub distance_m: f64,
    /// Travel time in seconds at free-flow speed.
    pub duration_s: f64,
    /// OSM way ID (for debugging / turn-by-turn).
    pub way_id: i64,
    /// Road class (motorway=1 .. residential=7).
    pub road_class: u8,
    /// Whether this is a one-way edge.
    pub oneway: bool,
    /// Optional road name.
    pub name: Option<String>,
    /// Edge geometry (sequence of intermediate coordinates, excluding endpoints).
    pub geometry: Vec<crate::Coord>,
}
