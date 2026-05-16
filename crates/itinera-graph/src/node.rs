use serde::{Deserialize, Serialize};

use crate::Coord;

/// Opaque node identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct NodeId(pub u32);

/// A node (intersection) in the road network.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Node {
    pub id: NodeId,
    pub coord: Coord,
    /// OSM node ID (for debugging / snapping).
    pub osm_id: i64,
    /// Contraction hierarchy level (0 = not contracted).
    pub ch_level: u16,
}
