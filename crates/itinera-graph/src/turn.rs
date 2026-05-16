use serde::{Deserialize, Serialize};

use crate::NodeId;

/// Turn restriction (e.g., no left turn).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TurnRestriction {
    /// The "via" node where the restriction applies.
    pub via_node: NodeId,
    /// OSM way ID of the "from" way.
    pub from_way: i64,
    /// OSM way ID of the "to" way.
    pub to_way: i64,
    /// Type of restriction.
    pub restriction_type: RestrictionType,
}

/// Type of turn restriction.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum RestrictionType {
    /// Prohibits this turn.
    No,
    /// Only this turn is allowed from the via node.
    Only,
}

/// Turn cost at a node (seconds penalty).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TurnCost {
    pub via_node: NodeId,
    pub from_edge_idx: u32,
    pub to_edge_idx: u32,
    pub cost_s: f64,
}
