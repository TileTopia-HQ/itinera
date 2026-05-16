use thiserror::Error;

/// Errors that can occur during routing.
#[derive(Debug, Error)]
pub enum RoutingError {
    #[error("no route found between {from} and {to}")]
    NoRoute { from: String, to: String },

    #[error("node not found: {0}")]
    NodeNotFound(u32),

    #[error("graph is empty")]
    EmptyGraph,

    #[error("invalid coordinate: lat={lat}, lon={lon}")]
    InvalidCoordinate { lat: f64, lon: f64 },
}
