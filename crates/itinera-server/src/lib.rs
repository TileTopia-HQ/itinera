//! # itinera-server
//!
//! HTTP API server for Itinera routing engine.
//! Provides OSRM-compatible route, isochrone, and nearest endpoints.

mod handlers;
mod state;

pub use handlers::router;
pub use state::AppState;
