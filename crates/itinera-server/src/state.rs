use std::sync::Arc;

use itinera_core::ContractionHierarchy;
use itinera_graph::{Graph, SpeedProfile};

/// Shared application state for the HTTP server.
#[derive(Clone)]
pub struct AppState {
    pub graph: Arc<Graph>,
    pub profile: SpeedProfile,
    /// Optional pre-built contraction hierarchy for fast queries.
    pub ch: Option<Arc<ContractionHierarchy>>,
}

impl AppState {
    #[must_use]
    pub fn new(graph: Graph, profile: SpeedProfile) -> Self {
        Self {
            graph: Arc::new(graph),
            profile,
            ch: None,
        }
    }

    #[must_use]
    pub fn with_ch(mut self, ch: ContractionHierarchy) -> Self {
        self.ch = Some(Arc::new(ch));
        self
    }
}
