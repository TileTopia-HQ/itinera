use thiserror::Error;

/// Errors during OSM import.
#[derive(Debug, Error)]
pub enum OsmError {
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    #[error("XML parse error: {0}")]
    XmlParse(String),

    #[error("missing node reference: {0}")]
    MissingNode(i64),

    #[error("unsupported format: {0}")]
    UnsupportedFormat(String),
}
