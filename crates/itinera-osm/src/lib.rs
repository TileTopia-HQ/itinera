//! # itinera-osm
//!
//! OpenStreetMap data import for road network extraction.
//! Supports XML (.osm) and PBF (.osm.pbf) parsing.

mod error;
pub mod parser;
mod pbf;
mod tags;

pub use error::OsmError;
pub use parser::{ImportStats, OsmImporter};
pub use pbf::parse_pbf;
pub use tags::{highway_to_road_class, is_oneway, max_speed_from_tags};
