use axum::{
    Router,
    extract::{Query, State},
    http::StatusCode,
    response::Json,
    routing::get,
};
use serde::{Deserialize, Serialize};
use tower_http::cors::CorsLayer;

use itinera_core::{Route, astar, dijkstra, isochrone};
use itinera_graph::{Coord, SpeedProfile};

use crate::state::AppState;

/// Build the HTTP router.
pub fn router(state: AppState) -> Router {
    Router::new()
        .route("/route", get(route_handler))
        .route("/nearest", get(nearest_handler))
        .route("/isochrone", get(isochrone_handler))
        .route("/health", get(health_handler))
        .layer(CorsLayer::permissive())
        .with_state(state)
}

// === Request/Response types ===

#[derive(Debug, Deserialize)]
struct RouteQuery {
    /// Source coordinate: "lat,lon"
    from: String,
    /// Target coordinate: "lat,lon"
    to: String,
    /// Algorithm: "dijkstra", "astar", or "ch" (default: "astar")
    algorithm: Option<String>,
    /// Profile: "car", "bicycle", "pedestrian", "truck" (default: "car")
    profile: Option<String>,
}

#[derive(Debug, Deserialize)]
struct NearestQuery {
    lat: f64,
    lon: f64,
}

#[derive(Debug, Deserialize)]
struct IsochroneQuery {
    lat: f64,
    lon: f64,
    /// Max travel time in seconds.
    max_seconds: f64,
    /// Profile: "car", "bicycle", "pedestrian", "truck" (default: "car")
    profile: Option<String>,
}

#[derive(Debug, Serialize)]
struct RouteResponse {
    distance_m: f64,
    duration_s: f64,
    geometry: Vec<[f64; 2]>,
    steps: Vec<StepResponse>,
}

#[derive(Debug, Serialize)]
struct StepResponse {
    distance_m: f64,
    duration_s: f64,
    name: Option<String>,
    maneuver: String,
}

#[derive(Debug, Serialize)]
struct NearestResponse {
    node_id: u32,
    lat: f64,
    lon: f64,
    distance_m: f64,
}

#[derive(Debug, Serialize)]
struct IsochroneResponse {
    reachable_nodes: usize,
    boundary: Vec<[f64; 2]>,
}

#[derive(Debug, Serialize)]
struct ErrorResponse {
    error: String,
}

// === Handlers ===

async fn health_handler() -> &'static str {
    "ok"
}

async fn route_handler(
    State(state): State<AppState>,
    Query(params): Query<RouteQuery>,
) -> Result<Json<RouteResponse>, (StatusCode, Json<ErrorResponse>)> {
    let from = parse_coord(&params.from).map_err(bad_request)?;
    let to = parse_coord(&params.to).map_err(bad_request)?;

    let profile = resolve_profile(params.profile.as_deref(), &state.profile)?;

    let source = state
        .graph
        .nearest_node(from)
        .ok_or_else(|| bad_request("no node found near source".to_string()))?;
    let target = state
        .graph
        .nearest_node(to)
        .ok_or_else(|| bad_request("no node found near target".to_string()))?;

    let algo = params.algorithm.as_deref().unwrap_or("astar");

    match algo {
        "ch" => {
            let ch = state.ch.as_ref().ok_or_else(|| {
                bad_request(
                    "contraction hierarchy not available; use 'astar' or 'dijkstra'".to_string(),
                )
            })?;
            let (cost, path) = ch.query(source, target, &profile).ok_or_else(|| {
                (
                    StatusCode::NOT_FOUND,
                    Json(ErrorResponse {
                        error: "no route found".to_string(),
                    }),
                )
            })?;

            let geometry: Vec<[f64; 2]> = path
                .iter()
                .filter_map(|nid| ch.graph.node_coord(*nid))
                .map(|c| [c.lat, c.lon])
                .collect();

            Ok(Json(RouteResponse {
                distance_m: cost * 50.0 / 3.6, // approximate from travel time
                duration_s: cost,
                geometry,
                steps: Vec::new(), // CH doesn't produce detailed steps
            }))
        }
        _ => {
            let route: Route = match algo {
                "dijkstra" => dijkstra(&state.graph, source, target, &profile),
                _ => astar(&state.graph, source, target, &profile),
            }
            .map_err(|e| {
                (
                    StatusCode::NOT_FOUND,
                    Json(ErrorResponse {
                        error: e.to_string(),
                    }),
                )
            })?;

            Ok(Json(RouteResponse {
                distance_m: route.distance_m,
                duration_s: route.duration_s,
                geometry: route.geometry.iter().map(|c| [c.lat, c.lon]).collect(),
                steps: route
                    .steps
                    .iter()
                    .map(|s| StepResponse {
                        distance_m: s.distance_m,
                        duration_s: s.duration_s,
                        name: s.name.clone(),
                        maneuver: format!("{:?}", s.maneuver),
                    })
                    .collect(),
            }))
        }
    }
}

async fn nearest_handler(
    State(state): State<AppState>,
    Query(params): Query<NearestQuery>,
) -> Result<Json<NearestResponse>, (StatusCode, Json<ErrorResponse>)> {
    let coord = Coord::new(params.lat, params.lon);
    let node_id = state
        .graph
        .nearest_node(coord)
        .ok_or_else(|| bad_request("graph is empty".to_string()))?;

    let node_coord = state.graph.node_coord(node_id).unwrap();
    let distance = coord.distance_to(node_coord);

    Ok(Json(NearestResponse {
        node_id: node_id.0,
        lat: node_coord.lat,
        lon: node_coord.lon,
        distance_m: distance,
    }))
}

async fn isochrone_handler(
    State(state): State<AppState>,
    Query(params): Query<IsochroneQuery>,
) -> Result<Json<IsochroneResponse>, (StatusCode, Json<ErrorResponse>)> {
    let coord = Coord::new(params.lat, params.lon);
    let profile = resolve_profile(params.profile.as_deref(), &state.profile)?;

    let source = state
        .graph
        .nearest_node(coord)
        .ok_or_else(|| bad_request("graph is empty".to_string()))?;

    let result = isochrone(&state.graph, source, params.max_seconds, &profile);

    Ok(Json(IsochroneResponse {
        reachable_nodes: result.nodes.len(),
        boundary: result.boundary.iter().map(|c| [c.lat, c.lon]).collect(),
    }))
}

// === Helpers ===

fn parse_coord(s: &str) -> Result<Coord, String> {
    let parts: Vec<&str> = s.split(',').collect();
    if parts.len() != 2 {
        return Err(format!(
            "invalid coordinate format: '{s}', expected 'lat,lon'"
        ));
    }
    let lat = parts[0]
        .trim()
        .parse::<f64>()
        .map_err(|e| format!("invalid latitude: {e}"))?;
    let lon = parts[1]
        .trim()
        .parse::<f64>()
        .map_err(|e| format!("invalid longitude: {e}"))?;
    Ok(Coord::new(lat, lon))
}

fn resolve_profile(
    name: Option<&str>,
    default: &SpeedProfile,
) -> Result<SpeedProfile, (StatusCode, Json<ErrorResponse>)> {
    match name {
        Some(name) => SpeedProfile::from_name(name).ok_or_else(|| {
            bad_request(format!(
                "unknown profile '{name}'; valid options: car, bicycle, pedestrian, truck"
            ))
        }),
        None => Ok(default.clone()),
    }
}

fn bad_request(msg: String) -> (StatusCode, Json<ErrorResponse>) {
    (StatusCode::BAD_REQUEST, Json(ErrorResponse { error: msg }))
}
