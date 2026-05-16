use std::cmp::Ordering;
use std::collections::BinaryHeap;

use itinera_graph::{Coord, Graph, NodeId, SpeedProfile};

use crate::error::RoutingError;
use crate::maneuver::annotate_maneuvers;
use crate::route::{Route, RouteStep, StepManeuver};

/// State with f-score for A*.
#[derive(Debug, Clone)]
struct AStarState {
    f_score: f64,
    g_score: f64,
    node: NodeId,
}

impl PartialEq for AStarState {
    fn eq(&self, other: &Self) -> bool {
        self.f_score == other.f_score
    }
}

impl Eq for AStarState {}

impl PartialOrd for AStarState {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for AStarState {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .f_score
            .partial_cmp(&self.f_score)
            .unwrap_or(Ordering::Equal)
    }
}

/// Heuristic: estimated travel time from coord to target using haversine distance
/// at a generous max speed (150 km/h) to ensure admissibility.
fn heuristic(from: Coord, to: Coord) -> f64 {
    let dist = from.distance_to(to);
    // time = distance / speed; 150 km/h = 41.67 m/s
    dist / 41.67
}

/// A* shortest path algorithm with haversine heuristic.
///
/// Typically 2-5x faster than Dijkstra for long-distance routes
/// due to directed search toward the target.
pub fn astar(
    graph: &Graph,
    source: NodeId,
    target: NodeId,
    profile: &SpeedProfile,
) -> Result<Route, RoutingError> {
    let n = graph.num_nodes();
    if n == 0 {
        return Err(RoutingError::EmptyGraph);
    }

    let src_idx = source.0 as usize;
    let tgt_idx = target.0 as usize;

    if src_idx >= n {
        return Err(RoutingError::NodeNotFound(source.0));
    }
    if tgt_idx >= n {
        return Err(RoutingError::NodeNotFound(target.0));
    }

    let target_coord = graph
        .node_coord(target)
        .ok_or(RoutingError::NodeNotFound(target.0))?;

    let mut g_scores = vec![f64::INFINITY; n];
    let mut prev: Vec<Option<u32>> = vec![None; n];
    let mut visited = vec![false; n];

    g_scores[src_idx] = 0.0;

    let source_coord = graph
        .node_coord(source)
        .ok_or(RoutingError::NodeNotFound(source.0))?;
    let initial_h = heuristic(source_coord, target_coord);

    let mut heap = BinaryHeap::new();
    heap.push(AStarState {
        f_score: initial_h,
        g_score: 0.0,
        node: source,
    });

    while let Some(AStarState { g_score, node, .. }) = heap.pop() {
        let node_idx = node.0 as usize;

        if node == target {
            break;
        }

        if visited[node_idx] {
            continue;
        }
        visited[node_idx] = true;

        if g_score > g_scores[node_idx] {
            continue;
        }

        for edge in graph.outgoing_edges(node) {
            let weight = graph.edge_weight(edge, profile);
            if weight == f64::INFINITY {
                continue;
            }

            let next = edge.to;
            let next_idx = next.0 as usize;
            let new_g = g_score + weight;

            if new_g < g_scores[next_idx] {
                g_scores[next_idx] = new_g;
                prev[next_idx] = Some(node.0);

                let next_coord = graph.node_coord(next).unwrap_or(target_coord);
                let h = heuristic(next_coord, target_coord);

                heap.push(AStarState {
                    f_score: new_g + h,
                    g_score: new_g,
                    node: next,
                });
            }
        }
    }

    if g_scores[tgt_idx] == f64::INFINITY {
        return Err(RoutingError::NoRoute {
            from: format!("{source:?}"),
            to: format!("{target:?}"),
        });
    }

    // Reconstruct path
    let mut path = Vec::new();
    let mut current = tgt_idx as u32;
    while current != source.0 {
        path.push(current);
        current = prev[current as usize].ok_or(RoutingError::NoRoute {
            from: format!("{source:?}"),
            to: format!("{target:?}"),
        })?;
    }
    path.push(source.0);
    path.reverse();

    // Build route
    let geometry: Vec<_> = path
        .iter()
        .filter_map(|&nid| graph.node_coord(NodeId(nid)))
        .collect();

    let maneuvers = annotate_maneuvers(graph, &path);
    let mut steps = Vec::new();
    let mut total_distance = 0.0;

    for (idx, window) in path.windows(2).enumerate() {
        let from = NodeId(window[0]);
        let to = NodeId(window[1]);

        if let Some(edge) = graph.outgoing_edges(from).iter().find(|e| e.to == to) {
            total_distance += edge.distance_m;
            let maneuver = maneuvers[idx].clone();
            steps.push(RouteStep {
                distance_m: edge.distance_m,
                duration_s: graph.edge_weight(edge, profile),
                name: edge.name.clone(),
                maneuver,
            });
        }
    }

    if let Some(last) = steps.last_mut() {
        last.maneuver = StepManeuver::Arrive;
    }

    Ok(Route {
        distance_m: total_distance,
        duration_s: g_scores[tgt_idx],
        node_ids: path,
        geometry,
        steps,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use itinera_graph::{Edge, Node};

    fn test_graph() -> Graph {
        let nodes = vec![
            Node {
                id: NodeId(0),
                coord: Coord::new(0.0, 0.0),
                osm_id: 1,
                ch_level: 0,
            },
            Node {
                id: NodeId(1),
                coord: Coord::new(0.0, 1.0),
                osm_id: 2,
                ch_level: 0,
            },
            Node {
                id: NodeId(2),
                coord: Coord::new(1.0, 0.0),
                osm_id: 3,
                ch_level: 0,
            },
            Node {
                id: NodeId(3),
                coord: Coord::new(1.0, 1.0),
                osm_id: 4,
                ch_level: 0,
            },
        ];

        let edges = vec![
            Edge {
                from: NodeId(0),
                to: NodeId(1),
                distance_m: 500.0,
                duration_s: 10.0,
                way_id: 1,
                road_class: 5,
                oneway: true,
                name: Some("A".into()),
                geometry: vec![],
            },
            Edge {
                from: NodeId(1),
                to: NodeId(3),
                distance_m: 500.0,
                duration_s: 10.0,
                way_id: 2,
                road_class: 5,
                oneway: true,
                name: Some("B".into()),
                geometry: vec![],
            },
            Edge {
                from: NodeId(0),
                to: NodeId(2),
                distance_m: 1250.0,
                duration_s: 25.0,
                way_id: 3,
                road_class: 5,
                oneway: true,
                name: Some("C".into()),
                geometry: vec![],
            },
            Edge {
                from: NodeId(2),
                to: NodeId(3),
                distance_m: 250.0,
                duration_s: 5.0,
                way_id: 4,
                road_class: 5,
                oneway: true,
                name: Some("D".into()),
                geometry: vec![],
            },
        ];

        Graph::build(nodes, edges)
    }

    #[test]
    fn test_astar_finds_same_path_as_dijkstra() {
        let g = test_graph();
        let profile = SpeedProfile::car();
        let route = astar(&g, NodeId(0), NodeId(3), &profile).unwrap();
        assert_eq!(route.node_ids, vec![0, 1, 3]);
    }

    #[test]
    fn test_astar_no_route() {
        let g = test_graph();
        let profile = SpeedProfile::car();
        let result = astar(&g, NodeId(3), NodeId(0), &profile);
        assert!(result.is_err());
    }
}
