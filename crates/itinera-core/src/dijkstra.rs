use std::cmp::Ordering;
use std::collections::BinaryHeap;

use itinera_graph::{Graph, NodeId, SpeedProfile};

use crate::error::RoutingError;
use crate::maneuver::annotate_maneuvers;
use crate::route::{Route, RouteStep, StepManeuver};

/// State in the priority queue.
#[derive(Debug, Clone)]
struct State {
    cost: f64,
    node: NodeId,
}

impl PartialEq for State {
    fn eq(&self, other: &Self) -> bool {
        self.cost == other.cost
    }
}

impl Eq for State {}

impl PartialOrd for State {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for State {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reverse ordering for min-heap
        other
            .cost
            .partial_cmp(&self.cost)
            .unwrap_or(Ordering::Equal)
    }
}

/// Dijkstra's shortest path algorithm.
///
/// Returns a `Route` from `source` to `target` using the given speed profile
/// to compute edge weights (travel time).
pub fn dijkstra(
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

    let mut dist = vec![f64::INFINITY; n];
    let mut prev: Vec<Option<u32>> = vec![None; n];
    let mut visited = vec![false; n];

    dist[src_idx] = 0.0;

    let mut heap = BinaryHeap::new();
    heap.push(State {
        cost: 0.0,
        node: source,
    });

    while let Some(State { cost, node }) = heap.pop() {
        let node_idx = node.0 as usize;

        if node == target {
            break;
        }

        if visited[node_idx] {
            continue;
        }
        visited[node_idx] = true;

        if cost > dist[node_idx] {
            continue;
        }

        for edge in graph.outgoing_edges(node) {
            let weight = graph.edge_weight(edge, profile);
            if weight == f64::INFINITY {
                continue;
            }

            let next = edge.to;
            let next_idx = next.0 as usize;
            let new_cost = cost + weight;

            if new_cost < dist[next_idx] {
                dist[next_idx] = new_cost;
                prev[next_idx] = Some(node.0);
                heap.push(State {
                    cost: new_cost,
                    node: next,
                });
            }
        }
    }

    if dist[tgt_idx] == f64::INFINITY {
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

    // Override last step maneuver to Arrive
    if let Some(last) = steps.last_mut() {
        last.maneuver = StepManeuver::Arrive;
    }

    Ok(Route {
        distance_m: total_distance,
        duration_s: dist[tgt_idx],
        node_ids: path,
        geometry,
        steps,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use itinera_graph::{Coord, Edge, Node};

    fn test_graph() -> Graph {
        // Simple 4-node graph:
        // 0 --10s--> 1 --10s--> 3
        //  \                    /
        //   ---25s--> 2 --5s--/
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
    fn test_dijkstra_shortest_path() {
        let g = test_graph();
        let profile = SpeedProfile::car();
        let route = dijkstra(&g, NodeId(0), NodeId(3), &profile).unwrap();
        // Shortest: 0 -> 1 -> 3 with total weight via profile
        // Edge weights: 500m at 50km/h = 36s each
        // vs 0->2->3: 1250*3.6/50 + 250*3.6/50 = 90+18 = 108s
        // 0->1->3: 36+36 = 72s
        assert_eq!(route.node_ids, vec![0, 1, 3]);
    }

    #[test]
    fn test_dijkstra_no_route() {
        let g = test_graph();
        let profile = SpeedProfile::car();
        // Node 3 has no outgoing edges, so 3->0 should fail
        let result = dijkstra(&g, NodeId(3), NodeId(0), &profile);
        assert!(result.is_err());
    }

    #[test]
    fn test_dijkstra_same_node() {
        let g = test_graph();
        let profile = SpeedProfile::car();
        let route = dijkstra(&g, NodeId(0), NodeId(0), &profile).unwrap();
        assert_eq!(route.distance_m, 0.0);
        assert_eq!(route.node_ids, vec![0]);
    }
}
