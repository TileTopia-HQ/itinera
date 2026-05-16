use std::cmp::Ordering;
use std::collections::BinaryHeap;

use itinera_graph::{Coord, Graph, NodeId, SpeedProfile};

/// Isochrone result — set of reachable nodes within a time budget.
#[derive(Debug, Clone)]
pub struct IsochroneResult {
    /// Nodes reachable within the time budget, with their travel time.
    pub nodes: Vec<(NodeId, f64)>,
    /// Convex hull or alpha-shape boundary of the isochrone (coords).
    pub boundary: Vec<Coord>,
}

#[derive(Debug, Clone)]
struct IsoState {
    cost: f64,
    node: NodeId,
}

impl PartialEq for IsoState {
    fn eq(&self, other: &Self) -> bool {
        self.cost == other.cost
    }
}

impl Eq for IsoState {}

impl PartialOrd for IsoState {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for IsoState {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .cost
            .partial_cmp(&self.cost)
            .unwrap_or(Ordering::Equal)
    }
}

/// Compute an isochrone: all nodes reachable from `source` within `max_seconds`.
///
/// Returns reachable nodes with their travel times, plus a convex hull boundary.
pub fn isochrone(
    graph: &Graph,
    source: NodeId,
    max_seconds: f64,
    profile: &SpeedProfile,
) -> IsochroneResult {
    let n = graph.num_nodes();
    let mut dist = vec![f64::INFINITY; n];
    let mut visited = vec![false; n];
    let mut reachable = Vec::new();

    let src_idx = source.0 as usize;
    if src_idx >= n {
        return IsochroneResult {
            nodes: Vec::new(),
            boundary: Vec::new(),
        };
    }

    dist[src_idx] = 0.0;
    let mut heap = BinaryHeap::new();
    heap.push(IsoState {
        cost: 0.0,
        node: source,
    });

    while let Some(IsoState { cost, node }) = heap.pop() {
        let node_idx = node.0 as usize;

        if visited[node_idx] {
            continue;
        }
        visited[node_idx] = true;

        if cost > max_seconds {
            break;
        }

        reachable.push((node, cost));

        for edge in graph.outgoing_edges(node) {
            let weight = graph.edge_weight(edge, profile);
            if weight == f64::INFINITY {
                continue;
            }

            let next = edge.to;
            let next_idx = next.0 as usize;
            let new_cost = cost + weight;

            if new_cost <= max_seconds && new_cost < dist[next_idx] {
                dist[next_idx] = new_cost;
                heap.push(IsoState {
                    cost: new_cost,
                    node: next,
                });
            }
        }
    }

    // Build convex hull boundary from reachable node coordinates
    let coords: Vec<Coord> = reachable
        .iter()
        .filter_map(|(nid, _)| graph.node_coord(*nid))
        .collect();

    let boundary = convex_hull(&coords);

    IsochroneResult {
        nodes: reachable,
        boundary,
    }
}

/// Simple convex hull using Gift Wrapping (Jarvis march).
fn convex_hull(points: &[Coord]) -> Vec<Coord> {
    if points.len() < 3 {
        return points.to_vec();
    }

    // Find leftmost point
    let mut start = 0;
    for (i, p) in points.iter().enumerate() {
        if p.lon < points[start].lon || (p.lon == points[start].lon && p.lat < points[start].lat) {
            start = i;
        }
    }

    let mut hull = Vec::new();
    let mut current = start;

    loop {
        hull.push(points[current]);
        let mut next = 0;

        for i in 1..points.len() {
            if next == current {
                next = i;
                continue;
            }
            let cross = cross_product(points[current], points[next], points[i]);
            if cross < 0.0
                || (cross == 0.0
                    && dist_sq(points[current], points[i]) > dist_sq(points[current], points[next]))
            {
                next = i;
            }
        }

        current = next;
        if current == start {
            break;
        }

        // Safety: prevent infinite loops
        if hull.len() > points.len() {
            break;
        }
    }

    hull
}

fn cross_product(o: Coord, a: Coord, b: Coord) -> f64 {
    (a.lon - o.lon) * (b.lat - o.lat) - (a.lat - o.lat) * (b.lon - o.lon)
}

fn dist_sq(a: Coord, b: Coord) -> f64 {
    (a.lat - b.lat).powi(2) + (a.lon - b.lon).powi(2)
}

#[cfg(test)]
mod tests {
    use super::*;
    use itinera_graph::{Edge, Node};

    fn grid_graph() -> Graph {
        // 3x3 grid:
        // 0-1-2
        // |   |
        // 3-4-5
        // |   |
        // 6-7-8
        let mut nodes = Vec::new();
        for i in 0..9 {
            let row = i / 3;
            let col = i % 3;
            nodes.push(Node {
                id: NodeId(i as u32),
                coord: Coord::new(row as f64 * 0.01, col as f64 * 0.01),
                osm_id: i as i64,
                ch_level: 0,
            });
        }

        let connections = [
            (0, 1),
            (1, 2),
            (0, 3),
            (2, 5),
            (3, 4),
            (4, 5),
            (3, 6),
            (5, 8),
            (6, 7),
            (7, 8),
            // Reverse directions
            (1, 0),
            (2, 1),
            (3, 0),
            (5, 2),
            (4, 3),
            (5, 4),
            (6, 3),
            (8, 5),
            (7, 6),
            (8, 7),
        ];

        let edges: Vec<Edge> = connections
            .iter()
            .enumerate()
            .map(|(i, &(from, to))| Edge {
                from: NodeId(from),
                to: NodeId(to),
                distance_m: 1000.0,
                duration_s: 60.0,
                way_id: i as i64,
                road_class: 5,
                oneway: true,
                name: None,
                geometry: vec![],
            })
            .collect();

        Graph::build(nodes, edges)
    }

    #[test]
    fn test_isochrone_limited_reach() {
        let g = grid_graph();
        let profile = SpeedProfile::car();
        // With 1000m edges at 50km/h class 5 -> each edge is 1000*3.6/50 = 72s
        // With budget of 80s, should reach immediate neighbors only
        let result = isochrone(&g, NodeId(4), 80.0, &profile);
        // Node 4 at cost 0, neighbors 3 and 5 at ~72s each
        assert!(result.nodes.len() >= 2);
        assert!(result.nodes.iter().any(|(n, _)| *n == NodeId(4)));
    }

    #[test]
    fn test_isochrone_full_reach() {
        let g = grid_graph();
        let profile = SpeedProfile::car();
        // Large budget should reach all nodes
        let result = isochrone(&g, NodeId(0), 10000.0, &profile);
        assert_eq!(result.nodes.len(), 9);
    }
}
