use std::collections::HashMap;
use std::path::Path;

use osmpbf::{Element, ElementReader};

use itinera_graph::{Coord, Edge, Graph, Node, NodeId, TurnRestriction};

use crate::error::OsmError;
use crate::parser::ImportStats;
use crate::tags::{highway_to_road_class, is_oneway};

/// Parse an OSM PBF file and build a routing graph.
pub fn parse_pbf(path: &Path) -> Result<(Graph, ImportStats), OsmError> {
    let reader = ElementReader::from_path(path)
        .map_err(|e| OsmError::Io(std::io::Error::other(e.to_string())))?;

    let mut osm_nodes: HashMap<i64, Coord> = HashMap::new();
    let mut ways: Vec<PbfWay> = Vec::new();
    let mut restrictions: Vec<PbfRestriction> = Vec::new();

    // First pass: collect all nodes, ways, and relations
    reader
        .for_each(|element| match element {
            Element::Node(node) => {
                osm_nodes.insert(node.id(), Coord::new(node.lat(), node.lon()));
            }
            Element::DenseNode(node) => {
                osm_nodes.insert(node.id(), Coord::new(node.lat(), node.lon()));
            }
            Element::Way(way) => {
                let tags: Vec<(String, String)> = way
                    .tags()
                    .map(|(k, v)| (k.to_string(), v.to_string()))
                    .collect();

                let highway = tags
                    .iter()
                    .find(|(k, _)| k == "highway")
                    .map(|(_, v)| v.clone())
                    .unwrap_or_default();

                if !highway.is_empty() && highway_to_road_class(&highway) > 0 {
                    ways.push(PbfWay {
                        way_id: way.id(),
                        node_refs: way.refs().collect(),
                        tags,
                        highway,
                    });
                }
            }
            Element::Relation(rel) => {
                let tags: Vec<(String, String)> = rel
                    .tags()
                    .map(|(k, v)| (k.to_string(), v.to_string()))
                    .collect();

                let is_restriction = tags.iter().any(|(k, v)| k == "type" && v == "restriction");

                if is_restriction {
                    let restriction_type = tags
                        .iter()
                        .find(|(k, _)| k == "restriction")
                        .map(|(_, v)| v.clone())
                        .unwrap_or_default();

                    let mut from_way = 0i64;
                    let mut to_way = 0i64;
                    let mut via_node = 0i64;

                    for member in rel.members() {
                        let role = member.role().unwrap_or("");
                        let mtype = member.member_type;
                        match (mtype, role) {
                            (osmpbf::RelMemberType::Way, "from") => from_way = member.member_id,
                            (osmpbf::RelMemberType::Way, "to") => to_way = member.member_id,
                            (osmpbf::RelMemberType::Node, "via") => via_node = member.member_id,
                            _ => {}
                        }
                    }

                    if from_way != 0 && to_way != 0 && via_node != 0 {
                        restrictions.push(PbfRestriction {
                            from_way,
                            to_way,
                            via_node,
                            restriction_type,
                        });
                    }
                }
            }
        })
        .map_err(|e| OsmError::Io(std::io::Error::other(e.to_string())))?;

    let mut stats = ImportStats {
        nodes_parsed: osm_nodes.len(),
        ways_parsed: ways.len(),
        ..Default::default()
    };

    // Determine graph nodes (intersections + endpoints)
    let mut node_usage: HashMap<i64, u32> = HashMap::new();
    for way in &ways {
        for (i, &nref) in way.node_refs.iter().enumerate() {
            let count = node_usage.entry(nref).or_insert(0);
            if i == 0 || i == way.node_refs.len() - 1 {
                *count += 2;
            } else {
                *count += 1;
            }
        }
    }

    let graph_node_osm_ids: Vec<i64> = node_usage
        .iter()
        .filter(|&(_, &count)| count >= 2)
        .map(|(&id, _)| id)
        .collect();

    let mut osm_to_internal: HashMap<i64, u32> = HashMap::new();
    let mut nodes = Vec::new();

    for (idx, &osm_id) in graph_node_osm_ids.iter().enumerate() {
        if let Some(&coord) = osm_nodes.get(&osm_id) {
            osm_to_internal.insert(osm_id, idx as u32);
            nodes.push(Node {
                id: NodeId(idx as u32),
                coord,
                osm_id,
                ch_level: 0,
            });
        }
    }

    stats.nodes_in_graph = nodes.len();

    // Build edges
    let mut edges = Vec::new();

    for way in &ways {
        let road_class = highway_to_road_class(&way.highway);
        if road_class == 0 {
            continue;
        }

        let oneway = is_oneway(&way.tags, &way.highway);
        let name: Option<String> = way
            .tags
            .iter()
            .find(|(k, _)| k == "name")
            .map(|(_, v)| v.clone());

        let mut segment_start = 0;
        for i in 1..way.node_refs.len() {
            let nref = way.node_refs[i];
            let is_graph_node = osm_to_internal.contains_key(&nref);
            let is_last = i == way.node_refs.len() - 1;

            if is_graph_node || is_last {
                let from_osm = way.node_refs[segment_start];
                let to_osm = nref;

                if let (Some(&from_id), Some(&to_id)) =
                    (osm_to_internal.get(&from_osm), osm_to_internal.get(&to_osm))
                {
                    let mut distance = 0.0;
                    let mut geometry = Vec::new();
                    for j in segment_start..i {
                        let a_osm = way.node_refs[j];
                        let b_osm = way.node_refs[j + 1];
                        if let (Some(&a_coord), Some(&b_coord)) =
                            (osm_nodes.get(&a_osm), osm_nodes.get(&b_osm))
                        {
                            distance += a_coord.distance_to(b_coord);
                            if j > segment_start {
                                geometry.push(a_coord);
                            }
                        }
                    }

                    edges.push(Edge {
                        from: NodeId(from_id),
                        to: NodeId(to_id),
                        distance_m: distance,
                        duration_s: 0.0,
                        way_id: way.way_id,
                        road_class,
                        oneway,
                        name: name.clone(),
                        geometry: geometry.clone(),
                    });

                    if !oneway {
                        let mut rev_geom = geometry;
                        rev_geom.reverse();
                        edges.push(Edge {
                            from: NodeId(to_id),
                            to: NodeId(from_id),
                            distance_m: distance,
                            duration_s: 0.0,
                            way_id: way.way_id,
                            road_class,
                            oneway: false,
                            name: name.clone(),
                            geometry: rev_geom,
                        });
                    }
                }

                segment_start = i;
            }
        }
    }

    stats.edges_created = edges.len();
    let mut graph = Graph::build(nodes, edges);

    // Add turn restrictions
    for restriction in &restrictions {
        if let Some(&via_internal) = osm_to_internal.get(&restriction.via_node) {
            let restriction_type = if restriction.restriction_type.starts_with("no_") {
                itinera_graph::turn::RestrictionType::No
            } else if restriction.restriction_type.starts_with("only_") {
                itinera_graph::turn::RestrictionType::Only
            } else {
                continue;
            };

            graph.restrictions.push(TurnRestriction {
                via_node: NodeId(via_internal),
                from_way: restriction.from_way,
                to_way: restriction.to_way,
                restriction_type,
            });
        }
    }

    Ok((graph, stats))
}

struct PbfWay {
    way_id: i64,
    node_refs: Vec<i64>,
    tags: Vec<(String, String)>,
    highway: String,
}

struct PbfRestriction {
    from_way: i64,
    to_way: i64,
    via_node: i64,
    restriction_type: String,
}
