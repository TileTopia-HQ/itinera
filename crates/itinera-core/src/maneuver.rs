use itinera_graph::{Graph, NodeId};

use crate::route::StepManeuver;

/// Determine the maneuver type based on the bearing change between two edges.
pub fn detect_maneuver(
    graph: &Graph,
    prev_node: NodeId,
    via_node: NodeId,
    next_node: NodeId,
) -> StepManeuver {
    let prev_coord = match graph.node_coord(prev_node) {
        Some(c) => c,
        None => return StepManeuver::Continue,
    };
    let via_coord = match graph.node_coord(via_node) {
        Some(c) => c,
        None => return StepManeuver::Continue,
    };
    let next_coord = match graph.node_coord(next_node) {
        Some(c) => c,
        None => return StepManeuver::Continue,
    };

    let bearing_in = prev_coord.bearing_to(via_coord);
    let bearing_out = via_coord.bearing_to(next_coord);

    let turn_angle = normalize_angle(bearing_out - bearing_in);

    angle_to_maneuver(turn_angle)
}

/// Normalize an angle to [-180, 180).
fn normalize_angle(angle: f64) -> f64 {
    let mut a = angle % 360.0;
    if a > 180.0 {
        a -= 360.0;
    }
    if a <= -180.0 {
        a += 360.0;
    }
    a
}

/// Convert a turn angle to a maneuver type.
/// Positive = right turn, negative = left turn.
fn angle_to_maneuver(angle: f64) -> StepManeuver {
    let abs_angle = angle.abs();

    if abs_angle < 10.0 {
        StepManeuver::Continue
    } else if abs_angle > 170.0 {
        StepManeuver::UTurn
    } else if angle > 0.0 {
        // Right turn
        if abs_angle < 45.0 {
            StepManeuver::TurnSlightRight
        } else if abs_angle < 135.0 {
            StepManeuver::TurnRight
        } else {
            StepManeuver::TurnSharpRight
        }
    } else {
        // Left turn
        if abs_angle < 45.0 {
            StepManeuver::TurnSlightLeft
        } else if abs_angle < 135.0 {
            StepManeuver::TurnLeft
        } else {
            StepManeuver::TurnSharpLeft
        }
    }
}

/// Build step maneuvers for a path, replacing generic Continue maneuvers
/// with directional maneuvers based on bearing changes.
pub fn annotate_maneuvers(graph: &Graph, path: &[u32]) -> Vec<StepManeuver> {
    let mut maneuvers = Vec::with_capacity(path.len());

    for (i, &node_id) in path.iter().enumerate() {
        if i == 0 {
            maneuvers.push(StepManeuver::Depart);
        } else if i == path.len() - 1 {
            maneuvers.push(StepManeuver::Arrive);
        } else {
            let prev = NodeId(path[i - 1]);
            let via = NodeId(node_id);
            let next = NodeId(path[i + 1]);
            maneuvers.push(detect_maneuver(graph, prev, via, next));
        }
    }

    maneuvers
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_normalize_angle() {
        assert!((normalize_angle(0.0)).abs() < f64::EPSILON);
        assert!((normalize_angle(360.0)).abs() < f64::EPSILON);
        assert!((normalize_angle(-180.0) - 180.0).abs() < f64::EPSILON);
        assert!((normalize_angle(270.0) - (-90.0)).abs() < f64::EPSILON);
    }

    #[test]
    fn test_angle_to_maneuver() {
        assert!(matches!(angle_to_maneuver(0.0), StepManeuver::Continue));
        assert!(matches!(angle_to_maneuver(5.0), StepManeuver::Continue));
        assert!(matches!(
            angle_to_maneuver(30.0),
            StepManeuver::TurnSlightRight
        ));
        assert!(matches!(angle_to_maneuver(90.0), StepManeuver::TurnRight));
        assert!(matches!(
            angle_to_maneuver(150.0),
            StepManeuver::TurnSharpRight
        ));
        assert!(matches!(angle_to_maneuver(175.0), StepManeuver::UTurn));
        assert!(matches!(
            angle_to_maneuver(-30.0),
            StepManeuver::TurnSlightLeft
        ));
        assert!(matches!(angle_to_maneuver(-90.0), StepManeuver::TurnLeft));
        assert!(matches!(
            angle_to_maneuver(-150.0),
            StepManeuver::TurnSharpLeft
        ));
    }
}
