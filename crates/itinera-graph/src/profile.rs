use serde::{Deserialize, Serialize};

/// Travel mode for routing profiles.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum TravelMode {
    Car,
    Bicycle,
    Pedestrian,
    Truck,
}

/// Speed profile mapping road classes to speeds (km/h).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SpeedProfile {
    pub mode: TravelMode,
    /// Speeds indexed by road_class (1-based, index 0 unused).
    pub speeds_kmh: [f64; 8],
}

impl SpeedProfile {
    /// Default car speed profile.
    #[must_use]
    pub fn car() -> Self {
        Self {
            mode: TravelMode::Car,
            speeds_kmh: [
                0.0,   // unused index 0
                130.0, // motorway
                100.0, // trunk
                80.0,  // primary
                60.0,  // secondary
                50.0,  // tertiary
                40.0,  // unclassified
                30.0,  // residential
            ],
        }
    }

    /// Default bicycle speed profile.
    #[must_use]
    pub fn bicycle() -> Self {
        Self {
            mode: TravelMode::Bicycle,
            speeds_kmh: [
                0.0,  // unused
                0.0,  // motorway (not allowed)
                25.0, // trunk (with bike lane)
                22.0, // primary
                20.0, // secondary
                18.0, // tertiary
                16.0, // unclassified
                15.0, // residential
            ],
        }
    }

    /// Default pedestrian speed profile.
    #[must_use]
    pub fn pedestrian() -> Self {
        Self {
            mode: TravelMode::Pedestrian,
            speeds_kmh: [
                0.0, // unused
                0.0, // motorway (not allowed)
                5.0, // trunk
                5.0, // primary
                5.0, // secondary
                5.0, // tertiary
                5.0, // unclassified
                5.0, // residential
            ],
        }
    }

    /// Default truck speed profile.
    #[must_use]
    pub fn truck() -> Self {
        Self {
            mode: TravelMode::Truck,
            speeds_kmh: [
                0.0,  // unused
                90.0, // motorway
                80.0, // trunk
                60.0, // primary
                50.0, // secondary
                40.0, // tertiary
                30.0, // unclassified
                20.0, // residential
            ],
        }
    }

    /// Get speed for a given road class. Returns 0 if road is not accessible.
    #[must_use]
    pub fn speed_for_class(&self, road_class: u8) -> f64 {
        self.speeds_kmh
            .get(road_class as usize)
            .copied()
            .unwrap_or(0.0)
    }

    /// Create a profile from a string identifier.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Self> {
        match name {
            "car" => Some(Self::car()),
            "bicycle" | "bike" => Some(Self::bicycle()),
            "pedestrian" | "foot" | "walk" => Some(Self::pedestrian()),
            "truck" | "hgv" => Some(Self::truck()),
            _ => None,
        }
    }
}
