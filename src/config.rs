use crate::math::random_range;

pub const HIGHWAY_SEGMENT_WIDTH: f64 = 16.0;
pub const HIGHWAY_BRANCH_POPULATION_THRESHOLD: f64 = 0.1;
pub const HIGHWAY_BRANCH_PROBABILITY: f64 = 0.05;
pub const DEFAULT_SEGMENT_WIDTH: f64 = 6.0;
pub const DEFAULT_SEGMENT_LENGTH: f64 = 300.0;
pub const EPSILON: f64 = 0.00000001;
pub const MINIMUM_INTERSECTION_DEVIATION: f64 = 30.0;
pub const ROAD_SNAP_DISTANCE: f64 = 50.0;
pub const NORMAL_BRANCH_TIME_DELAY_FROM_HIGHWAY: f64 = 5.0;
pub const NORMAL_BRANCH_POPULATION_THRESHOLD: f64 = 0.1;
pub const DEFAULT_BRANCH_PROBABILITY: f64 = 0.4;
pub const HIGHWAY_SEGMENT_LENGTH: f64 = 400.0;
pub const SEGMENT_COUNT_LIMIT: f64 = 200.0;

const BRANCH_ANGLE_DEV: f64 = 3.0;
const FORWARD_ANGLE_DEV: f64 = 15.0;

pub enum AngleDirection {
    Branch,
    Forward,
}

pub fn random_angle(direction: AngleDirection) -> f64 {
    let limit = match direction {
        AngleDirection::Branch => BRANCH_ANGLE_DEV,
        AngleDirection::Forward => FORWARD_ANGLE_DEV,
    };

    // non-linear distribution
    let non_uniform_norm = f64::powf(limit.abs(), 3.0);
    let mut val: f64 = 0.0;

    while val == 0.0 || rand::random::<f64>() < f64::powf(val.abs(), 3.0) / non_uniform_norm {
        val = random_range(-limit, limit);
    }

    val
}
