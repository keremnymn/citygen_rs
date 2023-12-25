use geo::Point;

use super::{
    math::*,
    segment::Road,
    utility::{extended_max, extended_min},
};

#[derive(Debug, Clone)]
pub struct CollisionObject {
    road: Road,
    collision_type: CollisionType,
    collision_properties: CollisionProperties,
    collision_revision: u32,
    limits_revision: Option<u32>,
    pub cached_limits: Option<CollisionLimits>,
    pub id: Option<usize>,
}

#[derive(Debug, Clone)]
pub enum CollisionType {
    Rect,
    Line,
    Circle,
}

#[derive(Debug, Clone)]
pub struct CollisionProperties {
    pub corners: Vec<Point>,
    pub start: Point,
    pub end: Point,
    pub center: Point,
    pub radius: f64,
    pub width: f64,
}

#[derive(Debug, Copy, Clone)]
pub struct CollisionLimits {
    pub x: f64,
    pub y: f64,
    pub width: f64,
    pub height: f64,
    pub id: Option<usize>,
}

impl CollisionObject {
    pub fn new(
        road: Road,
        collision_type: CollisionType,
        collision_properties: CollisionProperties,
        id: usize,
    ) -> Self {
        CollisionObject {
            road,
            collision_type,
            collision_properties,
            collision_revision: 1,
            limits_revision: None,
            cached_limits: Some(CollisionLimits {
                x: 0.0,
                y: 0.0,
                width: 0.0,
                height: 0.0,
                id: Some(id),
            }),
            id: Some(id),
        }
    }

    pub fn update_collision_properties(&mut self, start: Option<Point>, end: Option<Point>) {
        self.collision_revision += 1;
        if let Some(start) = start {
            self.collision_properties.start = start
        }

        if let Some(end) = end {
            self.collision_properties.end = end
        }
    }

    pub fn limits(&mut self) -> CollisionLimits {
        if self.collision_revision != self.limits_revision.unwrap_or_default() {
            self.limits_revision = Some(self.collision_revision);

            self.cached_limits = match self.collision_type {
                CollisionType::Rect => {
                    let min_x = self
                        .collision_properties
                        .corners
                        .iter()
                        .map(|&p| p.x())
                        .fold(f64::INFINITY, f64::min);
                    let min_y = self
                        .collision_properties
                        .corners
                        .iter()
                        .map(|&p| p.y())
                        .fold(f64::INFINITY, f64::min);
                    Some(CollisionLimits {
                        x: min_x,
                        y: min_y,
                        width: self
                            .collision_properties
                            .corners
                            .iter()
                            .map(|&p| p.x())
                            .fold(f64::NEG_INFINITY, f64::max)
                            - min_x,
                        height: self
                            .collision_properties
                            .corners
                            .iter()
                            .map(|&p| p.y())
                            .fold(f64::NEG_INFINITY, f64::max)
                            - min_y,
                        id: self.id,
                    })
                }
                CollisionType::Line => Some(CollisionLimits {
                    x: f64::min(
                        self.collision_properties.start.x(),
                        self.collision_properties.end.x(),
                    ),
                    y: f64::min(
                        self.collision_properties.start.y(),
                        self.collision_properties.end.y(),
                    ),
                    width: f64::abs(
                        self.collision_properties.start.x() - self.collision_properties.end.x(),
                    ),
                    height: f64::abs(
                        self.collision_properties.start.y() - self.collision_properties.end.y(),
                    ),
                    id: self.id,
                }),
                CollisionType::Circle => Some(CollisionLimits {
                    x: self.collision_properties.center.x() - self.collision_properties.radius,
                    y: self.collision_properties.center.y() - self.collision_properties.radius,
                    width: self.collision_properties.radius * 2.0,
                    height: self.collision_properties.radius * 2.0,
                    id: self.id,
                }),
            };
        }

        self.cached_limits.unwrap()
    }

    fn collide(&mut self, other: &mut CollisionObject) -> bool {
        // Implementation of collide function
        let obj_limits = self.limits();
        let other_limits = other.limits();

        if obj_limits.x + obj_limits.width < other_limits.x
            || other_limits.x + other_limits.width < obj_limits.x
            || obj_limits.y + obj_limits.height < other_limits.y
            || other_limits.y + other_limits.height < obj_limits.y
        {
            return false;
        }

        match self.collision_type {
            CollisionType::Circle => match other.collision_type {
                CollisionType::Rect => self
                    .rect_circle_collision(&other.collision_properties, &self.collision_properties),
                _ => false,
            },
            CollisionType::Rect => match other.collision_type {
                CollisionType::Rect => self
                    .rect_rect_intersection(&self.collision_properties, &other.collision_properties)
                    .is_some(),
                CollisionType::Line => self
                    .rect_rect_intersection(
                        &self.collision_properties,
                        &self.rect_props_from_line(&other.collision_properties),
                    )
                    .is_some(),
                CollisionType::Circle => self
                    .rect_circle_collision(&self.collision_properties, &other.collision_properties),
            },
            CollisionType::Line => match other.collision_type {
                CollisionType::Rect => self
                    .rect_rect_intersection(
                        &self.rect_props_from_line(&self.collision_properties),
                        &other.collision_properties,
                    )
                    .is_some(),
                CollisionType::Line => self
                    .rect_rect_intersection(
                        &self.rect_props_from_line(&self.collision_properties),
                        &self.rect_props_from_line(&other.collision_properties),
                    )
                    .is_some(),
                _ => false,
            },
        }
    }

    fn rect_circle_collision(
        &self,
        rect_props: &CollisionProperties,
        circle_props: &CollisionProperties,
    ) -> bool {
        let corners: &Vec<Point> = &rect_props.corners;

        // Check for corner intersections with circle
        for i in 0..corners.len() {
            if length2(corners[i], circle_props.center) <= circle_props.radius * circle_props.radius
            {
                return true;
            }
        }

        // Check for edge intersections with circle
        // from http://stackoverflow.com/a/1079478
        for i in 0..corners.len() {
            let start = corners[i];
            let end = corners[(i + 1) % corners.len()];
            let added_point: AddedPoint = distance_to_line(circle_props.center, start, end);
            if added_point.line_proj_2 > 0.0
                && added_point.line_proj_2 < added_point.length2
                && added_point.distance2 <= circle_props.radius * circle_props.radius
            {
                return true;
            }
        }

        let axes = [
            subtract_points(corners[3], corners[0]),
            subtract_points(corners[3], corners[2]),
        ];

        let projections = [
            project(subtract_points(circle_props.center, corners[0]), axes[0]),
            project(subtract_points(circle_props.center, corners[2]), axes[1]),
        ];

        if projections[0].dot_product < 0.0
            || length_v2(projections[0].projected) > length_v2(axes[0])
            || projections[1].dot_product < 0.0
            || length_v2(projections[1].projected) > length_v2(axes[1])
        {
            return false;
        }

        true
    }

    fn rect_props_from_line(&self, line_props: &CollisionProperties) -> CollisionProperties {
        let dir = subtract_points(line_props.end, line_props.start);
        let perp_dir = Point::new(-dir.y(), dir.x());
        let half_width_perp_dir =
            mult_v_scalar(perp_dir, 0.5 * line_props.width / length_v(perp_dir));

        CollisionProperties {
            corners: vec![
                add_points(line_props.start, half_width_perp_dir),
                subtract_points(line_props.start, half_width_perp_dir),
                subtract_points(line_props.end, half_width_perp_dir),
                add_points(line_props.end, half_width_perp_dir),
            ],
            start: Point::new(0.0, 0.0), // Placeholder, update accordingly
            end: Point::new(0.0, 0.0),   // Placeholder, update accordingly
            center: Point::new(0.0, 0.0), // Placeholder, update accordingly
            radius: 0.0,
            width: 0.0, // Placeholder, update accordingly
        }
    }

    fn rect_rect_intersection(
        &self,
        rect_a_props: &CollisionProperties,
        rect_b_props: &CollisionProperties,
    ) -> Option<Point<f64>> {
        let c_a = &rect_a_props.corners;
        let c_b = &rect_b_props.corners;

        // Generate axes
        let axes = [
            subtract_points(c_a[3], c_a[0]),
            subtract_points(c_a[3], c_a[2]),
            subtract_points(c_b[0], c_b[1]),
            subtract_points(c_b[0], c_b[3]),
        ];

        // List used to find axis with the minimum overlap
        // that axis is used as the response translation vector
        let mut axis_overlaps = Vec::new();

        for axis in axes.iter() {
            // Project rectangle points to axis
            let mut projected_vectors_a = Vec::new();
            let mut projected_vectors_b = Vec::new();

            for corner in c_a.iter() {
                projected_vectors_a.push(project(corner.clone(), *axis).projected);
            }

            for corner in c_b.iter() {
                projected_vectors_b.push(project(corner.clone(), *axis).projected);
            }

            // Calculate relative positions of rectangles on axis
            let positions_on_axis_a: Vec<f64> = projected_vectors_a
                .iter()
                .map(|v| dot_product(*v, *axis))
                .collect();

            let positions_on_axis_b: Vec<f64> = projected_vectors_b
                .iter()
                .map(|v| dot_product(*v, *axis))
                .collect();

            let (max_a, max_a_i) = extended_max(&positions_on_axis_a, |x| *x).unwrap();
            let (min_a, min_a_i) = extended_min(&positions_on_axis_a, |x| *x).unwrap();
            let (max_b, max_b_i) = extended_max(&positions_on_axis_b, |x| *x).unwrap();
            let (min_b, min_b_i) = extended_min(&positions_on_axis_b, |x| *x).unwrap();

            // If the rectangles don't overlap on at least one axis
            // they are not colliding
            if max_a < min_b || max_b < min_a {
                return None;
            } else {
                // Calculate the overlap between the rectangles on this axis
                let diff1 =
                    subtract_points(projected_vectors_a[max_a_i], projected_vectors_b[min_b_i]);
                let diff2 =
                    subtract_points(projected_vectors_b[max_b_i], projected_vectors_a[min_a_i]);

                if length_v2(diff1) < length_v2(diff2) {
                    axis_overlaps.push(diff1);
                } else {
                    // The rectangles overlap on the other side
                    // Invert the vector so that it will push out of the collision
                    axis_overlaps.push(mult_v_scalar(diff2, -1.0));
                }
            }
        }

        // Find axis with the minimum overlap
        let min_vector = axis_overlaps
            .iter()
            .min_by(|&v1, &v2| length_v2(*v1).partial_cmp(&length_v2(*v2)).unwrap());

        // Return displacement required to pull rectA from collision
        min_vector.map(|v| mult_v_scalar(*v, -1.0))
    }
}
