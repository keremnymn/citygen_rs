use std::f64::consts::PI;

use super::{config::EPSILON, segment::Road};
use geo::Point;
use rand::Rng;

pub struct DotProduct {
    pub dot_product: f64,
    pub projected: Point<f64>,
}

pub struct AddedPoint {
    pub distance2: f64,
    pub point_on_line: Point<f64>,
    pub line_proj_2: f64,
    pub length2: f64,
}

pub fn random_range(min: f64, max: f64) -> f64 {
    let mut rng = rand::thread_rng();
    rng.gen_range(min..max)
}

pub fn calculate_direction(r: &Road) -> f64 {
    let vector = subtract_points(r.end, r.start);
    let _cross_product = cross_product(Point::new(0.0, 1.0), vector);
    let _angle_between = angle_between(Point::new(0.0, 1.0), vector);
    -1.0 * sign(_cross_product) * _angle_between
}

pub fn length(point1: Point<f64>, point2: Point<f64>) -> f64 {
    let v = subtract_points(point2, point1);
    length_v(v)
}

pub fn length2(point1: Point, point2: Point) -> f64 {
    let v = subtract_points(point2, point1);
    length_v2(v)
}

pub fn distance_to_line(center: Point, start: Point, end: Point) -> AddedPoint {
    let ap = subtract_points(center, start);
    let ab = subtract_points(end, start);
    let result = project(ap, ab);

    let ad = result.projected;
    let d = add_points(start, ad);

    AddedPoint {
        distance2: length2(d, center),
        point_on_line: d,
        line_proj_2: sign(result.dot_product) * length_v2(ad),
        length2: length_v2(ab),
    }
}

pub fn project(v: Point<f64>, onto: Point<f64>) -> DotProduct {
    let dot_product = dot_product(v, onto);
    let projected = mult_v_scalar(onto, dot_product / length_v2(onto));

    DotProduct {
        dot_product,
        projected,
    }
}

pub fn subtract_points(point1: Point<f64>, point2: Point<f64>) -> Point<f64> {
    Point::new(point1.x() - point2.x(), point1.y() - point2.y())
}

pub fn length_v2(v: Point<f64>) -> f64 {
    v.x() * v.x() + v.y() * v.y()
}

pub fn mult_v_scalar(v: Point<f64>, n: f64) -> Point<f64> {
    Point::new(v.x() * n, v.y() * n)
}

pub fn length_v(v: Point<f64>) -> f64 {
    length_v2(v).sqrt()
}

pub fn add_points(point1: Point<f64>, point2: Point<f64>) -> Point<f64> {
    Point::new(point1.x() + point2.x(), point1.y() + point2.y())
}

pub fn dot_product(point1: Point<f64>, point2: Point<f64>) -> f64 {
    point1.x() * point2.x() + point1.y() * point2.y()
}

pub fn sign(x: f64) -> f64 {
    if x > 0.0 {
        1.0
    } else if x < 0.0 {
        -1.0
    } else {
        0.0
    }
}

pub fn cross_product(point1: Point<f64>, point2: Point<f64>) -> f64 {
    point1.x() * point2.y() - point1.y() * point2.x()
}

pub fn angle_between(v1: Point<f64>, v2: Point<f64>) -> f64 {
    let angle_rad = f64::acos((v1.x() * v2.x() + v1.y() * v2.y()) / (length_v(v1) * length_v(v2)));
    angle_rad * 180.0 / PI
}

pub fn equal_v(v1: Point<f64>, v2: Point<f64>) -> bool {
    let diff = subtract_points(v1, v2);
    let _length2 = length_v2(diff);

    _length2 < EPSILON
}

pub struct IntersectionResult {
    pub x: f64,
    pub y: f64,
    pub t: f64,
}

pub fn do_line_segments_intersect(
    p: Point<f64>,
    p2: Point<f64>,
    q: Point<f64>,
    q2: Point<f64>,
    omit_ends: bool,
) -> Option<IntersectionResult> {
    let r = subtract_points(p2, p);
    let s = subtract_points(q2, q);

    let u_numerator = cross_product(subtract_points(q, p), r);
    let denominator = cross_product(r, s);

    if u_numerator == 0.0 && denominator == 0.0 {
        return None;
        // colinear, so do they overlap?
        // return ((q.x - p.x < 0) != (q.x - p2.x < 0) != (q2.x - p.x < 0) != (q2.x - p2.x < 0)) ||
        //   ((q.y - p.y < 0) != (q.y - p2.y < 0) != (q2.y - p.y < 0) != (q2.y - p2.y < 0));
    }

    if denominator == 0.0 {
        // lines are parallel
        return None;
    }

    let u = u_numerator / denominator;
    let t = cross_product(subtract_points(q, p), s) / denominator;

    let do_segments_intersect = if !omit_ends {
        (t >= 0.0) && (t <= 1.0) && (u >= 0.0) && (u <= 1.0)
    } else {
        (t > 0.001) && (t < 1.0 - 0.001) && (u > 0.001) && (u < 1.0 - 0.001)
    };

    if do_segments_intersect {
        return Some(IntersectionResult {
            x: p.x() + t * r.x(),
            y: p.y() + t * r.y(),
            t,
        });
    }

    None
}
