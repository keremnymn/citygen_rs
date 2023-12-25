use std::f64::consts::PI;

use geo::Point;
// use rand::{rngs::StdRng, Rng, SeedableRng};

use super::{
    collision::{CollisionObject, CollisionProperties, CollisionType},
    config::*,
    generate::GlobalConfig,
    math::*,
};

pub enum SegmentDirection {
    Back,
    Forward,
    None,
}

#[derive(Debug, Clone, Copy)]
pub struct Road {
    pub start: Point<f64>,
    pub end: Point<f64>,
}

#[derive(Clone)]
pub struct Link {
    pub b: Vec<usize>,
    pub f: Vec<usize>,
}

#[derive(Clone, Copy)]
pub struct CollisionMetaInfo {
    pub highway: Option<bool>,
    pub severed: bool,
}

impl CollisionMetaInfo {
    pub fn new() -> Self {
        CollisionMetaInfo {
            highway: Some(false),
            severed: false,
        }
    }
}

#[derive(Clone)]
pub struct Segment {
    pub id: usize,
    pub collider: CollisionObject,
    pub road_revision: usize,
    dir_revision: usize,
    length_revision: usize,
    cached_dir: Option<f64>,
    cached_length: Option<f64>,
    pub r: Road,
    pub t: f64,
    pub q: CollisionMetaInfo,
    pub links: Link,
    pub prev_segment_to_link: Option<usize>,
}

impl PartialEq for Segment {
    fn eq(&self, other: &Self) -> bool {
        self.id == other.id
    }
}

impl Segment {
    pub fn set_start(&mut self, start: &Point<f64>) {
        self.r.start = *start;
        self.collider
            .update_collision_properties(Some(*start), None);
        self.road_revision += 1;
    }

    pub fn set_end(&mut self, end: &Point<f64>) {
        self.r.end = *end;
        self.collider.update_collision_properties(None, Some(*end));
        self.road_revision += 1;
    }

    pub fn new(start: Point, end: Point, t: &f64, q: &CollisionMetaInfo, id: &usize) -> Self {
        let width = if q.highway.is_some() {
            HIGHWAY_SEGMENT_WIDTH
        } else {
            DEFAULT_SEGMENT_WIDTH
        };

        let collision_properties = CollisionProperties {
            corners: Vec::new(),
            start,
            end,
            center: Point::new(0.0, 0.0),
            radius: 0.0,
            width,
        };

        let r = Road { start, end };

        let collider =
            CollisionObject::new(r, CollisionType::Line, collision_properties, id.clone());

        let links = Link {
            b: Vec::new(),
            f: Vec::new(),
        };

        Segment {
            id: id.clone(),
            collider,
            road_revision: 0,
            dir_revision: 1,
            length_revision: 1,
            cached_dir: Some(calculate_direction(&r)),
            cached_length: Some(length(r.start, r.end)),
            r,
            t: *t,
            q: *q,
            links,
            prev_segment_to_link: None,
        }
    }

    pub fn dir(&mut self) -> Option<f64> {
        if self.dir_revision != self.road_revision {
            self.dir_revision = self.road_revision;
            self.cached_dir = Some(calculate_direction(&self.r));
        }

        self.cached_dir
    }

    pub fn length(&mut self) -> Option<f64> {
        if self.length_revision != self.road_revision {
            self.length_revision = self.road_revision;
            self.cached_length = Some(length(self.r.start, self.r.end));
        }
        self.cached_length
    }

    pub fn start_is_backwards(&self, segments: &Vec<Segment>) -> bool {
        let mut _direction: SegmentDirection;
        let link_id: usize;

        if self.links.b.len() > 0 {
            _direction = SegmentDirection::Back;
            link_id = self.links.b[0];
        } else {
            _direction = SegmentDirection::Forward;
            link_id = self.links.f[0];
        };

        let link = segments.iter().find(|x| x.id == link_id).unwrap();

        match _direction {
            SegmentDirection::Back => {
                equal_v(link.r.start, self.r.start) || equal_v(link.r.end, self.r.start)
            }
            SegmentDirection::Forward => {
                equal_v(link.r.start, self.r.end) || equal_v(link.r.end, self.r.end)
            }
            SegmentDirection::None => false,
        }
    }

    pub fn links_for_end_containing(self, id: &usize) -> (Option<Vec<usize>>, SegmentDirection) {
        if self.links.b.contains(id) {
            return (Some(self.links.b), SegmentDirection::Back);
        } else if self.links.f.contains(id) {
            return (Some(self.links.f), SegmentDirection::Forward);
        } else {
            (None, SegmentDirection::None)
        }
    }

    pub fn split(
        &mut self,
        point: Point<f64>,
        segment: &mut Segment,
        global_config: &mut GlobalConfig,
    ) {
        let self_id: &usize = &self.id.clone();
        let start_is_backwards = self.start_is_backwards(&global_config.segments);

        global_config.last_id += 1;
        let mut split_part = Segment::from_existing(self, None, None, None, &global_config.last_id);
        let split_part_id = split_part.id.clone();

        split_part.set_end(&point);
        self.set_start(&point);

        // links are not copied using the preceding factory method
        // copy link array for the split part, keeping references the same
        split_part.links.b = self.links.b.clone();
        split_part.links.f = self.links.f.clone();

        let fix_links = match start_is_backwards {
            true => &mut split_part.links.b,
            false => &mut split_part.links.f,
        };

        for link_id in fix_links.iter() {
            let link: &mut Segment = global_config
                .segments
                .iter_mut()
                .find(|x| &x.id == link_id)
                .unwrap();

            if let Some(index) = link.links.b.iter().position(|x| x == self_id) {
                link.links.b[index] = split_part_id;
            } else if let Some(index) = link.links.f.iter().position(|x| x == self_id) {
                link.links.f[index] = split_part_id;
            }
        }

        let (first_split, second_split) = match start_is_backwards {
            true => (&mut split_part, self),
            false => (self, &mut split_part),
        };

        // ERROR PRONE - PLEASE CHECK HERE \\
        first_split.links.f.clear();
        first_split.links.f.push(segment.id);
        first_split.links.f.push(second_split.id);

        second_split.links.b.clear();
        second_split.links.b.push(segment.id);
        second_split.links.b.push(first_split.id);

        segment.links.f.push(first_split.id);
        segment.links.f.push(second_split.id);
        // - \\

        global_config.quad_tree.insert(split_part.collider.limits());
        global_config.segments.push(split_part);
    }

    pub fn from_existing(
        segment: &Segment,
        t: Option<f64>,
        r: Option<Road>,
        q: Option<CollisionMetaInfo>,
        id: &usize,
    ) -> Segment {
        let t = &t.unwrap_or(segment.t);
        let r = &r.unwrap_or(segment.r);
        let q = &q.unwrap_or(segment.q);

        Segment::new(r.start, r.end, t, q, id)
    }

    pub fn set_id(&mut self, id: usize) {
        self.id = id;
        self.collider.id = Some(id);
        self.collider.cached_limits.unwrap().id = Some(id);
    }

    pub fn using_direction(
        start: &Point<f64>,
        dir: Option<&f64>,
        length: Option<&f64>,
        t: &f64,
        q: CollisionMetaInfo,
        id: &usize,
    ) -> Segment {
        // default to east
        let dir = dir.unwrap_or(&90.0);
        let length = length.unwrap_or(&DEFAULT_SEGMENT_LENGTH);

        let end = Point::new(
            start.x() + length * f64::sin(*dir * PI / 180.0),
            start.y() + length * f64::cos(*dir * PI / 180.0),
        );

        Segment::new(start.clone(), end, &t, &q, id)
    }
}
