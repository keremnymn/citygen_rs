use geo::Point;
use noise::{NoiseFn, Simplex};

use super::{
    config::*,
    math::IntersectionResult,
    math::*,
    quad_tree::Quadtree,
    segment::{CollisionMetaInfo, Road, Segment, SegmentDirection},
    utility::*,
};

struct LocalConstraints {
    priority: i32,
    action: Option<LocalConstraintsAction>,
    other_arg: Option<LocalConstraintsArgument>,
    this_road: Option<Segment>,
    this_road_index: Option<usize>,
}

enum LocalConstraintsArgument {
    Intersection(IntersectionResult),
    Point(AddedPoint),
}

enum LocalConstraintsAction {
    IntersectionAction,
    SnapAction,
    RadiusAction,
}

pub struct GlobalConfig {
    pub segments: Vec<Segment>,
    pub last_id: usize,
    pub quad_tree: Quadtree,
    simplex: Simplex,
}

impl GlobalConfig {
    fn pop_on_road(&self, r: &Road) -> f64 {
        (self.population_at(r.start.x(), r.start.y()) + self.population_at(r.end.x(), r.end.y()))
            / 2.0
    }

    fn population_at(&self, x: f64, y: f64) -> f64 {
        let value1 = (self.simplex.get([x / 10000.0, y / 10000.0]) + 1.0) / 2.0;
        let value2 = (self.simplex.get([x / 20000.0 + 500.0, y / 20000.0 + 500.0]) + 1.0) / 2.0;
        let value3 = (self
            .simplex
            .get([x / 20000.0 + 1000.0, y / 20000.0 + 1000.0])
            + 1.0)
            / 2.0;
        f64::powf((value1 * value2 + value3) / 2.0, 2.0)
    }

    fn find_with_index(&mut self, target_id: usize) -> (Option<Segment>, Option<usize>) {
        let segments_iter = self.segments.clone();
        let (mut segment_opt, mut segment_index) = (None, None);

        for (index, segment) in segments_iter.into_iter().enumerate() {
            if segment.id == target_id {
                segment_opt = Some(segment);
                segment_index = Some(index);
            }
        }

        (segment_opt, segment_index)
    }

    fn local_constraints(&mut self, segment: &mut Segment) -> bool {
        let mut action = LocalConstraints {
            priority: 0,
            action: None,
            other_arg: None,
            this_road: None,
            this_road_index: None,
        };

        let matches = self.quad_tree.retrieve(&segment.collider.limits());

        if matches.len() < 1 {
            return true;
        }

        for i in 0..matches.len() - 1 {
            let _match = matches[i];

            let (this_road, this_road_index) = match self.find_with_index(_match.id.unwrap()) {
                (Some(road), Some(index)) => (road, index),
                _ => continue,
            };

            if action.priority <= 4 {
                let intersection = self.do_road_segments_intersect(&segment.r, &this_road.r);
                if intersection.is_some() {
                    action = LocalConstraints {
                        priority: 4,
                        action: Some(LocalConstraintsAction::IntersectionAction),
                        other_arg: Some(LocalConstraintsArgument::Intersection(
                            intersection.unwrap(),
                        )),
                        this_road: Some(this_road.clone()),
                        this_road_index: Some(this_road_index.clone()),
                    }
                }
            };
            if action.priority <= 3 {
                if length(segment.r.end, this_road.r.end) < ROAD_SNAP_DISTANCE {
                    action = LocalConstraints {
                        priority: 3,
                        action: Some(LocalConstraintsAction::RadiusAction),
                        this_road: Some(this_road.clone()),
                        this_road_index: Some(this_road_index.clone()),
                        other_arg: None,
                    }
                }
            };

            if action.priority <= 2 {
                let added_point: AddedPoint = distance_to_line(
                    segment.r.end,
                    this_road.clone().r.start,
                    this_road.clone().r.end,
                );

                if added_point.distance2 < ROAD_SNAP_DISTANCE * ROAD_SNAP_DISTANCE
                    && added_point.line_proj_2 >= 0.0
                    && added_point.line_proj_2 <= added_point.length2
                {
                    action = LocalConstraints {
                        priority: 2,
                        action: Some(LocalConstraintsAction::SnapAction),
                        other_arg: Some(LocalConstraintsArgument::Point(added_point)),
                        this_road: Some(this_road.clone()),
                        this_road_index: Some(this_road_index.clone()),
                    }
                }
            }
        }

        match action.action {
            Some(LocalConstraintsAction::IntersectionAction) => {
                let mut this_road = action.this_road.unwrap();

                if min_degree_difference(this_road.dir().unwrap(), segment.dir().unwrap())
                    < MINIMUM_INTERSECTION_DEVIATION
                {
                    return false;
                }

                let intersection = match action.other_arg.unwrap() {
                    LocalConstraintsArgument::Intersection(intersection) => intersection,
                    LocalConstraintsArgument::Point(_) => return false,
                };
                let intersection_point = Point::new(intersection.x, intersection.y);

                this_road.split(intersection_point, segment, self);
                self.segments[action.this_road_index.unwrap()] = this_road.clone();

                segment.set_end(&intersection_point);
                segment.q.severed = true;

                return true;
            }
            Some(LocalConstraintsAction::RadiusAction) => {
                let mut this_road = action.this_road.unwrap();
                let point = this_road.r.end;
                let direction: SegmentDirection;

                segment.r.end = point;
                segment.q.severed = true;

                let mut links = if this_road.clone().start_is_backwards(&self.segments) {
                    direction = SegmentDirection::Forward;
                    this_road.clone().links.f
                } else {
                    direction = SegmentDirection::Back;
                    this_road.clone().links.b
                };

                if links.iter().any(|link_id| {
                    let link_segment = self.segments.iter().find(|x| &x.id == link_id);
                    if let Some(link) = link_segment {
                        (equal_v(link.r.start, segment.r.end)
                            && equal_v(link.r.end, segment.r.start))
                            || (equal_v(link.r.start, segment.r.start)
                                && equal_v(link.r.end, segment.r.end))
                    } else {
                        true // CHECK AGAIN!!!
                    }
                }) {
                    return false;
                }

                for link_id in links.iter() {
                    let (link_segment, link_segment_index) = self.find_with_index(*link_id);

                    if link_segment.is_none() {
                        continue;
                    }

                    let (vec_to_push, direction) = link_segment
                        .unwrap()
                        .clone()
                        .links_for_end_containing(&this_road.id);

                    if let Some(mut link_links) = vec_to_push {
                        link_links.push(segment.id);

                        match direction {
                            SegmentDirection::Back => {
                                self.segments[link_segment_index.unwrap()].links.b = link_links;
                            }
                            SegmentDirection::Forward => {
                                self.segments[link_segment_index.unwrap()].links.f = link_links;
                            }
                            _ => (),
                        }

                        segment.links.f.push(*link_id);
                    }
                }
                links.push(segment.id);
                segment.links.f.push(this_road.id);

                match direction {
                    SegmentDirection::Forward => this_road.links.f = links,
                    SegmentDirection::Back => this_road.links.b = links,
                    SegmentDirection::None => (),
                }

                self.segments[action.this_road_index.unwrap()] = this_road;

                return true;
            }
            Some(LocalConstraintsAction::SnapAction) => {
                let added_point = match action.other_arg.unwrap() {
                    LocalConstraintsArgument::Point(added_point) => added_point,
                    LocalConstraintsArgument::Intersection(_) => return false,
                };
                let point = added_point.point_on_line;

                segment.set_end(&point);
                segment.q.severed = true;
                let mut this_road = action.this_road.unwrap();

                if min_degree_difference(this_road.dir().unwrap(), segment.dir().unwrap())
                    < MINIMUM_INTERSECTION_DEVIATION
                {
                    return false;
                }

                this_road.split(point, segment, self);
                self.segments[action.this_road_index.unwrap()] = this_road;

                return true;
            }
            None => (),
        }
        true
    }

    fn do_road_segments_intersect(&self, r1: &Road, r2: &Road) -> Option<IntersectionResult> {
        do_line_segments_intersect(r1.start, r1.end, r2.start, r2.end, true)
    }

    pub fn generate_segments(&mut self, previous_segment: &mut Segment) -> Vec<Segment> {
        let mut new_branches: Vec<Segment> = Vec::new();

        if !previous_segment.q.severed {
            let r_end = previous_segment.r.end;
            let len = previous_segment.length().unwrap();
            let dir = previous_segment.dir().unwrap();
            let some_dir = Some(&dir);

            // Used for highways or going straight on a normal branch
            let template_continue = |dir, id| {
                Segment::using_direction(
                    &r_end,
                    dir,
                    Some(&len),
                    &0.0,
                    previous_segment.q.clone(),
                    id,
                )
            };

            // Not using q, i.e., not highways
            let template_branch = |dir, id| {
                Segment::using_direction(
                    &r_end,
                    dir,
                    Some(&DEFAULT_SEGMENT_LENGTH),
                    if previous_segment.q.highway == Some(true) {
                        &NORMAL_BRANCH_TIME_DELAY_FROM_HIGHWAY
                    } else {
                        &0.0
                    },
                    CollisionMetaInfo::new(),
                    id,
                )
            };

            let _id = self.last_id.clone();
            let mut continue_straight = template_continue(some_dir, &_id);
            let straight_pop = self.pop_on_road(&continue_straight.r);

            if previous_segment.q.highway == Some(true) {
                let _random_angle = &dir + random_angle(AngleDirection::Forward);
                let random_angle_some = Some(&(_random_angle));

                let mut random_straight = template_continue(random_angle_some, &_id);

                let random_pop = self.pop_on_road(&random_straight.r);
                let road_pop: f64;

                if random_pop > straight_pop {
                    self.last_id += 1;
                    random_straight.set_id(self.last_id);

                    new_branches.push(random_straight);
                    road_pop = random_pop
                } else {
                    self.last_id += 1;
                    continue_straight.set_id(self.last_id);

                    new_branches.push(continue_straight.clone());
                    road_pop = straight_pop;
                }

                if road_pop > HIGHWAY_BRANCH_POPULATION_THRESHOLD {
                    if rand::random::<f64>() < HIGHWAY_BRANCH_PROBABILITY {
                        self.last_id += 1;

                        let left_highway_branch_angle =
                            &dir - 90.0 + random_angle(AngleDirection::Branch);
                        let left_highway_branch =
                            template_continue(Some(&left_highway_branch_angle), &self.last_id);
                        new_branches.push(left_highway_branch);
                    } else if rand::random::<f64>() < HIGHWAY_BRANCH_PROBABILITY {
                        self.last_id += 1;

                        let right_highway_branch_angle =
                            &dir + 90.0 + random_angle(AngleDirection::Branch);
                        let right_highway_branch =
                            template_continue(Some(&right_highway_branch_angle), &self.last_id);
                        new_branches.push(right_highway_branch);
                    }
                }
            } else if straight_pop > NORMAL_BRANCH_POPULATION_THRESHOLD {
                self.last_id += 1;
                continue_straight.set_id(self.last_id);

                new_branches.push(continue_straight);
            }

            if straight_pop > NORMAL_BRANCH_POPULATION_THRESHOLD {
                if rand::random::<f64>() < DEFAULT_BRANCH_PROBABILITY {
                    let left_branch_angle = &dir - 90.0 + random_angle(AngleDirection::Branch);
                    self.last_id += 1;
                    let left_branch = template_branch(Some(&(left_branch_angle)), &self.last_id);
                    new_branches.push(left_branch);
                } else if rand::random::<f64>() < DEFAULT_BRANCH_PROBABILITY {
                    let right_branch_angle = &dir + 90.0 + random_angle(AngleDirection::Branch);
                    self.last_id += 1;
                    let right_branch = template_branch(Some(&(right_branch_angle)), &self.last_id);
                    new_branches.push(right_branch);
                }
            }
        }

        for branch in new_branches.iter_mut() {
            branch.prev_segment_to_link = Some(previous_segment.id);
        }

        new_branches
    }

    fn generate_main(&mut self) -> (Vec<Segment>, Quadtree) {
        let mut priority_q: Vec<Segment> = Vec::new();

        self.last_id += 1;
        // Setup first segments in queue
        let mut root_segment = Segment::new(
            Point::new(0.0, 0.0),
            Point::new(HIGHWAY_SEGMENT_LENGTH, 0.0),
            &0.0,
            &CollisionMetaInfo {
                highway: Some(true),
                severed: false,
            },
            &self.last_id,
        );

        self.last_id += 1;
        let mut opposite_direction =
            Segment::from_existing(&root_segment, None, None, None, &self.last_id);

        let new_end = Point::new(
            root_segment.r.start.x() - HIGHWAY_SEGMENT_LENGTH,
            opposite_direction.r.end.y(),
        );

        opposite_direction.set_end(&new_end);
        opposite_direction.links.b.push(root_segment.id);
        root_segment.links.b.push(opposite_direction.id);

        priority_q.push(root_segment);
        priority_q.push(opposite_direction);

        while priority_q.len() > 0 && self.segments.len() < SEGMENT_COUNT_LIMIT as usize {
            let mut min_t: Option<f64> = None;
            let mut min_t_i: usize = 0;

            for (index, segment) in priority_q.iter().enumerate() {
                if min_t.is_none() || segment.t < min_t.unwrap() {
                    min_t = Some(segment.t);
                    min_t_i = index;
                }
            }

            let mut min_segment = priority_q.remove(min_t_i);
            let accepted = self.local_constraints(&mut min_segment);

            if accepted {
                if let Some(prev_segment_id) = min_segment.prev_segment_to_link {
                    let (prev_segment_opt, prev_segment_index) =
                        self.find_with_index(prev_segment_id);
                    let mut prev_segment = prev_segment_opt.unwrap();

                    for link_id in prev_segment.clone().links.f.clone() {
                        min_segment.links.b.push(link_id);

                        let (mut link_opt, mut link_index) = (None, None);

                        for (index, segment) in self.segments.iter_mut().enumerate() {
                            if segment.id == link_id {
                                link_opt = Some(segment);
                                link_index = Some(index);
                            }
                        }

                        let link = link_opt.unwrap();

                        let (changed_vec_opt, segment_direction) =
                            link.clone().links_for_end_containing(&prev_segment_id);

                        if let Some(mut changed_vec) = changed_vec_opt {
                            changed_vec.push(min_segment.id);

                            match segment_direction {
                                SegmentDirection::Back => link.links.b = changed_vec.clone(),
                                SegmentDirection::Forward => link.links.f = changed_vec.clone(),
                                _ => (),
                            }

                            self.segments[link_index.unwrap()] = link.clone();
                        }
                    }

                    prev_segment.links.f.push(min_segment.id);
                    min_segment.links.b.push(prev_segment.id);
                    self.segments[prev_segment_index.unwrap()] = prev_segment.clone();
                }

                let branches = self.generate_segments(&mut min_segment);
                for mut new_segment in branches {
                    new_segment.t = min_segment.t + 1.0 + new_segment.t;
                    priority_q.push(new_segment);
                }

                self.quad_tree.insert(min_segment.collider.limits());
                self.segments.push(min_segment);
            }
        }

        return (self.segments.clone(), self.quad_tree.clone());
    }
}

pub fn generate() -> (Vec<Segment>, Quadtree) {
    let quad_tree = Quadtree::from_config();

    let seed: u32 = 42;
    let simplex = Simplex::new(seed);

    let mut global_goals = GlobalConfig {
        segments: Vec::new(),
        last_id: 0,
        quad_tree,
        simplex,
    };
    global_goals.generate_main()
}
