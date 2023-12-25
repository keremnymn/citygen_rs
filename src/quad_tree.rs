use super::collision::CollisionLimits;

#[derive(Clone)]
pub struct Quadtree {
    max_objects: f64,
    max_levels: f64,
    level: f64,
    bounds: CollisionLimits,
    objects: Vec<CollisionLimits>,
    nodes: Vec<Quadtree>,
}

impl Quadtree {
    pub fn from_config() -> Quadtree {
        Quadtree {
            max_objects: 10.0,
            max_levels: 10.0,
            level: 0.0,
            bounds: CollisionLimits {
                x: -20000.0,
                y: -20000.0,
                width: 40000.0,
                height: 40000.0,
                id: None,
            },
            objects: Vec::new(),
            nodes: Vec::new(),
        }
    }

    fn new(bounds: CollisionLimits, max_objects: f64, max_levels: f64, level: f64) -> Quadtree {
        Quadtree {
            max_objects,
            max_levels,
            level,
            bounds,
            objects: Vec::new(),
            nodes: Vec::new(),
        }
    }

    fn split(&mut self) {
        let next_level = self.level + 1.0;
        let sub_width = (self.bounds.width / 2.0).round() as f64;
        let sub_height = (self.bounds.height / 2.0).round() as f64;
        let x = self.bounds.x.round() as f64;
        let y = self.bounds.y.round() as f64;

        // Top right node
        self.nodes.push(Quadtree::new(
            CollisionLimits {
                x: x + sub_width,
                y,
                width: sub_width,
                height: sub_height,
                id: None,
            },
            self.max_objects,
            self.max_levels,
            next_level,
        ));

        // Top left node
        self.nodes.push(Quadtree::new(
            CollisionLimits {
                x,
                y,
                width: sub_width,
                height: sub_height,
                id: None,
            },
            self.max_objects,
            self.max_levels,
            next_level,
        ));

        // Bottom left node
        self.nodes.push(Quadtree::new(
            CollisionLimits {
                x,
                y: y + sub_height,
                width: sub_width,
                height: sub_height,
                id: None,
            },
            self.max_objects,
            self.max_levels,
            next_level,
        ));

        // Bottom right node
        self.nodes.push(Quadtree::new(
            CollisionLimits {
                x: x + sub_width,
                y: y + sub_height,
                width: sub_width,
                height: sub_height,
                id: None,
            },
            self.max_objects,
            self.max_levels,
            next_level,
        ));
    }

    fn get_index(&self, p_rect: &CollisionLimits) -> i64 {
        let mut index = -1;
        let vertical_midpoint = self.bounds.x + (self.bounds.width / 2.0);
        let horizontal_midpoint = self.bounds.y + (self.bounds.height / 2.0);

        // pRect can completely fit within the top quadrants
        let top_quadrant =
            p_rect.y < horizontal_midpoint && p_rect.y + p_rect.height < horizontal_midpoint;

        // pRect can completely fit within the bottom quadrants
        let bottom_quadrant = p_rect.y > horizontal_midpoint;

        // pRect can completely fit within the left quadrants
        if p_rect.x < vertical_midpoint && p_rect.x + p_rect.width < vertical_midpoint {
            if top_quadrant {
                index = 1;
            } else if bottom_quadrant {
                index = 2;
            }
        // pRect can completely fit within the right quadrants
        } else if p_rect.x > vertical_midpoint {
            if top_quadrant {
                index = 0;
            } else if bottom_quadrant {
                index = 3;
            }
        }

        index
    }

    pub fn insert(&mut self, p_rect: CollisionLimits) {
        let mut i = 0;
        let mut index: i64;

        // If we have subnodes
        if !self.nodes.is_empty() {
            index = self.get_index(&p_rect);

            if index != -1 {
                if let Some(node) = self.nodes.get_mut(index as usize) {
                    node.insert(p_rect);
                    return;
                }
            }
        }

        self.objects.push(p_rect);

        if self.objects.len() as f64 > self.max_objects && self.level < self.max_levels {
            // Split if we don't already have subnodes
            if self.nodes.is_empty() {
                self.split();
            }

            // Add all objects to their corresponding subnodes
            while i < self.objects.len() {
                let _obj = &self.objects.get(i as usize).unwrap();
                index = self.get_index(_obj);

                if index != -1 {
                    if let Some(node) = self.nodes.get_mut(index as usize) {
                        node.insert(self.objects.remove(i as usize));
                    }
                } else {
                    i += 1;
                }
            }
        }
    }

    pub fn retrieve(&self, p_rect: &CollisionLimits) -> Vec<CollisionLimits> {
        let index = self.get_index(p_rect);
        let mut return_objects = self.objects.clone(); // Assuming Bounds implements Clone

        // If we have subnodes
        if self.nodes.as_slice().len() > 0 {
            // If pRect fits into a subnode
            if index != -1 {
                return_objects.extend(self.nodes[index as usize].retrieve(p_rect).iter().cloned());
            } else {
                // If pRect does not fit into a subnode, check it against all subnodes
                for node in &self.nodes {
                    return_objects.extend(node.retrieve(p_rect).iter().cloned());
                }
            }
        }

        return_objects
    }

    fn clear(&mut self) {
        self.objects.clear();

        for inner_node in &mut self.nodes {
            inner_node.clear();
        }
    }
}
