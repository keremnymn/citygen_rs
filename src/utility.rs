pub fn extended_min<T, F>(collection: &[T], mut selector: F) -> Option<(T, usize)>
where
    F: FnMut(&T) -> f64,
    T: Clone,
{
    if collection.is_empty() {
        return None;
    }

    let mut min_obj = collection[0].clone();
    let mut min_obj_i = 0;

    for (i, obj) in collection.iter().enumerate() {
        if selector(obj) < selector(&min_obj) {
            min_obj = obj.clone();
            min_obj_i = i;
        }
    }

    Some((min_obj, min_obj_i))
}

pub fn extended_max<T, F>(collection: &[T], mut selector: F) -> Option<(T, usize)>
where
    F: FnMut(&T) -> f64,
    T: Clone,
{
    if collection.is_empty() {
        return None;
    }

    let mut max_obj = collection[0].clone();
    let mut max_obj_i = 0;

    for (i, obj) in collection.iter().enumerate() {
        if selector(obj) > selector(&max_obj) {
            max_obj = obj.clone();
            max_obj_i = i;
        }
    }

    Some((max_obj, max_obj_i))
}

pub fn min_degree_difference(d1: f64, d2: f64) -> f64 {
    let diff = f64::abs(d1 - d2) % 180.0;
    f64::min(diff, f64::abs(diff - 180.0))
}
