mod collision;
mod config;
mod generate;
mod math;
mod quad_tree;
mod segment;
mod utility;

use geojson::{feature::Id, Feature, FeatureCollection, GeoJson, Geometry, Value};
use serde_json::Map;
use std::{fs::File, io::Write};

use generate::generate;

pub fn main() {
    let (segments, _) = generate();

    let mut features = Vec::new();

    for mut segment in segments.into_iter() {
        let coordinates = vec![
            vec![segment.r.start.x(), segment.r.start.y()],
            vec![segment.r.end.x(), segment.r.end.y()],
        ];
        let line_string_geometry = Geometry::new(Value::LineString(coordinates));

        let mut props = Map::new();
        let dir_val = serde_json::Number::from_f64(segment.dir().unwrap());
        props.insert("dir".to_string(), dir_val.into());

        let severed_val = serde_json::Value::Bool(segment.q.severed);
        props.insert("severed".to_string(), severed_val.into());

        let highway_val = if segment.q.highway.is_some() {
            serde_json::Value::Bool(segment.q.highway.unwrap())
        } else {
            serde_json::Value::Bool(false)
        };
        props.insert("highway".to_string(), highway_val.into());

        let feature = Feature {
            bbox: None,
            geometry: Some(line_string_geometry),
            id: Some(Id::Number(segment.id.into())),
            properties: Some(props),
            foreign_members: None,
        };

        features.push(feature);
    }

    let feature_collection = FeatureCollection {
        bbox: None,
        features,
        foreign_members: None,
    };

    let serialized = GeoJson::from(feature_collection).to_string();

    // Write the JSON string to a GeoJSON file
    let mut file = File::create("./output.geojson").unwrap();
    file.write_all(serialized.as_bytes()).unwrap();
}
