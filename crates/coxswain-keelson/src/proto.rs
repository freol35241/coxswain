//! Generated protobuf types from the vendored protos (protos/README.md).

pub mod core {
    include!(concat!(env!("OUT_DIR"), "/core.rs"));
}

pub mod keelson {
    include!(concat!(env!("OUT_DIR"), "/keelson.rs"));
}

pub mod foxglove {
    include!(concat!(env!("OUT_DIR"), "/foxglove.rs"));
}

pub mod coxswain {
    include!(concat!(env!("OUT_DIR"), "/coxswain.rs"));
}
