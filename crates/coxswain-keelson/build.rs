fn main() {
    let protoc = protoc_bin_vendored::protoc_bin_path().expect("vendored protoc");
    let includes = protoc_bin_vendored::include_path().expect("vendored protoc includes");
    // Build scripts are single-threaded; set_var is safe here.
    unsafe { std::env::set_var("PROTOC", protoc) };
    prost_build::Config::new()
        .compile_protos(
            &[
                "protos/Envelope.proto",
                "protos/payloads/Primitives.proto",
                "protos/payloads/EntityHealth.proto",
                "protos/payloads/foxglove/LocationFix.proto",
                "protos/coxswain_conn.proto",
            ],
            &[
                std::path::Path::new("protos"),
                std::path::Path::new("protos/payloads"),
                &includes,
            ],
        )
        .expect("proto codegen");
}
