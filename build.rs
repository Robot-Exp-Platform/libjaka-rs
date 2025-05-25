use std::path::Path;

fn main() {
    cxx_build::bridge("src/ffi/to_cxx.rs")
        .flag_if_supported("-std=c++17")
        .compile("mylib");

    let header_src = Path::new(r"target\cxxbridge\libjaka\src\ffi\to_cxx.rs.h");
    let header_dst = Path::new("include").join("libjaka.h");
    std::fs::create_dir_all("include").unwrap();
    std::fs::copy(header_src, &header_dst).unwrap();
    println!("cargo:rerun-if-changed=src/ffi/to_cxx.rs");
}
