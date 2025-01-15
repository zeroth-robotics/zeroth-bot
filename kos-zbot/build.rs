fn main() {
    #[cfg(feature = "milkv")]
    {
        println!("cargo:rustc-link-search=native=./firmware/cviwrapper");
        println!("cargo:rustc-link-lib=dylib=cviwrapper");
    }

    println!("cargo:rerun-if-changed=build.rs");
}
