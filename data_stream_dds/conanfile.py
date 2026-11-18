import os
from conan import ConanFile


_SDK_VERSION = (
    open(os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, "SDK_VERSION"))
    .read()
    .strip()
)


class DataStreamDdsConan(ConanFile):
    name = "data_stream_dds"
    version = "0"
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeDeps", "CMakeToolchain"
    requires = (
        f"plotjuggler_sdk/{_SDK_VERSION}",
        "nlohmann_json/3.12.0",
        "tinyxml2/10.0.0",
    )
    # CycloneDDS (including idlc) is provided by the native installation.
    # Uncompressed MCAP recording uses the vendored writer headers in contrib/mcap.
    # Bundle TinyXML2 statically so the build and installed plugin do not rely
    # on a system libtinyxml2-dev package or a separate TinyXML2 shared library.
    default_options = {"*:shared": False}
