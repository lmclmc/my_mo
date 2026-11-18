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
    # CycloneDDS 和 idlc 由本机安装提供，无压缩 MCAP 录制使用 contrib/mcap 中的头文件。
    # 静态链接 TinyXML2，使构建和安装后的插件由 Conan 依赖提供 XML 解析实现。
    default_options = {"*:shared": False}
