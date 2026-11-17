# MCAP writer headers

Upstream: https://github.com/foxglove/mcap/tree/releases/cpp/v2.1.1/cpp/mcap/include/mcap

Version: libmcap 2.1.1. License: MIT, see [LICENSE](LICENSE).

This directory contains the unmodified upstream writer and its required support
headers. Reader, compression libraries and command line tools are not included.
`dds_mcap_writer.cpp` compiles the implementation once, with LZ4/Zstd disabled;
the plugin writes standard, uncompressed MCAP records and a complete summary.
