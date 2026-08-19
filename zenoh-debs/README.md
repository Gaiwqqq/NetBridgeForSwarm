# Zenoh 1.9.0 Debian packages

These packages were built natively on Ubuntu 20.04 for NetBridge. Install the
directory matching the target architecture:

```bash
sudo dpkg -i ./zenoh-debs/"$(dpkg --print-architecture)"/*.deb
```

Each directory contains `libzenohc`, `libzenohc-dev`, and the architecture-
independent `libzenohcpp-dev` headers. Verify downloaded or copied packages
before installation:

```bash
(cd zenoh-debs && sha256sum -c SHA256SUMS)
```

The packages require Ubuntu 20.04 and use the Zenoh unstable reliability API
needed by NetBridge. Use `swarm_ros_bridge/scripts/build_zenoh_1_9_debs.sh` to
rebuild them on the native target architecture.
