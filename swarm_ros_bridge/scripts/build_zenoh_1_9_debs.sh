#!/usr/bin/env bash
set -euo pipefail

readonly ZENOH_VERSION="1.9.0"
readonly ZENOH_C_REPOSITORY="https://github.com/eclipse-zenoh/zenoh-c.git"
readonly ZENOH_CPP_REPOSITORY="https://github.com/eclipse-zenoh/zenoh-cpp.git"

usage() {
  echo "Usage: $0 --work-dir DIR --output-dir DIR [--jobs N]"
  echo "Build on each Ubuntu 20.04 target (x86_64 and ARM64); this script does not cross-compile."
}

work_dir=""
output_dir=""
jobs="$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 2)"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --work-dir)
      work_dir="$2"
      shift 2
      ;;
    --output-dir)
      output_dir="$2"
      shift 2
      ;;
    --jobs)
      jobs="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -z "$work_dir" || -z "$output_dir" ]]; then
  usage >&2
  exit 2
fi
if [[ "$work_dir" == "/" || "$output_dir" == "/" ]]; then
  echo "Refusing to use / as a build or output directory" >&2
  exit 2
fi

for command_name in git cmake cargo rustc cpack dpkg-deb; do
  if ! command -v "$command_name" >/dev/null 2>&1; then
    echo "Missing required command: $command_name" >&2
    echo "Install a Rust toolchain plus cmake, git and dpkg-dev in the Ubuntu 20.04 builder." >&2
    exit 1
  fi
done

mkdir -p "$work_dir" "$output_dir"
readonly work_dir_abs="$(cd "$work_dir" && pwd)"
readonly output_dir_abs="$(cd "$output_dir" && pwd)"
readonly source_dir="$work_dir_abs/source"
readonly build_dir="$work_dir_abs/build"
readonly stage_dir="$work_dir_abs/stage"
readonly deb_arch="$(dpkg --print-architecture)"
mkdir -p "$source_dir" "$build_dir" "$stage_dir"

clone_exact_tag() {
  local repository="$1"
  local destination="$2"
  if [[ ! -d "$destination/.git" ]]; then
    git clone --depth 1 --branch "$ZENOH_VERSION" "$repository" "$destination"
  fi
  local version
  version="$(tr -d '[:space:]' < "$destination/version.txt")"
  if [[ "$version" != "$ZENOH_VERSION" ]]; then
    echo "Version mismatch in $destination: expected $ZENOH_VERSION, got $version" >&2
    exit 1
  fi
}

clone_exact_tag "$ZENOH_C_REPOSITORY" "$source_dir/zenoh-c"
clone_exact_tag "$ZENOH_CPP_REPOSITORY" "$source_dir/zenoh-cpp"

cmake -S "$source_dir/zenoh-c" -B "$build_dir/zenoh-c" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr \
  -DCMAKE_INSTALL_LIBDIR=lib \
  -DDEBARCH="$deb_arch" \
  -DBUILD_SHARED_LIBS=ON \
  -DZENOHC_BUILD_WITH_UNSTABLE_API=ON \
  -DZENOHC_BUILD_WITH_SHARED_MEMORY=OFF \
  -DZENOHC_COPY_SOURCE_CARGO_LOCK=ON \
  -DZENOHC_BUILD_IN_SOURCE_TREE=OFF
cmake --build "$build_dir/zenoh-c" --config Release --parallel "$jobs"
DESTDIR="$stage_dir" cmake --install "$build_dir/zenoh-c" --config Release
(cd "$build_dir/zenoh-c" && cpack -G DEB -C Release)

cmake -S "$source_dir/zenoh-cpp" -B "$build_dir/zenoh-cpp" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr \
  -DCMAKE_INSTALL_LIBDIR=lib \
  -DCMAKE_PREFIX_PATH="$stage_dir/usr" \
  -DZENOHCXX_ZENOHC=ON \
  -DZENOHCXX_ZENOHPICO=OFF \
  -DZENOHCXX_ENABLE_TESTS=OFF \
  -DZENOHCXX_ENABLE_EXAMPLES=OFF
cmake --build "$build_dir/zenoh-cpp" --config Release --parallel "$jobs"
(cd "$build_dir/zenoh-cpp" && cpack -G DEB -C Release)

# A reused build tree may contain packages created before DEBARCH was set.
# Remove only this script's versioned outputs, then copy packages whose Debian
# architecture is valid for this native builder (plus architecture-independent
# headers).
find "$output_dir_abs" -maxdepth 1 -type f \
  \( -name "libzenohc_${ZENOH_VERSION}_*.deb" \
     -o -name "libzenohc-dev_${ZENOH_VERSION}_*.deb" \
     -o -name "libzenohcpp-dev_${ZENOH_VERSION}_*.deb" \) -delete
packages=()
while IFS= read -r package; do
  package_arch="$(dpkg-deb -f "$package" Architecture 2>/dev/null)"
  if [[ "$package_arch" == "$deb_arch" || "$package_arch" == "all" ]]; then
    packages+=("$package")
  fi
done < <(find "$build_dir/zenoh-c/packages" "$build_dir/zenoh-cpp/packages" \
  -maxdepth 1 -type f -name '*.deb' -print)
if [[ "${#packages[@]}" -lt 3 ]]; then
  echo "Expected three Zenoh Debian packages for $deb_arch, found ${#packages[@]}" >&2
  exit 1
fi
cp -f "${packages[@]}" "$output_dir_abs/"

echo "Zenoh $ZENOH_VERSION native packages for $(dpkg --print-architecture):"
find "$output_dir_abs" -maxdepth 1 -type f -name '*.deb' -print
echo "Install all generated packages, then build NetBridge normally with catkin."
