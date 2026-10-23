#!/usr/bin/env bash
# Builds the Puppet Pin OFX plugin and its tests with LLVM-MinGW (clang).
#
#   ./build.sh            plugin + tests
#   ./build.sh plugin     build/PuppetPin.ofx.bundle only
#   ./build.sh test       build everything and run the tests
#
# LLVM must point at an llvm-mingw toolchain on a short path (its linker fails on paths
# near the 260 character Windows limit). Run from a short path too, e.g. via a junction.
set -euo pipefail
cd "$(dirname "$0")"

LLVM="${LLVM:?set LLVM to your llvm-mingw folder, e.g. LLVM=/c/llvm-mingw ./build.sh test}"
CXX="$LLVM/bin/x86_64-w64-mingw32-clang++"
JOBS="${JOBS:-16}"
B=build
CXXFLAGS=(-std=c++17 -O2 -Wall -Wextra -Wno-unused-parameter -Wno-unused-variable -Wno-sign-compare
          -Wno-missing-field-initializers -DNOMINMAX -Ithird_party/openfx/include -Ithird_party/openfx/support/include)

CORE=(src/core/pp_geom.cpp src/core/pp_mesh.cpp src/core/pp_sparse.cpp src/core/pp_solver.cpp src/core/pp_render.cpp src/core/pp_rig.cpp)
SUPPORT=()
for f in ofxsCore ofxsImageEffect ofxsInteract ofxsLog ofxsMultiThread ofxsParams ofxsProperty ofxsPropertyValidation; do
    SUPPORT+=("third_party/openfx/support/Library/$f.cpp")
done

# Newest header we depend on: any change rebuilds our own objects.
newest_header=$(find src tests -name '*.h' -printf '%T@\n' | sort -n | tail -1)

obj_of() { local s="$1"; s="${s//\//_}"; echo "$B/obj/${s%.cpp}.o"; }

compile() {  # compile <src> [extra flags...]
    local src="$1"; shift
    local o; o=$(obj_of "$src")
    if [[ -f "$o" ]]; then
        local ot st; ot=$(stat -c %Y "$o"); st=$(stat -c %Y "$src")
        if (( ot >= st )) && { [[ "$src" == third_party/* ]] || (( ot >= ${newest_header%.*} )); }; then return 0; fi
    fi
    echo "  CXX $src"
    "$CXX" "${CXXFLAGS[@]}" "$@" -c "$src" -o "$o"
}

compile_all() {  # compile_all <files...> (in parallel)
    local pids=() f
    for f in "$@"; do
        local extra=()
        [[ "$f" == third_party/* ]] && extra=(-w)
        compile "$f" "${extra[@]}" &
        pids+=($!)
        if (( ${#pids[@]} >= JOBS )); then wait "${pids[0]}"; pids=("${pids[@]:1}"); fi
    done
    local rc=0
    for p in "${pids[@]}"; do wait "$p" || rc=1; done
    return $rc
}

objs() { local f; for f in "$@"; do obj_of "$f"; done; }

mkdir -p "$B/obj"
target="${1:-all}"

build_plugin() {
    compile_all "${CORE[@]}" "${SUPPORT[@]}" src/ofx/PuppetPin.cpp
    mkdir -p "$B/PuppetPin.ofx.bundle/Contents/Win64"
    echo "  LINK PuppetPin.ofx"
    "$CXX" -shared $(objs src/ofx/PuppetPin.cpp "${CORE[@]}" "${SUPPORT[@]}") \
        -o "$B/PuppetPin.ofx.bundle/Contents/Win64/PuppetPin.ofx" -static
}

build_installer() {
    build_plugin
    mkdir -p "$B/installer"
    python installer/make_icon.py "$B/installer/puppetpin.ico" "$B/installer/icon_preview.png" >/dev/null
    "$LLVM/bin/x86_64-w64-mingw32-windres" -I . installer/setup.rc -O coff -o "$B/installer/setup_res.o"
    echo "  LINK PuppetPin-1.5-Setup.exe"
    "$CXX" -std=c++17 -O2 -municode -mwindows installer/setup.cpp "$B/installer/setup_res.o"         -o "$B/installer/PuppetPin-1.5-Setup.exe" -static -lshell32 -lole32 -luuid
}

build_tests() {
    compile_all "${CORE[@]}" tests/core_test.cpp tests/ofx_host_test.cpp
    echo "  LINK core_test.exe"
    "$CXX" $(objs "${CORE[@]}" tests/core_test.cpp) -o "$B/core_test.exe" -static
    echo "  LINK ofx_host_test.exe"
    "$CXX" $(objs tests/ofx_host_test.cpp) -o "$B/ofx_host_test.exe" -static
}

case "$target" in
    plugin) build_plugin ;;
    tests) build_tests ;;
    all) build_plugin; build_tests ;;
    installer) build_installer ;;
    test)
        build_plugin; build_tests
        mkdir -p tests/out
        "$B/core_test.exe" tests/out
        "$B/ofx_host_test.exe" "$B/PuppetPin.ofx.bundle/Contents/Win64/PuppetPin.ofx" tests/out
        ;;
    clean) rm -rf "$B" ;;
    *) echo "unknown target $target"; exit 1 ;;
esac
echo "done"
