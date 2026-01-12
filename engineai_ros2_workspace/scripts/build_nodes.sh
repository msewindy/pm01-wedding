#!/bin/bash
# Gets the source directory
root_dir="$(realpath -s $(cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)/..)"

# Define node lists for different hosts
# Nodes to build on example
EXAMPLE_NODES=(
    "interface_protocol"
    "rdk_framework"
    "interface_example"
)

# Nodes to build on Orin
APPLICATION_NODES=(
    "interface_protocol"
    "rdk_framework"
    "robot_manager"
    "sound_hub"
    "system_monitor"
)

SIM_NODES=(
    "mujoco_simulator"
    "interface_protocol"
    "interface_example"
    "wedding_interaction"
)
# Default target host is example
TARGET_HOST="example"

# Default build type is Release
BUILD_TYPE="Release"

# Clean build cache function (recursively clean all CMakeCache.txt and CMakeFiles)
clean_build_cache() {
    local packages_to_clean="$@"
    if [ -z "$packages_to_clean" ]; then
        echo "清理所有构建缓存..."
        find "$root_dir/build" -name "CMakeCache.txt" -type f -delete 2>/dev/null
        find "$root_dir/build" -type d -name "CMakeFiles" -exec rm -rf {} + 2>/dev/null
        echo "所有构建缓存已清理"
    else
        echo "清理指定包的构建缓存: $packages_to_clean"
        for pkg in $packages_to_clean; do
            if [ -d "$root_dir/build/$pkg" ]; then
                find "$root_dir/build/$pkg" -name "CMakeCache.txt" -type f -delete 2>/dev/null
                find "$root_dir/build/$pkg" -type d -name "CMakeFiles" -exec rm -rf {} + 2>/dev/null
                echo "已清理 $pkg 的构建缓存（包括所有依赖项）"
            else
                echo "警告: $pkg 的构建目录不存在"
            fi
        done
    fi
}

# Function to recursively find and clean invalid CMakeCache.txt files
clean_invalid_caches() {
    local build_dir="$1"
    local cleaned_count=0
    
    # Find all CMakeCache.txt files recursively
    while IFS= read -r -d '' cache_file; do
        # Check if cache contains wrong source path
        if grep -q "/home/lingjing/project/engine_ai" "$cache_file" 2>/dev/null; then
            local pkg_dir=$(dirname "$cache_file")
            local pkg_name=$(basename "$pkg_dir")
            echo "检测到 $pkg_name 包含旧路径的缓存，正在清理..."
            rm -rf "$cache_file" "$pkg_dir/CMakeFiles" 2>/dev/null
            ((cleaned_count++))
        fi
    done < <(find "$build_dir" -name "CMakeCache.txt" -type f -print0 2>/dev/null)
    
    if [ $cleaned_count -gt 0 ]; then
        echo "已清理 $cleaned_count 个包含旧路径的缓存文件"
    fi
}

# Parse command line arguments
if [[ "$1" == "clean" ]]; then
    # Clean mode: clean all or specified packages
    if [[ -n "$2" ]]; then
        clean_build_cache "$2"
    else
        clean_build_cache
    fi
    exit 0
elif [[ -n "$1" ]]; then
    TARGET_HOST="$1"
fi

# Validate target host
if [[ "$TARGET_HOST" != "example" && "$TARGET_HOST" != "app" && "$TARGET_HOST" != "sim" ]]; then
    echo "Error: Invalid target host '$TARGET_HOST'"
    echo "Available hosts: example, app, sim"
    exit 1
fi

# Select the appropriate node list based on target host
if [[ "$TARGET_HOST" == "example" ]]; then
    NODES=("${EXAMPLE_NODES[@]}")
    echo "Building for example nodes"
elif [[ "$TARGET_HOST" == "app" ]]; then
    NODES=("${APPLICATION_NODES[@]}")
    echo "Building for application nodes"
elif [[ "$TARGET_HOST" == "sim" ]]; then
    NODES=("${SIM_NODES[@]}")
    echo "Building for simulation nodes"
fi

echo "Build type: $BUILD_TYPE"

# Create packages argument for colcon build
PACKAGES_ARG=""
if [[ ${#NODES[@]} -gt 0 ]]; then
    PACKAGES_ARG="--packages-select ${NODES[*]}"
fi

# Create build directory if it doesn't exist
mkdir -p "$root_dir/build"

# Auto-detect and clean invalid CMakeCache.txt files (with wrong paths)
# This includes all dependencies in _deps directories
echo "检查 CMake 缓存文件..."
clean_invalid_caches "$root_dir/build"

# Run colcon build with the selected options
echo "Running build with the following nodes: ${NODES[*]}"
cd "$root_dir" && \
colcon build \
    --cmake-args -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
    --build-base build \
    --install-base install \
    $PACKAGES_ARG

if [ $? -eq 0 ]; then
    echo "Build successful!"
    echo "To use the built packages, run:"
    echo "source /opt/ros/humble/setup.bash"
    echo "source $root_dir/install/setup.bash"
else
    echo "Build failed!"
    exit 1
fi
