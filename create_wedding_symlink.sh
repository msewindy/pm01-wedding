#!/bin/bash
# 创建软链接脚本：在 engineai_ros2_workspace/src 下创建指向 wedding_jeston 的软链接

# 获取脚本所在目录（项目根目录）
project_root="$(realpath -s $(cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd))"

# 定义源目录和目标链接路径
src_dir="$project_root/engineai_ros2_workspace/src"
target_dir="$(realpath -s $project_root/wedding_jeston)"
link_path="$src_dir/wedding_jeston"

# 检查源目录是否存在
if [ ! -d "$src_dir" ]; then
    echo "错误: 源目录不存在: $src_dir"
    exit 1
fi

# 检查目标目录是否存在
if [ ! -d "$target_dir" ]; then
    echo "错误: 目标目录不存在: $target_dir"
    exit 1
fi

# 检查软链接是否已存在
if [ -L "$link_path" ]; then
    echo "警告: 软链接已存在: $link_path"
    echo "当前指向: $(readlink -f $link_path)"
    read -p "是否要删除并重新创建? (y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm "$link_path"
        echo "已删除旧软链接"
    else
        echo "操作已取消"
        exit 0
    fi
elif [ -e "$link_path" ]; then
    echo "错误: 路径已存在但不是软链接: $link_path"
    exit 1
fi

# 创建软链接
echo "正在创建软链接..."
echo "源目录: $target_dir"
echo "链接路径: $link_path"

ln -s "$target_dir" "$link_path"

if [ $? -eq 0 ]; then
    echo "软链接创建成功!"
    echo "软链接: $link_path -> $target_dir"
else
    echo "软链接创建失败!"
    exit 1
fi
