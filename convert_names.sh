#!/bin/bash

function convert_to_camel_case() {
  local var_name=$1
  local camel_case=$(echo "$var_name" | sed -r 's/_([a-z])/\U\1/g; s/^[a-z]/\u&/')
  echo "$camel_case"
}

# 인자 확인
if [ "$#" -ne 2 ]; then
  echo "Usage: $0 <package_name> <test_node>"
  exit 1
fi

pkg_name="$1"
test_node="$2"
class_name=$(convert_to_camel_case "$test_node")
upper_pkg_name="${pkg_name^^}_H_"

echo "convert this template to the following names"
echo "package name: $pkg_name"
echo "node name: $test_node"
echo "node's class name: $class_name"

# 현재 디렉토리에서 시작하여 하위 디렉토리 탐색
function process_directory() {
  local dir="$1"

  # 디렉토리 이름 변경
  if [[ -d "$dir" ]]; then
    new_dir=$(echo "$dir" | sed "s/ros2_node_template/$pkg_name/g")
    if [[ "$new_dir" != "$dir" ]]; then
      mv "$dir" "$new_dir"
      echo "Renamed directory: $dir -> $new_dir"
      dir="$new_dir" # 변경된 디렉토리 이름으로 업데이트
    fi

    # 하위 디렉토리 처리
    for subdir in "$dir"/*/; do
      process_directory "$subdir"
    done
  fi

  # 파일 이름 및 내용 변경
  for file in "$dir"/*; do
    if [[ -f "$file" ]]; then
      if [[ "$file" == "./convert_names.sh" ]]; then
        continue
      fi

      if [[ "$(basename "$file")" == "ros2_node_template.h" ]]; then
        cmd="sed -i "s/ROS2_NODE_TEMPLATE_H_/$upper_pkg_name/g" "$file""
        echo "found header file $(basename "$file")"
        echo $cmd
        sed -i "s/ROS2_NODE_TEMPLATE_H_/$upper_pkg_name/g" "$file"
      fi

      new_file=$(echo "$file" | sed "s/ros2_node_template/$pkg_name/g")
      if [[ "$new_file" != "$file" ]]; then
        mv "$file" "$new_file"
        echo "Renamed file: $file -> $new_file"
        file="$new_file" # 변경된 파일 이름으로 업데이트
      fi

      # 파일 내용 변경
      sed -i "s/ros2_node_template/$pkg_name/g" "$file"
      sed -i "s/node_name/$test_node/g" "$file"
      sed -i "s/NodeName/$class_name/g" "$file"
      echo "Updated contents of file: $file"
    fi
  done
}

cd ..
mv ros2_node_template $pkg_name
cd $pkg_name

process_directory "."
rm .git -rf

