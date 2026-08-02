#!/bin/bash

# ==========================================
# 打印带颜色的信息，方便观察执行过程
# ==========================================
print_info() { echo -e "\e[32m[INFO] $1\e[0m"; }
print_warn() { echo -e "\e[33m[WARN] $1\e[0m"; }

# ==========================================
# 步骤 1：处理 sensor_meta.pb.txt
# ==========================================
print_info "开始处理 Step 1: sensor_meta.pb.txt"
FILE1="/apollo_workspace/modules/perception/data/conf/sensor_meta.pb.txt"
LINK1="/apollo/modules/perception/data/conf/sensor_meta.pb.txt"

# 确保目录存在，不存在则自动创建文件
mkdir -p "$(dirname "$FILE1")"
touch "$FILE1"

# 检查文件中是否已经包含目标内容（以 "front_6mm" 为标志进行检查）
if ! grep -q "name: \"front_6mm\"" "$FILE1"; then
    print_info "向 $FILE1 追加 sensor_meta 节点..."
    cat <<EOF >> "$FILE1"

sensor_meta {
    name: "front_6mm"
    type: MONOCULAR_CAMERA
    orientation: FRONT
}
sensor_meta {
    name: "front_12mm"
    type: MONOCULAR_CAMERA
    orientation: FRONT
}
EOF
else
    print_info "节点已存在于 $FILE1，跳过追加。"
fi

# 删除旧链接（-f 强制删除，即使不存在也不会报错）并创建新链接
rm -f "$LINK1"
ln -s "$FILE1" "$LINK1"
print_info "软链接已建立: $LINK1 -> $FILE1"


# ==========================================
# 步骤 2：处理 radar_front_extrinsics.yaml
# ==========================================
print_info "开始处理 Step 2: radar_front_extrinsics.yaml"
FILE2="/apollo_workspace/modules/perception/data/params/radar_front_extrinsics.yaml"
LINK2="/apollo/modules/perception/data/params/radar_front_extrinsics.yaml"

mkdir -p "$(dirname "$FILE2")"

# 直接使用单尖括号 (>) 覆盖或创建新文件，写入内容
cat <<EOF > "$FILE2"
child_frame_id: radar_front
transform:
  translation:
    x: 2.0
    y: 0.0
    z: 2.0
  rotation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: novatel
EOF

rm -f "$LINK2"
ln -s "$FILE2" "$LINK2"
print_info "软链接已建立: $LINK2 -> $FILE2"


# ==========================================
# 步骤 3：处理 velodyne64_novatel_extrinsics.yaml
# ==========================================
print_info "开始处理 Step 3: velodyne64_novatel_extrinsics.yaml"
FILE3_SRC="/apollo_workspace/modules/drivers/lidar/velodyne/params/velodyne64_novatel_extrinsics_example.yaml"
FILE3_DST="/apollo_workspace/modules/drivers/lidar/velodyne/params/velodyne64_novatel_extrinsics.yaml"
LINK3="/apollo/modules/drivers/lidar/velodyne/params/velodyne64_novatel_extrinsics.yaml"

mkdir -p "$(dirname "$FILE3_DST")"

# 检查源样例文件是否存在，存在则复制，否则创建一个空的（防止报错）
if [ -f "$FILE3_SRC" ]; then
    cp "$FILE3_SRC" "$FILE3_DST"
    print_info "已成功复制 $FILE3_SRC 到 $FILE3_DST"
else
    print_warn "源文件 $FILE3_SRC 不存在，仅创建空的 $FILE3_DST"
    touch "$FILE3_DST"
fi

rm -f "$LINK3"
ln -s "$FILE3_DST" "$LINK3"
print_info "软链接已建立: $LINK3 -> $FILE3_DST"


# ==========================================
# 步骤 4：处理 static_transform_conf.pb.txt
# ==========================================
print_info "开始处理 Step 4: static_transform_conf.pb.txt"
FILE4="/apollo_workspace/modules/transform/conf/static_transform_conf.pb.txt"
LINK4="/apollo/modules/transform/conf/static_transform_conf.pb.txt"

mkdir -p "$(dirname "$FILE4")"

# 覆盖并写入新内容
cat <<EOF > "$FILE4"
extrinsic_file {
    frame_id: "novatel"
    child_frame_id: "velodyne64"
    file_path: "modules/drivers/lidar/velodyne/params/velodyne64_novatel_extrinsics.yaml"
    enable: true
}
extrinsic_file {
    frame_id: "localization"
    child_frame_id: "novatel"
    file_path: "modules/localization/msf/params/novatel_localization_extrinsics.yaml"
    enable: true
}
extrinsic_file {
    frame_id: "localization"
    child_frame_id: "imu"
    file_path: "modules/localization/msf/params/imu_localization_extrinsics.yaml"
    enable: true
}
extrinsic_file {
    frame_id: "velodyne64"
    child_frame_id: "front_6mm"
    file_path: "modules/perception/data/params/front_6mm_extrinsics.yaml"
    enable: true
}
extrinsic_file {
    frame_id: "velodyne64"
    child_frame_id: "front_12mm"
    file_path: "modules/perception/data/params/front_12mm_extrinsics.yaml"
    enable: true
}
extrinsic_file {
    frame_id: "novatel"
    child_frame_id: "radar_front"
    file_path: "modules/perception/data/params/radar_front_extrinsics.yaml"
    enable: true
}
EOF

rm -f "$LINK4"
ln -s "$FILE4" "$LINK4"
print_info "软链接已建立: $LINK4 -> $FILE4"

print_info "所有操作执行完毕！"
