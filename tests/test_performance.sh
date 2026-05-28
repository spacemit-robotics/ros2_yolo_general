#!/bin/bash
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
set -eo pipefail
export PERCEPTION_MODULE=yolo_general
export PERCEPTION_NODE=yolo_general_node
export PERCEPTION_PACKAGES=yolo_general
export PERCEPTION_CONFIG_REL=config/yolov8.yaml
export PERCEPTION_TOPIC=/yolo_general/boxes
export PERCEPTION_IMAGE_TOPIC=/test/image_raw
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${SCRIPT_DIR}/vision_performance.sh"
