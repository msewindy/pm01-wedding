#!/bin/bash

echo "=== Jetson FFMPEG Hardware Encoder Test ==="
echo "Generating test input (test_src.mp4)..."
ffmpeg -y -f lavfi -i testsrc=duration=5:size=1280x720:rate=30 -pix_fmt yuv420p test_src.mp4 > /dev/null 2>&1

function test_encoder() {
    ENC_NAME=$1
    PIX_FMT=$2
    OUT_FILE="test_${ENC_NAME}_${PIX_FMT}.mp4"
    
    echo "----------------------------------------"
    echo "Testing encoder: $ENC_NAME (pix_fmt: $PIX_FMT)"
    
    # Run ffmpeg command
    ffmpeg -y -i test_src.mp4 -c:v $ENC_NAME -pix_fmt $PIX_FMT -b:v 4M $OUT_FILE > "${OUT_FILE}.log" 2>&1
    RET=$?
    
    if [ $RET -eq 0 ]; then
        echo "✅ SUCCESS! Output saved to $OUT_FILE"
        echo "Log tail:"
        tail -n 3 "${OUT_FILE}.log"
    else
        echo "❌ FAILED (Return Code: $RET)"
        echo "Error details:"
        grep -i "error" "${OUT_FILE}.log" | head -n 5
        grep -i "Could not" "${OUT_FILE}.log" | head -n 5
    fi
}

# Check available encoders
echo ""
echo "Checking available encoders..."
ffmpeg -encoders 2>/dev/null | grep -E "h264_v4l2m2m|h264_nvmpi|h264_omx|nvenc"

# Test 1: h264_v4l2m2m with yuv420p (Failed previously)
test_encoder "h264_v4l2m2m" "yuv420p"

# Test 2: h264_v4l2m2m with nv12 (Common requirement for V4L2)
test_encoder "h264_v4l2m2m" "nv12"

# Test 3: h264_nvmpi (If installed via jetson-ffmpeg)
test_encoder "h264_nvmpi" "yuv420p"

# Test 4: h264_omx (Legacy, might work)
test_encoder "h264_omx" "yuv420p"

echo "----------------------------------------"
echo "Tests completed. Please check which one succeeded."
