#!/usr/bin/env python3
"""
下载 MediaPipe 所需的模型文件
"""

import os
import urllib.request
import hashlib
from pathlib import Path


# 模型信息：(文件名, URL)
MODELS = {
    'face_landmarker.task': 
        'https://storage.googleapis.com/mediapipe-models/face_landmarker/face_landmarker/float16/1/face_landmarker.task',
    'blaze_face_short_range.tflite':
        'https://storage.googleapis.com/mediapipe-models/face_detector/blaze_face_short_range/float16/1/blaze_face_short_range.tflite',
}


def get_model_dir() -> Path:
    """获取模型目录"""
    model_dir = Path.home() / '.mediapipe' / 'models'
    model_dir.mkdir(parents=True, exist_ok=True)
    return model_dir


def download_file(url: str, dest: Path, retries: int = 3) -> bool:
    """下载文件，支持重试"""
    for attempt in range(retries):
        try:
            print(f"  尝试 {attempt + 1}/{retries}: 下载 {url}")
            
            # 添加 headers 避免某些服务器拒绝
            request = urllib.request.Request(url, headers={
                'User-Agent': 'Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36'
            })
            
            with urllib.request.urlopen(request, timeout=60) as response:
                data = response.read()
                
            # 检查数据完整性
            if len(data) < 1000:
                print(f"  ⚠️ 下载的数据太小 ({len(data)} bytes)，可能不完整")
                continue
            
            # 写入文件
            with open(dest, 'wb') as f:
                f.write(data)
            
            print(f"  ✅ 下载成功: {dest.name} ({len(data) / 1024 / 1024:.2f} MB)")
            return True
            
        except Exception as e:
            print(f"  ❌ 下载失败: {e}")
            if attempt < retries - 1:
                import time
                time.sleep(2)
    
    return False


def main():
    print("=" * 60)
    print("MediaPipe 模型下载工具")
    print("=" * 60)
    
    model_dir = get_model_dir()
    print(f"\n模型目录: {model_dir}\n")
    
    success_count = 0
    failed_models = []
    
    for filename, url in MODELS.items():
        dest = model_dir / filename
        
        if dest.exists():
            size = dest.stat().st_size
            if size > 1000:  # 文件有效
                print(f"✅ {filename} 已存在 ({size / 1024 / 1024:.2f} MB)")
                success_count += 1
                continue
            else:
                print(f"⚠️ {filename} 存在但可能损坏，重新下载...")
                dest.unlink()
        
        print(f"\n📥 下载 {filename}...")
        if download_file(url, dest):
            success_count += 1
        else:
            failed_models.append(filename)
    
    print("\n" + "=" * 60)
    print(f"完成: {success_count}/{len(MODELS)} 个模型")
    
    if failed_models:
        print(f"\n❌ 失败的模型:")
        for m in failed_models:
            print(f"   - {m}")
        print("\n请手动下载这些模型到:")
        print(f"   {model_dir}")
    else:
        print("\n✅ 所有模型下载成功！")
    
    print("=" * 60)


if __name__ == '__main__':
    main()
