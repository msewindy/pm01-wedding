#!/usr/bin/env python3
"""
预录音频生成脚本

使用此脚本批量生成婚礼机器人的语音文件。

【重要】为保证声音一致性，推荐统一使用 Piper TTS：
- 预录音频：开发电脑上用 Piper 生成
- 动态对话：Jetson 上用同一 Piper 模型实时生成
- 结果：机器人始终使用同一种声音

用法：
    # 【推荐】使用 piper（保证声音一致）
    python generate_audio.py --engine piper --model /path/to/zh_CN-huayan-medium.onnx
    
    # 使用 edge-tts（仅用于测试，声音与 piper 不同！）
    python generate_audio.py --engine edge
    
    # 列出所有需要生成的语音
    python generate_audio.py --list

安装 Piper：
    # Linux x86_64
    pip install piper-tts
    
    # 下载中文模型
    wget https://huggingface.co/rhasspy/piper-voices/resolve/main/zh/zh_CN/huayan/medium/zh_CN-huayan-medium.onnx
    wget https://huggingface.co/rhasspy/piper-voices/resolve/main/zh/zh_CN/huayan/medium/zh_CN-huayan-medium.onnx.json

【推荐】使用 Edge TTS 预生成固定语音（音质最好）：
    pip install edge-tts
    python generate_audio.py --engine edge --voice zh-CN-XiaoxiaoNeural
    
    然后将生成的 audio_resources/ 目录复制到 Jetson 上
"""

import argparse
import asyncio
import os
import sys
from pathlib import Path
from typing import Dict

# 调整 sys.path 以便导入模块
try:
    from wedding_interaction.audio.speech_resources import SPEECH_LIBRARY, DEFAULT_TTS_CONFIG
except ImportError:
    # 允许直接运行脚本
    sys.path.append(str(Path(__file__).parent.parent))
    from wedding_interaction.audio.speech_resources import SPEECH_LIBRARY, DEFAULT_TTS_CONFIG



def get_output_dir() -> Path:
    """获取音频输出目录"""
    script_dir = Path(__file__).parent.parent
    output_dir = script_dir / "audio_resources"
    output_dir.mkdir(exist_ok=True)
    return output_dir


async def generate_with_edge_tts(
    speech_id: str,
    text: str,
    output_dir: Path,
    voice: str = DEFAULT_TTS_CONFIG["voice"],
    rate: str = DEFAULT_TTS_CONFIG["rate"],
) -> bool:
    """
    使用 edge-tts 生成音频
    
    需要安装：pip install edge-tts
    可用声音：
    - zh-CN-XiaoxiaoNeural (女声，自然)
    - zh-CN-XiaoyiNeural (女声，活泼)
    - zh-CN-YunjianNeural (男声，新闻)
    - zh-CN-YunxiNeural (男声，活泼)
    """
    try:
        import edge_tts
    except ImportError:
        print("请安装 edge-tts: pip install edge-tts")
        return False
    
    output_file = output_dir / f"{speech_id}.mp3"
    
    try:
        communicate = edge_tts.Communicate(text, voice, rate=rate)
        await communicate.save(str(output_file))
        print(f"✓ {speech_id}: {text[:20]}... -> {output_file.name}")
        return True
    except Exception as e:
        print(f"✗ {speech_id}: {e}")
        return False


def generate_with_piper(
    speech_id: str,
    text: str,
    output_dir: Path,
    model: str,
    speaker_id: int = 0,
    length_scale: float = 1.0,
) -> bool:
    """
    使用 Piper TTS 生成音频（Python API）
    
    【推荐】使用此方法保证声音一致性
    
    安装：
        pip install piper-tts
        
    下载中文模型：
        # huayan 模型（女声，推荐）
        wget https://huggingface.co/rhasspy/piper-voices/resolve/main/zh/zh_CN/huayan/medium/zh_CN-huayan-medium.onnx
        wget https://huggingface.co/rhasspy/piper-voices/resolve/main/zh/zh_CN/huayan/medium/zh_CN-huayan-medium.onnx.json
    
    Args:
        speech_id: 语音 ID
        text: 要合成的文本
        output_dir: 输出目录
        model: 模型文件路径 (.onnx)
        speaker_id: 说话人 ID（多说话人模型时使用）
        length_scale: 语速控制（<1.0 加快，>1.0 减慢）
    """
    import wave
    import struct
    
    # 检查 piper-tts 是否可用
    try:
        from piper import PiperVoice
        from piper.config import SynthesisConfig
    except ImportError:
        print("✗ piper-tts 未安装！请运行: pip install piper-tts")
        return False
    
    # 检查模型文件
    model_path = os.path.expanduser(model)
    if not os.path.exists(model_path):
        print(f"✗ 模型文件不存在: {model_path}")
        print("  请下载: https://huggingface.co/rhasspy/piper-voices/tree/main/zh/zh_CN/huayan/medium")
        return False
    
    output_file = output_dir / f"{speech_id}.wav"
    
    try:
        # 使用全局缓存的 voice 对象（避免重复加载模型）
        global _piper_voice, _piper_model_path
        
        if '_piper_voice' not in globals() or _piper_model_path != model_path:
            print(f"  加载模型: {os.path.basename(model_path)}...")
            _piper_voice = PiperVoice.load(model_path)
            _piper_model_path = model_path
        
        voice = _piper_voice
        
        # 配置合成参数
        syn_config = SynthesisConfig(
            length_scale=length_scale,
        )
        
        # 生成音频 - synthesize 返回 AudioChunk 迭代器
        audio_chunks = []
        for chunk in voice.synthesize(text, syn_config):
            # AudioChunk 有 audio_int16_bytes 属性
            audio_chunks.append(chunk.audio_int16_bytes)
        
        if not audio_chunks:
            print(f"✗ {speech_id}: 未生成音频数据")
            return False
        
        # 合并音频数据
        audio_data = b''.join(audio_chunks)
        
        # 获取采样率（从模型配置中读取）
        sample_rate = voice.config.sample_rate
        
        # 写入 WAV 文件
        with wave.open(str(output_file), 'wb') as wav_file:
            wav_file.setnchannels(1)  # 单声道
            wav_file.setsampwidth(2)  # 16-bit
            wav_file.setframerate(sample_rate)
            wav_file.writeframes(audio_data)
        
        if output_file.exists():
            # 获取文件大小
            size_kb = output_file.stat().st_size / 1024
            print(f"✓ {speech_id}: {text[:20]}... -> {output_file.name} ({size_kb:.1f}KB)")
            return True
        else:
            print(f"✗ {speech_id}: 文件未生成")
            return False
            
    except Exception as e:
        print(f"✗ {speech_id}: {e}")
        import traceback
        traceback.print_exc()
        return False


async def main():
    parser = argparse.ArgumentParser(description="生成婚礼机器人预录音频")
    parser.add_argument(
        "--engine",
        choices=["edge", "piper"],
        default="edge",
        help="TTS 引擎 (default: edge)",
    )
    parser.add_argument(
        "--model",
        type=str,
        default=None,
        help="Piper 模型路径",
    )
    parser.add_argument(
        "--voice",
        type=str,
        default=DEFAULT_TTS_CONFIG["voice"],
        help=f"Edge TTS 声音 (default: {DEFAULT_TTS_CONFIG['voice']})",
    )
    parser.add_argument(
        "--rate",
        type=str,
        default=DEFAULT_TTS_CONFIG["rate"],
        help=f"语速调整 (default: {DEFAULT_TTS_CONFIG['rate'].replace('%', '%%')})",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="列出所有语音条目",
    )
    parser.add_argument(
        "--only",
        type=str,
        default=None,
        help="只生成指定 ID 的语音（逗号分隔）",
    )
    
    args = parser.parse_args()
    
    # 列出所有语音
    if args.list:
        print("\n=== 语音库 ===")
        for speech_id, text in SPEECH_LIBRARY.items():
            print(f"  {speech_id}: {text}")
        print(f"\n共 {len(SPEECH_LIBRARY)} 条语音")
        return
    
    output_dir = get_output_dir()
    print(f"\n输出目录: {output_dir}")
    print(f"TTS 引擎: {args.engine}")
    
    # 筛选要生成的语音
    if args.only:
        ids = [s.strip() for s in args.only.split(",")]
        speeches = {k: v for k, v in SPEECH_LIBRARY.items() if k in ids}
    else:
        speeches = SPEECH_LIBRARY
    
    print(f"待生成: {len(speeches)} 条\n")
    
    # 生成
    success = 0
    failed = 0
    
    for speech_id, text in speeches.items():
        if args.engine == "edge":
            ok = await generate_with_edge_tts(
                speech_id, text, output_dir,
                voice=args.voice,
                rate=args.rate,
            )
        elif args.engine == "piper":
            if not args.model:
                print("Piper 需要指定模型路径: --model xxx.onnx")
                return
            ok = generate_with_piper(speech_id, text, output_dir, args.model)
        
        if ok:
            success += 1
        else:
            failed += 1
    
    print(f"\n=== 完成 ===")
    print(f"成功: {success}, 失败: {failed}")
    print(f"音频文件位于: {output_dir}")


if __name__ == "__main__":
    asyncio.run(main())

