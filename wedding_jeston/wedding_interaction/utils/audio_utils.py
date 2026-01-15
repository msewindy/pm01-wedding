
import pyaudio
import logging
import re

def find_audio_device(pa: pyaudio.PyAudio, device_id_param: int = -1, target_name: str = "USB Audio", logger=None):
    """
    Find audio device index.
    
    Args:
        pa: PyAudio instance
        device_id_param: User configured device ID. If >= 0, returns this index directly.
        target_name: Substring to search for in device name if device_id_param < 0.
        logger: Logger instance (optional)
        
    Returns:
        (index, info_dict) tuple. 
        If device not found or invalid, returns (None, None) or falls back to default if specified logic allows.
    """
    if logger is None:
        logger = logging.getLogger("AudioUtils")
        
    input_device_index = None
    device_info = None

    if device_id_param >= 0:
        try:
            input_device_index = device_id_param
            device_info = pa.get_device_info_by_index(input_device_index)
            logger.info(f"Using manual audio device index: {input_device_index} ({device_info.get('name')})")
        except Exception as e:
            logger.error(f"Invalid manual audio device index {device_id_param}: {e}")
            return None, None
    else:
        # Auto-detect target_name
        num_devices = pa.get_device_count()
        found = False
        for i in range(num_devices):
            try:
                info = pa.get_device_info_by_index(i)
                name = info.get('name', '')
                # Check for target name and input capability
                if target_name in name and info.get('maxInputChannels') > 0:
                    input_device_index = i
                    device_info = info
                    logger.info(f"Auto-detected '{target_name}' device: '{name}' at index {i}")
                    found = True
                    break
            except Exception:
                continue
        
        if not found:
            logger.warning(f"No '{target_name}' device found. Using system default.")
            try:
                default_info = pa.get_default_input_device_info()
                input_device_index = default_info['index']
                device_info = default_info
                logger.info(f"Using default input device: '{device_info.get('name')}' at index {input_device_index}")
            except Exception as e:
                 logger.error(f"Failed to get default input device: {e}")
                 return None, None

    return input_device_index, device_info

def get_alsa_card_index_from_info(device_info: dict) -> int:
    """
    Extract ALSA card index from PyAudio device info, if available.
    Typical name format: "USB Audio: - (hw:1,0)" -> Card 1
    
    Returns:
        Card index (int). Returns 0 if parsing fails (safe default).
    """
    if not device_info:
        return 0
    
    name = device_info.get('name', '')
    # Regex to find (hw:X,Y)
    match = re.search(r'\(hw:(\d+),', name)
    if match:
        try:
            return int(match.group(1))
        except ValueError:
            pass
            
    return 0
