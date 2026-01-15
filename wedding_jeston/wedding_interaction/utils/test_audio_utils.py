
import unittest
from unittest.mock import MagicMock
import sys
import os

# Add current dir to path to import audio_utils
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import audio_utils
import pyaudio

class TestAudioUtils(unittest.TestCase):
    def test_find_audio_device_manual(self):
        pa = MagicMock()
        mock_info = {'name': 'Manual Device', 'index': 5}
        pa.get_device_info_by_index.return_value = mock_info
        
        idx, info = audio_utils.find_audio_device(pa, device_id_param=5)
        self.assertEqual(idx, 5)
        self.assertEqual(info, mock_info)
        
    def test_find_audio_device_auto_usb(self):
        pa = MagicMock()
        pa.get_device_count.return_value = 3
        
        # Device 0: System Default
        # Device 1: USB Audio
        # Device 2: Other
        
        def get_info(i):
            if i == 0: return {'name': 'HDA Intel', 'maxInputChannels': 2, 'index': 0}
            if i == 1: return {'name': 'USB Audio Device', 'maxInputChannels': 1, 'index': 1}
            if i == 2: return {'name': 'Other', 'maxInputChannels': 1, 'index': 2}
            return {}
            
        pa.get_device_info_by_index.side_effect = get_info
        
        idx, info = audio_utils.find_audio_device(pa, device_id_param=-1, target_name="USB Audio")
        self.assertEqual(idx, 1)
        self.assertEqual(info['name'], 'USB Audio Device')

    def test_find_audio_device_fallback(self):
        pa = MagicMock()
        pa.get_device_count.return_value = 1
        pa.get_device_info_by_index.return_value = {'name': 'HDA Intel', 'maxInputChannels': 2, 'index': 0}
        pa.get_default_input_device_info.return_value = {'name': 'HDA Intel', 'index': 0}
        
        # Searching for USB, but only HDA exists
        idx, info = audio_utils.find_audio_device(pa, device_id_param=-1, target_name="USB Audio")
        
        # Should fallback to default (0)
        self.assertEqual(idx, 0)
        self.assertEqual(info['name'], 'HDA Intel')

    def test_get_alsa_card_index(self):
        info1 = {'name': 'USB Audio Device (hw:1,0)'}
        self.assertEqual(audio_utils.get_alsa_card_index_from_info(info1), 1)
        
        info2 = {'name': 'HDA Intel (hw:0,0)'}
        self.assertEqual(audio_utils.get_alsa_card_index_from_info(info2), 0)
        
        info3 = {'name': 'Unknown Device'}
        self.assertEqual(audio_utils.get_alsa_card_index_from_info(info3), 0)

if __name__ == '__main__':
    unittest.main()
