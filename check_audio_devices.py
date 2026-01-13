import pyaudio

p = pyaudio.PyAudio()
print("Default Input Device Info:")
try:
    default_info = p.get_default_input_device_info()
    print(default_info)
except Exception as e:
    print(f"Error getting default input device: {e}")

print("\nAll Host APIs:")
for i in range(p.get_host_api_count()):
    print(p.get_host_api_info_by_index(i))

print("\nAll Input Devices:")
for i in range(p.get_device_count()):
    dev = p.get_device_info_by_index(i)
    if dev['maxInputChannels'] > 0:
        print(f"Index {i}: {dev['name']} (Channels: {dev['maxInputChannels']}, Rate: {dev['defaultSampleRate']})")

p.terminate()
