import whisper
import sounddevice as sd
import numpy as np
import scipy.io.wavfile as wavfile
import tempfile
import os
import time
import subprocess
import asyncio
import edge_tts
from playsound import playsound
import os
import sys

async def play_chinese_tts(text, voice="zh-CN-XiaoyiNeural", output_filename="output.mp3"):
    """
    使用 edge-tts 生成中文语音并播放
    :param text: 要转换为语音的中文文本
    :param voice: 使用的语音模型 (例如: zh-CN-XiaoyiNeural, zh-CN-XiaohanNeural 等)
    :param output_filename: 生成的音频文件名
    """
    communicate = edge_tts.Communicate(text, voice)
    await communicate.save(output_filename)

    # print(f"生成音频文件: {output_filename}")

    try:
        playsound(output_filename)
        # print("播放完成")
    except Exception as e:
        pass
        # print(f"播放音频时发生错误: {e}")
    finally:
        # 可以选择删除生成的音频文件
        # os.remove(output_filename)
        pass

def play_tts(txt:str):
    chinese_text = "你好，这是一个中文语音合成的测试。Hello, this is a chinese tts"
    # 你可以在这里尝试不同的中文语音模型
    voice_model = "zh-CN-XiaoyiNeural" # 女声
    # voice_model = "zh-CN-XiaohanNeural" # 男声
    # voice_model = "zh-CN-YunxiNeural"  # 男声
    # voice_model = "zh-CN-YunzeNeural"  # 男声

    # 创建保存音频的目录 (如果不存在)
    output_dir = "tts_output"
    os.makedirs(output_dir, exist_ok=True)
    output_file = os.path.join(output_dir, "test_chinese.mp3")

    asyncio.run(play_chinese_tts(txt, voice=voice_model, output_filename=output_file))

def record(time):


    # 2. 配置录音参数
    fs = 16000  # 采样率 (Whisper 模型通常使用 16kHz)
    duration = time # 录音时长 (秒)
    channels = 1 # 声道数 (单声道即可)

    # 1. 加载 Whisper 模型
    # 确保 download_root 目录存在，并且有足够的空间下载模型
    model_name = "large-v2"
    download_root = '/mnt/DataDisk/MODELS'
    filename = "temp.wav"

    # print(f"Loading Whisper model '{model_name}' from '{download_root}'...")
    try:
        model = whisper.load_model(model_name, download_root=download_root)
        # print("Model loaded successfully.")
    except Exception as e:
        print(f"Error loading model: {e}")
        print("Please check the model name and download_root path, and ensure you have internet access for the first download.")
        exit()


    # 3. 录制音频
    try:
        # 使用 sounddevice 录制音频
        # cmd = ["arecord", "-D", "plughw:2,0","-r", str(fs),"-d", str(duration), "-f", "cd", filename]
        # subprocess.run(cmd)
        play_tts("开始录音")
        audio_data = sd.rec(int(duration * fs), samplerate=fs, channels=channels, dtype='float32')
        sd.wait()  # 等待录音完成
        wavfile.write(filename, fs,audio_data)
        # print("Recording finished.")
    except Exception as e:
        # print(f"Error during recording: {e}")
        # print("Please check your microphone setup and permissions.")
        exit()

    # print(f"Recording for {duration} seconds...")

    # 4. 将录制的音频保存为临时 WAV 文件 (Whisper 也可以直接处理 numpy array，但保存为文件更通用)
    # 或者直接处理 numpy array (更高效)
    # Option A: Save to a temporary file
    # with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp_file:
    #     temp_filename = tmp_file.name
    #     wavfile.write(temp_filename, fs, (audio_data * 32767).astype(np.int16)) # Convert to 16-bit PCM
    # print(f"Audio saved to temporary file: {temp_filename}")
    #
    # # 5. 使用 Whisper 模型进行转录 (从文件)
    # print("Transcribing audio from file...")
    # try:
    #     result = model.transcribe(temp_filename)
    #     transcribed_text = result["text"]
    #     print("Transcription:")
    #     print(transcribed_text)
    # except Exception as e:
    #     print(f"Error during transcription: {e}")
    # finally:
    #     # 清理临时文件
    #     if os.path.exists(temp_filename):
    #         os.remove(temp_filename)

    # Option B: Transcribe directly from numpy array (more efficient)
    # print("Transcribing audio from numpy array...")
    try:
        start_time = time.time()
        # Whisper's transcribe method can directly accept a numpy array of the audio
        result = model.transcribe(filename, language="zh")
        end_time = time.time()
        transcribed_text = result["text"]
        # print("Transcription:")
        print(transcribed_text)
        return transcribed_text
        # print(f"Transcription time: {end_time - start_time:.2f} seconds")
    except Exception as e:
        print(f"Error during transcription: {e}")
    return "识别失败"
    
'''
├─ 听觉感知类
│ ├─ 音频采集
│ ├─ 语音转文本
│ └─ 文本转语音
'''

# 音频采集
def audio_capture(time=10):
    print("Audio captured")
    question = record(time)
    return f"audio capture : {question}"

# 文本转语音
def text_to_speech(text):
     # 打印到speech.txt
    with open("speech.txt", "a") as f:
        f.write("speech:" + text + "\n")

    voice_model = "zh-CN-XiaoyiNeural" # 女声
    # voice_model = "zh-CN-XiaohanNeural" # 男声
    # voice_model = "zh-CN-YunxiNeural"  # 男声
    # voice_model = "zh-CN-YunzeNeural"  # 男声

    # 创建保存音频的目录 (如果不存在)
    output_dir = "tts_output"
    os.makedirs(output_dir, exist_ok=True)
    output_file = os.path.join(output_dir, "test_chinese.mp3")

    asyncio.run(play_chinese_tts(text, voice=voice_model, output_filename=output_file))
    
    # 保存到speech.txt
    with open("speech.txt", "a") as f:
        f.write("\n" + text + "\n")
    return "Playing audio over"



'''
├─ 听觉感知类
│ ├─ 音频采集
│ ├─ 语音转文本
│ └─ 文本转语音
'''
class AudioSensor():
    def __init__(self, audio_capture=audio_capture, audio_text_to_speech=text_to_speech):
        super().__init__()
        self.audio_capture = audio_capture
        self.text_to_speech = audio_text_to_speech
        

        self.tools_info = """
            - audio_capture(time: int) # 功能描述: 音频采集 参数描述: 时间(整数形式)
        """

        self.func_tools_list = [
            {
                "type": "function",
                "function": {
                    "name": "audio_capture",
                    "description": "Capture audio",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "audio_data": {
                                "type": "string",
                                "description": "The audio data to capture",
                            }
                        },
                        "required": ["audio_data"]
                    },
                }
            },
        ]

    # 获取AudioSensor类的tools信息 提供给function_call使用
    def get_func_tools_info(self):
        return self.tools_info
    def get_func_tools_list(self):
        return self.func_tools_list
    # 调用func_call函数来执行相应的功能
    def func_call(self, func_name, arguments):
        if func_name == "audio_capture":
            return self.capture_audio(arguments["time"])
        elif func_name == "audio_text_to_speech":
            return self.text_to_speech(arguments["text"])
        else:
            raise ValueError(f"Function {func_name} not recognized.")

    # 调试接口
    def capture_audio(self, time):
        return self.audio_capture(time)

    def text_to_speech(self, text):
        return self.text_to_speech(text)


if __name__ == "__main__":
    # 测试AudioSensor类
    audio_sensor = AudioSensor()
    
    # 测试音频采集
    audio_data = audio_sensor.capture_audio(5)
    print(f"Captured audio data: {audio_data}")
    
    # 测试语音转文本
    text = audio_sensor.speech_to_text("这是测试语言，请检查麦克风")
