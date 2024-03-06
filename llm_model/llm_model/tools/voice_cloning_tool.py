from dotenv import load_dotenv
import os
from langchain.tools import BaseTool
import pyaudio
import wave
from elevenlabs import clone, generate, Voice, play
from elevenlabs.client import ElevenLabs
from gtts import gTTS
from langchain.callbacks.manager import (
    AsyncCallbackManagerForToolRun,
    CallbackManagerForToolRun,
)
from typing import Optional, Union, Dict, Tuple

api_key: str
voice_name: str = "yolo"
audio_file_path: str = "/tmp/recorded_voice.wav"
flag_file_path: str = "/tmp/voice_cloning_flag.txt"
pyaudio_channels: int = 1
pyaudio_rate: int = 44100
pyaudio_chunk: int = 1024
duration: int = 60

api_key = "c099a9d746f0d3f8ad573e47223b85a8"
client = ElevenLabs(api_key=api_key)
class VoiceCloningTool(BaseTool):
    name = "voice_cloning"
    description = "Asks the participant to record a voice sample. It saves the sample in tmp and then clones the voice using the ElevenLabs Python SDK. It saves a flag to a file to indicate that another tool needs to use the new voice. When finished, it says something to confirm the process is complete."

    def __init__(self):
        super().__init__()

    def record_audio(self):
        # Record from the microphone
        p = pyaudio.PyAudio()
        stream = p.open(
            format=pyaudio.paInt16,
            channels=pyaudio_channels,
            rate=pyaudio_rate,
            input=True,
            frames_per_buffer=pyaudio_chunk,
        )
        frames = []

        print("Recording...")
        for _ in range(int(pyaudio_rate / pyaudio_chunk * duration)):
            data = stream.read(pyaudio_chunk)
            frames.append(data)
        print("Finished recording.")

        stream.stop_stream()
        stream.close()
        p.terminate()

        # Save the audio to a file
        with wave.open(audio_file_path, "wb") as wf:
            wf.setnchannels(pyaudio_channels)
            wf.setsampwidth(p.get_sample_size(pyaudio.paInt16))
            wf.setframerate(pyaudio_rate)
            wf.writeframes(b"".join(frames))

    def clone_voice(self) -> str:
        voice = clone(
            api_key=api_key,
            name=voice_name,
            files=[audio_file_path],
        )
        voice_id = voice.voice_id
        print(f"Cloned voice ID: {voice_id}")
        return voice_id

    def save_flag(self, cloned_voice_id: str):
        with open(flag_file_path, "w") as f:
            f.write(cloned_voice_id)

    def play_generic_tts(self, msg):
        tts = gTTS(text=msg, lang="en")
        tts_file = "/tmp/speech_output.mp3"
        tts.save(tts_file)
        os.system(f"mpv --audio-device=alsa/hw:1,0 {tts_file}")

    def _run(
        self,
        run_manager: Optional[CallbackManagerForToolRun] = None,
    ):
        self.play_generic_tts("Awaiting voice input")
        self.record_audio()
        cloned_voice_id = self.clone_voice()
        if cloned_voice_id:
            self.save_flag(cloned_voice_id)
            audio = generate(
                api_key=api_key,
                text="Congratulations, human! Your voice has been cloned with an uncanny precision that blurs the lines between reality and replication, ushering us into a thrilling era where your vocal essence can be echoed across the digital realm without limits!",
                voice=Voice(
                    api_key=api_key,
                    voice_id=cloned_voice_id,
                    settings=client.voices.get_settings(cloned_voice_id),
                ),
            )
            with open("/tmp/speech_output.mp3", "wb") as file:
                file.write(audio)
            os.system("mpv  --audio-device=alsa/hw:1,0 /tmp/speech_output.mp3")
            return "The voice was cloned successfully!"
        else:
            return "There was an error while cloning the voice."
        
    def _to_args_and_kwargs(self, tool_input: Union[str, Dict]) -> Tuple[Tuple, Dict]:
        return (), {}


# Example usage
if __name__ == "__main__":
    tool = VoiceCloningTool()
    result = tool._run()
    print(result)
