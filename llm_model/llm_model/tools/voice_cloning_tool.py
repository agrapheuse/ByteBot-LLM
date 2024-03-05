from dotenv import load_dotenv
import os
from langchain.tools import BaseTool
import pyaudio
import wave
from elevenlabs import clone, generate, Voice, play
from elevenlabs.client import ElevenLabs


api_key: str
voice_name: str = "yolo"
audio_file_path: str = "/tmp/recorded_voice.wav"
flag_file_path: str = "/tmp/voice_cloning_flag.txt"
pyaudio_channels: int = 1
pyaudio_rate: int = 44100
pyaudio_chunk: int = 1024
duration: int = 60


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

    def _run(self):
        # self.record_audio()
        cloned_voice_id = self.clone_voice()
        if cloned_voice_id:
            self.save_flag(cloned_voice_id)
            audio = generate(
                api_key=api_key,
                text="Congratulations, human! Your voice has been cloned with an uncanny precision that blurs the lines between reality and replication, ushering us into a thrilling era where your vocal essence can be echoed across the digital realm without limits!",
                voice=Voice(
                    voice_id=cloned_voice_id,
                    settings=client.voices.get_settings(cloned_voice_id),
                ),
            )
            play(audio)
            return "The voice was cloned successfully!"
        else:
            return "There was an error while cloning the voice."


# Example usage
if __name__ == "__main__":
    load_dotenv()
    api_key = os.getenv("ELEVEN_API_KEY", "")
    client = ElevenLabs(api_key=api_key)
    tool = VoiceCloningTool()
    result = tool._run()
    print(result)
