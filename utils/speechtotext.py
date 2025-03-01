from flask_socketio import SocketIO
import whisper
import pyaudio
import numpy as np
import threading
import wave
import time

socketio = SocketIO(cors_allowed_origins="*")  # Allow WebSocket connections

# Load Whisper model
model = whisper.load_model("base")

# Audio recording settings
RATE = 16000
CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RECORD_SECONDS = 5

audio = pyaudio.PyAudio()

def record_and_transcribe():
    """Function to record audio and transcribe"""
    global socketio  # Ensure socketio is recognized

    stream = audio.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)
    frames = []

    print("Recording...")

    for _ in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
        data = stream.read(CHUNK, exception_on_overflow=False)
        frames.append(data)

    print("Transcribing...")

    # Convert audio to NumPy array
    audio_data = np.frombuffer(b"".join(frames), dtype=np.int16).astype(np.float32) / 32768.0
    audio_data = np.copy(audio_data)

    # Transcribe with Whisper
    result = model.transcribe(audio_data, fp16=False)
    transcription = result.get("text", "")

    print("Transcription:", transcription)

    if transcription:  # Ensure there's a transcription before sending
        print("Emitting transcription result...")
        socketio.emit("transcription_result", {"transcription": transcription})

    stream.stop_stream()
    stream.close()

@socketio.on("start_recording")
def handle_voice_command():
    """Start recording when the frontend triggers it."""
    threading.Thread(target=record_and_transcribe).start()
