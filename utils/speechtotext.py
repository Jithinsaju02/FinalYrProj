import whisper
import pyaudio
import numpy as np
import wave
import time

# Load the Whisper model
print("Loading Whisper model...")
model = whisper.load_model("base")

# Audio recording settings
RATE = 16000  # Sampling rate
CHUNK = 1024  # Buffer size
FORMAT = pyaudio.paInt16
CHANNELS = 1
RECORD_SECONDS = 5  # Duration of each segment

# Initialize PyAudio
audio = pyaudio.PyAudio()

# Open a new stream for recording
stream = audio.open(
    format=FORMAT,
    channels=CHANNELS,
    rate=RATE,
    input=True,
    frames_per_buffer=CHUNK
)

print("Listening...")

# File to save transcriptions
transcript_file = "transcriptions.txt"

# Main loop for recording and transcribing
try:
    while True:
        print("Recording segment...")
        frames = []

        # Record audio segment
        for _ in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
            data = stream.read(CHUNK, exception_on_overflow=False)
            frames.append(data)

        print("Transcribing...")

        # Convert audio segment to NumPy array
        audio_data = np.frombuffer(b"".join(frames), dtype=np.int16)

        # Normalize audio data to float32 range (-1 to 1)
        audio_data = audio_data.astype(np.float32) / 32768.0

        # Convert to a writable array
        audio_data = np.copy(audio_data)

        # Transcribe using Whisper
        result = model.transcribe(audio_data, fp16=False)

        # Extract transcription
        transcription = result["text"]
        print("Transcription:", transcription)

        # Append transcription to the file
        with open(transcript_file, "a") as f:
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            f.write(f"[{timestamp}] {transcription}\n")

except KeyboardInterrupt:
    print("Stopped listening.")

finally:
    # Clean up resources
    print("Cleaning up...")
    stream.stop_stream()
    stream.close()
    audio.terminate()
    print("Transcriptions saved to:", transcript_file)
