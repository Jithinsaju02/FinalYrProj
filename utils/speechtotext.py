import whisper
import pyaudio
import numpy as np
import threading

import re  # ✅ Import regex for better replacements

# ✅ Add this dictionary inside speechtotext.py
common_mistakes = {
    "touring lab": "turing lab",
    "force lab": "foss lab",
    "false lab": "foss lab",
    "confess room": "conference room"
}

# Load Whisper model
model = whisper.load_model("base")

# Audio recording settings
RATE = 16000
CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RECORD_SECONDS = 5

audio = pyaudio.PyAudio()

# ✅ Common Mistakes Map (before sending to processing)
# common_mistakes = {
#     "force lab": "foss lab",
#     "4th lab": "foss lab",
#     "fourth lab": "foss lab",
#     "false lab": "foss lab",
#     "touring lab": "turing lab",
#     "confess room": "conference room"
# }


def correct_transcription(text):
    """Corrects common misinterpretations in speech-to-text results."""
    text = text.lower().strip()

    # ✅ Handle number-based errors
    text = re.sub(r"\b4th\b|\bfourth\b", "foss", text)  

    # ✅ Apply direct correction
    if text in common_mistakes:
        corrected = common_mistakes[text]
        print(f"🔄 Auto-corrected: {text} → {corrected}")
        return corrected

    return text  # Return corrected or original text


# def clean_transcription(text):
#     """Fix common transcription errors."""
#     text = text.lower().strip()
#     return common_mistakes.get(text, text)  # Auto-correct if needed

def record_and_transcribe(socketio, app):
    """Function to record audio and transcribe"""
    try:
        # Open audio stream
        stream = audio.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)
        frames = []

        print("Recording audio...")

        for _ in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
            data = stream.read(CHUNK, exception_on_overflow=False)
            frames.append(data)

        stream.stop_stream()
        stream.close()
        print("Finished recording. Transcribing...")

        # Convert to NumPy array
        audio_data = np.frombuffer(b"".join(frames), dtype=np.int16).astype(np.float32) / 32768.0
          # Transcribe using Whisper
        result = model.transcribe(audio_data, fp16=False)
        transcription = result["text"].lower().strip()
        print("Raw Transcription:", transcription)

        # ✅ Apply correction function
        corrected_transcription = correct_transcription(transcription)

        # Emit corrected transcription
        with app.app_context():
            socketio.emit("transcription_result", {"transcription": corrected_transcription})
    except Exception as e:
        print(f"Error in record_and_transcribe(): {e}")
