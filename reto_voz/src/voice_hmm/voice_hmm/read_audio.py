
import wave
import numpy as np

def read_wav(path):
    with wave.open(path, "rb") as wf:
        s_rate = wf.getframerate()
        n_channels = wf.getnchannels()
        s_width = wf.getsampwidth()
        n_frames = wf.getnframes()
        raw = wf.readframes(n_frames)

    if s_width != 2:
        raise ValueError("Use 16-bit PCM WAV files.")

    audio = np.frombuffer(raw, dtype=np.int16).astype(np.float64)

    if n_channels == 2:
        audio = audio.reshape(-1, 2).mean(axis=1)

    audio = audio / 32768.0
    return audio, s_rate

