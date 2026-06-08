import os
import subprocess
import time

import numpy as np

from .read_audio import read_wav_int16
from .train import WORDS

# Config de grabacion
SAMP_RATE = 16000
CHANNELS = 1
DURATION = 2.0

OUTPUT_DIR = "data/test"
SAMPLES_PER_WORD = 1
DEVICE = "plughw:0,6"

WORD = "baja" # cambiar para palabra especifica
SAMPLE_INDEX = None # cambiar para indice especifico
LIST_DEVICES = False

def record_audio(path, duration, samp_rate, alsa_device):
    duration = int(round(duration))

    command = [
        "arecord",
        "-D", alsa_device,
        "-f", "S16_LE",
        "-r", str(samp_rate),
        "-c", str(CHANNELS),
        "-d", str(duration),
        path,
    ]

    print("Recording...")
    print(" ".join(command))
    subprocess.run(command, check=True)

def print_audio_level_from_file(path):
    audio, samp_rate = read_wav_int16(path)

    peak = int(np.max(np.abs(audio))) if len(audio) else 0
    rms = float(np.sqrt(np.mean(audio.astype(np.float64) ** 2))) if len(audio) else 0.0

    print(f"Audio level: peak={peak}, rms={rms:.1f}, sr={samp_rate}")
    
    # Checar que el audio no este en silencio
    if peak == 0:
        print("WARNING: recorded silence. Check microphone/input device.")
    elif peak < 500:
        print("WARNING: audio level is very low.")

def main():
    if WORD is not None and WORD not in WORDS:
        raise ValueError(f"Palabra no valida: {WORD}")

    if SAMPLE_INDEX is not None and WORD is None:
        raise ValueError("SAMPLE_INDEX requiere WORD")

    if LIST_DEVICES:
        subprocess.run(["arecord", "-l"], check=False)
        return

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    print("=== HMM Voice Dataset Recorder ===")
    print(f"Sample rate: {SAMP_RATE} Hz")
    print(f"Channels: {CHANNELS}")
    print(f"Duration: {DURATION} seconds")
    print(f"Samples per word: {SAMPLES_PER_WORD}")
    print(f"Output dir: {OUTPUT_DIR}")
    print(f"ALSA device: {DEVICE}")
    print()

    input("Press ENTER to start recording...")

    words_to_record = [WORD] if WORD else WORDS

    for word in words_to_record:
        word_dir = os.path.join(OUTPUT_DIR, word)
        os.makedirs(word_dir, exist_ok=True)

        print()
        print("=" * 40)
        print(f"WORD: {word.upper()}")
        print("=" * 40)
        input(f"Press ENTER when ready to record '{word}'...")

        if SAMPLE_INDEX is None:
            sample_indices = range(1, SAMPLES_PER_WORD + 1)
        else:
            sample_indices = [SAMPLE_INDEX]

        for i in sample_indices:
            filename = f"{word}_mj_{i:03d}.wav"
            path = os.path.join(word_dir, filename)

            print()
            print(f"Recording sample {i:03d} for word '{word}'")
            print(f"Say: {word}")
            for i in range(2, 0, -1):
                print(f"Starting in {i}...")
                time.sleep(1)

            record_audio(
                path=path,
                duration=DURATION,
                samp_rate=SAMP_RATE,
                alsa_device=DEVICE,
            )

            print_audio_level_from_file(path)
            print(f"Saved: {path}")
            time.sleep(0.5)

    print()
    print("Dataset recording complete.")


if __name__ == "__main__":
    main()
