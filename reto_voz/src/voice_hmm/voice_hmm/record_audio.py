import os
import wave
import time
import argparse
import subprocess
import numpy as np


SAMPLE_RATE = 16000
CHANNELS = 1
DURATION_SECONDS = 2.0

WORDS = [
    "avanza",
    "retrocede",
    "derecha",
    "izquierda",
    "alto",
    "empieza",
    "sube",
    "baja",
    "gira",
    "busca"
]

OUTPUT_DIR = "data/train"
SAMPLES_PER_WORD = 20

# En tu caso, por lo que salió en arecord -l:
# card 0, device 6: DMIC Raw
DEFAULT_ALSA_DEVICE = "plughw:0,6"


def record_audio_arecord(path, duration_seconds, sample_rate, alsa_device):
    """
    Record audio using ALSA arecord directly.

    Saves:
        16-bit PCM WAV
        mono
        sample_rate Hz
    """
    duration_seconds = int(round(duration_seconds))

    command = [
        "arecord",
        "-D", alsa_device,
        "-f", "S16_LE",
        "-r", str(sample_rate),
        "-c", str(CHANNELS),
        "-d", str(duration_seconds),
        path
    ]

    print("Recording with command:")
    print(" ".join(command))

    try:
        subprocess.run(command, check=True)
    except subprocess.CalledProcessError as e:
        raise RuntimeError(
            "arecord failed. Try another ALSA device, for example hw:0,6 or plughw:0,0"
        ) from e


def read_wav_int16(path):
    """
    Read a WAV file as int16 NumPy array for checking audio level.
    """
    with wave.open(path, "rb") as wf:
        channels = wf.getnchannels()
        sample_width = wf.getsampwidth()
        sample_rate = wf.getframerate()
        frames = wf.getnframes()
        raw = wf.readframes(frames)

    if sample_width != 2:
        raise ValueError("Expected 16-bit WAV.")

    audio = np.frombuffer(raw, dtype=np.int16)

    if channels == 2:
        audio = audio.reshape(-1, 2).mean(axis=1).astype(np.int16)

    return audio, sample_rate


def print_audio_level_from_file(path):
    audio, sr = read_wav_int16(path)

    peak = int(np.max(np.abs(audio))) if len(audio) else 0
    rms = float(np.sqrt(np.mean(audio.astype(np.float64) ** 2))) if len(audio) else 0.0

    print(f"Audio level: peak={peak}, rms={rms:.1f}, sr={sr}")

    if peak == 0:
        print("WARNING: recorded silence. Check microphone/input device before continuing.")
    elif peak < 500:
        print("WARNING: audio level is very low. Try speaking closer or checking input gain.")


def countdown(seconds=3):
    for i in range(seconds, 0, -1):
        print(f"Starting in {i}...")
        time.sleep(1)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Record a WAV dataset for the HMM + VQ isolated-word recognizer using ALSA arecord."
    )

    parser.add_argument(
        "--output-dir",
        default=OUTPUT_DIR,
        help="Destination directory, for example data/train or data/test.",
    )

    parser.add_argument(
        "--samples-per-word",
        type=int,
        default=SAMPLES_PER_WORD,
        help="Number of recordings to capture for each word.",
    )

    parser.add_argument(
        "--duration",
        type=float,
        default=DURATION_SECONDS,
        help="Duration in seconds for each recording. arecord works best with integer seconds.",
    )

    parser.add_argument(
        "--word",
        choices=WORDS,
        help="Record only one word.",
    )

    parser.add_argument(
        "--sample-index",
        type=int,
        help="Record only one numbered sample for --word, for example 7.",
    )

    parser.add_argument(
        "--alsa-device",
        default=DEFAULT_ALSA_DEVICE,
        help="ALSA input device. For your Dell internal mic, try plughw:0,6.",
    )

    parser.add_argument(
        "--list-alsa-devices",
        action="store_true",
        help="Run arecord -l and exit.",
    )

    return parser.parse_args()


def main():
    args = parse_args()

    if args.list_alsa_devices:
        subprocess.run(["arecord", "-l"], check=False)
        return

    if args.sample_index is not None and args.word is None:
        raise ValueError("--sample-index requires --word")

    os.makedirs(args.output_dir, exist_ok=True)

    print("=== HMM Voice Dataset Recorder using arecord ===")
    print(f"Sample rate: {SAMPLE_RATE} Hz")
    print(f"Channels: {CHANNELS}")
    print(f"Duration per recording: {args.duration} seconds")
    print(f"Samples per word: {args.samples_per_word}")
    print(f"Output directory: {args.output_dir}")
    print(f"ALSA device: {args.alsa_device}")
    print()

    input("Press ENTER to start recording the dataset...")

    words_to_record = [args.word] if args.word else WORDS

    for word in words_to_record:
        word_dir = os.path.join(args.output_dir, word)
        os.makedirs(word_dir, exist_ok=True)

        print()
        print("=" * 50)
        print(f"WORD: {word.upper()}")
        print("=" * 50)

        input(f"Press ENTER when ready to record '{word}'...")

        if args.sample_index is None:
            sample_indices = range(1, args.samples_per_word + 1)
        else:
            sample_indices = [args.sample_index]

        for i in sample_indices:
            filename = f"{word}_{i:03d}.wav"
            path = os.path.join(word_dir, filename)

            print()
            if args.sample_index is None:
                print(f"Recording {i}/{args.samples_per_word} for word '{word}'")
            else:
                print(f"Recording sample {i:03d} for word '{word}'")

            print(f"Say: {word}")

            countdown(2)

            record_audio_arecord(
                path=path,
                duration_seconds=args.duration,
                sample_rate=SAMPLE_RATE,
                alsa_device=args.alsa_device
            )

            print_audio_level_from_file(path)

            print(f"Saved: {path}")

            time.sleep(0.5)

    print()
    print("Dataset recording complete!")


if __name__ == "__main__":
    main()