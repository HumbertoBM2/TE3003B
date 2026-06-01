import argparse
import glob
import os
import time

import numpy as np

from .read_audio import read_wav
from .record_audio import (
    DEFAULT_ALSA_DEVICE,
    SAMPLE_RATE,
    print_audio_level_from_file,
    record_audio_arecord,
)
from .recognizer_core import HMMVoiceRecognizer
from .train import WORDS


def predict_file(recognizer, path):
    audio, sample_rate = read_wav(path)
    best_word, scores = recognizer.predict(audio, sample_rate)

    sorted_scores = sorted(scores.items(), key=lambda item: item[1], reverse=True)
    scores_text = ", ".join(f"{word}:{score:.3f}" for word, score in sorted_scores)

    print(f"{path}")
    print(f"  reconocido: {best_word}")
    print(f"  scores: {scores_text}")

    return best_word


def evaluate_directory(recognizer, data_dir):
    confusion = np.zeros((len(WORDS), len(WORDS)), dtype=int)
    word_to_index = {word: i for i, word in enumerate(WORDS)}
    total = 0
    correct = 0

    for true_word in WORDS:
        pattern = os.path.join(data_dir, true_word, "*.wav")
        paths = sorted(glob.glob(pattern))

        if not paths:
            print(f"Sin WAVs para {true_word}: {os.path.join(data_dir, true_word)}")
            continue

        for path in paths:
            predicted_word = predict_file(recognizer, path)
            total += 1

            if predicted_word == true_word:
                correct += 1

            if predicted_word in word_to_index:
                confusion[word_to_index[true_word], word_to_index[predicted_word]] += 1

    if total == 0:
        print("No se encontraron WAVs para evaluar.")
        return

    print()
    print(f"Accuracy: {correct}/{total} = {correct / total:.3f}")
    print()
    print("Matriz de confusion")
    print("filas=real, columnas=predicho")
    print(" " * 12 + " ".join(f"{word[:4]:>4}" for word in WORDS))

    for i, word in enumerate(WORDS):
        values = " ".join(f"{confusion[i, j]:4d}" for j in range(len(WORDS)))
        print(f"{word[:10]:>10}  {values}")


def predict_live(recognizer, path, duration, alsa_device):
    os.makedirs(os.path.dirname(path), exist_ok=True)

    print("Habla despues de la cuenta regresiva.")
    for i in range(2, 0, -1):
        print(f"Grabando en {i}...")
        time.sleep(1)

    record_audio_arecord(
        path=path,
        duration_seconds=duration,
        sample_rate=SAMPLE_RATE,
        alsa_device=alsa_device,
    )
    print_audio_level_from_file(path)
    return predict_file(recognizer, path)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Test trained HMM + VQ voice models on one WAV or a dataset directory."
    )
    parser.add_argument(
        "--models-dir",
        default="models",
        help="Directory with codebook.npy and *_hmm.npz files.",
    )
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--wav", help="Path to one WAV file to recognize.")
    group.add_argument(
        "--data-dir",
        help="Dataset directory with one subdirectory per word, for example data/test.",
    )
    group.add_argument(
        "--live",
        action="store_true",
        help="Record one live sample with arecord and recognize it.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=2.0,
        help="Duration in seconds for --live recording.",
    )
    parser.add_argument(
        "--alsa-device",
        default=DEFAULT_ALSA_DEVICE,
        help="ALSA input device for --live, for example plughw:0,6.",
    )
    parser.add_argument(
        "--live-wav",
        default="tmp/live_test.wav",
        help="Where to save the temporary live WAV.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    recognizer = HMMVoiceRecognizer(args.models_dir, WORDS)

    if args.wav:
        predict_file(recognizer, args.wav)
    elif args.data_dir:
        evaluate_directory(recognizer, args.data_dir)
    else:
        predict_live(recognizer, args.live_wav, args.duration, args.alsa_device)


if __name__ == "__main__":
    main()
