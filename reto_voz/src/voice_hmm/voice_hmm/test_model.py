import glob
import os
import time

import numpy as np

from .read_audio import read_wav
from .record_audio import (
    DEVICE,
    SAMP_RATE,
    print_audio_level_from_file,
    record_audio,
)
from .recognizer_core import HMMVoiceRecognizer
from .train import WORDS


MODELS_DIR = "models"

# Cambiar entre wav, data o live
TEST_MODE = "data"

WAV_PATH = "data/test/baja/baja_m_007.wav"
DATA_DIR = "data/test"
LIVE_WAV_PATH = "tmp/live_test.wav"
LIVE_DURATION = 2.0
ALSA_DEVICE = DEVICE


def predict_file(recognizer, path):
    audio, samp_rate = read_wav(path)
    best_word, scores = recognizer.predict(audio, samp_rate)

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

    record_audio(
        path=path,
        duration=duration,
        samp_rate=SAMP_RATE,
        alsa_device=alsa_device,
    )
    print_audio_level_from_file(path)
    return predict_file(recognizer, path)


def main():
    recognizer = HMMVoiceRecognizer(MODELS_DIR, WORDS)

    if TEST_MODE == "wav":
        predict_file(recognizer, WAV_PATH)
    elif TEST_MODE == "data":
        evaluate_directory(recognizer, DATA_DIR)
    elif TEST_MODE == "live":
        predict_live(
            recognizer,
            LIVE_WAV_PATH,
            LIVE_DURATION,
            ALSA_DEVICE,
        )
    else:
        raise ValueError("TEST_MODE debe ser 'wav', 'data' o 'live'")


if __name__ == "__main__":
    main()
