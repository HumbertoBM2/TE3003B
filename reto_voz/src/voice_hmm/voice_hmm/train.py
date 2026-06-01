# train.py
import os
import glob
import numpy as np

from .read_audio import read_wav
from .mfcc import extract_mfcc
from .vq import train_lbg, quantize_mfcc
from .hmm import train_hmm_counts

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

def collect_training_mfcc(data_dir):
    all_mfcc = []
    mfcc_by_word = {}

    for word in WORDS:
        mfcc_by_word[word] = []
        files = glob.glob(os.path.join(data_dir, word, "*.wav"))

        if not files:
            raise FileNotFoundError(
                f"No WAV files found for '{word}' in {os.path.join(data_dir, word)}"
            )

        for path in files:
            x, sr = read_wav(path)
            mfcc = extract_mfcc(x, sr)

            mfcc_by_word[word].append(mfcc)
            all_mfcc.append(mfcc)

    if not all_mfcc:
        raise FileNotFoundError(f"No training WAV files found in {data_dir}")

    X = np.vstack(all_mfcc)
    return X, mfcc_by_word


def train_system(data_dir="data/train"):
    os.makedirs("models", exist_ok=True)
    X, mfcc_by_word = collect_training_mfcc(data_dir)

    print("Training codebook...")
    codebook = train_lbg(X, target_size=256)

    np.save("models/codebook.npy", codebook)

    for word in WORDS:
        sequences = []

        for mfcc in mfcc_by_word[word]:
            O = quantize_mfcc(mfcc, codebook)
            sequences.append(O)

        hmm = train_hmm_counts(
            sequences,
            n_states=5,
            n_symbols=256,
            epsilon=1e-6
        )

        np.savez(
            f"models/{word}_hmm.npz",
            A=hmm["A"],
            B=hmm["B"],
            pi=hmm["pi"],
            n_states=hmm["n_states"],
            n_symbols=hmm["n_symbols"]
        )

        print("Saved:", word)


if __name__ == "__main__":
    train_system()
