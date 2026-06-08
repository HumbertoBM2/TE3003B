import glob
import os

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np

from .hmm import train_hmm_counts
from .mfcc import extract_mfcc
from .read_audio import read_wav
from .vq import quantize_mfcc


DATA_DIR = "data/train"
MODELS_DIR = "models"
OUTPUT_DIR = "reports/hmm_heatmaps"

WORDS_TO_PLOT = ["avanza", "alto", "derecha","izquierda","sube"]

N_STATES = 5
N_SYMBOLS = 256
EPSILON = 1e-6


def train_hmm_counts_without_smoothing(sequences, n_states=5, n_symbols=256):
    pi = np.zeros(n_states)
    pi[0] = 1.0

    B_counts = np.zeros((n_states, n_symbols))
    durations = [[] for _ in range(n_states)]

    for observations in sequences:
        total_frames = len(observations)
        boundaries = np.linspace(0, total_frames, n_states + 1).astype(int)

        for state in range(n_states):
            start = boundaries[state]
            end = boundaries[state + 1]
            segment = observations[start:end]

            durations[state].append(len(segment))

            for symbol in segment:
                B_counts[state, symbol] += 1

    B = np.zeros_like(B_counts)
    row_sums = B_counts.sum(axis=1, keepdims=True)
    nonzero_rows = row_sums[:, 0] > 0
    B[nonzero_rows] = B_counts[nonzero_rows] / row_sums[nonzero_rows]

    A = np.zeros((n_states, n_states))

    for state in range(n_states):
        if state == n_states - 1:
            A[state, state] = 1.0
        else:
            avg_duration = np.mean(durations[state])
            avg_duration = max(avg_duration, 1.01)
            A[state, state] = (avg_duration - 1) / avg_duration
            A[state, state + 1] = 1 / avg_duration

    A = A / A.sum(axis=1, keepdims=True)

    return {
        "A": A,
        "B": B,
        "pi": pi,
        "n_states": n_states,
        "n_symbols": n_symbols,
    }


def load_word_sequences(data_dir, word, codebook):
    pattern = os.path.join(data_dir, word, "*.wav")
    paths = sorted(glob.glob(pattern))

    if not paths:
        raise FileNotFoundError(f"No WAV files found for '{word}' in {pattern}")

    sequences = []

    for path in paths:
        audio, sample_rate = read_wav(path)
        mfcc = extract_mfcc(audio, sample_rate)
        observations = quantize_mfcc(mfcc, codebook)
        sequences.append(observations)

    return sequences


def load_final_hmm(models_dir, word):
    path = os.path.join(models_dir, f"{word}_hmm.npz")
    data = np.load(path)

    return {
        "A": data["A"],
        "B": data["B"],
        "pi": data["pi"],
        "n_states": int(data["n_states"]),
        "n_symbols": int(data["n_symbols"]),
    }


def build_snapshots(sequences, models_dir, word, n_states, n_symbols, epsilon):
    midpoint = max(1, len(sequences) // 2)

    return [
        (
            "Inicial",
            train_hmm_counts_without_smoothing(
                sequences[:1],
                n_states=n_states,
                n_symbols=n_symbols,
            ),
        ),
        (
            "Intermedia",
            train_hmm_counts_without_smoothing(
                sequences[:midpoint],
                n_states=n_states,
                n_symbols=n_symbols,
            ),
        ),
        (
            "Final",
            load_final_hmm(models_dir, word)
            if os.path.exists(os.path.join(models_dir, f"{word}_hmm.npz"))
            else train_hmm_counts(
                sequences,
                n_states=n_states,
                n_symbols=n_symbols,
                epsilon=epsilon,
            ),
        ),
    ]


def plot_word_heatmaps(word, snapshots, output_dir):
    fig, axes = plt.subplots(2, 3, figsize=(15, 6), constrained_layout=True)
    fig.suptitle(f"HMM heatmaps - {word}", fontsize=14)

    for col, (stage, hmm) in enumerate(snapshots):
        ax_a = axes[0, col]
        ax_b = axes[1, col]

        image_a = ax_a.imshow(hmm["A"], aspect="equal", cmap="viridis", vmin=0.0)
        ax_a.set_title(f"{stage} - A")
        ax_a.set_xlabel("Estado destino")
        ax_a.set_ylabel("Estado origen")
        ax_a.set_xticks(range(hmm["n_states"]))
        ax_a.set_yticks(range(hmm["n_states"]))
        fig.colorbar(image_a, ax=ax_a, fraction=0.046, pad=0.04)

        image_b = ax_b.imshow(hmm["B"], aspect="auto", cmap="magma", vmin=0.0)
        ax_b.set_title(f"{stage} - B")
        ax_b.set_xlabel("Indice VQ")
        ax_b.set_ylabel("Estado")
        ax_b.set_yticks(range(hmm["n_states"]))
        fig.colorbar(image_b, ax=ax_b, fraction=0.046, pad=0.04)

    os.makedirs(output_dir, exist_ok=True)
    output_path = os.path.join(output_dir, f"{word}_hmm_heatmaps.png")
    fig.savefig(output_path, dpi=180)
    plt.close(fig)

    return output_path


def main():
    codebook_path = os.path.join(MODELS_DIR, "codebook.npy")
    codebook = np.load(codebook_path)

    output_paths = []

    for word in WORDS_TO_PLOT:
        sequences = load_word_sequences(DATA_DIR, word, codebook)
        snapshots = build_snapshots(
            sequences,
            MODELS_DIR,
            word,
            N_STATES,
            N_SYMBOLS,
            EPSILON,
        )
        output_paths.append(plot_word_heatmaps(word, snapshots, OUTPUT_DIR))

    print("Heatmaps guardados:")
    for path in output_paths:
        print(path)


if __name__ == "__main__":
    main()
