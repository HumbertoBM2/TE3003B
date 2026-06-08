import os
import numpy as np

from .mfcc import extract_mfcc
from .vq import quantize_mfcc
from .hmm import forward_log


class HMMVoiceRecognizer:
    def __init__(self, models_dir, words):
        self.models_dir = models_dir
        self.words = words

        codebook_path = os.path.join(models_dir, "codebook.npy")
        self.codebook = np.load(codebook_path)

        self.hmms = {}
        self.last_num_frames = 0

        for word in words:
            hmm_path = os.path.join(models_dir, f"{word}_hmm.npz")
            data = np.load(hmm_path)

            self.hmms[word] = {
                "A": data["A"],
                "B": data["B"],
                "pi": data["pi"],
                "n_states": int(data["n_states"]),
                "n_symbols": int(data["n_symbols"])
            }

    def predict(self, audio, sample_rate):
        """
        audio: np.array float64 in range [-1, 1]
        sample_rate: int
        """

        if len(audio) == 0:
            self.last_num_frames = 0
            return None, {}

        mfcc = extract_mfcc(audio, sample_rate)

        if len(mfcc) == 0:
            self.last_num_frames = 0
            return None, {}

        observations = quantize_mfcc(mfcc, self.codebook)
        self.last_num_frames = len(observations)

        scores = {}

        for word, hmm in self.hmms.items():
            scores[word] = forward_log(observations, hmm)

        best_word = max(scores, key=scores.get)

        return best_word, scores
