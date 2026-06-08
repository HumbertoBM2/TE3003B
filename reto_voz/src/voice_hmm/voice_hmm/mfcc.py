import numpy as np

def trim_silence_by_energy(
    x,
    sr,
    frame_ms=20,
    hop_ms=10,
    threshold_ratio=0.08,
    min_threshold=1e-5,
    padding_ms=80,
):  
    frame_len = int(sr * frame_ms / 1000)
    hop_len = int(sr * hop_ms / 1000)

    if len(x) < frame_len:
        return x

    n_frames = 1 + int((len(x) - frame_len) / hop_len)
    energies = np.zeros(n_frames)

    for i in range(n_frames):
        start = i * hop_len
        frame = x[start:start + frame_len]
        energies[i] = np.mean(frame ** 2)

    max_energy = np.max(energies)

    if max_energy <= min_threshold:
        return x

    threshold = max(min_threshold, threshold_ratio * max_energy)
    active = np.where(energies >= threshold)[0]

    if len(active) == 0:
        return x

    padding = int(sr * padding_ms / 1000)
    start = max(0, active[0] * hop_len - padding)
    end = min(len(x), active[-1] * hop_len + frame_len + padding)

    if end <= start:
        return x

    return x[start:end]


def pre_emphasis(x, alpha=0.97):
    return np.append(x[0], x[1:] - alpha * x[:-1])


def frame_signal(x, sr, frame_ms=25, hop_ms=10):
    frame_len = int(sr * frame_ms / 1000)
    hop_len = int(sr * hop_ms / 1000)

    if len(x) < frame_len:
        x = np.pad(x, (0, frame_len - len(x)))

    n_frames = 1 + int((len(x) - frame_len) / hop_len)

    frames = np.zeros((n_frames, frame_len))

    for i in range(n_frames):
        start = i * hop_len
        frames[i] = x[start:start + frame_len]

    return frames


def hamming_window(N):
    n = np.arange(N)
    return 0.54 - 0.46 * np.cos(2 * np.pi * n / (N - 1))


def hz_to_mel(hz):
    return 2595 * np.log10(1 + hz / 700)


def mel_to_hz(mel):
    return 700 * (10 ** (mel / 2595) - 1)


def mel_filterbank(sr, n_fft, n_filters=26):
    f_min = 0
    f_max = sr / 2

    mel_min = hz_to_mel(f_min)
    mel_max = hz_to_mel(f_max)

    mel_points = np.linspace(mel_min, mel_max, n_filters + 2)
    hz_points = mel_to_hz(mel_points)

    bins = np.floor((n_fft + 1) * hz_points / sr).astype(int)

    filters = np.zeros((n_filters, n_fft // 2 + 1))

    for m in range(1, n_filters + 1):
        left = bins[m - 1]
        center = bins[m]
        right = bins[m + 1]

        for k in range(left, center):
            filters[m - 1, k] = (k - left) / (center - left)

        for k in range(center, right):
            filters[m - 1, k] = (right - k) / (right - center)

    return filters


def dct_manual(x, n_mfcc=13):
    """
    x shape: (T, n_filters)
    returns: (T, n_mfcc)
    """
    T, N = x.shape
    result = np.zeros((T, n_mfcc))

    for k in range(n_mfcc):
        for n in range(N):
            result[:, k] += x[:, n] * np.cos(np.pi * k * (2 * n + 1) / (2 * N))

    return result


def extract_mfcc(
    x,
    sr,
    n_mfcc=13,
    n_filters=26,
    n_fft=512,
    trim_silence=True,
):
    if trim_silence:
        x = trim_silence_by_energy(x, sr)

    x = pre_emphasis(x)

    frames = frame_signal(x, sr)
    window = hamming_window(frames.shape[1])
    frames = frames * window

    spectrum = np.fft.rfft(frames, n=n_fft)
    power = (np.abs(spectrum) ** 2) / n_fft

    filters = mel_filterbank(sr, n_fft, n_filters)
    mel_energy = power @ filters.T

    mel_energy = np.maximum(mel_energy, 1e-12)
    log_mel = np.log(mel_energy)

    mfcc = dct_manual(log_mel, n_mfcc)
    mfcc = mfcc - np.mean(mfcc, axis=0)

    return mfcc
