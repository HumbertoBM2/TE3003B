import numpy as np

def train_hmm_counts(sequences, n_states=5, n_symbols=256, epsilon=1e-6):
    pi = np.zeros(n_states)
    pi[0] = 1.0

    B_counts = np.zeros((n_states, n_symbols))
    durations = [[] for _ in range(n_states)]

    for O in sequences:
        T = len(O)
        boundaries = np.linspace(0, T, n_states + 1).astype(int)

        for s in range(n_states):
            start = boundaries[s]
            end = boundaries[s + 1]
            segment = O[start:end]

            durations[s].append(len(segment))

            for symbol in segment:
                B_counts[s, symbol] += 1

    B = B_counts + epsilon
    B = B / B.sum(axis=1, keepdims=True)

    A = np.zeros((n_states, n_states))

    for s in range(n_states):
        if s == n_states - 1:
            A[s, s] = 1.0
        else:
            avg_duration = np.mean(durations[s])
            avg_duration = max(avg_duration, 1.01)

            A[s, s] = (avg_duration - 1) / avg_duration
            A[s, s + 1] = 1 / avg_duration

    A = A / A.sum(axis=1, keepdims=True)

    return {
        "A": A,
        "B": B,
        "pi": pi,
        "n_states": n_states,
        "n_symbols": n_symbols
    }


def logsumexp(values):
    m = np.max(values)

    if np.isneginf(m):
        return -np.inf

    return m + np.log(np.sum(np.exp(values - m)))


def safe_log(probabilities):
    result = np.full_like(probabilities, -np.inf, dtype=np.float64)
    positive = probabilities > 0
    result[positive] = np.log(probabilities[positive])
    return result


def forward_log(O, hmm):
    A = hmm["A"]
    B = hmm["B"]
    pi = hmm["pi"]
    N = hmm["n_states"]
    T = len(O)

    log_A = safe_log(A)
    log_B = safe_log(B)
    log_pi = safe_log(pi)

    log_alpha = np.full((T, N), -np.inf)

    log_alpha[0] = log_pi + log_B[:, O[0]]

    for t in range(1, T):
        for j in range(N):
            log_alpha[t, j] = log_B[j, O[t]] + logsumexp(
                log_alpha[t - 1] + log_A[:, j]
            )

    return logsumexp(log_alpha[-1])
