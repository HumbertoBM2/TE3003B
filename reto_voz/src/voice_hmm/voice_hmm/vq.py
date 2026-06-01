import numpy as np

def nearest_codewords(X, codebook):
    """
    X: (T, D)
    codebook: (K, D)
    returns indices: (T,)
    """
    distances = np.sum((X[:, None, :] - codebook[None, :, :]) ** 2, axis=2)
    return np.argmin(distances, axis=1)


def kmeans_refine(X, codebook, max_iter=50, tol=1e-4):
    prev_distortion = None

    for _ in range(max_iter):
        labels = nearest_codewords(X, codebook)

        new_codebook = np.copy(codebook)

        for k in range(len(codebook)):
            points = X[labels == k]
            if len(points) > 0:
                new_codebook[k] = np.mean(points, axis=0)

        codebook = new_codebook

        distortion = np.mean(np.sum((X - codebook[labels]) ** 2, axis=1))

        if prev_distortion is not None:
            if abs(prev_distortion - distortion) < tol:
                break

        prev_distortion = distortion

    return codebook


def train_lbg(X, target_size=256, epsilon=0.01):
    codebook = np.mean(X, axis=0, keepdims=True)

    while len(codebook) < target_size:
        codebook_plus = codebook * (1 + epsilon)
        codebook_minus = codebook * (1 - epsilon)

        codebook = np.vstack([codebook_plus, codebook_minus])
        codebook = kmeans_refine(X, codebook)

        print("Codebook size:", len(codebook))

    return codebook


def quantize_mfcc(mfcc, codebook):
    return nearest_codewords(mfcc, codebook)