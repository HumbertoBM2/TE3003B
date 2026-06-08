from glob import glob
import os

from setuptools import setup


package_name = "voice_hmm"

setup(
    name=package_name,
    version="0.0.0",
    packages=["voice_hmm"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/voice_hmm"]),
        (os.path.join("share", package_name), ["package.xml", "README_voice_hmm.txt"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "models"), glob("models/*")),
        (os.path.join("share", package_name, "scripts"), glob("scripts/*.bash")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="your_name",
    maintainer_email="your_email@example.com",
    description="Isolated word recognition using manual MFCC VQ HMM",
    license="MIT",
    entry_points={
        "console_scripts": [
            "voice_recognition_node = voice_hmm.voice_recognition_node:main",
            "voice_trigger_node = voice_hmm.voice_trigger_node:main",
            "record_audio = voice_hmm.record_audio:main",
            "train_voice_hmm = voice_hmm.train:train_system",
            "test_voice_hmm = voice_hmm.test_model:main",
            "plot_hmm_heatmaps = voice_hmm.plot_hmm_heatmaps:main",
        ],
    },
)
