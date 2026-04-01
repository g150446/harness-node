#!/usr/bin/env python3
"""
play_wav.py — Simple WAV file player for macOS

Usage:
    python3 play_wav.py [wav_file]

Default: test.wav in the project root
"""

import subprocess
import sys
from pathlib import Path


def get_wav_file(argv: list[str]) -> Path:
    """Get WAV file path from arguments or use default."""
    if len(argv) > 1:
        return Path(argv[1])
    # Default to test.wav in project root
    return Path(__file__).parent.parent / "test.wav"


def play_wav(file_path: Path) -> None:
    """Play WAV file using macOS afplay command."""
    if not file_path.exists():
        print(f"Error: File not found: {file_path}")
        sys.exit(1)

    print(f"Playing: {file_path}")
    subprocess.run(["afplay", str(file_path)], check=True)


def main():
    wav_file = get_wav_file(sys.argv)
    play_wav(wav_file)
    print("Playback complete.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted.")
    except subprocess.CalledProcessError as e:
        print(f"Error playing audio: {e}")
        sys.exit(1)
