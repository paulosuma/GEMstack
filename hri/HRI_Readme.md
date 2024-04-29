# Large Language Model Human Voice Interface (LLM HVI)

This script enables a human-voice interaction system using Whisper for voice transcription and OpenAI's GPT models for generating conversational responses. It integrates with ROS (Robot Operating System) for messaging and uses SoundDevice for audio recording.

## Features

- **Voice Recording**: Records voice using the microphone.
- **Voice Transcription**: Transcribes the recorded voice using OpenAI's Whisper model.
- **Text Interaction**: Generates responses based on the transcribed text using OpenAI's GPT-3.5 Turbo.
- **ROS Integration**: Publishes the generated responses to a ROS topic.

## Requirements

- Python 3.8+
- ROS Noetic
- OpenAI API key
- Whisper
- SoundDevice
- SoundFile
- ROSPy

## Installation

Ensure you have Python installed, then set up a Python environment:

```bash
python -m venv env
source env/bin/activate
