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

Install the necessary Python packages:

```bash
pip install openai whisper sounddevice soundfile rospy

## Setup

OpenAI API Key: Insert your OpenAI API key in the script where api_key="" is located.
ROS Environment: Ensure that your ROS environment is correctly set up and that the ROS master is running.
Audio Device: Verify that your audio input device is properly configured for recording.
Usage

To run the script, navigate to the script's directory and execute:

```bash
python llm_hvi.py



