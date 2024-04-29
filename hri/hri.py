from openai import OpenAI
import rospy
from std_msgs.msg import String
import whisper
import sounddevice as sd
import io
import soundfile as sf
import json

rospy.init_node('LLM_HVI', anonymous=True)
pub = rospy.Publisher('/LLM_HVI', String, queue_size=10)

# Load API key from a configuration file
with open('config.json', 'r') as config_file:
    config = json.load(config_file)
    api_key = config['openai_api_key']

client = OpenAI(
    api_key=api_key,
)

def record_and_save_audio(file_path, duration=10, sample_rate=16000):
    """Record audio from the microphone and save it to a file."""
    print("Recording...")
    recording = sd.rec(int(duration * sample_rate), samplerate=sample_rate, channels=1, dtype='float32')
    sd.wait()
    print("Recording stopped.")
    
    # Save the recording to a WAV file
    sf.write(file_path, recording, sample_rate)
    print(f"Audio saved to {file_path}")


def transcribe_file(file_path, model_type='base'):
    """Transcribe the specified audio file using Whisper."""
    model = whisper.load_model(model_type)
    result = model.transcribe(file_path)
    return result['text']


def interactive_chat(last_output, audio_txt,prompt):
    # keyboard input
    human_input = input("Human command: "+  audio_txt)

    messages = [
                {
                    "role": "system",
                    "content": prompt},
                {"role": "assistant", "content": f'Pervious settings: {last_output}'},
                {"role": "user", "content": human_input},
            ]

    prediction = client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=messages,
            temperature=0.9,
            max_tokens=500,
            top_p=1,
            frequency_penalty=0,
            presence_penalty=0.6,
            # stop=[""]
            )
    
    reply = prediction.choices[0].message.content
    print(reply)
    pub.publish(String(reply))
    return reply

last_output = "None"
while(1):
    try:
        record_and_save_audio("output.wav")
        file_path = 'output.wav'  # Path to the audio file
        transcription = transcribe_file(file_path)
        prompt_file_path='./prompt/basic.txt'
        with open(prompt_file_path, 'r', encoding='utf-8') as file:
            prompt = file.read()
        last_output = interactive_chat(last_output,transcription ,prompt)
    except rospy.ROSInterruptException:
        exit()

    # try:
    #     last_output = interactive_chat(last_output)
    # except KeyboardInterrupt:
    #     break
