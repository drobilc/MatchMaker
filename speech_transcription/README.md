## Speech transcription node

Note: this node has not been tested yet, it might work it might not.

This node handles speech recognition. When a request is received, the node starts listening on the microphone. When the user stops talking the node transcribes the speech and tries to understand it (albeit in a very simple way).

To use this node first install the dependencies

```shell
sudo apt-get install portaudio19-dev
pip2 install PyAudio==0.2.11
pip2 install SpeechRecognition

sudo apt install swig
pip2 install pocketsphinx
```