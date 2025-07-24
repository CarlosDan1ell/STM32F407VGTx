import serial
import wave
import numpy as np
import os


# === CONFIGURAÇÕES ===
porta = 'COM15'  # Altere conforme seu caso
baudrate = 115200
tempo_segundos = 10  # tempo de gravação
amostragem = 16000  # mesma frequência usada no STM32
arquivo_saida = 'audio_saida.wav'

# === CONEXÃO SERIAL ===
ser = serial.Serial(porta, baudrate)
print(f"[INFO] Lendo dados da porta {porta} por {tempo_segundos}s...")

pcm_data = bytearray()

# === LEITURA ===
tempo_total = int(amostragem * tempo_segundos * 2)  # 2 bytes por int16_t

while len(pcm_data) < tempo_total:
    if ser.in_waiting:
        pcm_data += ser.read(ser.in_waiting)

ser.close()
print("[INFO] Captura finalizada.")

# === CONVERSÃO E SALVAMENTO ===
# Converte para array numpy tipo int16
samples = np.frombuffer(pcm_data, dtype=np.int16)

# Salva no formato .wav
with wave.open(arquivo_saida, 'wb') as wavfile:
    wavfile.setnchannels(1)        # Mono
    wavfile.setsampwidth(2)        # 2 bytes (16 bits)
    wavfile.setframerate(amostragem)
    wavfile.writeframes(samples.tobytes())

if os.path.exists(arquivo_saida):
    print(f"[DEBUG] Arquivo existe em: {os.path.abspath(arquivo_saida)}")
else:
    print("[ERRO] Arquivo .wav não foi criado!")
