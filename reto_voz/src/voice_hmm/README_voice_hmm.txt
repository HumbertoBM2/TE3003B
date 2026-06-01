VOICE_HMM - Reconocimiento de comandos con HMM + VQ

Resumen breve
-------------
Este paquete ROS 2 reconoce palabras aisladas usando un pipeline hecho a mano:

WAV -> MFCC -> VQ/codebook -> HMM por palabra -> Forward log-space

El nodo principal graba audio con arecord, clasifica el comando y publica el
resultado en /voice/recognized_command. Si el audio esta muy bajo o el modelo no
tiene una decision clara, publica "ninguna".


Que pasarle a otra persona
--------------------------
Pasar la carpeta completa del paquete:

src/voice_hmm/

Debe incluir:

- package.xml
- setup.py
- setup.cfg
- resource/voice_hmm
- voice_hmm/
- launch/
- config/
- models/
- scripts/

La carpeta models/ es importante porque contiene el modelo ya entrenado:

- models/codebook.npy
- models/*_hmm.npz


Dependencias
------------
La otra computadora necesita:

- ROS 2 Humble
- Python 3
- numpy
- arecord / ALSA

En Ubuntu, arecord normalmente viene con:

sudo apt install alsa-utils


Compilar el paquete
-------------------
Desde la raiz del workspace:

cd ~/8vo/reto_voz
colcon build --packages-select voice_hmm


Ambiente recomendado
--------------------
En cada terminal que vaya a usar ROS:

cd ~/8vo/reto_voz
source install/voice_hmm/share/voice_hmm/scripts/voice_hmm_env.bash

Ese script deja ROS en modo local para que las terminales se vean entre si.


Probar microfono
----------------
Listar dispositivos:

arecord -l

En esta compu funciono:

plughw:0,6

Probar grabacion directa:

arecord -D plughw:0,6 -f S16_LE -r 16000 -c 1 -d 2 test_mic.wav
aplay test_mic.wav


Grabar mas dataset
------------------
Desde el paquete:

cd ~/8vo/reto_voz/src/voice_hmm

Grabar entrenamiento:

python3 -m voice_hmm.record_audio --output-dir data/train --samples-per-word 20 --duration 2 --alsa-device plughw:0,6

Grabar test:

python3 -m voice_hmm.record_audio --output-dir data/test --samples-per-word 5 --duration 2 --alsa-device plughw:0,6

Regrabar una sola muestra, ejemplo baja_007.wav:

python3 -m voice_hmm.record_audio --output-dir data/train --word baja --sample-index 7 --duration 2 --alsa-device plughw:0,6


Entrenar
--------
Desde el paquete:

cd ~/8vo/reto_voz/src/voice_hmm
python3 -m voice_hmm.train

Esto actualiza los archivos en models/.

Despues de entrenar, recompilar para que ROS instale los modelos nuevos:

cd ~/8vo/reto_voz
colcon build --packages-select voice_hmm


Probar modelo sin ROS
---------------------
Probar hablando en vivo:

cd ~/8vo/reto_voz/src/voice_hmm
python3 -m voice_hmm.test_model --live --alsa-device plughw:0,6 --duration 2

Probar un WAV:

python3 -m voice_hmm.test_model --wav data/train/baja/baja_007.wav

Evaluar carpeta de test:

python3 -m voice_hmm.test_model --data-dir data/test


Usar nodo ROS
-------------
Terminal 1:

cd ~/8vo/reto_voz
source install/voice_hmm/share/voice_hmm/scripts/voice_hmm_env.bash
ros2 launch voice_hmm launch_voice_hmm.py

Terminal 2, ver resultado:

cd ~/8vo/reto_voz
source install/voice_hmm/share/voice_hmm/scripts/voice_hmm_env.bash
ros2 topic echo /voice/recognized_command std_msgs/msg/String

Terminal 3, pedir una grabacion con ENTER:

cd ~/8vo/reto_voz
source install/voice_hmm/share/voice_hmm/scripts/voice_hmm_env.bash
ros2 run voice_hmm voice_trigger_node

Cada vez que se presiona ENTER en Terminal 3, el nodo graba 2 segundos,
reconoce la palabra y publica el resultado.


Topicos
-------
Entrada:

/voice/listen_flag        std_msgs/msg/Bool

Salida:

/voice/recognized_command std_msgs/msg/String
/voice/log_likelihoods    std_msgs/msg/String


Palabras actuales
-----------------
avanza
retrocede
derecha
izquierda
alto
empieza
sube
baja
gira
busca


Notas importantes
-----------------
- Si ros2 topic list no muestra los topicos, revisar que todas las terminales
  hayan hecho source del script voice_hmm_env.bash.
- Si el audio sale como "ninguna", revisar volumen del microfono o bajar un poco
  min_audio_peak en config/voice_hmm.yaml.
- Si se cambia el dataset o se reentrena, volver a ejecutar colcon build para
  instalar los modelos actualizados en install/.
