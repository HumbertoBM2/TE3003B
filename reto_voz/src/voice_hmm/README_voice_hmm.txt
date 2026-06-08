VOICE_HMM - Reconocimiento de comandos con HMM + VQ

Resumen breve
-------------
Este paquete ROS 2 reconoce palabras aisladas usando un pipeline hecho a mano:

WAV -> MFCC -> VQ/codebook -> HMM por palabra -> Forward log-space

El nodo principal graba audio con arecord, clasifica el comando y publica el
resultado en /voice/recognized_command. Si el audio esta muy bajo o el modelo no
tiene una decision clara, publica "ninguna".


Pipeline completo de entrenamiento y reconocimiento
---------------------------------------------------
El sistema se entrena como un reconocedor de palabras aisladas. Cada comando
tiene varios WAVs de ejemplo, y al final del entrenamiento quedan dos tipos de
archivos en models/:

- codebook.npy: diccionario global de VQ usado por todas las palabras.
- <palabra>_hmm.npz: un HMM entrenado para una palabra especifica.

La idea completa es:

1. Grabar dataset

   Los audios viven en carpetas separadas por palabra:

   data/train/avanza/*.wav
   data/train/retrocede/*.wav
   data/train/derecha/*.wav
   ...

   El script voice_hmm.record_audio graba WAVs mono, 16-bit PCM, a 16000 Hz.
   Por defecto graba 20 muestras por palabra y las guarda como:

   data/train/<palabra>/<palabra>_001.wav
   data/train/<palabra>/<palabra>_002.wav
   ...

   El nombre de la carpeta es importante porque train.py usa esas carpetas para
   saber la etiqueta real de cada audio.

2. Leer WAVs

   voice_hmm.read_audio.read_wav abre cada archivo WAV y valida que sea 16-bit
   PCM. Si el WAV tiene dos canales, los mezcla a mono promediando ambos
   canales. Despues convierte las muestras int16 al rango flotante [-1, 1].

   Salida de esta etapa:

   audio: arreglo NumPy de muestras de audio
   sample_rate: frecuencia de muestreo del WAV

3. Extraer MFCCs

   voice_hmm.mfcc.extract_mfcc convierte la senal de audio en una secuencia de
   vectores acusticos. Cada vector describe un pedacito corto del audio.

   Pasos internos:

   - Recorte de silencio: trim_silence_by_energy busca las regiones con energia
     suficiente y conserva un poco de padding antes y despues. Esto ayuda a que
     el HMM aprenda la palabra, no el silencio alrededor.
   - Pre-enfasis: pre_emphasis aplica x[t] - 0.97*x[t-1] para resaltar cambios
     rapidos de la senal.
   - Ventaneo: frame_signal divide el audio en ventanas de 25 ms con salto de
     10 ms. Cada ventana representa un instante acustico.
   - Hamming: cada frame se multiplica por una ventana Hamming para reducir
     artefactos en la FFT.
   - Espectro de potencia: se calcula rfft con n_fft=512 y luego potencia.
   - Banco Mel: mel_filterbank usa 26 filtros en escala Mel para aproximar como
     percibimos frecuencias de voz.
   - Log energia: se aplica log a las energias Mel.
   - DCT: dct_manual reduce esas 26 energias a 13 coeficientes MFCC.
   - Normalizacion: se resta la media por coeficiente para reducir diferencias
     de volumen/canal entre grabaciones.

   Salida de esta etapa:

   mfcc: matriz de forma (T, 13)

   T cambia segun la duracion util de la palabra. Una palabra mas larga produce
   mas frames.

4. Construir el codebook VQ

   voice_hmm.train.collect_training_mfcc junta los MFCCs de todas las palabras
   en una sola matriz grande X. Luego voice_hmm.vq.train_lbg entrena un
   codebook de 256 centroides usando LBG:

   - Empieza con un solo centroide: la media de todos los MFCCs.
   - Duplica cada centroide con una pequena perturbacion: centro*(1+epsilon) y
     centro*(1-epsilon).
   - Refina los centroides con k-means.
   - Repite hasta llegar a 256 centroides.

   Este codebook es global. No pertenece a una sola palabra; define el
   vocabulario acustico comun que despues usan todos los HMMs.

   Archivo generado:

   models/codebook.npy

5. Cuantizar MFCCs a simbolos discretos

   Los HMMs implementados en este paquete trabajan con observaciones discretas,
   no con vectores continuos. Por eso cada vector MFCC se convierte al indice
   del centroide mas cercano del codebook:

   MFCC frame -> indice 0..255

   voice_hmm.vq.quantize_mfcc hace esta conversion con distancia euclidiana al
   cuadrado. El resultado para un audio es una secuencia como:

   [12, 12, 87, 44, 44, 201, ...]

   Esta secuencia se llama O en el codigo.

6. Entrenar un HMM por palabra

   voice_hmm.hmm.train_hmm_counts entrena un HMM independiente para cada
   comando. Por ejemplo:

   models/avanza_hmm.npz
   models/alto_hmm.npz
   models/busca_hmm.npz

   Cada HMM tiene:

   - pi: probabilidad inicial. Siempre empieza en el estado 0.
   - A: matriz de transicion entre estados.
   - B: matriz de emision, o probabilidad de observar cada simbolo VQ en cada
     estado.
   - n_states: numero de estados, actualmente 5.
   - n_symbols: numero de simbolos, actualmente 256.

   La topologia es izquierda-a-derecha. Eso significa que el modelo representa
   la palabra como una progresion temporal:

   estado 0 -> estado 1 -> estado 2 -> estado 3 -> estado 4

   Un estado puede quedarse en si mismo o avanzar al siguiente. El ultimo estado
   se queda en si mismo. Esta estructura encaja con palabras habladas porque una
   palabra avanza en el tiempo y no deberia regresar a sonidos anteriores.

   Como se estiman B y A:

   - Cada secuencia de entrenamiento se parte en 5 segmentos iguales.
   - Los simbolos VQ del primer segmento cuentan para el estado 0, los del
     segundo para el estado 1, etc.
   - B se calcula contando cuantas veces aparece cada simbolo en cada estado.
   - Se suma epsilon=1e-6 para suavizado, evitando probabilidades exactamente 0.
   - A se calcula a partir de la duracion promedio de cada segmento. Si un
     estado dura mas frames, recibe mas probabilidad de quedarse en si mismo; si
     dura menos, recibe mas probabilidad de avanzar.

   Este entrenamiento no usa Baum-Welch. Es un entrenamiento por conteos y
   alineacion uniforme, suficiente para un reconocedor pequeno de comandos
   aislados.

7. Guardar los modelos

   train.py guarda:

   models/codebook.npy
   models/avanza_hmm.npz
   models/retrocede_hmm.npz
   models/derecha_hmm.npz
   models/izquierda_hmm.npz
   models/alto_hmm.npz
   models/empieza_hmm.npz
   models/sube_hmm.npz
   models/baja_hmm.npz
   models/gira_hmm.npz
   models/busca_hmm.npz

   Estos archivos son el reconocedor entrenado. Sin ellos, el nodo ROS no tiene
   codebook ni HMMs que cargar.

8. Construir el reconocedor en memoria

   voice_hmm.recognizer_core.HMMVoiceRecognizer es la clase que arma el
   reconocedor para inferencia. Al inicializarse recibe:

   - models_dir: carpeta donde estan codebook.npy y los *_hmm.npz.
   - words: lista de palabras que debe cargar.

   En __init__ carga:

   - models/codebook.npy
   - models/<palabra>_hmm.npz para cada palabra configurada

   Despues deja todo en memoria:

   - self.codebook
   - self.hmms[palabra]

   En predict(audio, sample_rate), el reconocedor repite el mismo preproceso que
   se uso en entrenamiento:

   audio -> MFCC -> VQ -> secuencia de simbolos

   Luego evalua esa misma secuencia contra todos los HMMs. Para cada palabra
   llama voice_hmm.hmm.forward_log, que calcula el log-likelihood:

   score(palabra) = log P(observaciones | HMM de esa palabra)

   El mejor comando es la palabra con mayor score.

9. Rechazo de comandos dudosos

   El nodo ROS no publica siempre la mejor palabra. Primero revisa:

   - min_audio_peak: si el audio esta muy bajo, publica reject_word.
   - min_score_margin: si la diferencia entre el mejor score y el segundo mejor
     score es pequena, publica reject_word.

   En config/voice_hmm.yaml, reject_word es "ninguna".

   Esto evita que ruido o pronunciaciones poco claras se conviertan en comandos
   falsos.

10. Instalar modelos para ROS

   setup.py incluye models/* como archivos instalables del paquete. Por eso,
   despues de entrenar, hay que volver a compilar con colcon build. Asi los
   nuevos modelos pasan de:

   src/voice_hmm/models/

   a:

   install/voice_hmm/share/voice_hmm/models/

   El nodo ROS carga por defecto desde la carpeta instalada usando
   get_package_share_directory("voice_hmm"). Si no se recompila despues de
   entrenar, ROS puede seguir usando modelos viejos.


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

Antes de grabar, abrir voice_hmm/record_audio.py y ajustar las variables de
arriba:

OUTPUT_DIR = "data/train"
SAMPLES_PER_WORD = 20
DURATION_SECONDS = 2.0
DEFAULT_ALSA_DEVICE = "plughw:0,6"
WORD = None
SAMPLE_INDEX = None

Luego correr:

python3 -m voice_hmm.record_audio

Para grabar test, cambiar:

OUTPUT_DIR = "data/test"
SAMPLES_PER_WORD = 10

Para regrabar una sola muestra, ejemplo baja_007.wav:

OUTPUT_DIR = "data/train"
WORD = "baja"
SAMPLE_INDEX = 7

Despues correr otra vez:

python3 -m voice_hmm.record_audio


Entrenar
--------
Desde el paquete:

cd ~/8vo/reto_voz/src/voice_hmm
python3 -m voice_hmm.train

Esto ejecuta todo el entrenamiento:

- Lee los WAVs de data/train/<palabra>/.
- Extrae MFCCs de cada grabacion.
- Entrena un codebook VQ global de 256 simbolos.
- Convierte cada grabacion a una secuencia de simbolos VQ.
- Entrena un HMM de 5 estados para cada palabra.
- Guarda codebook.npy y los *_hmm.npz en models/.

Despues de entrenar, recompilar para que ROS instale los modelos nuevos:

cd ~/8vo/reto_voz
colcon build --packages-select voice_hmm

Si se quiere usar los modelos recien entrenados sin ROS, no hace falta compilar;
python3 -m voice_hmm.test_model desde src/voice_hmm lee directamente models/.
Para el nodo ROS, si hace falta recompilar porque el nodo carga los modelos
instalados en install/voice_hmm/share/voice_hmm/models/.


Probar modelo sin ROS
---------------------
Abrir voice_hmm/test_model.py y cambiar las variables de arriba.

Para probar hablando en vivo:

TEST_MODE = "live"
LIVE_DURATION = 2.0
ALSA_DEVICE = "plughw:0,6"

Probar un WAV:

TEST_MODE = "wav"
WAV_PATH = "data/train/baja/baja_007.wav"

Evaluar carpeta de test:

TEST_MODE = "data"
DATA_DIR = "data/test"

Luego correr:

cd ~/8vo/reto_voz/src/voice_hmm
python3 -m voice_hmm.test_model

La prueba sin ROS tambien construye el reconocedor igual que el nodo:

1. Crea HMMVoiceRecognizer(models_dir, WORDS).
2. Carga codebook.npy.
3. Carga un HMM por palabra.
4. Extrae MFCCs del audio de prueba.
5. Cuantiza los MFCCs con el codebook.
6. Calcula un score log-space para cada HMM.
7. Imprime la palabra con mayor score y la lista de scores.

Cuando TEST_MODE = "data", el script recorre todas las carpetas por palabra,
cuenta aciertos y muestra una matriz de confusion. Esa matriz sirve para ver que
palabras se estan confundiendo entre si.


Generar heatmaps de A y B
-------------------------
Para el entregable de verificacion, se pueden generar mapas de calor de las
matrices A y B para exactamente 3 palabras:

Abrir voice_hmm/plot_hmm_heatmaps.py y ajustar:

WORDS_TO_PLOT = ["avanza", "alto", "busca"]

Luego correr:

cd ~/8vo/reto_voz/src/voice_hmm
python3 -m voice_hmm.plot_hmm_heatmaps

Esto guarda imagenes PNG en:

reports/hmm_heatmaps/

Cada imagen contiene exclusivamente heatmaps, no tablas crudas de los 256
valores de B. Para cada palabra se muestran:

- Inicial: conteos despues de segmentar linealmente la primera grabacion de esa
  palabra, sin smoothing.
- Intermedia: conteos acumulados hasta la mitad de las grabaciones de
  entrenamiento de esa palabra, sin smoothing.
- Final: modelo completo ya listo, usando el HMM guardado en models/ con
  smoothing epsilon=1e-6 aplicado.

La fila A permite verificar la estructura Bakis: diagonal y superdiagonal. La
fila B permite verificar sparsity: pocos indices VQ tienen probabilidad alta y
la mayoria queda cerca de cero.

Tambien se pueden escoger otras 3 palabras:

WORDS_TO_PLOT = ["derecha", "izquierda", "retrocede"]


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
