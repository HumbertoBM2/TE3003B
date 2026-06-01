# Contexto del Proyecto: Reconocimiento de Palabras Aisladas con HMM + VQ en ROS 2

## 1. Objetivo general

El objetivo del proyecto es implementar un sistema de reconocimiento de palabras aisladas usando un pipeline clásico de procesamiento de voz basado en:

```text
Audio WAV → MFCC → Cuantización Vectorial (VQ) → Secuencia de símbolos → HMM por palabra → Forward en log-space → Comando reconocido
```

El sistema debe reconocer comandos de voz aislados, por ejemplo:

```text
avanza, retrocede, derecha, izquierda, alto, empieza, sube, baja, gira, busca
```

La idea final es que este reconocedor pueda integrarse a un proyecto ROS 2 en Python, donde un nodo reciba audio, ejecute el reconocimiento y publique el comando detectado para que otro nodo del robot lo use.

---

## 2. Restricciones importantes

El pipeline debe ser implementado manualmente, sin usar librerías que ya hagan reconocimiento de voz o extracción automática de características.

### Permitido

Se puede usar:

```text
numpy
wave
os
json
matplotlib, solo para gráficas/evidencia
rclpy para ROS 2
std_msgs para mensajes ROS 2
sounddevice solo para grabar audios, no para reconocer
```

### No usar

No se deben usar librerías que hagan el trabajo principal del pipeline, como:

```text
librosa
sklearn
scipy para HMM/MFCC/logsumexp
hmmlearn
torchaudio
speech_recognition
python_speech_features
```

La idea es que el código implemente por cuenta propia:

```text
preprocesamiento
framing
ventaneo
MFCC
VQ/LBG
HMM por conteos
Forward en log-space
matriz de confusión
```

NumPy sí está permitido como motor matemático para arreglos, FFT y operaciones matriciales.

---

## 3. Lo que debe cumplir el reto

El reto se llama:

```text
Reconocimiento de palabras aisladas con HMM
Guía de Proyecto: Reconocedor de Palabras Aisladas (HMM + VQ)
```

Debe cumplir los siguientes puntos.

---

## 4. Dataset de audio

Se deben grabar varias muestras por palabra.

Estructura recomendada:

```text
data/
├── train/
│   ├── avanza/
│   │   ├── avanza_001.wav
│   │   ├── avanza_002.wav
│   │   └── ...
│   ├── retrocede/
│   ├── derecha/
│   └── ...
│
└── test/
    ├── avanza/
    ├── retrocede/
    ├── derecha/
    └── ...
```

Configuración recomendada de grabación:

```text
sample rate: 16000 Hz
canales: mono
formato: 16-bit PCM WAV
duración por grabación: 2 segundos
muestras por palabra para train: 20 a 30 mínimo
muestras por palabra para test: 5 a 10 mínimo
```

Se preparó un script de grabación llamado conceptualmente `record_dataset.py`, que debe:

1. Crear automáticamente las carpetas de cada palabra.
2. Grabar una cantidad configurable de muestras por palabra.
3. Guardar cada grabación como WAV mono de 16 bits.
4. Separar datos de entrenamiento y prueba usando `OUTPUT_DIR = "data/train"` o `OUTPUT_DIR = "data/test"`.

---

## 5. Preprocesamiento y extracción de características

Cada archivo WAV debe convertirse en una matriz de MFCC.

### Flujo de MFCC requerido

```text
WAV
↓
normalización de int16 a float [-1, 1]
↓
recorte opcional de silencio con energía corta
↓
pre-emphasis
↓
framing: ventanas de 25 ms
↓
hop: 10 ms
↓
ventana Hamming
↓
FFT con np.fft.rfft
↓
espectro de potencia
↓
banco de filtros Mel manual
↓
log de energías Mel
↓
DCT manual
↓
13 coeficientes MFCC
↓
normalización cepstral, por ejemplo restar media
```

Parámetros recomendados:

```text
sample_rate = 16000
frame_ms = 25
hop_ms = 10
n_fft = 512
n_filters = 26
n_mfcc = 13
pre_emphasis_alpha = 0.97
```

El código no debe llamar funciones externas tipo `librosa.feature.mfcc`. La extracción MFCC debe estar hecha con NumPy.

---

## 6. Cuantización Vectorial VQ

Después de extraer MFCC, se debe entrenar un codebook global de exactamente 256 vectores.

### Requisito principal

```text
Cada vector MFCC debe mapearse al centroide más cercano del codebook.
Cada grabación debe convertirse en una secuencia de enteros de 0 a 255.
```

Ejemplo esperado:

```text
avanza.wav → O = [12, 12, 15, 200, 201, 45, ..., 193]
```

### Entrenamiento del codebook

El codebook debe entrenarse manualmente con LBG/K-means propio:

1. Juntar todos los MFCC de todas las palabras de entrenamiento.
2. Iniciar con un centroide global.
3. Dividir centroides con perturbación, por ejemplo `±epsilon`.
4. Refinar con K-means implementado manualmente.
5. Repetir hasta tener 256 centroides.

No usar `sklearn.KMeans`.

Funciones esperadas:

```text
nearest_codewords(X, codebook)
kmeans_refine(X, codebook)
train_lbg(X, target_size=256)
quantize_mfcc(mfcc, codebook)
```

---

## 7. Definición de HMM por palabra

Se deben crear 10 modelos HMM, uno por palabra.

Cada modelo debe tener:

```text
λ = (A, B, π)
```

Donde:

```text
A = matriz de transición de estados, tamaño N x N
B = matriz de emisiones, tamaño N x 256
π = distribución inicial, tamaño N
N = número de estados, entre 4 y 8
M = número de símbolos, exactamente 256
```

### Topología requerida

Cada HMM debe ser tipo Bakis / Left-to-Right.

Solo se permiten transiciones:

```text
i → i
i → i + 1
```

Todas las demás transiciones deben ser 0.

Ejemplo para 5 estados:

```text
A =
[0.90 0.10 0.00 0.00 0.00]
[0.00 0.90 0.10 0.00 0.00]
[0.00 0.00 0.90 0.10 0.00]
[0.00 0.00 0.00 0.90 0.10]
[0.00 0.00 0.00 0.00 1.00]
```

Distribución inicial recomendada:

```text
π = [1, 0, 0, ..., 0]
```

---

## 8. Entrenamiento obligatorio por conteos

No se debe empezar usando Baum-Welch ni entrenamiento de caja negra.

El entrenamiento inicial debe hacerse con ingeniería de conteos.

### Procedimiento

Para cada palabra:

1. Tomar todas sus secuencias de símbolos VQ.
2. Dividir cada secuencia linealmente en N segmentos, donde N es el número de estados.
3. Asignar cada segmento a un estado.
4. Para cada estado, contar cuántas veces aparece cada símbolo de 0 a 255.
5. Normalizar esos conteos para obtener la matriz B.
6. Estimar la matriz A a partir de la duración promedio de cada segmento.
7. Aplicar smoothing obligatorio a B.

### Segmentación lineal

Si hay 5 estados, cada estado representa aproximadamente 20% de la señal:

```text
estado 0 → 0% a 20%
estado 1 → 20% a 40%
estado 2 → 40% a 60%
estado 3 → 60% a 80%
estado 4 → 80% a 100%
```

### Matriz B

La matriz B representa:

```text
B[estado, símbolo] = P(símbolo | estado)
```

Debe entrenarse contando apariciones de símbolos en cada segmento.

### Smoothing obligatorio

Como hay 256 símbolos, muchos no aparecerán en entrenamiento. Para evitar probabilidades cero:

```text
B = B_counts + epsilon
epsilon = 1e-6
B = B / suma_por_fila
```

Cada fila de B debe sumar exactamente 1.

### Matriz A

Si un estado dura en promedio `d` frames:

```text
aii = (d - 1) / d
ai,i+1 = 1 / d
```

Último estado:

```text
A[N-1, N-1] = 1
```

Cada fila de A debe sumar exactamente 1.

---

## 9. Baum-Welch

Baum-Welch no debe usarse como primer paso.

Razón:

```text
Baum-Welch es un optimizador local. Si se inicializa aleatoriamente, puede caer en mínimos locales y perder la correspondencia entre estados y partes reales de la palabra.
```

Uso permitido:

```text
Solo como refinamiento opcional después de que el modelo por conteos ya funcione.
```

Si la precisión baja después de Baum-Welch, no usarlo para la entrega final.

---

## 10. Reconocimiento con Forward

Para reconocer una palabra nueva:

1. Recibir audio.
2. Extraer MFCC.
3. Convertir MFCC a símbolos usando el codebook VQ.
4. Evaluar la secuencia O en los 10 HMMs usando Forward.
5. Escoger la palabra con mayor log-likelihood.

Decisión:

```text
palabra_reconocida = argmax_j log P(O | λ_j)
```

### Forward en log-space

Debe implementarse en espacio logarítmico para evitar underflow.

Funciones esperadas:

```text
logsumexp(values)
forward_log(O, hmm)
```

No usar `scipy.special.logsumexp`; implementarlo manualmente con NumPy.

---

## 11. Entregables y evidencias obligatorias

El código y reporte deben poder demostrar que el modelo no es caja negra.

### 1. Secuencia VQ

Mostrar ejemplo de una grabación convertida a índices:

```text
avanza_001.wav → [12, 12, 15, 200, 201, 45, ..., 193]
```

### 2. Sparsity en B

Mostrar para un estado, por ejemplo estado 0 de `avanza`, que B tiene picos en algunos índices y casi cero en la mayoría.

Ejemplo de evidencia:

```text
Top emissions estado 0:
índice 12 → 0.21
índice 15 → 0.18
índice 43 → 0.09
...
```

También se puede graficar `B[0]` con matplotlib.

### 3. Diagonalidad en A

Mostrar que A tiene valores fuertes en la diagonal y superdiagonal, y ceros en el resto.

### 4. Log-likelihoods

Para cada audio de prueba, imprimir los scores de los 10 modelos.

Ejemplo:

```text
avanza: -240.3
retrocede: -390.8
derecha: -318.2
stop: -501.5
...
Comando reconocido: avanza
```

### 5. Matriz de confusión 10x10

Evaluar el sistema sobre `data/test` y construir una matriz 10x10.

Analizar errores:

```text
- similitud fonética
- ruido
- mala cuantización
- pocas grabaciones
- comandos muy cortos
- mala segmentación por silencio
```

---

## 12. Integración en ROS 2

La parte de entrenamiento no debe correr en ROS 2.

En ROS 2 solo se usa el sistema ya entrenado:

```text
modelos entrenados → nodo ROS 2 → recibe audio → reconoce comando → publica resultado
```

### Paquete sugerido

```text
voice_hmm_recognizer/
├── package.xml
├── setup.py
├── resource/
│   └── voice_hmm_recognizer
├── voice_hmm_recognizer/
│   ├── __init__.py
│   ├── voice_recognition_node.py
│   ├── recognizer_core.py
│   ├── mfcc.py
│   ├── vq.py
│   └── hmm.py
├── models/
│   ├── codebook.npy
│   ├── avanza_hmm.npz
│   ├── retrocede_hmm.npz
│   └── ...
├── launch/
│   └── voice_hmm.launch.py
└── config/
    └── voice_hmm.yaml
```

### Tópicos ROS 2 recomendados

El nodo de reconocimiento debe suscribirse a:

```text
/voice/listen_flag       std_msgs/Bool
/voice/audio_chunk       std_msgs/Int16MultiArray
```

Debe publicar:

```text
/voice/recognized_command    std_msgs/String
/voice/log_likelihoods       std_msgs/String
```

### Comportamiento del nodo

```text
Si /voice/listen_flag = True:
    empezar a guardar audio en un buffer

Mientras listen_flag = True:
    agregar audio de /voice/audio_chunk al buffer

Cuando /voice/listen_flag cambia a False:
    convertir buffer int16 a float [-1, 1]
    extraer MFCC
    cuantizar con codebook
    correr Forward en los 10 HMMs
    publicar palabra reconocida
    publicar log-likelihoods
    limpiar buffer
```

### Separación recomendada

El nodo de voz solo debe publicar la palabra reconocida.

Otro nodo debe encargarse de mover el robot:

```text
voice_recognition_node
        ↓
/voice/recognized_command
        ↓
command_router_node
        ↓
/cmd_vel, motor de lift, stop de emergencia, etc.
```

---

## 13. Archivos principales del código

### Para entrenamiento offline

```text
record_dataset.py        graba audios WAV
train_codebook.py        entrena codebook VQ de 256 centroides
train_hmms.py            entrena un HMM por palabra usando conteos
evaluate.py              genera matriz de confusión
```

### Para ROS 2 runtime

```text
mfcc.py                  extracción manual de MFCC con NumPy
vq.py                    cuantización con codebook cargado
hmm.py                   Forward log-space
recognizer_core.py       clase HMMVoiceRecognizer
voice_recognition_node.py nodo ROS 2
```

---

## 14. Modelos guardados

El entrenamiento debe producir:

```text
models/codebook.npy
models/avanza_hmm.npz
models/retrocede_hmm.npz
models/derecha_hmm.npz
models/izquierda_hmm.npz
models/stop_hmm.npz
models/start_hmm.npz
models/lift_hmm.npz
models/baja_hmm.npz
models/gira_hmm.npz
models/alto_hmm.npz
```

Cada `.npz` debe contener:

```text
A
B
pi
n_states
n_symbols
```

---

## 15. Configuración final recomendada

```text
Audio:
    16 kHz, mono, 16-bit WAV

MFCC:
    frame = 25 ms
    hop = 10 ms
    FFT = 512 puntos
    filtros Mel = 26
    coeficientes MFCC = 13

VQ:
    LBG/K-means manual
    codebook global = 256 centroides
    distancia euclidiana

HMM:
    10 modelos, uno por palabra
    5 estados para empezar
    Bakis / Left-to-Right
    π = [1, 0, 0, 0, 0]
    B por conteos + epsilon = 1e-6
    A por duración promedio

Reconocimiento:
    Forward en log-space
    Decisión por máximo log-likelihood

ROS 2:
    Nodo separado de control del robot
    Nodo de voz solo publica el comando reconocido
```

---

## 16. Orden recomendado de implementación

```text
1. Crear script record_dataset.py.
2. Grabar datos train y test.
3. Implementar lectura WAV.
4. Implementar MFCC manual con NumPy.
5. Verificar que cada audio produce matriz MFCC de shape (T, 13).
6. Implementar LBG/K-means manual.
7. Entrenar codebook de 256.
8. Convertir audios a secuencias de índices 0-255.
9. Entrenar un HMM por palabra con conteos.
10. Verificar A y B.
11. Implementar Forward log-space.
12. Probar reconocimiento de un archivo individual.
13. Evaluar todos los archivos de test.
14. Generar matriz de confusión.
15. Integrar modelos entrenados al paquete ROS 2.
16. Crear nodo voice_recognition_node.
17. Probar con audio publicado por tópico.
18. Conectar salida a un command_router_node.
```

---

## 17. Criterios para saber si funciona

El sistema puede considerarse funcionando si:

```text
- Cada audio se convierte en una secuencia de símbolos 0-255.
- Cada matriz B tiene filas que suman 1.
- Cada matriz A tiene filas que suman 1.
- A respeta la topología Left-to-Right.
- Forward no produce underflow porque trabaja en log-space.
- Para audios de prueba, el modelo correcto suele tener el mayor log-likelihood.
- La matriz de confusión muestra mayoría de valores en la diagonal.
- El nodo ROS 2 publica un comando en /voice/recognized_command.
```

---

## 18. Riesgos comunes

```text
1. Pocos datos de entrenamiento.
2. Grabar train y test en condiciones muy diferentes.
3. Silencios muy largos al inicio o final.
4. Codebook mal entrenado o con centroides vacíos.
5. B sin smoothing, causando probabilidades cero.
6. Usar probabilidades normales en Forward y provocar underflow.
7. Mezclar sample rates diferentes entre entrenamiento y runtime.
8. Cambiar parámetros MFCC después de entrenar.
9. Entrenar un codebook diferente para test o runtime.
10. Intentar usar Baum-Welch antes de que el modelo por conteos funcione.
```

---

## 19. Resumen corto para README

Este proyecto implementa un reconocedor de palabras aisladas usando un pipeline clásico de voz: extracción manual de MFCC, cuantización vectorial con codebook de 256 centroides, entrenamiento de HMMs Bakis por palabra usando segmentación lineal y conteos, y reconocimiento mediante el algoritmo Forward en espacio logarítmico. La integración con ROS 2 se realiza mediante un nodo que escucha audio por tópico, ejecuta el pipeline entrenado y publica el comando reconocido como `std_msgs/String`.
