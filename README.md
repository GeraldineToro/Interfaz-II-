# Interfaz-II-
Indice 

1.  [Hola Mundo](#ejercicio-n1-cueck) <br>
2.  [Led intermitente](#ejercicio-n2-led-intermitente) <br>
3.  [Led con potenciometro](#ejercicio-n-3-led-con-potenciometro) <br>
4.  [Led con pulsador](#ejercicio-n4-led-pulsador) <br>
5.  [Semaforo](#ejercicio-n5-semaforo) <br>   
##### Ejercicio n°1: cueck

```js
void setup() {
  Serial.begin(9600); // Inicia la comunicación serie a 9600 bps
  Serial.println("Cueck"); // Envía "Cueck" al monitor serie
}

void loop() {
  // No es necesario poner nada en el loop para este ejemplo
}
```
<img scr="https://raw.githubusercontent.com/GeraldineToro/Interfaz-II-/refs/heads/main/img/ejercicio%201.png"/>

##### Ejercicio n° 2: Led intermitente

```js
void setup() {  // Configuración inicial (ej: pines como entrada/salida)
  pinMode(13, OUTPUT);  // Pin 13 como salida
}

void loop() {   // Se repite infinitamente
  digitalWrite(13, HIGH);  // Encender LED
  delay(1000);             // Esperar 1 segundo
  digitalWrite(13, LOW);   // Apagar LED
  delay(1000);             // Esperar 1 segundo
}
```
<img src="https://raw.githubusercontent.com/GeraldineToro/Interfaz-II-/refs/heads/main/img/led%20intermitente.png"/>
```

##### Ejercicio n° 3: Led con potenciometro

```js
void setup() {
  pinMode(13, OUTPUT);  // Pin PWM (símbolo ~)
}
void loop() {
  int valor = analogRead(A0);           // Leer potenciómetro (0-1023)
  int brillo = map(valor, 0, 1023, 0, 255);  // Convertir a rango PWM
  analogWrite(13, brillo);               // Ajustar brillo
}
```
<img src="https://raw.githubusercontent.com/GeraldineToro/Interfaz-II-/refs/heads/main/img/potenciometro.png"/>
```

##### Ejercicio n°4: Led pulsador

```js
void setup() {
  pinMode(2, INPUT);  // Botón como entrada
  pinMode(13, OUTPUT);
}
void loop() {
  if (digitalRead(2) == HIGH) {  // Si se presiona el botón
    digitalWrite(13, HIGH);
  } else {
    digitalWrite(13, LOW);
  }
}
```
<img src="https://raw.githubusercontent.com/GeraldineToro/Interfaz-II-/refs/heads/main/img/pulsador.png"/>
```

##### Ejercicio n°5: Semaforo

```js// C++ code - Semáforo Autos y Peatones

// Definición de pines
int LED_1 = 6;  // Luz roja autos
int LED_2 = 7;  // Luz amarilla autos
int LED_3 = 8;  // Luz verde autos
int LED_4 = 9;  // Luz verde peatones
int LED_5 = 10; // Luz roja peatones

void setup() {
  // Configuramos todos los pines como salida
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_3, OUTPUT);
  pinMode(LED_4, OUTPUT);
  pinMode(LED_5, OUTPUT);
}

void loop() {
  // 🚦 Fase 1: Autos en verde, peatones en rojo
  digitalWrite(LED_1, LOW);   // Rojo autos apagado
  digitalWrite(LED_2, LOW);   // Amarillo autos apagado
  digitalWrite(LED_3, HIGH);  // Verde autos encendido
  digitalWrite(LED_4, LOW);   // Verde peatones apagado
  digitalWrite(LED_5, HIGH);  // Rojo peatones encendido
  delay(5000); // 5 segundos

  // 🚦 Fase 2: Amarillo autos, peatones siguen en rojo
  digitalWrite(LED_3, LOW);   // Verde autos apagado
  digitalWrite(LED_2, HIGH);  // Amarillo autos encendido
  delay(2000); // 2 segundos
  digitalWrite(LED_2, LOW);   // Amarillo autos apagado

  // 🚦 Fase 3: Rojo autos, verde peatones
  digitalWrite(LED_1, HIGH);  // Rojo autos encendido
  digitalWrite(LED_5, LOW);   // Rojo peatones apagado
  digitalWrite(LED_4, HIGH);  // Verde peatones encendido
  delay(5000); // 5 segundos

  // 🚦 Fase 4: Rojo autos, rojo peatones (tiempo intermedio)
 // digitalWrite(LED_4, LOW);   // Verde peatones apagado
 // digitalWrite(LED_5, HIGH);  // Rojo peatones encendido
  //delay(2000); // 2 segundos
}
```
##### <img src="https://raw.githubusercontent.com/GeraldineToro/Interfaz-II-/refs/heads/main/img/semaforo.png"/>
```

##### Eercicio n°6: Arduino-Processing

Arduino

```js
unsigned int ADCValue;
void setup(){
    Serial.begin(9600);
}

void loop(){

 int val = analogRead(0);
   val = map(val, 0, 300, 0, 255);
    Serial.println(val);
delay(50);
}
```
Processing

```js
import processing.serial.*;

Serial myPort;  // Crear objeto de la clase Serial
static String val;    // Datos recibidos desde el puerto serial
int sensorVal = 0;

void setup()
{
  background(0); 
  //fullScreen(P3D);
   size(1080, 720);
   noStroke();
  noFill();
  String portName = "COM3";// Cambia el número (en este caso) para que coincida con el puerto correspondiente conectado a tu Arduino. 

  //myPort = new Serial(this, "/dev/cu.usbmodem1101", 9600);
  myPort = new Serial(this, Serial.list()[0], 9600);

}

void draw()
{
  if ( myPort.available() > 0) {  // Si hay datos disponibles,
  val = myPort.readStringUntil('\n'); 
  try {
   sensorVal = Integer.valueOf(val.trim());
  }
  catch(Exception e) {
  ;
  }
  println(sensorVal); // léelos y guárdalos en vals!
  }  
 //background(0);
  // Escala el valor de mouseX de 0 a 640 a un rango entre 0 y 175
  float c = map(sensorVal, 0, width, 0, 400);
  // Escala el valor de mouseX de 0 a 640 a un rango entre 40 y 300
  float d = map(sensorVal, 0, width, 40,500);
  fill(255, c, 0);
  ellipse(width/2, height/2, d, d);   
}
```
<img src="https://raw.githubusercontent.com/GeraldineToro/Interfaz-II-/refs/heads/main/img/arduino-processing.png"/>
```

##### Ejercicio n°7: Arduino + boton + processing

Arduino
```js
int buttonPin = 2;  // Pin del botón
int buttonState = 0;

void setup() {
  pinMode(buttonPin, INPUT_PULLUP); // Botón con resistencia interna
  Serial.begin(9600);
}

void loop() {
  buttonState = digitalRead(buttonPin);

  if (buttonState == HIGH) {   // Botón presionado
    Serial.println(1);        // Enviar un "1" a Processing
    delay(200);               // Evitar rebotes
  }
}
```
Processing
```js
import processing.serial.*;

Serial myPort;
ArrayList<PVector> circles; 

void setup() {
  size(1920, 1080);
  background(0);
  
  // Ajusta el nombre del puerto según tu Arduino
  println(Serial.list());
  myPort = new Serial(this, "/dev/cu.usbmodem1101", 9600);
  //myPort = new Serial(this, Serial.list()[0], 9600);
  
  circles = new ArrayList<PVector>();
}

void draw() {
  //background(0);
  
  // Dibujar círculos almacenados
  fill(0, 0, 0);
  //noStroke();
  stroke(255, 0, 0);
  for (PVector c : circles) {
    ellipse(c.x, c.y, 30, 30);
  }
  
  // Revisar si llega algo de Arduino
  if (myPort.available() > 0) {
    String val = myPort.readStringUntil('\n');
    if (val != null) {
      val = trim(val);
      if (val.equals("1")) {
        // Cada vez que se aprieta el botón, agregar un círculo en posición aleatoria
        circles.add(new PVector(random(width), random(height)));
      }
    }
  }
}
```

##### Ejercicio n°8: Arduino + boton + poteniometro + processing
Arduino
```js
int buttonPin = 2;       // Pin del botón
int potPin = A0;         // Pin del potenciómetro
int buttonState = 0;

void setup() {
  pinMode(buttonPin, INPUT_PULLUP); // Botón con resistencia interna
  Serial.begin(9600);
}

void loop() {
  buttonState = digitalRead(buttonPin);

  if (buttonState == HIGH) {   // Botón presionado
    int potValue = analogRead(potPin);   // 0 - 1023
    Serial.print("BTN,");     // etiqueta para Processing
    Serial.println(potValue); // mando el valor junto con el evento
    delay(200);               // debounce simple
  }
}
```
Processing
```js
import processing.serial.*;

Serial myPort;
ArrayList<CircleData> circles; 

void setup() {
  size(1200, 720);
  background(0);
  
  // Ajusta el puerto según tu Arduino
  println(Serial.list());
  myPort = new Serial(this, "/dev/cu.usbmodem1101", 9600);
  //myPort = new Serial(this, Serial.list()[0], 9600);
  
  circles = new ArrayList<CircleData>();
}

void draw() {
  //background(0);
  
  // Dibujar todos los círculos guardados
  //fill(0, 150, 255);
  //noStroke();
  fill(0, 0, 0);
  stroke(255, 0, 0);
  for (CircleData c : circles) {
    ellipse(c.x, c.y, c.size, c.size);
  }
  
  // Leer datos de Arduino
  if (myPort.available() > 0) {
    String val = myPort.readStringUntil('\n');
    if (val != null) {
      val = trim(val);
      if (val.startsWith("BTN")) {
        // Extraer el valor del potenciómetro
        String[] parts = split(val, ',');
        if (parts.length == 2) {
          float potVal = float(parts[1]);
          float circleSize = map(potVal, 0, 1023, 10, 100); // tamaño 10-100 px
          circles.add(new CircleData(random(width), random(height), circleSize));
        }
      }
    }
  }
}

// Clase para guardar datos de cada círculo
class CircleData {
  float x, y, size;
  CircleData(float x, float y, float size) {
    this.x = x;
    this.y = y;
    this.size = size;
  }
}
```
##### Ejercicio n°9: Botonera con audio

Processing

```js
// Importamos librería para comunicación serial
import processing.serial.*;
// Importamos librería Minim para manejar audio
import ddf.minim.*;

// Declaramos el objeto serial para comunicarnos con Arduino
Serial myPort;
// Objeto principal de Minim
Minim minim;
// Array de reproductores de audio (3 pistas)
AudioPlayer[] players;
// Variable para guardar el índice de la pista que está sonando
int currentTrack = -1;  // -1 significa que no hay pista activa al inicio

void setup() {
  size(400, 200); // Ventana de 400x200 píxeles
  
  // --- Configuración del puerto serial ---
  printArray(Serial.list()); // Muestra en consola la lista de puertos disponibles
  myPort = new Serial(this, Serial.list()[0], 9600); // Abrimos el primer puerto de la lista a 9600 baudios
  
  // --- Configuración de audio ---
  minim = new Minim(this); // Inicializamos Minim
  players = new AudioPlayer[3]; // Creamos un array de 3 reproductores
  
  // Cargamos los 3 archivos de audio desde la carpeta "data"
  players[0] = minim.loadFile("audio1.mp3", 2048); 
  players[1] = minim.loadFile("audio2.mp3", 2048); 
  players[2] = minim.loadFile("audio3.mp3", 2048); 
}

void draw() {
  background(0); // Fondo negro
  fill(255);     // Color blanco para el texto
  textSize(16);  // Tamaño del texto
  
  // Mostramos en pantalla qué botón está activo
  text("Botón actual: " + (currentTrack == -1 ? "ninguno" : currentTrack), 20, 40);
}

void serialEvent(Serial myPort) {
  // Leemos la cadena que llega desde Arduino hasta el salto de línea
  String inString = trim(myPort.readStringUntil('\n'));
  
  // Si no llega nada, salimos
  if (inString == null) return;

  // --- Si el mensaje recibido empieza con "B" significa que es un botón ---
  if (inString.startsWith("B")) {
    // Quitamos la letra "B" y separamos el mensaje en partes (ejemplo "0:0")
    String[] parts = split(inString.substring(1), ':');
    
    // Si realmente recibimos dos partes (índice y estado)
    if (parts.length == 2) {
      int buttonIndex = int(parts[0]); // Número del botón (0,1,2)
      int state = int(parts[1]);       // Estado del botón (0 = presionado, 1 = suelto)
      
      // Si el botón fue presionado (LOW = 0 en Arduino)
      if (state == 0) { 
        playTrack(buttonIndex); // Llamamos a la función para reproducir la pista correspondiente
      }
    }
  }
}

// --- Función que reproduce una pista según el botón ---
void playTrack(int index) {
  // Si ya había una pista sonando, la pausamos y la rebobinamos al inicio
  if (currentTrack != -1 && players[currentTrack].isPlaying()) {
    players[currentTrack].pause();
    players[currentTrack].rewind();
  }
  
  // Reproducimos en bucle la pista seleccionada
  players[index].loop();
  
  // Actualizamos la variable para saber cuál es la pista activa
  currentTrack = index;
}
```
Arduino

```js
// --- Configuración de botones ---
const int numButtons = 3;
const int buttonPins[numButtons] = {2, 4, 7};
const int ledButtonPins[numButtons] = {9, 10, 11}; // LEDs botones

// --- Configuración de potenciómetros ---
const int numPots = 2;
const int potPins[numPots] = {A0, A1};
const int ledPotPins[numPots] = {3, 5}; // LEDs PWM

// Variables de estados previos
int lastButtonState[numButtons];
int lastPotValue[numPots];

void setup() {
  Serial.begin(9600);

  // Configurar botones y LEDs
  for (int i = 0; i < numButtons; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
    pinMode(ledButtonPins[i], OUTPUT);
    lastButtonState[i] = digitalRead(buttonPins[i]);
  }

  // Configurar LEDs de potenciómetros
  for (int i = 0; i < numPots; i++) {
    pinMode(ledPotPins[i], OUTPUT);
    lastPotValue[i] = analogRead(potPins[i]);
  }
}

void loop() {
  // Leer y enviar botones
  for (int i = 0; i < numButtons; i++) {
    int buttonState = digitalRead(buttonPins[i]);

    // LED se enciende cuando botón está presionado
    digitalWrite(ledButtonPins[i], buttonState == LOW ? HIGH : LOW);

    if (buttonState != lastButtonState[i]) {  // enviar cambios
      Serial.print("B");
      Serial.print(i); 
      Serial.print(":");
      Serial.println(buttonState);
      lastButtonState[i] = buttonState;
    }
  }

  // Leer y enviar potenciómetros
  for (int i = 0; i < numPots; i++) {
    int potValue = analogRead(potPins[i]); // 0–1023
    int pwmValue = potValue / 4;           // 0–255

    // Ajustar LED según valor
    analogWrite(ledPotPins[i], pwmValue);

    if (abs(pwmValue - lastPotValue[i]) > 2) { // evitar ruido
      Serial.print("P");
      Serial.print(i);
      Serial.print(":");
      Serial.println(pwmValue);
      lastPotValue[i] = pwmValue;
    }
  }

  delay(10);
}
```
##### Nota n° 1: Semaforo + botón

```js

//Semáforo de Autos y Peatones controlado con botón

// LEDs
int LED_1 = 6;  // Rojo autos
int LED_2 = 7;  // Amarillo autos
int LED_3 = 8;  // Verde autos
int LED_4 = 9;  // Verde peatones
int LED_5 = 10; // Rojo peatones

// Pin del botón
int botonPin = 2;

//Variable añadida para detectar la pulsación única 
int ultimoEstadoBoton = HIGH;

void setup() {
  // Configurar pines LED como salida
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_3, OUTPUT);
  pinMode(LED_4, OUTPUT);
  pinMode(LED_5, OUTPUT);

  // Configurar botón como entrada con resistencia interna pull-up
  pinMode(botonPin, INPUT_PULLUP);

  // semáforo en espera (Rojo autos, Rojo peatones)
  digitalWrite(LED_1, HIGH);  // Rojo autos ON
  digitalWrite(LED_2, LOW);
  digitalWrite(LED_3, LOW);
  digitalWrite(LED_4, LOW);   // Verde peatones OFF
  digitalWrite(LED_5, HIGH);  // Rojo peatones ON
}

void loop() {
  // Leer el estado actual del botón
  int estadoBoton = digitalRead(botonPin);
  
  //  Comprueba si el botón acaba de ser presionado (de HIGH a LOW)
  if (estadoBoton == LOW && ultimoEstadoBoton == HIGH) {
    
    // 🚦 Fase 1: Autos en verde, peatones en rojo
    digitalWrite(LED_1, LOW);   // Rojo autos OFF
    digitalWrite(LED_2, LOW);   // Amarillo autos OFF
    digitalWrite(LED_3, HIGH);  // Verde autos ON
    digitalWrite(LED_4, LOW);   // Verde peatones OFF
    digitalWrite(LED_5, HIGH);  // Rojo peatones ON
    delay(5000); // 5 segundos

    // 🚦 Fase 2: Amarillo autos, peatones siguen en rojo
    digitalWrite(LED_3, LOW);   // Verde autos OFF
    digitalWrite(LED_2, HIGH);  // Amarillo autos ON
    delay(2000); // 2 segundos
    digitalWrite(LED_2, LOW);   // Amarillo autos OFF

    // 🚦 Fase 3: Rojo autos, verde peatones
    digitalWrite(LED_1, HIGH);  // Rojo autos ON
    digitalWrite(LED_5, LOW);   // Rojo peatones OFF
    digitalWrite(LED_4, HIGH);  // Verde peatones ON
    delay(5000); // 5 segundos

    // 🚦 Fase 4: Rojo autos, rojo peatones (pausa)
    digitalWrite(LED_4, LOW);   // Verde peatones OFF
    digitalWrite(LED_5, HIGH);  // Rojo peatones ON
    delay(2000); // 2 segundos
    
    // Regresa al estado de espera 

    digitalWrite(LED_1, HIGH);  // Rojo autos ON
    digitalWrite(LED_2, LOW);
    digitalWrite(LED_3, LOW);
    digitalWrite(LED_4, LOW);   // Verde peatones OFF
    digitalWrite(LED_5, HIGH);  // Rojo peatones ON
  }

##### Ejercicio 10: Sensor Sharp

Codigo Arduino
```js
// Definir el pin del sensor Sharp
int sharpPin = A0;

void setup() {
  Serial.begin(9600); // Iniciar comunicación serial
}

void loop() {
  int sensorValue = analogRead(sharpPin); // Leer valor del sensor
  Serial.println(sensorValue); // Enviar valor a Processing
  delay(100); // Esperar un momento
}


  ultimoEstadoBoton = estadoBoton;
}
``` 

Codigo Processing
```js
import processing.serial.*;

Serial myPort;  // Create object from Serial class
static String val;    // Data received from the serial port
int sensorVal = 0;

void setup()
{
  background(0); 
  //fullScreen(P3D);
   size(1080, 720);
   noStroke();
  noFill();
  String portName = "COM5";// Change the number (in this case ) to match the corresponding port number connected to your Arduino. 

  myPort = new Serial(this, "/dev/cu.usbmodem1101", 9600);
}

void draw()
{
  if ( myPort.available() > 0) {  // If data is available,
  val = myPort.readStringUntil('\n'); 
  try {
   sensorVal = Integer.valueOf(val.trim());
  }
  catch(Exception e) {
  ;
  }
  println(sensorVal); // read it and store it in vals!
  }  
 //background(0);
  // Scale the mouseX value from 0 to 640 to a range between 0 and 175
  float c = map(sensorVal, 0, width, 0, 400);
  // Scale the mouseX value from 0 to 640 to a range between 40 and 300
  float d = map(sensorVal, 0, width, 40,500);
  fill(255, c, 0);
  ellipse(width/2, height/2, d, d);   

}

```
##### Eercicio 11: Sensor de Humedad.

Codigo Arduino

```js
void setup()
{
  Serial.begin(9600);// abre el puerto serial y Establece la velocidad en baudios a 9600 bps
}
void loop()
{
  int sensorValue;
  sensorValue = analogRead(0);   //conectar el sensor de humedad al pin analogo 0
  Serial.println(sensorValue); //imprime el valor a serial.
  delay(200);
}
```
##### Ejercicio 12: Cuerpo, video y sensor.

Codigo Arduino

```js
// --- Sensor Sharp conectado al pin A0 ---
int sensorPin = A0;
int valor;

void setup() {
  Serial.begin(9600);
}

void loop() {
  valor = analogRead(sensorPin);
  Serial.println(valor);
  delay(50); // envío cada 50 ms
}
```
Codigo Processing

```js
// --- Librerías necesarias ---
import processing.serial.*;
import processing.video.*;

// --- Variables de cámara y serial ---
Capture cam;
Serial myPort;

// --- Variables del sensor ---
float sensorValue = 0;
float suavizado = 0;

// --- Parámetros para detección de silueta ---
float umbral = 100; // controla el contraste para definir la silueta

void setup() {
  size(1280, 720);
  background(0);
  
  // --- Inicializar cámara ---
  String[] cameras = Capture.list();
  if (cameras.length == 0) {
    println("No se encontró cámara.");
    exit();
  } else {
    println("Cámara encontrada: " + cameras[0]);
    cam = new Capture(this, cameras[0]);
    cam.start();
  }
  
  // --- Inicializar puerto serie (Arduino) ---
  // Puedes ver la lista de puertos con println(Serial.list());
  String portName = Serial.list()[0]; 
  myPort = new Serial(this, "/dev/cu.usbmodem1101", 9600);
  //myPort = new Serial(this, portName, 9600);
}

void draw() {
  background(0);
  
  // --- Leer datos del sensor ---
  while (myPort.available() > 0) {
    String inString = trim(myPort.readStringUntil('\n'));
    if (inString != null) {
      sensorValue = float(inString);
      suavizado = lerp(suavizado, sensorValue, 0.1);
    }
  }
  
  // --- Mapear los valores del sensor ---
  float escala = map(suavizado, 0, 1023, 1.5, 0.5); // tamaño de la silueta
  float alpha = map(suavizado, 0, 1023, 255, 80);   // opacidad según distancia
  
  // --- Captura de video ---
  if (cam.available()) {
    cam.read();
  }

  // --- Dibujar silueta desde la cámara ---
  cam.loadPixels();
  loadPixels();
  
  for (int y = 0; y < cam.height; y++) {
    for (int x = 0; x < cam.width; x++) {
      int loc = x + y * cam.width;
      color c = cam.pixels[loc];
      float brillo = brightness(c);
      
      // Si el brillo es menor que el umbral, dibujamos píxel blanco (silueta)
      if (brillo < umbral) {
        int px = int(x * escala);
        int py = int(y * escala);
        if (px < width && py < height) {
          stroke(255, alpha);
          point(px, py);
        }
      }
    }
  }
}
```

#### Ejercicio 13: Promedio de imagenes.

Codigo Arduino

```js
void setup() {
  Serial.begin(9600);
}

void loop() {
  int potValue = analogRead(A0);
  Serial.println(potValue);
  delay(20);
}
```
Codigo Processing

```js
import processing.serial.*;

Serial myPort;
PImage[] imgs;
int numImages = 3;
PImage avgImg;
float mixAmount = 0;

void setup() {
  size(800, 600);
  println(Serial.list());
  
  //Cambia el índice según tu puerto (0, 1, 2, etc.)
  myPort = new Serial(this, Serial.list()[0], 9600);
  //myPort = new Serial(this, "/dev/cu.usbmodem1101", 9600);
  myPort.bufferUntil('\n');

  // Cargar imágenes
  imgs = new PImage[numImages];
  imgs[0] = loadImage("img1.jpg");
  imgs[1] = loadImage("img2.jpg");
  imgs[2] = loadImage("img3.jpg");

  avgImg = createImage(imgs[0].width, imgs[0].height, RGB);
}

void draw() {
  // Dibujar la imagen promedio según el valor del potenciómetro
  background(0);
  calcAverage(mixAmount);
  image(avgImg, 0, 0, width, height);
  
  fill(255);
  textSize(20);
  text("Mezcla: " + nf(mixAmount, 1, 2), 20, height - 20);
}

void serialEvent(Serial p) {
  String val = p.readStringUntil('\n');
  if (val != null) {
    val = trim(val);
    float sensor = float(val);
    mixAmount = map(sensor, 0, 1023, 0, 1); // 0 a 1
  }
}

void calcAverage(float t) {
  avgImg.loadPixels();

  for (int i = 0; i < avgImg.pixels.length; i++) {
    color c1 = imgs[0].pixels[i];
    color c2 = imgs[1].pixels[i];
    color c3 = imgs[2].pixels[i];

    // Promedio ponderado según el potenciómetro
    float r = red(c1)*(1-t) + red(c2)*t*0.5 + red(c3)*t*0.5;
    float g = green(c1)*(1-t) + green(c2)*t*0.5 + green(c3)*t*0.5;
    float b = blue(c1)*(1-t) + blue(c2)*t*0.5 + blue(c3)*t*0.5;

    avgImg.pixels[i] = color(r, g, b);
  }
  avgImg.updatePixels();
}

```

##### Ejercicio 14: Promedio de imagenes + potenciometro
Codigo Arduino

```js
void setup() {
  Serial.begin(9600);
}

void loop() {
  int potValue = analogRead(A0);
  Serial.println(potValue);
  delay(20);
}
```
Codigo Processing

```js
// --- Librerías necesarias ---
// Importa la librería de comunicación serial para conectar con Arduino
import processing.serial.*;
// Importa la clase File de Java para listar archivos y carpetas
import java.io.File;

// --- Comunicación serial con Arduino ---
// Variable que contendrá el objeto de puerto serial (conexión con Arduino)
Serial myPort;
// Variable que guarda el valor leído del potenciómetro (0..1023)
float potValue = 0;

// --- Variables de imágenes ---
// Arreglo dinámico que contendrá todas las imágenes cargadas desde la carpeta
PImage[] imgs;
// Imagen donde se almacenará el resultado del promedio/interpolación
PImage avgImg;

// --- Configuración inicial ---
void setup() {
  // Define el tamaño de la ventana de Processing (ancho, alto)
  size(745, 1024);
  
  // Cargar imágenes desde carpeta "data/imagenes"
  // Llama a la función que busca todas las imágenes dentro de esa carpeta
  imgs = loadImagesFromFolder("imagenes");
  // Imprime en la consola cuántas imágenes se cargaron (útil para debug)
  println("Imágenes cargadas: " + imgs.length);
  
  // Redimensionar todas las imágenes al tamaño del lienzo para que coincidan pixel a pixel
  for (int i = 0; i < imgs.length; i++) {
    imgs[i].resize(width, height); // redimensiona cada imagen al ancho y alto de la ventana
  }
  
  // Crea una imagen vacía del tamaño del lienzo donde guardaremos el promedio
  avgImg = createImage(width, height, RGB);
  
  // Conectar con Arduino (ver lista de puertos)
  // Muestra en consola la lista de puertos seriales disponibles (para identificar cuál usar)
  printArray(Serial.list());
  // Alternativa automática (comentada): abrir el primer puerto disponible a 9600 baudios
  myPort = new Serial(this, Serial.list()[0], 9600);
  // Abrir un puerto específico (ejemplo para macOS). Ajusta según el puerto real en tu sistema.
  //myPort = new Serial(this, "/dev/cu.usbmodem1101", 9600);
  // Nota: si no funciona el puerto, revisa la salida de printArray(Serial.list()) y usa el nombre correcto.
}

// --- Bucle principal ---
// draw() se ejecuta continuamente (aprox. 60 veces por segundo)
void draw() {
  // Pinta el fondo de negro en cada frame
  background(0);
  // Llama a la función que lee datos desde el puerto serial (actualiza potValue)
  readSerial();
  
  // Si no hay imágenes o sólo hay una, no hacemos nada (necesitamos al menos 2 para interpolar)
  if (imgs == null || imgs.length < 2) return;
  
  // Mapear el valor del potenciómetro (0..1023) al rango de índices entre 0 y imgs.length-1
  // Esto permite moverse a lo largo de la secuencia de imágenes
  float mixValue = map(potValue, 0, 1023, 0, imgs.length - 1);
  
  // Calcular el promedio/interpolación entre las dos imágenes vecinas según mixValue
  avgImagesWeighted(mixValue);
  
  // Mostrar la imagen promedio resultante en la pantalla, en la posición (0,0)
  image(avgImg, 0, 0);
  
  // Mostrar texto con el valor actual del potenciómetro en la esquina inferior izquierda
  fill(255); // color blanco para el texto
  text("Valor pot: " + nf(potValue, 1, 0), 10, height - 10); // nf para formatear el número
}

// --- Función que calcula el promedio ponderado entre imágenes ---
// mix es un valor flotante que indica la posición entre imágenes (ej. 2.3 -> entre img2 e img3)
void avgImagesWeighted(float mix) {
  // Accede al arreglo de píxeles de avgImg para poder modificarlos directamente
  avgImg.loadPixels();
  
  // Asegura que mix esté dentro del rango válido [0, imgs.length - 1]
  mix = constrain(mix, 0, imgs.length - 1);
  
  // i1 es el índice de la imagen "inferior" (por ejemplo 2 en 2.3)
  int i1 = floor(mix);
  // i2 es la imagen siguiente (i1 + 1), pero sin pasarse del último índice
  int i2 = min(i1 + 1, imgs.length - 1);
  // t es la fracción entre i1 e i2 (por ejemplo, 0.3 si mix es 2.3)
  float t = mix - i1;
  
  // Cargar los píxeles de las dos imágenes que vamos a mezclar
  imgs[i1].loadPixels();
  imgs[i2].loadPixels();
  
  // Recorre todos los píxeles de la imagen objetivo
  for (int i = 0; i < avgImg.pixels.length; i++) {
    // Coge el color del píxel i de la imagen i1
    color c1 = imgs[i1].pixels[i];
    // Coge el color del píxel i de la imagen i2
    color c2 = imgs[i2].pixels[i];
    
    // Interpola por separado cada componente de color (rojo, verde, azul)
    // red(c1) obtiene la componente roja del color c1
    float r = lerp(red(c1), red(c2), t);
    // green(c1) obtiene la componente verde del color c1
    float g = lerp(green(c1), green(c2), t);
    // blue(c1) obtiene la componente azul del color c1
    float b = lerp(blue(c1), blue(c2), t);
    
    // Crea un nuevo color a partir de las componentes interpoladas y lo asigna al píxel i
    avgImg.pixels[i] = color(r, g, b);
  }
  
  // Aplica los cambios realizados en el arreglo de píxeles a la imagen avgImg
  avgImg.updatePixels();
}

// --- Leer valor del potenciómetro desde Arduino ---
// Lee datos desde el puerto serial hasta encontrar saltos de línea y los convierte a número
void readSerial() {
  // Mientras el puerto exista y tenga bytes disponibles para leer...
  while (myPort != null && myPort.available() > 0) {
    // Lee una línea completa hasta '\n' (salto de línea)
    String val = myPort.readStringUntil('\n');
    if (val != null) {
      // Elimina espacios y caracteres de control al inicio/final
      val = trim(val);
      // Si la cadena no está vacía, la convierte a float y la asigna a potValue
      if (val.length() > 0) {
        potValue = float(val);
      }
    }
  }
}

// --- Cargar todas las imágenes desde una carpeta ---
// Devuelve un arreglo PImage[] con todas las imágenes JPG/PNG encontradas en data/folderName
PImage[] loadImagesFromFolder(String folderName) {
  // Construye la ruta absoluta a la carpeta dentro de la carpeta data del sketch
  String path = sketchPath("data/" + folderName);
  // Crea un objeto File apuntando a esa carpeta
  File folder = new File(path);
  // Lista todos los archivos dentro de la carpeta (puede devolver null si no existe)
  File[] files = folder.listFiles();
  
  // Si files es null, la carpeta no existe o no tiene permisos -> avisar y devolver null
  if (files == null) {
    println("Carpeta no encontrada: " + path);
    return null;
  }
  
  // Crea una lista dinámica para almacenar las PImage cargadas
  ArrayList<PImage> loaded = new ArrayList<PImage>();
  // Recorre cada archivo encontrado en la carpeta
  for (File f : files) {
    // Obtiene el nombre del archivo y lo convierte a minúsculas para comparar extensiones
    String fname = f.getName().toLowerCase();
    // Si termina en .jpg o .png, lo cargamos
    if (fname.endsWith(".jpg") || fname.endsWith(".png")) {
      // loadImage busca en data/folderName el archivo y devuelve un PImage
      PImage img = loadImage(folderName + "/" + f.getName());
      // Si la imagen se cargó correctamente, la agregamos a la lista
      if (img != null) loaded.add(img);
    }
  }
  
  // Convierte la ArrayList a un arreglo PImage[] y lo retorna
  return loaded.toArray(new PImage[loaded.size()]);
}
```
##### Ejercicio Caos organico: Nota 2

Codigo Arduino

```js
// --- Pines de conexión ---
const int btnColor = 4;
const int btnSize  = 6;
const int btnDrop  = 8;
const int btnReset = 10;

const int joyX = A0;
const int joyY = A1;
const int joyBtn = 2;  // si usas el botón del joystick

// LED RGB (de cátodo común)
const int ledR = 9;
const int ledG = 11;
const int ledB = 13;

void setup() {
  Serial.begin(9600);

  // Botones con resistencia pull-up interna
  pinMode(btnColor, INPUT_PULLUP);
  pinMode(btnSize,  INPUT_PULLUP);
  pinMode(btnDrop,  INPUT_PULLUP);
  pinMode(btnReset, INPUT_PULLUP);
  pinMode(joyBtn, INPUT_PULLUP);

  // LED RGB como salida
  pinMode(ledR, OUTPUT);
  pinMode(ledG, OUTPUT);
  pinMode(ledB, OUTPUT);

  apagarLED();
}

void loop() {
  // Lectura de botones (invertida por INPUT_PULLUP)
  int bColor = !digitalRead(btnColor);
  int bSize  = !digitalRead(btnSize);
  int bDrop  = !digitalRead(btnDrop);
  int bReset = !digitalRead(btnReset);

  // Lectura del joystick
  int xVal = analogRead(joyX);
  int yVal = analogRead(joyY);
  xVal = map(xVal, 0, 1023, -100, 100);
  yVal = map(yVal, 0, 1023, -100, 100);

  // Enviar datos a Processing
  Serial.print(xVal); Serial.print(",");
  Serial.print(yVal); Serial.print(",");
  Serial.print(bColor); Serial.print(",");
  Serial.print(bSize); Serial.print(",");
  Serial.print(bDrop); Serial.print(",");
  Serial.println(bReset);

  // Recibir mensajes desde Processing
  if (Serial.available() > 0) {
    String msg = Serial.readStringUntil('\n');
    msg.trim();

    if (msg == "RED") parpadeoRojo();
    else if (msg == "GREEN") parpadeoVerde();
    else if (msg == "BLUE") parpadeoAzul();
    else if (msg == "RESET") apagarLED();
  }

  delay(100);
}

// --- Funciones de LED RGB ---
void parpadeoRojo() {
  analogWrite(ledR, 255);
  analogWrite(ledG, 0);
  analogWrite(ledB, 0);
  delay(100);
  apagarLED();
}

void parpadeoVerde() {
  analogWrite(ledR, 0);
  analogWrite(ledG, 255);
  analogWrite(ledB, 0);
  delay(100);
  apagarLED();
}

void parpadeoAzul() {
  analogWrite(ledR, 0);
  analogWrite(ledG, 0);
  analogWrite(ledB, 255);
  delay(100);
  apagarLED();
}

void apagarLED() {
  analogWrite(ledR, 0);
  analogWrite(ledG, 0);
  analogWrite(ledB, 0);
}
```
Codigo Processing

```js
// --- Comunicación Serial con Arduino ---
import processing.serial.*;
Serial myPort;

// --- Variables del juego ---
ArrayList<Forma> formas;
Forma activa;
boolean cayendo = false;

// Paleta de colores inspirada en Jean Arp ("Plant Hammer")
color[] paleta = {
  #F5D5AE, #EBA89A, #D7C7D0, #9FC7AA, #7AB8BF
};

void setup() {
  size(800, 800);
  smooth();
  colorMode(HSB, 360, 100, 100);
  noStroke();

  // --- Configurar el puerto correcto (CAMBIA si no es COM6) ---
  String portName = "COM6";
  println("Intentando conectar con " + portName);
  myPort = new Serial(this, portName, 9600);
  myPort.bufferUntil('\n');

  // --- Inicialización del juego ---
  formas = new ArrayList<Forma>();
  activa = new Forma(width/2, 100, paleta[0], 40);
}

void draw() {
  background(245, 15, 95);

  // Dibujar todas las formas
  for (Forma f : formas) {
    f.display();
  }

  // Dibujar la forma activa
  activa.display();

  // Movimiento y caída
  if (cayendo) {
    activa.caer();
  }

  // Si la forma activa toca el suelo
  if (activa.y > height - activa.tam/2) {
    formas.add(activa);
    activa = new Forma(width/2, 100, paleta[int(random(paleta.length))], 40);
    cayendo = false;
  }

  // Verificar colisiones entre formas
  verificarColisiones();
}

// --- Lectura de datos desde Arduino ---
void serialEvent(Serial p) {
  String data = trim(p.readStringUntil('\n'));
  if (data == null) return;

  String[] v = split(data, ',');
  if (v.length != 6) return;

  int joyX = int(v[0]);
  int joyY = int(v[1]);
  int bColor = int(v[2]);
  int bSize  = int(v[3]);
  int bDrop  = int(v[4]);
  int bReset = int(v[5]);

  // Movimiento horizontal con el joystick
  activa.x += joyX * 0.05;
  activa.x = constrain(activa.x, activa.tam/2, width - activa.tam/2);

  // --- Controles de botones ---
  if (bColor == 1) {
    activa.cambiarColor();
  }

  if (bSize == 1) {
    activa.cambiarTam();
  }

  if (bDrop == 1 && !cayendo) {
    cayendo = true;
    myPort.write("GREEN\n");  // Indica caída (LED verde)
  }

  if (bReset == 1) {
    reiniciarJuego();
    myPort.write("RESET\n");
  }
}

// --- Verificar colisiones entre formas ---
void verificarColisiones() {
  for (int i = 0; i < formas.size(); i++) {
    Forma f = formas.get(i);
    float d = dist(f.x, f.y, activa.x, activa.y);
    if (d < (f.tam/2 + activa.tam/2)) {
      // Si son del mismo color y tamaño → fusionar
      if (f.col == activa.col && abs(f.tam - activa.tam) < 5) {
        myPort.write("BLUE\n"); // LED azul por fusión
        formas.remove(i);
        activa = new Forma(f.x, f.y, f.col, f.tam * 1.3);
        return;
      } else {
        // Si son diferentes → colapso
        myPort.write("RED\n"); // LED rojo por colapso
        reiniciarJuego();
        return;
      }
    }
  }
}

// --- Reiniciar el juego ---
void reiniciarJuego() {
  formas.clear();
  activa = new Forma(width/2, 100, paleta[int(random(paleta.length))], 40);
  cayendo = false;
  println("Juego reiniciado");
}

// --- Clase de Forma orgánica ---
class Forma {
  float x, y, tam;
  color col;

  Forma(float x, float y, color c, float t) {
    this.x = x;
    this.y = y;
    this.col = c;
    this.tam = t;
  }

  void display() {
    fill(col);
    beginShape();
    for (float a = 0; a < TWO_PI; a += 0.3) {
      float r = tam/2 + 10 * noise(x * 0.01 + cos(a), y * 0.01 + sin(a));
      vertex(x + cos(a) * r, y + sin(a) * r);
    }
    endShape(CLOSE);
  }

  void caer() {
    y += 3;
  }

  void cambiarColor() {
    col = paleta[int(random(paleta.length))];
  }

  void cambiarTam() {
    tam = random(30, 80);
  }
}
```


