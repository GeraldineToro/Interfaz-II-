# Interfaz-II-

##### Ejercicio n° 1: Cueck

```js
void setup() {
  Serial.begin(9600); // Inicia la comunicación serie a 9600 bps
  Serial.println("Cueck"); // Envía "Cueck" al monitor serie
}

void loop() {
  // No es necesario poner nada en el loop para este ejemplo
}
```
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
