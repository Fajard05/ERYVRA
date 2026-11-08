#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// #include <SoftwareSerial.h>   // BT deshabilitado (módulo no conectado)

// ---------------- OLED 128x64 I2C ----------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR    0x3C
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ---------------- Bluetooth HC-05 (RX, TX) ----------------
// SoftwareSerial BT(2, 3);   // BT deshabilitado (módulo no conectado)

// ---------------- Pines de hardware ----------------
const int pinSensor  = A0;
const int ledVerde   = 8;
const int ledAmarillo = 9;
const int ledRojo    = 10;

// ---------------- Parámetros de medición ----------------
const unsigned long DURACION_PRUEBA   = 30000UL;  // 30 s
const unsigned long INTERVALO_MUESTRA = 20UL;     // 50 muestras/s (no bloqueante)
const unsigned long ANTI_REBOTE       = 300UL;    // máx. 200 BPM (corta dobles detecciones)
const int  AMPLITUD_MINIMA            = 25;       // amplitud mínima para aceptar latidos
const int  N_FILTRO                   = 4;        // muestras del promedio móvil
const int  ZONA_GRAFICA_Y             = 15;       // filas 0..14 = encabezado

// ---------------- Estado de la prueba ----------------
bool pruebaActiva        = false;
unsigned long tiempoInicio = 0;
unsigned long ultimaMuestra = 0;

// ---------------- Filtro de suavizado (promedio móvil) ----------------
int  muestrasFiltro[N_FILTRO];
long sumaFiltro = 0;
int  idxFiltro = 0;

// ---------------- Algoritmo de pulso (umbral adaptativo) ----------------
int  maxReciente = 0;
int  minReciente = 1023;
unsigned long ultimoDecay = 0;
bool latidoDetectado = false;
unsigned long ultimaContada = 0;
int  contadorLatidos = 0;

// ---------------- Gráfica OLED ----------------
int xPos = 0;
int yPosAnt = 32;
int ultimoSegundo = -1;
int ultimoBpmMostrado = -1;

void setup() {
  pinMode(ledVerde, OUTPUT);
  pinMode(ledAmarillo, OUTPUT);
  pinMode(ledRojo, OUTPUT);
  apagarLEDs();

  // BT.begin(9600);         // BT deshabilitado (módulo no conectado)
  Wire.setClock(400000);   // I2C a 400 kHz: display ~4x más rápido

  // Si la OLED no responde, el sistema sigue funcionando (LEDs)
  oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);

  // BT.println("{\"evento\":\"ready\"}");   // BT deshabilitado
  iniciarPrueba();
}

void loop() {
  unsigned long ahora = millis();

  if (pruebaActiva) {
    // Fin de la prueba
    if (ahora - tiempoInicio >= DURACION_PRUEBA) {
      mostrarResultado();
      return;
    }
    // Muestreo a intervalo fijo y no bloqueante
    if (ahora - ultimaMuestra >= INTERVALO_MUESTRA) {
      ultimaMuestra = ahora;
      muestrear(ahora);
      oled.display();
    }
  } else {
    // Espera de reinicio por Bluetooth (deshabilitado: módulo no conectado).
    // Sin BT, reinicia con el pulsador RESET del Arduino.
    // if (BT.available()) {
    //   char c = BT.read();
    //   if (c == 'R' || c == 'r') iniciarPrueba();
    // }
  }
}

// -------------------------------------------------------------
// FASE 1: muestreo, detección de latidos y graficado
// -------------------------------------------------------------
void muestrear(unsigned long ahora) {
  // -- Filtro de promedio móvil: mata el ruido del sensor sensible --
  sumaFiltro -= muestrasFiltro[idxFiltro];
  muestrasFiltro[idxFiltro] = analogRead(pinSensor);
  sumaFiltro += muestrasFiltro[idxFiltro];
  idxFiltro++;
  if (idxFiltro >= N_FILTRO) idxFiltro = 0;
  int valor = (int)(sumaFiltro / N_FILTRO);

  // -- Umbral adaptativo: seguimiento de pico/mínimo con decaimiento --
  unsigned long dt = ahora - ultimoDecay;
  ultimoDecay = ahora;
  int paso = 1 + (int)((dt * 20UL) / 1000UL);   // decae ~20 cuentas/s

  if (valor > maxReciente) maxReciente = valor;
  else                     maxReciente = max(maxReciente - paso, valor);

  if (valor < minReciente) minReciente = valor;
  else                     minReciente = min(minReciente + paso, valor);

  int amplitud   = maxReciente - minReciente;
  int umbralAlto = minReciente + (amplitud * 3) / 4;  // entrada al latido (más exigente)
  int umbralBajo = minReciente + amplitud / 4;        // salida: exige caída real (corta el bajo diacrótico)

  // -- Detección de latido con histéresis + periodo refractario --
  if (!latidoDetectado && amplitud >= AMPLITUD_MINIMA && valor >= umbralAlto) {
    latidoDetectado = true;
    if (ahora - ultimaContada >= ANTI_REBOTE) {
      contadorLatidos++;
      ultimaContada = ahora;
    }
  } else if (latidoDetectado && valor <= umbralBajo) {
    latidoDetectado = false;
  }

  // -- Dibujo de la onda (auto-escalado según la amplitud actual) --
  int centro = (maxReciente + minReciente) / 2;
  int media  = max(25, amplitud / 2 + 10);
  int yPos = map(valor, centro - media, centro + media, 63, ZONA_GRAFICA_Y);
  yPos = constrain(yPos, ZONA_GRAFICA_Y, 63);

  oled.drawLine(xPos - 1, yPosAnt, xPos, yPos, SSD1306_WHITE);
  yPosAnt = yPos;
  xPos++;

  if (xPos >= SCREEN_WIDTH) {           // reinicia eje X
    xPos = 0;
    oled.fillRect(0, ZONA_GRAFICA_Y, SCREEN_WIDTH,
                  SCREEN_HEIGHT - ZONA_GRAFICA_Y, SSD1306_BLACK);
  }

  // -- Encabezado: solo se redibuja cuando cambia el segundo o el BPM --
  unsigned long transcurrido = ahora - tiempoInicio;
  int segundosRestantes = (int)((DURACION_PRUEBA - transcurrido) / 1000);
  int bpmActual = (transcurrido > 0)
                    ? (int)((contadorLatidos * 60000UL) / transcurrido)
                    : 0;

  if (segundosRestantes != ultimoSegundo || bpmActual != ultimoBpmMostrado) {
    oled.fillRect(0, 0, SCREEN_WIDTH, ZONA_GRAFICA_Y, SSD1306_BLACK);
    oled.setCursor(0, 0);
    oled.print("Midiendo ");
    oled.print(segundosRestantes);
    oled.print("s  ");
    oled.print(bpmActual);
    oled.print("bpm");
    ultimoSegundo = segundosRestantes;
    ultimoBpmMostrado = bpmActual;
  }
}

// -------------------------------------------------------------
// FASE 2: resultado, LEDs y trama JSON
// -------------------------------------------------------------
void mostrarResultado() {
  int bpmFinal = (int)((contadorLatidos * 60000UL) / DURACION_PRUEBA);
  const char* estado;

  apagarLEDs();

  if (contadorLatidos == 0) {
    estado = "Sin senal";                    // dedo fuera / ruido
    digitalWrite(ledAmarillo, HIGH);
  } else if (bpmFinal < 60) {
    estado = "Bradicardia";
    digitalWrite(ledAmarillo, HIGH);
  } else if (bpmFinal <= 100) {
    estado = "Normal";
    digitalWrite(ledVerde, HIGH);
  } else {
    estado = "Taquicardia";
    digitalWrite(ledRojo, HIGH);
  }

  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.println("RESULTADO FINAL (30s)");
  oled.setTextSize(2);
  oled.setCursor(0, 18);
  oled.print(bpmFinal);
  oled.println(" BPM");
  oled.setTextSize(1);
  oled.setCursor(0, 45);
  oled.print("Estado: ");
  oled.println(estado);
  oled.setCursor(0, 55);
  oled.print("Pulsa RESET p/repetir");
  oled.display();

  // Trama JSON por Bluetooth (deshabilitado: módulo no conectado)
  // char json[80];
  // snprintf(json, sizeof(json), "{\"bpm\":%d,\"estado\":\"%s\"}", bpmFinal, estado);
  // BT.println(json);   // la app debe leer hasta '\n'

  pruebaActiva = false;
}

// -------------------------------------------------------------
// Reinicio de la prueba
// -------------------------------------------------------------
void iniciarPrueba() {
  contadorLatidos   = 0;
  latidoDetectado   = false;
  maxReciente       = 0;
  minReciente       = 1023;
  xPos              = 0;
  yPosAnt           = 32;
  ultimoSegundo     = -1;
  ultimoBpmMostrado = -1;
  sumaFiltro        = 0;
  idxFiltro         = 0;
  for (int i = 0; i < N_FILTRO; i++) muestrasFiltro[i] = 0;
  apagarLEDs();

  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);
  oled.println("Coloca el dedo en");
  oled.println("el sensor...");
  oled.println();
  oled.println("Medicion: 30 segundos");
  oled.display();

  // Calentamiento: muestrea el sensor para inicializar min/max
  unsigned long t0 = millis();
  while (millis() - t0 < 1500) {
    int v = analogRead(pinSensor);
    if (v > maxReciente) maxReciente = v;
    if (v < minReciente) minReciente = v;
    delay(20);
  }
  ultimaContada = millis() - ANTI_REBOTE;   // el 1er latido siempre cuenta

  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.print("Midiendo 30s");
  oled.display();

  tiempoInicio  = millis();
  ultimaMuestra = tiempoInicio;
  ultimoDecay   = tiempoInicio;
  pruebaActiva  = true;
}

void apagarLEDs() {
  digitalWrite(ledVerde, LOW);
  digitalWrite(ledAmarillo, LOW);
  digitalWrite(ledRojo, LOW);
}
