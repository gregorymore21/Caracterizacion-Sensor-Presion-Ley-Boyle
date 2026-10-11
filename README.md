# Caracterización de un Sensor de Presión Manométrica con la Ley de Boyle

**Informe de Laboratorio N.º 2 — Taller de Equipos Biomédicos II**  
*Escuela Profesional de Ingeniería Biomédica — Universidad Nacional Mayor de San Marcos (UNMSM)*

---

## 1. Resumen

En esta experiencia se buscó caracterizar el sensor piezoresistivo de presión manométrica **MPS20N0040D-D** mediante un esquema de compresión volumétrica basado en la Ley de Boyle-Mariotte, empleando un módulo ADC **HX710B** de 24 bits e interfaz con un microcontrolador ESP32. En las pruebas iniciales a presión atmosférica se verificó un nivel de offset base ($C_{0,i}$) de **3 581 655 cuentas** con un ruido de lectura ($\sigma_0$) de **49.38 cuentas**. 

Asimismo, mediante inyección de agua en la vía neumática, se determinó un volumen muerto no desplazable de $V_{\text{med}} = 0.77\text{ mL}$. Debido a una fuga en las uniones del circuito cerrado, el barrido de presiones de la Ley de Boyle se vió limitado por la pérdida de masa constante. A partir de la prueba de hermeticidad en punto de saturación, se determinó una cota inferior de sensibilidad de **$S \ge 1.45 \times 10^5\text{ cuentas/kPa}$** y una cota superior de resolución de **$R_p \le 0.00034\text{ kPa}$ ($0.0035\text{ cmH}_2\text{O}$)**.

---

## 2. Diagrama Termodinámico y de Sistema

El ensayo se fundamenta en la relación isotérmica de Boyle-Mariotte ($P_1 V_1 = P_2 V_2$), donde el volumen total del gas comprende un volumen variable ($V_j$) accionado por la jeringa y un volumen muerto ($V_m$) entrampado en los acoples y el puerto del sensor.

![Diagrama Termodinámico](fig1_diagrama_termodinamico.png)
*Figura 1. Diagrama termodinámico y volumétrico del sistema neumático (Jeringa graduada, llave de tres vías y sensor).*

![Diagrama de Bloques](fig4_diagrama_bloques.png)
*Figura 2. Diagrama de bloques funcional del circuito neumático, adquisición, procesamiento y visualización.*

---

## 3. Esquemático Eléctrico del Montaje

El transductor piezoresistivo en puente de Wheatstone se acopla al ADC $\Sigma-\Delta$ de 24 bits (HX710B). Las líneas de comunicación serie de dos hilos (SCK y DOUT) se interconectan a los pines GPIO18 y GPIO19 del ESP32, alimentando el módulo a 3.3 V.

![Esquemático del Circuito](fig5_esquematico_circuito.png)
*Figura 3. Circuito esquemático de interconexión entre el ESP32 DevKit V1, el módulo HX710B + MPS20N0040D-D y las puntas del osciloscopio.*

---

## 4. Código Fuente de Adquisición (`caracterizacion_presion.ino`)

El firmware implementado en el ESP32 lee los 24 bits del conversor mediante la manipulación directa de reloj en SCK, promedia un bloque de $N=20$ muestras y evalúa la presión de referencia calculada mediante la Ley de Boyle:

$$P_{\text{ref}} = P_{\text{atm}} \cdot \left( \frac{V_0 - V_j}{V_j + V_m} \right)$$

```cpp
#include <Arduino.h>

// Definición de pines para la conexión
#define HX710_DOUT_PIN 19
#define HX710_SCK_PIN  21

// Parámetros y constantes de configuración
const double ATMOSPHERIC_P_KPA = 101.3;  // Presión atmosférica en kPa
const double INITIAL_VOL_ML    = 1.00;   // Volumen inicial en mL
const double DEAD_VOL_ML       = 0.76;   // Volumen muerto en mL
const double MAX_SAFE_P_KPA    = 35.0;   // Límite de seguridad
const int SAMPLE_COUNT         = 20;     // Muestras promediadas

// Funciones de lectura y cálculo del sensor
bool readSensorRaw(long &rawData);
bool getAverageAndStdDev(double &meanVal, double &stdDevVal);
double computeReferencePressure(double vj);
## 5. Resultados Metrológicos y Respuesta Físico-Digital

### Tabla Consolidada de Parámetros Medidos y Calculados

| Parámetro / Condición | Símbolo | Valor Experimental | Unidad |
| :--- | :---: | :---: | :---: |
| Lecturas promediadas por punto | $N$ | **20** | muestras |
| Offset inicial a presión atmosférica | $C_{0,i}$ | **3 581 655.00** | cuentas |
| Ruido de lectura a cero presión | $\sigma_0$ | **49.38** | cuentas |
| Volumen medio de llave y manguera | $V_{\text{med}}$ | **0.77** | mL |
| Presión atmosférica calculada del laboratorio ($h=75\text{ m}$) | $P_{\text{atm}}$ | **100.40** | kPa |
| Marca de parada de seguridad del émbolo | $V_{j,\text{min}}$ | **0.548** | mL |
| Cota inferior de Sensibilidad | $S$ | **$\ge 1.45 \times 10^5$** | cuentas/kPa |
| Cota superior de Resolución | $R_p$ | **$\le 0.00034$** | kPa |
| Cota superior de Resolución Fisiológica | $R_p$ | **$\le 0.0035$** | $\text{cmH}_2\text{O}$ |

### Gráfico de Lecturas vs. Presión de Referencia ($P_{\text{ref}}$)

![Curva de Cuentas vs Pref](fig11_curva_cuentas_pref.png)
*Figura 4. Relación entre la lectura digital en cuentas y la presión de referencia $P_{\text{ref}}$, destacando el punto de cero y la saturación del ADC.*

---

## 6. Componentes e Instrumental Utilizados

* Microcontrolador ESP32 DevKit V1 (Alimentación a 3.3 V)
* Módulo sensor de presión manométrica MPS20N0040D-D (Rango 0–40 kPa) + ADC HX710B de 24 bits
* Jeringas graduadas de 1 mL (Medición volumétrica y neumática)
* Llave de tres vías y tramo de manguera de silicona
* Osciloscopio digital de banco (Verificación del protocolo de comunicación de 2 hilos)
* Protoboard y jumpers de conexión

---

## 7. Integrantes

* **Dávila Pucuhuayla, Jazmín Sarai** — *Código: 23190369*
* **Leon Vasquez, Jimmy Fabricio** — *Código: 23190378*
* **More Quispe, Gregory Martin** — *Código: 23190125*
* **Sotomayor Villanueva, Genny Solangch** — *Código: 23190393*

**Docente:** Dra. María Elisia Armas Alvarado  
**Curso:** Taller de Equipos Biomédicos II — Universidad Nacional Mayor de San Marcos (UNMSM)
