# Actuador – Proyecto Final Ingeniería Electrónica

Firmware para un nodo actuador basado en **ESP32-C6**, encargado de regular la
apertura de un servomotor en función de una consigna de temperatura recibida
por **MQTT**. El controlador implementado es un **PI discreto** obtenido por
discretización vía Tustin a partir de un diseño continuo en MATLAB.

## Características principales

- Microcontrolador: ESP32-C6 (entorno Arduino).
- Actuador: servomotor controlado por PWM (50 Hz, 500–2400 µs).
- Control:
  - Controlador PI discreto (Tustin).
  - Salida lógica 0–100 % mapeada a 0–180° de servo.
  - Lazo ejecutado periódicamente mediante tarea FreeRTOS.
- Comunicación:
  - Conexión WiFi en modo estación.
  - Enlace MQTT con broker configurado en la red de laboratorio.
  - Suscripción a:
    - `emu/t_sp`  – consigna de temperatura (SP).
    - `emu/t_act` – temperatura de proceso (PV).
    - `emu/kp_set` – ganancia proporcional.
    - `emu/ti_set` – tiempo integral.
  - Publicación de:
    - `emu/servo_pos` – posición del servo (grados).
    - `emu/u_out`     – salida lógica del PI (0–100 %).

## Estructura del proyecto

```text
.
├─ src/              # Código fuente (main.cpp, etc.)
├─ include/          # Headers (si aplica)
├─ docs/
│  ├─ html/          # Documentación HTML generada por Doxygen
│  └─ latex/         # refman.tex / refman.pdf (documentación LaTeX)
├─ Doxyfile          # Configuración de Doxygen
└─ README.md
