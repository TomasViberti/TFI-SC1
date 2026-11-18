# Control de velocidad de un motor DC para una cinta transportadora de envases

Trabajo Final Integrador de **Sistemas de Control 1**, FCEFyN - UNC (2024).
Autor: Tomás Viberti.

## Descripción

Control de velocidad de la cinta transportadora de una máquina de reciclaje de
envases retornables (RVM), accionada por un motor DC de imán permanente
(Maxon A-max 22 con reductora GP 22 A). El trabajo incluye:

- Modelado de la planta y funciones de transferencia (motor, reductora, driver, sensor).
- Análisis de estabilidad, tipo de sistema y respuesta temporal.
- Diseño de un controlador PI por cancelación del polo dominante, con lugar de raíces.
- Simulación de la respuesta al escalón, la acción de control y el rechazo de la
  perturbación por ingreso de un envase.

## Contenido

- `Viberti.TFI.SCI 2024.pdf`: informe del trabajo.
- `TFI_cinta_transportadora_envases.m`: script de Octave con todas las simulaciones (Secciones 2 a 5).
