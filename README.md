# ODB1_Firmware

![GAUL Banner](doc/logo-full.webp)

Code for the on-board computer (ODB1) of the GAUL for the Launch Canada 2024 competition.

## LAUNCH CANADA 2024

Seulement le mode INFLIGHT est utilisé. Voici les données envoyées:

```
- header.flightmode (toujours 01)
- header.pyro0 (toujours 0 car les igniters ne sont pas installées pour launch canada)
- header.pyro1 (toujours 0 car les igniters ne sont pas installées pour launch canada)
- header.accelerometer (s'il y a une erreur avec l'ICM pendant l'initialisation de la fusée)
- header.barometer (s'il y a une erreur avec le BMP280 pendant l'initialisation de la fusée)
- header.gps_fix (si le gps a un fix avec les satellites)
- header.sd (s'il y a une erreur avec la carte sd pendant l'initialisation de la fusée)

- altitude_filtered (metre, float)
- temperature (C, float)
- latitude (deg, float)
- longitude (deg, float)
- gyroX (deg, float)
- gyroY (deg, float)
- gyroZ (deg, float)
- accX (deg, float)
- accY (deg, float)
- accZ (deg, float)
- kalmanRoll (deg, float)
- kalmanPitch (deg, float)
```

L'ICM (accéléromètre) est utilisé pour déterminer si le moteur est en train de pousser.

Si le moteur n'est pas en train de pousser, le BMP280 est utilisé pour savoir si la fusée est en train de monter ou de descendre.

Si la fusée est en train de descendre et que le moteur est éteint, déclancher le pyro0 (drogue chute).

Si le drogue est déployé et que l'altitude est inférieur à un threshold, déclancher le pyro1 (main chute).

**[Driver details](doc/driver.md)**
