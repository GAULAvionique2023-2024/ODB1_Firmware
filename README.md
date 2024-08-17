# ODB1_Firmware

![GAUL Banner](doc/logo-full.webp)

Code de l'ODB1 du GAUL pour la compétition Launch Canada 2024.

## LAUNCH CANADA 2024

Seulement le mode INFLIGHT est utilisé. Voici les données qu'il envoie:

```
- header.flightmode (toujours 01)
- header.pyro0 (toujours 0 car les igniters ne sont pas installées pour launch canada)
- header.pyro1 (toujours 0 car les igniters ne sont pas installées pour launch canada)
- header.accelerometer (0 s'il y a une erreur avec l'ICM pendant l'initialisation de la fusée, sinon 1)
- header.barometer (0 s'il y a une erreur avec le BMP280 pendant l'initialisation de la fusée, sinon 1)
- header.gps_fix (en tout temps si le gps a un fix avec les satellites)
- header.sd (0 s'il y a une erreur avec la carte sd pendant l'initialisation de la fusée, sinon 1)

- altitude_filtered (metre, float)
- temperature (C, float)
- latitude (deg, float)
- longitude (deg, float)
- gyroX (deg/s, float)
- gyroY (deg/s, float)
- gyroZ (deg/s, float)
- accX (G, float)
- accY (G, float)
- accZ (G, float)
- angle_roll_acc (deg, float) (yaw de la fusée)
- angle_pitch_acc (deg, float) (pitch de la fusée)
```

L'ICM (accéléromètre) est utilisé pour déterminer si le moteur est allumé.

Si le moteur n'est pas allumé (pas en Mach lock), le BMP280 (baromètre) est utilisé pour savoir si la fusée est en train de monter ou de descendre.

Si la fusée est au sol (altitude inférieur à un threshold), ne rien faire.

Si le moteur est éteint et que la fusée est en train de descendre, déclancher le pyro0 (drogue chute).

Si le drogue est déployé et que l'altitude est inférieur à un threshold, déclancher le pyro1 (main chute).

**Informations sur les drivers:** [Driver details](doc/driver.md)
