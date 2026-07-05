# Notes terrain

## Ce qui a ete confirme

- l'ESP32-S3 peut lire le flux lidar via UART sans blocage de `loop()`
- le webserver local expose la carte 2D
- `freq 20000` est le reglage PWM le plus stable observe
- `pwm 500` est un bon point de depart pour la rotation

## Ce qui reste a ajuster

- la calibration angulaire fine si un offset visuel subsiste
- les seuils de rejet si un environnement reflecissant genere encore quelques faux points
- un mode de test plus serre si on veut comparer plusieurs rapports PWM/frequence
