# Pinout ESP32-S3

Brochage retenu pour le LDS C102 sur DevKitC-1:

| Signal | GPIO | Remarque |
| --- | --- | --- |
| PWM / MOT_EN | GPIO14 | sortie PWM materielle |
| RX lidar | GPIO18 | entree UART vers l'ESP |
| TX lidar optionnel | GPIO17 | desactive par defaut |
| GND | GND | masse commune obligatoire |

## Notes pratiques

- garder le câblage court sur RX
- partager la masse entre le LDS et l'ESP
- si le rotor pompe, tester d'abord `freq 20000` puis ajuster `pwm`
- l'UART est utilise pour la lecture du flux, pas `SoftwareSerial`

