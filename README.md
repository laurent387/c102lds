# ESP32-S3 + Xiaomi LDS C102

Repository de travail pour piloter et diagnostiquer un lidar Xiaomi LDS C102 sur ESP32-S3 DevKitC-1 avec Arduino/PlatformIO.

Le firmware fournit:

- PWM moteur sur `GPIO14`
- RX lidar sur `GPIO18`
- TX optionnel sur `GPIO17`, desactive par defaut
- affichage brut HEX/ASCII avec timestamp
- serveur web avec carte 2D
- commandes USB pour le moteur, le baud, le PWM et les essais de trame

## Etat du firmware

Le projet inclus `src/main_s3.cpp`, qui a ete teste sur une carte ESP32-S3 branchée au LDS.

Observations utiles:

- le LDS parle sur une trame `55 AA ...`
- le flux web `/scan` expose les points polaires
- le moteur tient mieux avec `freq=20000`
- un `pwm` autour de `500` a donne une rotation exploitable sur le banc

## Brochage retenu

| ESP32-S3 DevKitC-1 | GPIO | LDS |
| --- | --- | --- |
| GPIO14 | PWM / MOT_EN | commande moteur |
| GPIO18 | RX | TX lidar vers ESP |
| GPIO17 | TX optionnel | RX lidar depuis ESP |
| GND | GND | masse commune |

## Commandes serie USB

```text
help
motor on
motor off
pwm 0-1023
freq 1000
freq 5000
freq 10000
freq 20000
invertpwm 0
invertpwm 1
baud 9600|19200|38400|57600|115200|230400
status
send $
send startlds$
send start
send stop
```

## Webserver

L'ESP32-S3 publie une page 2D et un endpoint JSON:

- `/` ou `/map` : vue carte
- `/scan` : donnees de scan et metriques
- `/cmd?c=...` : envoi de commande

## Build

```bash
pio run
pio run -t upload
pio device monitor -b 115200
```

## Fichiers importants

- `src/main_s3.cpp` : firmware principal
- `docs/pinout.md` : brochage et connexions
- `docs/protocol.md` : notes sur les trames et le decode
- `docs/notes.md` : observations terrain
