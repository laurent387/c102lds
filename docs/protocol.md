# Notes protocole LDS C102

Le flux observe sur ce capteur correspond a des trames qui commencent par:

```text
55 AA
```

Points retenus pendant le reverse:

- chaque paquet contient plusieurs mesures de distance et de qualite
- les angles sont reconstruits a partir des champs de debut/fin de paquet
- le flux utile a ete vu a `115200` bauds sur le banc
- les commandes `start`, `stop` et `startlds$` sont conservees comme essais de dialogue

## Remarques de decodage

- le firmware actuel ne traite pas ce capteur comme un LDS-02 TurtleBot3 classique
- le decodeur ajuste les points sur `360` bins
- les points parasites restants ont ete filtres par qualite et voisinage angulaire

## Hypotheses de travail

- `pwm 500`
- `freq 20000`
- `baud 115200`

Ces valeurs sont celles qui ont donne le meilleur comportement mecanique et le flux le plus exploitable lors des essais.
