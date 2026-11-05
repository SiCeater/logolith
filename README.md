# logolith

Firmware de contrôleur de vol d'Aerolith, un cinelifter maison qui porte un hybride plein format (classe A7S III).
Écrit de zéro, sur FreeRTOS, pour la LithCore (STM32F405RGT6), en restant portable vers d'autres cartes.

> Branche `dev-robin` : base minimale. Un bootloader USB maison, et une application FreeRTOS dont une tâche fait
> clignoter la LED bleue et une autre gère le port série USB.

## Arborescence

```
app/                  Application : main, tâches, et sa configuration (config/ : FreeRTOS, TinyUSB).
bootloader/           Bootloader USB DFU : lance l'application ou en reçoit une nouvelle.
boards/lithcore/      Tout ce qui est propre à la carte : brochage, horloges, plan mémoire, options du MCU.
cmake/                Toolchain, dépendances, fonctions de build (bin/hex, flash).
tools/                Scripts côté PC (flash USB).
.github/workflows/    CI : compile debug et release à chaque push.
```

Seul `boards/` touche aux registres de la carte. Les dépendances (FreeRTOS-Kernel, TinyUSB, CMSIS 6, CMSIS du
STM32F4) ne sont pas copiées dans le dépôt : CMake les télécharge au premier configure, sur un tag figé dans
[cmake/dependencies.cmake](cmake/dependencies.cmake).

Pour porter sur une autre carte : créer `boards/<carte>/` avec la même interface que `board.h` et son propre
`memory.ld`, puis configurer avec `-DLOGOLITH_BOARD=<carte>`.

## Plan mémoire

| Zone | Adresse | Taille | Contenu |
| --- | --- | --- | --- |
| Secteurs 0-1 | `0x08000000` | 32 Ko | Bootloader |
| Secteurs 2-3 | `0x08008000` | 32 Ko | Réservé aux réglages (pas encore utilisé) |
| Secteurs 4-11 | `0x08010000` | 960 Ko | Application |
| Fin de RAM | `0x2001FFF0` | 16 o | Demande de passage en DFU, survit au reset |

Défini une seule fois, dans [boards/lithcore/memory.ld](boards/lithcore/memory.ld).

## Prérequis

- [Arm GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) (`arm-none-eabi-gcc` 12 ou plus),
  dans le `PATH` ou indiquée par la variable d'environnement `ARM_GCC_PATH` (dossier `bin/`).
- CMake 3.25 ou plus, et Ninja.
- Python 3 avec `pip install -r tools/requirements.txt` (pyserial, pyusb, libusb-package).
- [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html), seulement pour la première
  installation et le flash par SWD.

## Compiler

```bash
cmake --workflow --preset debug     # ou release
```

Sortie : `build/debug/app/logolith.{elf,bin,hex,map}` et `build/debug/bootloader/bootloader.{elf,bin,hex,map}`.

## Flasher

```bash
cmake --build --preset debug --target flash
```

[tools/flash.py](tools/flash.py) choisit le chemin selon ce qui est branché :

1. **Application en marche** (port série « LithCore », `0483:5740`) : le script ouvre le port à 1200 bauds puis coupe
   DTR, comme pour les cartes Arduino et RP2040. L'application se débranche, écrit une demande en fin de RAM et
   redémarre. Le bootloader la lit et reste en DFU au lieu de lancer l'application.
2. **Bootloader en DFU** (« LithCore bootloader », `1209:0001`) : le script envoie `logolith.bin` par DFU 1.1, le
   bootloader l'écrit puis redémarre sur la nouvelle application. Compter 4 s en tout.
3. **Bootloader ROM de ST** (bouton BOOT maintenu au branchement) : le script installe bootloader et application avec
   STM32CubeProgrammer. C'est la première installation, et le recours si le bootloader lui-même est cassé.

Sous Windows, le bootloader annonce des descripteurs Microsoft OS 2.0 : WinUSB se charge tout seul, sans Zadig. La
toute première connexion sur un PC prend une quinzaine de secondes, le temps de cette installation.

Le bootloader garde la table des vecteurs de l'application (premier bloc) en RAM et ne l'écrit qu'à la fin du
transfert. Un flash interrompu laisse donc une application invalide : le bootloader reste en DFU, et il suffit de
relancer la commande.

Par SWD avec une sonde ST-Link (bootloader + application) :

```bash
cmake --build --preset debug --target flash-swd
```

Le PID `1209:0001` est le PID de test de [pid.codes](https://pid.codes) : à remplacer par un PID attribué avant de
diffuser la carte.

## LED d'état

| LED | Signification |
| --- | --- |
| Bleue qui clignote (1 Hz) | Application : l'ordonnanceur tourne |
| Rouge fixe, bleue éteinte | Application : erreur fatale du noyau (assert ou dépassement de pile) |
| Rouge fixe, bleue qui clignote | Application : le quartz 8 MHz n'a pas démarré, la carte tourne sur l'oscillateur interne |
| Rouge qui clignote vite | Bootloader en mode DFU, prêt à recevoir une application |
| Bleue très rapide, rouge éteinte | Bootloader planté (HardFault) |

## Licence

[MIT](LICENSE)
