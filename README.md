# logolith

Firmware du contrôleur de vol d'Aerolith, cinelifter maison pour hybride plein format (classe A7S III).
Cible : LithCore (STM32F405RGT6). FreeRTOS, écrit de zéro.

## Arborescence

```
app/              application FreeRTOS et sa config (FreeRTOS, TinyUSB)
bootloader/       bootloader USB DFU
boards/lithcore/  brochage, horloges, plan mémoire, options du MCU
cmake/            toolchain, dépendances, cibles de flash
tools/            scripts PC
```

Seul `boards/` accède aux registres. Dépendances (FreeRTOS-Kernel, TinyUSB, CMSIS) téléchargées au configure,
versions figées dans [cmake/dependencies.cmake](cmake/dependencies.cmake).

Autre carte : créer `boards/<carte>/` (même interface que `board.h`, son `memory.ld`), configurer avec
`-DLOGOLITH_BOARD=<carte>`.

## Prérequis

- Arm GNU Toolchain (`arm-none-eabi-gcc` 12+), dans le `PATH` ou via `ARM_GCC_PATH` (dossier `bin/`)
- CMake 3.25+, Ninja
- Python 3 : `pip install -r tools/requirements.txt`
- STM32CubeProgrammer (première installation, SWD)

## Compiler

```bash
cmake --workflow --preset debug     # ou release
```

Sorties dans `build/debug/app/` et `build/debug/bootloader/`.

## Flasher

```bash
cmake --build --preset debug --target flash
```

[tools/flash.py](tools/flash.py) détecte l'état de la carte :

| Carte vue comme | Action |
| --- | --- |
| Port série « LithCore » (`0483:5740`) | ouverture à 1200 bauds, l'application redémarre dans le bootloader, puis flash |
| « LithCore bootloader » (`1209:0001`) | envoi de l'application en DFU 1.1 (environ 4 s) |
| « STM32 BOOTLOADER » (bouton BOOT maintenu au branchement) | installation bootloader + application via STM32CubeProgrammer |

Première installation et bootloader cassé : bouton BOOT. Ensuite plus besoin.

Windows charge WinUSB tout seul pour le bootloader. Première connexion sur un PC : environ 15 s.

Flash interrompu : le bootloader reste en DFU, relancer la commande.

SWD avec un ST-Link (bootloader + application) :

```bash
cmake --build --preset debug --target flash-swd
```

`1209:0001` est le PID de test de [pid.codes](https://pid.codes), à remplacer avant diffusion.

## Plan mémoire

| Zone | Adresse | Taille | Contenu |
| --- | --- | --- | --- |
| Secteurs 0-1 | `0x08000000` | 32 Ko | bootloader |
| Secteurs 2-3 | `0x08008000` | 32 Ko | réservé aux réglages |
| Secteurs 4-11 | `0x08010000` | 960 Ko | application |
| Fin de RAM | `0x2001FFF0` | 16 o | demande de passage en DFU, conservée au reset |

Source : [boards/lithcore/memory.ld](boards/lithcore/memory.ld).

## LED

| LED | Signification |
| --- | --- |
| Bleue 1 Hz | application en marche |
| Rouge fixe, bleue éteinte | erreur fatale du noyau (assert, dépassement de pile) |
| Rouge fixe, bleue 1 Hz | quartz 8 MHz absent, horloge interne |
| Rouge rapide | bootloader en DFU |
| Bleue très rapide | bootloader planté |

## Licence

[MIT](LICENSE)
