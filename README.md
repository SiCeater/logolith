# logolith

Firmware de contrôleur de vol d'Aerolith, un cinelifter maison qui porte un hybride plein format (classe A7S III).
Écrit de zéro, sur FreeRTOS, pour la LithCore (STM32F405RGT6), en restant portable vers d'autres cartes.

> Branche `dev-robin` : base minimale, une tâche FreeRTOS fait clignoter la LED bleue.

## Arborescence

```
app/                  Application : main, tâches. Ne touche jamais aux registres directement.
boards/lithcore/      Tout ce qui est propre à la carte : brochage, horloges, script de liens, options du MCU.
config/               Configuration des bibliothèques (FreeRTOSConfig.h).
cmake/                Toolchain, dépendances, fonctions de build (bin/hex, flash).
.github/workflows/    CI : compile debug et release à chaque push.
```

Les dépendances (FreeRTOS-Kernel, CMSIS 6, CMSIS du STM32F4) ne sont pas copiées dans le dépôt :
CMake les télécharge au premier configure, sur un tag figé dans [cmake/dependencies.cmake](cmake/dependencies.cmake).

Pour porter sur une autre carte : créer `boards/<carte>/` avec la même interface que `board.h`, puis configurer avec
`-DLOGOLITH_BOARD=<carte>`.

## Prérequis

- [Arm GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) (`arm-none-eabi-gcc` 12 ou plus),
  dans le `PATH` ou indiquée par la variable d'environnement `ARM_GCC_PATH` (dossier `bin/`).
- CMake 3.25 ou plus, et Ninja.
- [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) pour flasher.

## Compiler

```bash
cmake --workflow --preset debug     # ou release
```

Sortie : `build/debug/app/logolith.{elf,bin,hex,map}`.

## Flasher

Par USB : maintenir le bouton BOOT en branchant la carte (elle apparaît en « STM32 BOOTLOADER »), puis :

```bash
cmake --build --preset debug --target flash-dfu
```

Par SWD avec une sonde ST-Link :

```bash
cmake --build --preset debug --target flash-swd
```

## LED d'état

| LED | Signification |
| --- | --- |
| Bleue qui clignote (1 Hz) | L'ordonnanceur tourne |
| Rouge fixe, bleue éteinte | Erreur fatale du noyau (assert ou dépassement de pile) |
| Rouge fixe, bleue qui clignote | Le quartz 8 MHz n'a pas démarré, la carte tourne sur l'oscillateur interne |

## Licence

[MIT](LICENSE)
