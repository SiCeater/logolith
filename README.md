# logolith

Firmware du contrôleur de vol d'Aerolith. Cible : LithCore (STM32F405RGT6, quartz 8 MHz).

Branche `base` : point de départ minimal. FreeRTOS, horloge à 168 MHz sur le quartz, LED bleue à 1 Hz.

## Arborescence

```
src/            code du firmware (main.c, clock.c)
config/         FreeRTOSConfig.h
linker/         plan mémoire (script de liens)
cmake/          toolchain, dépendances, cibles de flash
.vscode/        build, flash et debug dans VS Code
.github/        CI : format et compilation debug + release à chaque push
```

Dépendances (FreeRTOS-Kernel, CMSIS 6, CMSIS STM32F4, drivers LL de ST, fichier SVD) téléchargées au premier
configure, versions figées dans [cmake/dependencies.cmake](cmake/dependencies.cmake).

Accès au matériel par les registres (CMSIS) ou par la couche LL de ST (`#include "stm32f4xx_ll_gpio.h"`, etc.), sans
HAL. Doc de la LL : UM1725 de ST.

## Installer

| Outil | Rôle |
| --- | --- |
| [Arm GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) 13+ | compilateur et `arm-none-eabi-gdb`, dans le `PATH` ou via `ARM_GCC_PATH` (dossier `bin/`) |
| CMake 3.25+, Ninja | build |
| [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) | flash USB (DFU) et SWD |
| [xPack OpenOCD](https://xpack-dev-tools.github.io/openocd-xpack/) | serveur de debug pour la sonde ST-Link |
| VS Code | extensions proposées à l'ouverture du dossier (CMake Tools, clangd, Cortex-Debug) |

Outils hors du `PATH` : les indiquer dans les réglages utilisateur de VS Code (`cortex-debug.openocdPath`,
`cortex-debug.armToolchainPath`, et `ARM_GCC_PATH` dans `cmake.environment` et `terminal.integrated.env.windows`).

## Démarrer

```bash
git clone -b base https://github.com/SiCeater/logolith.git
cd logolith
cmake --workflow --preset debug
```

Ouvrir le dossier dans VS Code, accepter les extensions recommandées.

## Au quotidien

| Action | VS Code | Terminal |
| --- | --- | --- |
| Compiler | `Ctrl+Shift+B` | `cmake --workflow --preset debug` |
| Flasher par USB | tâche « Flasher par USB (DFU) » | `cmake --build --preset debug --target flash` |
| Flasher par SWD | tâche « Flasher par SWD (ST-Link) » | `cmake --build --preset debug --target flash-swd` |
| Déboguer | `F5` | |

Flash USB : maintenir BOOT en branchant la carte (elle apparaît en « STM32 BOOTLOADER »), puis lancer le flash. Le
firmware démarre tout seul à la fin.

Debug : sonde ST-Link sur le connecteur SWD (SWDIO, SWCLK, GND, 3V3). `F5` compile, flashe, s'arrête sur `main`.
Points d'arrêt, registres des périphériques (fichier SVD) et liste des tâches FreeRTOS dans le panneau de debug.

Sorties dans `build/debug/` : `logolith.elf` (debug), `.bin`, `.hex`, `.map` (place occupée par chaque fonction).

Affichage : bannière au configure, rapport en fin de build (étapes, mémoire, durée). `NO_COLOR=1` retire les couleurs,
`LOGOLITH_ASCII=1` dessine en ASCII pur. Hors des tâches VS Code, un terminal Windows doit être en UTF-8 (`chcp 65001`).

## LED

| LED | Signification |
| --- | --- |
| Bleue 1 Hz | FreeRTOS tourne |
| Rouge fixe | quartz absent, ou erreur fatale (HardFault, assert FreeRTOS, dépassement de pile) |

## Règles

- C17. `clang-format` à l'enregistrement (réglé dans VS Code), vérifié par la CI.
- Zéro warning : ils sont traités comme des erreurs.
- Pas d'allocation dynamique : tâches et objets FreeRTOS en version `...Static`.
- Interruptions de priorité NVIC 0 à 4 : aucun appel FreeRTOS. De 5 à 15 : seulement les fonctions `...FromISR`.
- Pas de `LL_Init1msTick` ni de `LL_mDelay` : ils reprogramment SysTick, que FreeRTOS utilise. Dans une tâche,
  `vTaskDelay`.
- Valeur tirée d'une datasheet ou du manuel de référence (RM0090) : citer la section en commentaire.

## Licence

[MIT](LICENSE)
