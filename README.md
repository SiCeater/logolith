# logolith

Firmware du contrôleur de vol d'Aerolith.

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

## Démarrer

Première fois, dans l'ordre.

1. Installer :
   - Arm GNU Toolchain (`arm-none-eabi-gcc` 12+), dans le `PATH` ou via `ARM_GCC_PATH` (dossier `bin/`)
   - CMake 3.25+, Ninja
   - Python 3
   - STM32CubeProgrammer (première installation, SWD)
2. Récupérer le code :
   ```bash
   git clone https://github.com/SiCeater/logolith.git
   cd logolith
   git switch dev-robin
   pip install -r tools/requirements.txt
   ```
3. Compiler. Le premier configure télécharge les dépendances (internet requis, environ 1 min) :
   ```bash
   cmake --workflow --preset debug
   ```
4. Installer bootloader et application : brancher la carte en maintenant BOOT, relâcher, puis
   ```bash
   cmake --build --preset debug --target flash
   ```
   LED bleue à 1 Hz : tout est en place.
5. Éditeur : VS Code avec l'extension clangd. `.clangd` pointe vers `build/debug/compile_commands.json`. Ajouter
   dans les réglages VS Code `"clangd.arguments": ["--query-driver=**/arm-none-eabi-gcc*"]` pour que clangd trouve
   les en-têtes de la toolchain.

## Développer

### Boucle de travail

```bash
cmake --build --preset debug --target flash
```

Compile ce qui a changé puis flashe sans bouton, environ 5 s. Carte muette après un plantage : BOOT au branchement,
même commande.

La CI GitHub compile debug et release à chaque push. Elle doit rester verte.

### Où mettre le code

| Besoin | Emplacement |
| --- | --- |
| Accès matériel : broches, SPI, UART, timers, DMA | `boards/lithcore/`, exposé par une fonction `board_*` dans `board.h` |
| Driver d'un composant (ICM-42688, LPS22HH, ESC) | `drivers/<composant>/`, bibliothèque CMake à créer avec le premier driver, utilise `board.h` |
| Tâches, logique de vol | `app/src/<module>.c` et `app/include/<module>.h` |
| Configuration FreeRTOS, TinyUSB | `app/config/` |
| Dépendance externe | `cmake/dependencies.cmake`, tag figé, jamais copiée dans le dépôt |

### Ajouter une tâche

Modèle : [app/src/heartbeat.c](app/src/heartbeat.c).

1. `app/include/imu.h` :
   ```c
   #pragma once

   void imu_start(void);
   ```
2. `app/src/imu.c` :
   ```c
   #include "imu.h"

   #include "FreeRTOS.h"
   #include "task.h"

   #define IMU_PERIOD_MS 1
   #define IMU_STACK     (configMINIMAL_STACK_SIZE * 2)
   #define IMU_PRIORITY  (tskIDLE_PRIORITY + 5)

   static StackType_t stack[IMU_STACK];
   static StaticTask_t tcb;

   static void imu_task(void *arg)
   {
       (void)arg;
       TickType_t wake = xTaskGetTickCount();

       for (;;) {
           // lecture et traitement
           xTaskDelayUntil(&wake, pdMS_TO_TICKS(IMU_PERIOD_MS));
       }
   }

   void imu_start(void)
   {
       xTaskCreateStatic(imu_task, "imu", IMU_STACK, NULL, IMU_PRIORITY, stack, &tcb);
   }
   ```
3. Ajouter `src/imu.c` à `add_executable` dans [app/CMakeLists.txt](app/CMakeLists.txt).
4. Appeler `imu_start()` dans [app/src/main.c](app/src/main.c), avant `vTaskStartScheduler()`.

Priorités (0 à 7, la plus haute gagne) :

| Priorité | Usage |
| --- | --- |
| 6 | boucle de contrôle |
| 5 | capteurs |
| 4 | radio (RC) |
| 3 | télémétrie, OSD |
| 2 | USB |
| 1 | heartbeat, tâches de fond |

### Règles de codage

- C17, pas de C++.
- Format : `clang-format -i <fichier>` avant chaque commit ([.clang-format](.clang-format) : 4 espaces, 110 colonnes).
- Nommage en `snake_case`, préfixé par le module : `imu_start`, `board_led_set`. Macros et constantes en
  `MAJUSCULES`, types en `_t`.
- Tout ce qui n'est pas dans le header est `static`. Un header par module, `#pragma once`, le strict nécessaire.
- Types à taille fixe (`uint32_t`, `int16_t`) pour registres, trames et mesures.
- Pas d'allocation dynamique : ni `malloc`, ni `xTaskCreate`. Tâches, queues, mutex en version `...Static`.
- Registres et broches uniquement dans `boards/`. L'application passe par `board.h`.
- Interruptions courtes, sans attente. FreeRTOS seulement via les fonctions `...FromISR`, et seulement depuis une
  interruption de priorité NVIC 5 à 15. Priorités 0 à 4 : jamais d'appel FreeRTOS.
- Données partagées entre tâches : queue, notification ou mutex. Pas de variable globale modifiée par plusieurs tâches.
- Tâche périodique : `xTaskDelayUntil`, jamais de boucle d'attente.
- Zéro warning (`-Wall -Wextra -Wshadow -Wconversion`).
- Commentaires en français, sur le pourquoi. Valeur tirée d'une datasheet : citer le document et la section.

### Git

- Une branche par sujet, partie de `dev-robin`, fusionnée par PR.
- Commits petits, chacun compile. Titre court qui décrit le changement, corps qui explique pourquoi.
- Bootloader modifié : le réinstaller avec BOOT, il ne se met pas à jour lui-même.

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
