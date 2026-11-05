#pragma once

// Port série USB (CDC). Ouvrir puis fermer le port à 1200 bauds redémarre la carte dans le bootloader
// Logolith, en mode DFU : on la flashe sans toucher au bouton BOOT (voir tools/flash.py).
void usb_start(void);
