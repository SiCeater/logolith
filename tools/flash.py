#!/usr/bin/env python3
"""Flashe la LithCore par USB.

Selon ce qui est branché :
- bootloader Logolith en DFU : envoie l'application ;
- application Logolith (port série) : l'ouvre à 1200 bauds pour la renvoyer dans le bootloader,
  puis envoie l'application ;
- bootloader ROM de ST (bouton BOOT maintenu au branchement) : installe bootloader + application
  avec STM32_Programmer_CLI. C'est le chemin de la toute première installation.

Dépendances : pip install -r tools/requirements.txt
"""

import argparse
import re
import subprocess
import sys
import time

import libusb_package
import serial
import serial.tools.list_ports
import usb.core
import usb.util

APP_VID, APP_PID = 0x0483, 0x5740        # port série de l'application
BOOT_VID, BOOT_PID = 0x1209, 0x0001      # bootloader Logolith (DFU 1.1)
DFU_TOUCH_BAUDRATE = 1200
WAIT_S = 30  # la première fois sur un PC, Windows installe WinUSB pour le bootloader : compter ~15 s

# DFU 1.1 : requêtes et états
DFU_DNLOAD, DFU_GETSTATUS, DFU_CLRSTATUS, DFU_ABORT = 1, 3, 4, 6
STATE_IDLE, STATE_DNBUSY, STATE_DNLOAD_IDLE, STATE_MANIFEST_SYNC, STATE_MANIFEST, STATE_ERROR = 2, 4, 5, 6, 7, 10
STATUS_TEXT = {
    0x03: "écriture", 0x04: "effacement", 0x07: "vérification", 0x08: "adresse hors de la zone application",
    0x09: "image incomplète", 0x0A: "image invalide (table des vecteurs)",
}


def find_bootloader():
    return libusb_package.find(idVendor=BOOT_VID, idProduct=BOOT_PID)


def app_ports():
    return [p.device for p in serial.tools.list_ports.comports() if (p.vid, p.pid) == (APP_VID, APP_PID)]


def rom_dfu_port(cli):
    if not cli:
        return None
    out = subprocess.run([cli, "-l", "usb"], capture_output=True, text=True).stdout
    ports = re.findall(r"Device Index\s*:\s*(\S+)", out)
    return ports[0] if ports else None


def wait_for(predicate, what):
    deadline = time.monotonic() + WAIT_S
    while time.monotonic() < deadline:
        result = predicate()
        if result:
            return result
        time.sleep(0.2)
    sys.exit(f"{what} n'est pas apparu après {WAIT_S} s.")


def touch_1200(port):
    print(f"Passage dans le bootloader via {port}...")
    try:
        with serial.Serial(port, DFU_TOUCH_BAUDRATE) as s:
            s.dtr = True
            time.sleep(0.05)
            s.dtr = False
    except (serial.SerialException, OSError):
        pass  # la carte se débranche pendant la fermeture : normal


class Dfu:
    def __init__(self, dev):
        self.dev = dev
        dev.set_configuration()
        intf = dev.get_active_configuration()[(0, 0)]
        self.intf = intf.bInterfaceNumber
        usb.util.claim_interface(dev, self.intf)
        self.transfer_size = self._transfer_size(intf)

    @staticmethod
    def _transfer_size(intf):
        extra = bytes(intf.extra_descriptors)
        while len(extra) >= 2:
            length, kind = extra[0], extra[1]
            if kind == 0x21 and length >= 7:  # descripteur fonctionnel DFU
                return extra[5] | extra[6] << 8
            extra = extra[length:]
        return 1024

    def status(self):
        s = self.dev.ctrl_transfer(0xA1, DFU_GETSTATUS, 0, self.intf, 6)
        return s[0], s[1] | s[2] << 8 | s[3] << 16, s[4]

    def wait_idle(self, wanted):
        while True:
            status, poll_ms, state = self.status()
            if status != 0 or state == STATE_ERROR:
                self.dev.ctrl_transfer(0x21, DFU_CLRSTATUS, 0, self.intf, None)
                sys.exit(f"Erreur du bootloader : {STATUS_TEXT.get(status, hex(status))}")
            if state in wanted:
                return state
            time.sleep(poll_ms / 1000)

    def download(self, image):
        self.dev.ctrl_transfer(0x21, DFU_ABORT, 0, self.intf, None)
        size = self.transfer_size
        blocks = (len(image) + size - 1) // size
        for n in range(blocks):
            self.dev.ctrl_transfer(0x21, DFU_DNLOAD, n, self.intf, image[n * size:(n + 1) * size])
            self.wait_idle({STATE_DNLOAD_IDLE})
            print(f"\r  {min((n + 1) * size, len(image)) // 1024} / {len(image) // 1024} Ko", end="", flush=True)
        print()
        # Bloc vide : fin du transfert, le bootloader écrit la table des vecteurs et vérifie l'image.
        self.dev.ctrl_transfer(0x21, DFU_DNLOAD, blocks, self.intf, None)
        self.wait_idle({STATE_IDLE})
        usb.util.release_interface(self.dev, self.intf)


def flash_with_bootloader(app_bin):
    image = open(app_bin, "rb").read()
    print(f"Bootloader Logolith trouvé, envoi de {app_bin} ({len(image) // 1024} Ko)")
    Dfu(find_bootloader()).download(image)
    port = wait_for(app_ports, "Le port série de l'application")
    print(f"Application démarrée ({port[0]}).")


def install_with_rom(cli, port, bootloader_elf, app_elf):
    print(f"Bootloader ROM de ST sur {port} : installation du bootloader et de l'application")
    result = subprocess.run([cli, "-c", f"port={port}", "-w", bootloader_elf, "-v", "-w", app_elf, "-v",
                             "-s", "0x08000000"])
    if result.returncode:
        sys.exit(result.returncode)
    port = wait_for(app_ports, "Le port série de l'application")
    print(f"Application démarrée ({port[0]}).")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--app-bin", required=True)
    parser.add_argument("--app-elf", required=True)
    parser.add_argument("--bootloader-elf", required=True)
    parser.add_argument("--cli", help="chemin de STM32_Programmer_CLI (installation initiale)")
    args = parser.parse_args()

    if find_bootloader():
        flash_with_bootloader(args.app_bin)
        return

    ports = app_ports()
    if len(ports) > 1:
        sys.exit(f"Plusieurs cartes branchées ({', '.join(ports)}) : n'en garde qu'une.")
    if ports:
        touch_1200(ports[0])
        wait_for(find_bootloader, "Le bootloader Logolith")
        flash_with_bootloader(args.app_bin)
        return

    rom = rom_dfu_port(args.cli)
    if rom:
        install_with_rom(args.cli, rom, args.bootloader_elf, args.app_elf)
        return

    sys.exit("Carte introuvable. Pour une première installation, maintiens BOOT en la branchant.")


if __name__ == "__main__":
    main()
