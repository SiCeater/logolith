/**
 * flash_storage.c
 * Stockage persistant dans le secteur 11 de la flash interne (garanties : voir flash_storage.h).
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * RAPPELS FLASH STM32F405 (RM0090 §3)
 * ═══════════════════════════════════════════════════════════════════════════
 * Carte des secteurs (1 Mo) :
 *   0-3   : 16 Ko chacun   0x08000000 – 0x0800FFFF   ← vecteurs + début du firmware
 *   4     : 64 Ko          0x08010000 – 0x0801FFFF
 *   5-11  : 128 Ko chacun  0x08020000 – 0x080FFFFF
 *   → secteur 11 = 0x080E0000 – 0x080FFFFF, le plus loin du code.
 *
 * Règles physiques de la flash :
 *   - Effacer met tous les bits à 1 (0xFF). On ne peut effacer qu'un SECTEUR
 *     ENTIER (ici 128 Ko), et ça prend ~1-2 s.
 *   - Programmer ne peut que faire passer des bits de 1 à 0. Pour réécrire un
 *     mot déjà programmé, il faut effacer tout le secteur.
 *   - Endurance : 10 000 cycles d'effacement minimum par secteur.
 *   - Pendant un effacement ou une programmation, toute lecture flash (y compris
 *     le fetch d'instructions et des vecteurs d'IRQ) attend : le CPU est figé
 *     ~1-2 s pendant un effacement. Acceptable au sol, désarmé ; jamais en vol.
 *
 * D'où le format en JOURNAL : au lieu d'effacer/réécrire à chaque sauvegarde,
 * on écrit chaque nouvelle version dans le slot vierge suivant. On n'efface
 * que quand les 512 slots sont pleins. 512 sauvegardes par effacement ×
 * 10 000 effacements = endurance qui n'est plus un sujet.
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * FORMAT D'UN SLOT (256 octets)
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *   offset 0   magic    u32   0x4C4F4731 ("LOG1") — slot utilisé par ce module
 *          4   len      u16   taille du payload en octets
 *          6   seq      u16   n° de sauvegarde (info/debug, pas utilisé pour choisir)
 *          8   payload  len octets (ex : logolith_config_t)
 *     8+len    crc32    u32   CRC32 de [magic .. fin du payload]
 *          …   0xFF jusqu'à la fin du slot (non programmé)
 *
 * Secteur :  [slot 0][slot 1]...[slot k-1][slot k = vierge (tout 0xFF)]...[slot 511]
 *            └─ anciennes versions ─┘ └ courante ┘
 *
 * Lecture  : on cherche le premier slot vierge k ; la version courante est k-1.
 *            Si k-1 est corrompu (CRC faux, coupure de courant pendant
 *            l'écriture), on remonte vers k-2, k-3, … jusqu'au premier slot valide.
 * Écriture : dans le slot k. Si k = 512 (plein) → effacement du secteur, puis slot 0.
 *
 * Cas de coupure d'alimentation :
 *   - pendant la programmation d'un slot → slot partiel, CRC faux → ignoré, la
 *     version précédente reste utilisée. La sauvegarde suivante ira au slot
 *     d'après (le slot partiel n'est pas vierge).
 *   - pendant l'effacement (1 fois / 512) → toutes les versions sont perdues :
 *     au boot, "aucune calibration", il faut recalibrer. Cas rare et détecté.
 */

#include "flash_storage.h"
#include "stm32f4xx.h"   /* registres FLASH->CR/SR/KEYR/ACR et masques CMSIS FLASH_CR_*, FLASH_SR_* */
#include <string.h>

/* Il n'y a pas de driver LL pour la flash sur F4 (seulement la HAL) : on
 * pilote les registres directement, en suivant les séquences de RM0090 §3.6. */

#define FLASH_STORE_MAGIC       0x4C4F4731UL   /* "LOG1" */
#define FLASH_STORE_NSLOTS      (FLASH_STORE_SIZE / FLASH_STORE_SLOT_SIZE)   /* 512 */

/* Clés de déverrouillage de FLASH->CR (RM0090 §3.5.1) : il faut écrire KEY1
 * puis KEY2 dans FLASH->KEYR. Une mauvaise séquence bloque la flash jusqu'au reset. */
#define FLASH_KEY1              0x45670123UL
#define FLASH_KEY2              0xCDEF89ABUL

/* Flags d'erreur de FLASH->SR (RM0090 §3.8.4) :
 *   PGSERR : séquence de programmation incorrecte
 *   PGPERR : parallélisme (PSIZE) incohérent avec l'accès
 *   PGAERR : adresse mal alignée
 *   WRPERR : secteur protégé en écriture (option bytes)
 *   SOP    : erreur d'opération */
#define FLASH_SR_ERRORS         (FLASH_SR_PGSERR | FLASH_SR_PGPERR | FLASH_SR_PGAERR | \
                                 FLASH_SR_WRPERR | FLASH_SR_SOP)

typedef struct {
    uint32_t magic;
    uint16_t len;
    uint16_t seq;
} slot_hdr_t;   /* 8 octets = FLASH_STORE_HDR_SIZE */

/* ── CRC32 (le même que zip/Ethernet : poly 0x04C11DB7 en forme réfléchie 0xEDB88320) ──
 * Version bit à bit, sans table : ~250 octets à vérifier par slot, donc
 * négligeable, et ça évite 1 Ko de table en flash. On n'utilise pas le
 * périphérique CRC du STM32 : il calcule un CRC32 non réfléchi, mot par mot,
 * moins pratique pour un payload de taille quelconque. */
static uint32_t crc32(const uint8_t *p, uint32_t n)
{
    uint32_t c = 0xFFFFFFFFUL;
    while (n--) {
        c ^= *p++;
        for (uint8_t k = 0; k < 8U; k++)
            c = (c & 1U) ? (c >> 1) ^ 0xEDB88320UL : (c >> 1);
    }
    return ~c;
}

/* La flash est mappée en mémoire : on la lit comme de la RAM. */
static inline const uint8_t *slot_addr(uint32_t i)
{
    return (const uint8_t *)(FLASH_STORE_BASE + i * FLASH_STORE_SLOT_SIZE);
}

/* Slot vierge = entièrement à 0xFF (état après effacement). On vérifie tout le
 * slot, pas seulement le magic : un slot dont l'écriture a été coupée après
 * quelques mots ne doit pas être considéré comme libre. */
static bool slot_is_erased(uint32_t i)
{
    const uint32_t *w = (const uint32_t *)slot_addr(i);
    for (uint32_t k = 0; k < FLASH_STORE_SLOT_SIZE / 4U; k++)
        if (w[k] != 0xFFFFFFFFUL) return false;
    return true;
}

/* Slot valide = bon magic + bonne taille attendue + CRC correct.
 * Vérifier `len` évite de charger un ancien enregistrement d'une autre taille
 * (struct de config modifiée entre deux firmwares). config.c vérifie en plus
 * le champ `version`. memcpy plutôt que des casts : pas d'hypothèse d'alignement. */
static bool slot_is_valid(uint32_t i, uint16_t len)
{
    const uint8_t *s = slot_addr(i);
    slot_hdr_t h;
    memcpy(&h, s, sizeof(h));
    if (h.magic != FLASH_STORE_MAGIC || h.len != len || len > FLASH_STORE_MAX_PAYLOAD) return false;
    uint32_t crc;
    memcpy(&crc, s + FLASH_STORE_HDR_SIZE + len, 4);
    return crc == crc32(s, FLASH_STORE_HDR_SIZE + len);
}

/* Index du premier slot vierge, c'est-à-dire le nombre de slots déjà utilisés.
 * Renvoie NSLOTS si le secteur est plein. Les slots sont écrits dans l'ordre,
 * donc tout ce qui suit le premier slot vierge est vierge aussi. */
static uint32_t first_free_slot(void)
{
    uint32_t i = 0;
    while (i < FLASH_STORE_NSLOTS && !slot_is_erased(i)) i++;
    return i;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Primitives flash bas niveau (RM0090 §3.6)
 * ═══════════════════════════════════════════════════════════════════════════ */

/* FLASH->CR est verrouillé au reset (bit LOCK). On le déverrouille juste
 * avant d'écrire et on le reverrouille juste après : un bug ailleurs dans le
 * firmware ne peut pas écrire la flash par accident. */
static void flash_unlock(void)
{
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;
    }
}

static void flash_lock(void) { FLASH->CR |= FLASH_CR_LOCK; }

/* Attend la fin de l'opération en cours (BSY retombe), retourne false si un
 * flag d'erreur est levé, puis efface les flags. Ils s'effacent en écrivant 1
 * ("rc_w1"), d'où l'affectation directe : écrire 0 sur les autres bits ne
 * change rien. */
static bool flash_wait(void)
{
    while (FLASH->SR & FLASH_SR_BSY) { }
    bool ok = (FLASH->SR & FLASH_SR_ERRORS) == 0U;
    FLASH->SR = FLASH_SR_ERRORS | FLASH_SR_EOP;
    return ok;
}

/* Le F4 a un cache de données flash (ART accelerator, FLASH->ACR.DCEN). Après
 * un effacement ou une programmation, il peut encore contenir les anciennes
 * valeurs : la relecture de vérification lirait l'ancien contenu. On le
 * réinitialise. Séquence imposée : DCRST ne marche que cache désactivé
 * (RM0090 §3.5.2), comme FLASH_FlushCaches() de la HAL. */
static void flash_flush_dcache(void)
{
    if (FLASH->ACR & FLASH_ACR_DCEN) {
        FLASH->ACR &= ~FLASH_ACR_DCEN;
        FLASH->ACR |= FLASH_ACR_DCRST;
        FLASH->ACR &= ~FLASH_ACR_DCRST;
        FLASH->ACR |= FLASH_ACR_DCEN;
    }
}

/* Efface le secteur 11, et uniquement lui : le numéro est une constante, il n'y
 * a aucun paramètre qui permettrait d'effacer un autre secteur (qui contiendrait
 * du code).
 * Séquence RM0090 §3.6.3 : SER=1 (sector erase) + SNB = n° de secteur, puis STRT=1.
 * PSIZE=x32 : parallélisme 32 bits, autorisé pour une alim de 2,7 à 3,6 V
 * (notre cas, 3,3 V). En x8 ou x16, l'effacement serait plus lent. */
static bool flash_erase_store_sector(void)
{
    flash_wait();
    FLASH->CR &= ~(FLASH_CR_PSIZE | FLASH_CR_SNB);
    FLASH->CR |= FLASH_CR_PSIZE_1                       /* PSIZE = 0b10 → x32 */
              |  (FLASH_STORE_SECTOR << FLASH_CR_SNB_Pos)
              |  FLASH_CR_SER;
    FLASH->CR |= FLASH_CR_STRT;                         /* lance l'effacement (~1-2 s, CPU figé) */
    bool ok = flash_wait();
    FLASH->CR &= ~(FLASH_CR_SER | FLASH_CR_SNB);
    flash_flush_dcache();
    return ok;
}

/* Programme nwords mots de 32 bits à partir de addr.
 * Séquence RM0090 §3.6.4 : PG=1 et PSIZE=x32, puis une simple écriture
 * mémoire 32 bits à l'adresse flash ; on attend BSY=0 entre chaque mot.
 * __DSB() force l'écriture à partir sur le bus avant qu'on interroge BSY. */
static bool flash_program_words(uint32_t addr, const uint32_t *w, uint32_t nwords)
{
    /* Garde-fou : refuse toute écriture hors du secteur 11 ou mal alignée, même
     * si un appelant futur se trompait d'adresse. Le code ne peut pas être écrasé. */
    if (addr < FLASH_STORE_BASE || addr + nwords * 4U > FLASH_STORE_END || (addr & 3U)) return false;

    bool ok = flash_wait();
    FLASH->CR &= ~FLASH_CR_PSIZE;
    FLASH->CR |= FLASH_CR_PSIZE_1 | FLASH_CR_PG;
    for (uint32_t k = 0; k < nwords && ok; k++) {
        *(volatile uint32_t *)(addr + 4U * k) = w[k];
        __DSB();
        ok = flash_wait();
    }
    FLASH->CR &= ~FLASH_CR_PG;
    flash_flush_dcache();
    return ok;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * API
 * ═══════════════════════════════════════════════════════════════════════════ */

bool FlashStore_Load(void *data, uint16_t len)
{
    if (len == 0U || len > FLASH_STORE_MAX_PAYLOAD) return false;

    /* Du dernier slot écrit vers le premier : le premier valide rencontré est
     * la version la plus récente. `i-- > 0` : décrémente avant le corps, donc
     * parcourt first_free-1 … 0, et sort proprement si first_free == 0. */
    for (uint32_t i = first_free_slot(); i-- > 0U; ) {
        if (slot_is_valid(i, len)) {
            memcpy(data, slot_addr(i) + FLASH_STORE_HDR_SIZE, len);
            return true;
        }
    }
    return false;   /* secteur vierge (1er boot ou mass erase) ou aucun slot valide */
}

bool FlashStore_Save(const void *data, uint16_t len)
{
    if (len == 0U || len > FLASH_STORE_MAX_PAYLOAD) return false;

    /* On construit l'image complète du slot en RAM (alignée sur 32 bits, puisqu'on
     * programme par mots), puis on la programme d'un coup. `static` : 256 octets
     * hors de la pile. Pré-remplie à 0xFF = valeur "non programmée". */
    static uint32_t img[FLASH_STORE_SLOT_SIZE / 4U];
    uint8_t *b = (uint8_t *)img;
    memset(img, 0xFF, sizeof(img));

    uint32_t slot = first_free_slot();
    uint16_t seq  = 0;
    if (slot > 0U) {    /* seq = celui du slot précédent + 1 (informatif) */
        slot_hdr_t prev;
        memcpy(&prev, slot_addr(slot - 1U), sizeof(prev));
        seq = (uint16_t)(prev.seq + 1U);
    }

    slot_hdr_t h = { FLASH_STORE_MAGIC, len, seq };
    memcpy(b, &h, sizeof(h));
    memcpy(b + FLASH_STORE_HDR_SIZE, data, len);
    uint32_t crc = crc32(b, FLASH_STORE_HDR_SIZE + len);
    memcpy(b + FLASH_STORE_HDR_SIZE + len, &crc, 4);
    /* nb de mots à programmer = en-tête + payload + CRC, arrondi au mot supérieur.
     * Le reste du slot reste à 0xFF sans être programmé. */
    uint32_t nwords = (FLASH_STORE_HDR_SIZE + len + 4U + 3U) / 4U;

    flash_unlock();
    FLASH->SR = FLASH_SR_ERRORS | FLASH_SR_EOP;   /* efface d'éventuels flags d'erreur précédents */
    bool ok = true;
    if (slot >= FLASH_STORE_NSLOTS) {             /* secteur plein → effacement, retour au slot 0 */
        ok   = flash_erase_store_sector();
        slot = 0;
    }
    if (ok) ok = flash_program_words((uint32_t)slot_addr(slot), img, nwords);
    flash_lock();

    /* Vérification finale en relisant la flash (et pas la RAM) : on ne renvoie
     * true que si l'enregistrement est réellement relisible. */
    return ok && slot_is_valid(slot, len);
}
