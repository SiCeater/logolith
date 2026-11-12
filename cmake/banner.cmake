# Bannière affichée à chaque configure : logo, cible, compilateur, versions des dépendances, outils.

include(${CMAKE_CURRENT_LIST_DIR}/term.cmake)
term_init(OFF)

if("$ENV{LOGOLITH_ASCII}")
    set(_logo
        "  _                     _ _ _   _     "
        " | |    ___   __ _  ___ | (_) |_| |__  "
        " | |   / _ \\ / _` |/ _ \\| | | __| '_ \\ "
        " | |__| (_) | (_| | (_) | | | |_| | | |"
        " |_____\\___/ \\__, |\\___/|_|_|\\__|_| |_|"
        "             |___/                     ")
else()
    set(_logo
        "  ██╗      ██████╗  ██████╗  ██████╗ ██╗     ██╗████████╗██╗  ██╗"
        "  ██║     ██╔═══██╗██╔════╝ ██╔═══██╗██║     ██║╚══██╔══╝██║  ██║"
        "  ██║     ██║   ██║██║  ███╗██║   ██║██║     ██║   ██║   ███████║"
        "  ██║     ██║   ██║██║   ██║██║   ██║██║     ██║   ██║   ██╔══██║"
        "  ███████╗╚██████╔╝╚██████╔╝╚██████╔╝███████╗██║   ██║   ██║  ██║"
        "  ╚══════╝ ╚═════╝  ╚═════╝  ╚═════╝ ╚══════╝╚═╝   ╚═╝   ╚═╝  ╚═╝")
endif()

# Dégradé du cyan au violet, une teinte par ligne. Le texte est assemblé par concaténation :
# les codes couleur contiennent des « ; », que CMake prendrait pour des séparateurs de liste.
set(_shades 51 45 39 33 63 99)
set(_out "\n")
foreach(i RANGE 5)
    list(GET _logo ${i} _row)
    list(GET _shades ${i} _shade)
    if(C_RESET STREQUAL "")
        string(APPEND _out "${_row}\n")
    else()
        string(APPEND _out "${TERM_ESC}[38;5;${_shade}m${_row}${C_RESET}\n")
    endif()
endforeach()

# Ligne d'information : libellé aligné, valeur.
function(_banner_row out label value)
    term_pad(l "${label}" 13)
    set(${out} "  ${C_GREY}${l}${C_RESET}${value}" PARENT_SCOPE)
endfunction()

get_filename_component(_preset "${CMAKE_BINARY_DIR}" NAME)
string(TOUPPER "${CMAKE_BUILD_TYPE}" _bt)
string(STRIP "${CMAKE_C_FLAGS_${_bt}}" _flags)
get_filename_component(_cc "${CMAKE_C_COMPILER}" NAME_WE)

if(STM32_PROGRAMMER_CLI)
    set(_flash "${C_GREEN}${G_OK}${C_RESET} STM32_Programmer_CLI ${C_GREY}${G_DOT} cibles flash et flash-swd${C_RESET}")
else()
    set(_flash "${C_RED}${G_KO}${C_RESET} STM32_Programmer_CLI introuvable ${C_GREY}${G_DOT} pas de cible de flash${C_RESET}")
endif()
if(CMAKE_COMPILE_WARNING_AS_ERROR)
    set(_werror "${C_GREEN}${G_OK}${C_RESET} warnings traités comme des erreurs")
else()
    set(_werror "${C_YELLOW}${G_KO}${C_RESET} warnings non bloquants")
endif()

_banner_row(r1 "cible" "${C_BOLD}LithCore${C_RESET} ${C_GREY}${G_DOT}${C_RESET} STM32F405RG ${C_GREY}${G_DOT}${C_RESET} Cortex-M4F ${C_GREY}${G_DOT}${C_RESET} quartz 8 MHz")
_banner_row(r2 "preset" "${C_CYAN}${_preset}${C_RESET} ${C_GREY}(${CMAKE_BUILD_TYPE} ${_flags})${C_RESET}")
_banner_row(r3 "compilateur" "${_cc} ${C_BOLD}${CMAKE_C_COMPILER_VERSION}${C_RESET}")
_banner_row(r4 "dépendances" "FreeRTOS ${C_BLUE}${LOGOLITH_FREERTOS_TAG}${C_RESET} ${C_GREY}${G_DOT}${C_RESET} CMSIS ${C_BLUE}${LOGOLITH_CMSIS_CORE_TAG}${C_RESET} ${C_GREY}${G_DOT}${C_RESET} CMSIS F4 ${C_BLUE}${LOGOLITH_CMSIS_F4_TAG}${C_RESET} ${C_GREY}${G_DOT}${C_RESET} LL ${C_BLUE}${LOGOLITH_STM32F4_LL_TAG}${C_RESET}")
_banner_row(r5 "contrôles" "${_werror}")
_banner_row(r6 "flash" "${_flash}")

string(APPEND _out
    "  ${C_GREY}firmware du contrôleur de vol Aerolith ${G_DOT} v${PROJECT_VERSION}${C_RESET}\n\n"
    "${r1}\n${r2}\n${r3}\n${r4}\n${r5}\n${r6}\n")
term_print("${_out}")
