# Rapport de fin de build : étapes parcourues, occupation mémoire, durée.
# Lancé en mode script par les cibles définies dans cmake/firmware.cmake :
#   MODE=start   note l'heure de début du build dans STAMP
#   MODE=report  affiche le rapport
# Autres variables : STAMP, ELF, SIZE_TOOL, LINKER_SCRIPT, BUILD_TYPE, SOURCES, OBJ_DIR, GCC_VERSION

cmake_minimum_required(VERSION 3.25)

function(_now_ms out)
    string(TIMESTAMP t "%s.%f")
    string(REPLACE "." ";" t "${t}")
    list(GET t 0 s)
    list(GET t 1 us)
    string(REGEX REPLACE "^0+([0-9])" "\\1" us "${us}") # « 000123 » serait lu en octal
    math(EXPR ms "${s} * 1000 + ${us} / 1000")
    set(${out} ${ms} PARENT_SCOPE)
endfunction()

if(MODE STREQUAL "start")
    _now_ms(now)
    file(WRITE "${STAMP}" "${now}")
    return()
endif()

include(${CMAKE_CURRENT_LIST_DIR}/term.cmake)
term_init(ON) # Ninja retire les couleurs si sa sortie n'est pas un terminal

set(WIDTH 77) # largeur du cadre mémoire, alignée sur la ligne des étapes

# --- Mémoire : régions du script de liens, sections de l'ELF ------------------------------------

file(READ "${LINKER_SCRIPT}" ld)
string(REGEX MATCHALL "[A-Z]+[ \t]*\\([a-z]+\\)[ \t]*:[ \t]*ORIGIN[ \t]*=[ \t]*0x[0-9A-Fa-f]+[ \t]*,[ \t]*LENGTH[ \t]*=[ \t]*[0-9]+[KM]?" regions "${ld}")
set(region_names "")
foreach(r IN LISTS regions)
    string(REGEX MATCH "^([A-Z]+).*ORIGIN[ \t]*=[ \t]*(0x[0-9A-Fa-f]+).*LENGTH[ \t]*=[ \t]*([0-9]+)([KM]?)" _ "${r}")
    set(name ${CMAKE_MATCH_1})
    math(EXPR origin "${CMAKE_MATCH_2}")
    set(length ${CMAKE_MATCH_3})
    if(CMAKE_MATCH_4 STREQUAL "K")
        math(EXPR length "${length} * 1024")
    elseif(CMAKE_MATCH_4 STREQUAL "M")
        math(EXPR length "${length} * 1048576")
    endif()
    list(APPEND region_names ${name})
    set(${name}_ORIGIN ${origin})
    set(${name}_LENGTH ${length})
    set(${name}_USED 0)
endforeach()

execute_process(COMMAND "${SIZE_TOOL}" -A -d "${ELF}" OUTPUT_VARIABLE size_out RESULT_VARIABLE rc)
if(NOT rc EQUAL 0)
    message(WARNING "rapport de build : ${SIZE_TOOL} a échoué")
    return()
endif()

set(stack 0)
string(REPLACE "\n" ";" size_lines "${size_out}")
foreach(line IN LISTS size_lines)
    if(NOT line MATCHES "^(\\.[A-Za-z0-9_.]+)[ \t]+([0-9]+)[ \t]+([0-9]+)")
        continue()
    endif()
    set(sec ${CMAKE_MATCH_1})
    set(sz ${CMAKE_MATCH_2})
    set(addr ${CMAKE_MATCH_3})
    if(sz EQUAL 0)
        continue()
    endif()
    set(SEC_${sec} ${sz})

    if(sec STREQUAL "._user_heap_stack")
        set(stack ${sz}) # pile de main() réservée : comptée en RAM, comme le fait l'éditeur de liens
    endif()
    foreach(name IN LISTS region_names)
        math(EXPR end "${${name}_ORIGIN} + ${${name}_LENGTH}")
        if(addr GREATER_EQUAL ${name}_ORIGIN AND addr LESS end)
            math(EXPR ${name}_USED "${${name}_USED} + ${sz}")
        endif()
    endforeach()
    # .data vit en RAM mais sa valeur initiale est stockée en flash : elle compte des deux côtés.
    if(sec STREQUAL ".data" AND DEFINED FLASH_USED)
        math(EXPR FLASH_USED "${FLASH_USED} + ${sz}")
    endif()
endforeach()

# --- Contexte : durée, à jour ou non, objets compilés --------------------------------------------

set(duration "")
set(up_to_date OFF)
if(EXISTS "${STAMP}")
    file(READ "${STAMP}" start_ms)
    _now_ms(now_ms)
    math(EXPR elapsed "(${now_ms} - ${start_ms}) / 100")
    term_decimal(duration ${elapsed})
    file(TIMESTAMP "${ELF}" elf_s "%s")
    math(EXPR start_s "${start_ms} / 1000")
    if(elf_s LESS start_s)
        set(up_to_date ON)
    endif()
endif()

file(GLOB_RECURSE objs "${OBJ_DIR}/*.obj" "${OBJ_DIR}/*.o")
list(LENGTH objs nobjs)
get_filename_component(elf_name "${ELF}" NAME)

# --- Étapes du build : quatre cases reliées ------------------------------------------------------

set(BOX 17) # largeur d'une case, bordures comprises : 4 cases et 3 liens tiennent en 79 colonnes
set(OK "${C_GREEN}${G_OK}${C_RESET}")
if(up_to_date)
    set(OK "${C_GREY}${G_OK}${C_RESET}")
endif()

set(titles "sources" "compilation" "édition" "binaires")
set(details "${SOURCES} fichiers" "${nobjs} objets" "de liens" "bin hex map")

math(EXPR inner "${BOX} - 2")
term_repeat(h "${G_H}" ${inner})
set(top "")
set(mid "")
set(low "")
set(bot "")
foreach(i RANGE 3)
    list(GET titles ${i} t)
    list(GET details ${i} d)
    term_pad(t " ${OK} ${C_BOLD}${t}${C_RESET}" ${inner})
    term_pad(d "   ${C_CYAN}${d}${C_RESET}" ${inner})
    if(i EQUAL 0)
        set(link "")
        set(gap "")
    else()
        set(link " ${C_GREY}${G_ARROW}${C_RESET} ")
        term_width(lw "${link}")
        term_repeat(gap " " ${lw})
    endif()
    string(APPEND top "${gap}${C_GREY}${G_TL}${h}${G_TR}${C_RESET}")
    string(APPEND mid "${link}${C_GREY}${G_V}${C_RESET}${t}${C_GREY}${G_V}${C_RESET}")
    string(APPEND low "${gap}${C_GREY}${G_V}${C_RESET}${d}${C_GREY}${G_V}${C_RESET}")
    string(APPEND bot "${gap}${C_GREY}${G_BL}${h}${G_BR}${C_RESET}")
endforeach()

# --- Mémoire : une barre par région ---------------------------------------------------------------
# Texte assemblé par concaténation : les codes couleur contiennent des « ; », séparateurs de liste.

set(mem_lines "")
foreach(name IN LISTS region_names)
    term_bar(bar ${${name}_USED} ${${name}_LENGTH} 34)
    term_size(used ${${name}_USED})
    term_size(total ${${name}_LENGTH})
    term_percent(pct ${${name}_USED} ${${name}_LENGTH})
    term_pad(label "${C_BOLD}${name}${C_RESET}" 7)
    term_pad(used "${used}" 8 RIGHT)
    term_pad(total "${total}" 7)
    term_pad(pct "${pct}" 7 RIGHT)
    term_box_line(l ${WIDTH} "${label}${bar} ${used} ${C_GREY}/${C_RESET} ${total}${C_BOLD}${pct}${C_RESET}")
    string(APPEND mem_lines "  ${l}\n")
endforeach()

set(detail_line "")
set(sep "")
foreach(sec .text .rodata .data .bss)
    if(DEFINED SEC_${sec})
        term_size(s ${SEC_${sec}})
        string(APPEND detail_line "${sep}${C_GREY}${sec}${C_RESET} ${s}")
        set(sep " ${C_GREY}${G_DOT}${C_RESET} ")
    endif()
endforeach()
if(stack GREATER 0)
    term_size(s ${stack})
    string(APPEND detail_line "${sep}${C_GREY}pile${C_RESET} ${s}")
endif()
term_box_line(detail ${WIDTH} "${detail_line}")
term_box_line(blank ${WIDTH} "")

term_box_top(mem_top ${WIDTH} "${C_BOLD}mémoire${C_RESET}")
term_box_bottom(mem_bot ${WIDTH})

# --- Pied : verdict ---------------------------------------------------------------------------------

if(up_to_date)
    set(verdict "${C_GREY}${G_OK}${C_RESET} ${C_BOLD}rien à recompiler${C_RESET}")
else()
    set(verdict "${C_GREEN}${G_OK} build ${BUILD_TYPE} réussi${C_RESET}")
endif()
if(NOT duration STREQUAL "")
    string(APPEND verdict " en ${C_BOLD}${duration} s${C_RESET}")
endif()
string(APPEND verdict "  ${C_GREY}${G_DOT} gcc ${GCC_VERSION} ${G_DOT} ${elf_name}${C_RESET}")

string(CONCAT out
    "\n  ${top}\n  ${mid}\n  ${low}\n  ${bot}\n\n"
    "  ${mem_top}\n"
    "${mem_lines}"
    "  ${blank}\n  ${detail}\n  ${mem_bot}\n"
    "  ${verdict}\n")
term_print("${out}")
