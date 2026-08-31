#include "security.h"
#include "global.h"

void connection_lost_routine()
{

    if (debug_warn)
        print_to_console("\nremote connection lost !\n", 26);

    // ICI DESACTIVER LA LOOP CONTROL ET SECURITY
    R_LED_On();

    // do
    // {
        
    // }
    // while(connection non re-established);

    R_LED_Off();

    if(debug_warn)
        print_to_console("\nremote connection re-etablished !\n", 35);
}