#include <assert.h>
#include <string.h>

#include "../main/proximity.h"

int main(void)
{
    assert(proximity_strength(-100) == 0);
    assert(proximity_strength(-95) == 0);
    assert(proximity_strength(-50) == 100);
    assert(proximity_strength(-40) == 100);
    assert(proximity_click_ms(-95) == PROXIMITY_CLICK_SLOWEST_MS);
    assert(proximity_click_ms(-50) == PROXIMITY_CLICK_FASTEST_MS);
    assert(proximity_band(-45) == 0);
    assert(proximity_band(-86) == 4);
    assert(strcmp(proximity_status(-60), "SAME TABLE") == 0);
    assert(strcmp(proximity_status(-72), "SAME ROOM") == 0);
    assert(strcmp(proximity_status(-85), "FAR") == 0);
    assert(strcmp(proximity_status(-86), "VERY FAR") == 0);
    assert(strcmp(proximity_trend(-50, -60), "WARMER") == 0);
    assert(strcmp(proximity_trend(-70, -60), "COLDER") == 0);
    assert(strcmp(proximity_trend(-60, -60), "STEADY") == 0);
}
