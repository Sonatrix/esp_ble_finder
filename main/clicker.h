#pragma once

#include <stdbool.h>

void clicker_start(void);
void clicker_set_enabled(bool enabled);
void clicker_update(bool fresh, int rssi);
