#pragma once

/* Same radio bounds as findphone: laptop-on-phone is about -50 dBm. */
#define PROXIMITY_TOUCHING_DBM (-50)
#define PROXIMITY_DISTANT_DBM  (-95)

#define PROXIMITY_CLICK_FASTEST_MS 70
#define PROXIMITY_CLICK_SLOWEST_MS 1000

static inline int proximity_clamp(int value, int lo, int hi)
{
    if (value < lo) {
        return lo;
    }
    if (value > hi) {
        return hi;
    }
    return value;
}

static inline int proximity_strength(int rssi)
{
    const int span = PROXIMITY_TOUCHING_DBM - PROXIMITY_DISTANT_DBM;
    return proximity_clamp((rssi - PROXIMITY_DISTANT_DBM) * 100 / span, 0, 100);
}

static inline int proximity_band(int rssi)
{
    if (rssi >= -45) {
        return 0;
    }
    if (rssi >= -60) {
        return 1;
    }
    if (rssi >= -72) {
        return 2;
    }
    if (rssi >= -85) {
        return 3;
    }
    return 4;
}

static inline const char *proximity_status(int rssi)
{
    if (rssi >= -45) {
        return "ARM'S REACH";
    }
    if (rssi >= -60) {
        return "SAME TABLE";
    }
    if (rssi >= -72) {
        return "SAME ROOM";
    }
    if (rssi >= -85) {
        return "FAR";
    }
    return "VERY FAR";
}

static inline const char *proximity_trend(int rssi, int previous)
{
    if (rssi >= previous + 2) {
        return "WARMER";
    }
    if (rssi <= previous - 2) {
        return "COLDER";
    }
    return "STEADY";
}

static inline int proximity_click_ms(int rssi)
{
    const float ramp = (float)(rssi - PROXIMITY_DISTANT_DBM)
                       / (float)(PROXIMITY_TOUCHING_DBM - PROXIMITY_DISTANT_DBM);
    const float clamped = ramp < 0.f ? 0.f : ramp > 1.f ? 1.f : ramp;
    return (int)(PROXIMITY_CLICK_SLOWEST_MS
                 - clamped * (PROXIMITY_CLICK_SLOWEST_MS - PROXIMITY_CLICK_FASTEST_MS));
}
