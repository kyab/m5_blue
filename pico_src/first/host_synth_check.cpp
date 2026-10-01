// Host-side smoke check for Synth.hpp and Y-zone / X-accidental mapping.
// Build: clang++ -std=c++17 -O0 -o /tmp/host_synth_check host_synth_check.cpp && /tmp/host_synth_check

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include "Synth.hpp"

static const int16_t kFull = 4096;
static const int kZones = 9;
static const int kSemis[9] = {0, 2, 4, 5, 7, 9, 11, 12, 14};
static const int16_t kXDead = static_cast<int16_t>(0.2f * static_cast<float>(kFull));

static int map_y(int16_t y) {
    int32_t yy = y;
    if (yy < -kFull) yy = -kFull;
    if (yy > kFull) yy = kFull;
    const int32_t span = (int32_t)kFull * 2;
    int32_t pos = yy + kFull;
    int zone = (int)((pos * kZones) / span);
    if (zone >= kZones) zone = kZones - 1;
    return kSemis[zone];
}

static int map_x(int16_t x) {
    if (x < -kXDead) return -1;
    if (x > kXDead) return 1;
    return 0;
}

static void expect_eq(const char* name, int got, int want) {
    if (got != want) {
        std::printf("FAIL %s: got %d want %d\n", name, got, want);
        std::exit(1);
    }
}

int main() {
    expect_eq("y=0 -> ソ", map_y(0), 7);
    expect_eq("y=-4096 -> 低ド", map_y(-4096), 0);
    expect_eq("y=+4096 -> 高レ", map_y(4096), 14);
    expect_eq("x dead", map_x(0), 0);
    expect_eq("x dead edge-", map_x(-kXDead), 0);
    expect_eq("x dead edge+", map_x(kXDead), 0);
    expect_eq("x flat", map_x((int16_t)(-kXDead - 1)), -1);
    expect_eq("x sharp", map_x((int16_t)(kXDead + 1)), 1);

    Synth synth;
    synth.reset();
    synth.setSemitone(7);
    int16_t buf[8] = {0};
    synth.gen(buf, 4);
    // sine starts at phase 0 → first sample near 0
    if (std::abs((int)buf[0]) > 50) {
        std::printf("FAIL first sample not near 0: %d\n", (int)buf[0]);
        return 1;
    }

    // Mid-note setSemitone keeps generating without crashing; phase continuity smoke.
    synth.setSemitone(12);
    synth.gen(buf, 4);

    std::printf("host_synth_check OK\n");
    return 0;
}
