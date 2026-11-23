#include "Hardware/Rembrandt/Display/DisplayClock.hpp"
#include <IOKit/IOLib.h>

extern uint32_t mmioRead(uint32_t reg);
extern void mmioWrite(uint32_t reg, uint32_t value);
extern uint64_t vramRead64(uint64_t offset);
extern bool vramWrite32(uint64_t offset, uint32_t value);
extern uint32_t vbiosSmcSend(uint32_t message, uint32_t parameter);

namespace {

constexpr uint32_t segmentDword = 0x34c0;
DisplayStream externalStream;

uint32_t getHdmiClock(const DisplayStream &stream) {
    uint64_t clock = static_cast<uint64_t>(stream.pixelClockKhz) * 10;
    if (stream.encoding == 4) clock /= 2;
    switch (stream.depth) {
        case 3: return static_cast<uint32_t>(clock * 5 / 4);
        case 4: return static_cast<uint32_t>(clock * 6 / 4);
        case 6: return static_cast<uint32_t>(clock * 2);
        default: return static_cast<uint32_t>(clock);
    }
}

bool startOtgClock(uint32_t crtc, uint32_t clock100Hz) {
    if (crtc > 3) return false;
    uint32_t otgClock = segmentDword + 0x1b86 + crtc * 0x80;
    uint32_t odmClock = segmentDword + 0x1acf + crtc * 0x10;
    uint32_t rate = 0xc0 + 0x80 + crtc * 4;
    uint32_t before = mmioRead(otgClock);
    if ((before & (1u << 8)) && (mmioRead(rate) & 0x18)) return true;
    uint32_t count = 0;
    if (before & 1) {
        mmioWrite(otgClock, before & ~1u);
        while ((mmioRead(otgClock) & (1u << 8)) && count++ < 1000) IODelay(10);
    }
    uint32_t hz = clock100Hz * 100 < 600000000u ? clock100Hz * 100 : 600000000u;
    mmioWrite(rate + 1, hz);
    mmioWrite(rate + 2, 600000000u);
    mmioWrite(rate, mmioRead(rate) | 0x10);
    count = 0;
    while (!(mmioRead(rate) & 0x80) && count++ < 1000) IODelay(10);
    mmioWrite(odmClock, mmioRead(odmClock) | 3);
    mmioWrite(otgClock, mmioRead(otgClock) | 3);
    count = 0;
    bool locked;
    while (!(locked = (mmioRead(otgClock) & (1u << 8)) != 0) && count++ < 1000) IODelay(10);
    return locked;
}

}

const DisplayStream &getExternalDisplayStream() {
    return externalStream;
}

void setExternalDisplayStream(uint32_t pixelClockKhz, uint32_t encoding, uint32_t depth, uint32_t link) {
    externalStream = {pixelClockKhz, encoding, depth, link};
}

void configureHdmiPixelClock(uint64_t ring, uint32_t offset, uint32_t clock, HdmiClockDiagnostic &diagnostic) {
    const auto &stream = getExternalDisplayStream();
    uint64_t data = vramRead64(ring + offset + 8);
    uint32_t word2 = static_cast<uint32_t>(data), word3 = static_cast<uint32_t>(data >> 32);
    if (((word2 >> 16) & 0xff) != 3) return;
    uint32_t crtc = word3 & 0xff;
    bool locked = startOtgClock(crtc, clock);
    diagnostic.clock100Hz = clock;
    diagnostic.crtc = static_cast<int32_t>(crtc);
    diagnostic.otgLocked = locked;
    diagnostic.stream = stream;
    if (!stream.pixelClockKhz || getHdmiClock(stream) != clock) return;
    static uint32_t currentMhz = 600;
    uint32_t mhz = static_cast<uint32_t>((static_cast<uint64_t>(stream.pixelClockKhz) * 101 + 99999) / 100000);
    mhz = mhz < 600 ? 600 : mhz > 1200 ? 1200 : mhz;
    if (mhz != currentMhz) {
        vbiosSmcSend(0x4, mhz);
        vbiosSmcSend(0x6, mhz);
        currentMhz = mhz;
    }
    uint32_t ratio = stream.depth == 3 ? 1 : stream.depth == 4 ? 2 : stream.depth == 6 ? 3 : 0;
    vramWrite32(ring + offset + 8, word2 | (stream.encoding == 4 ? 0x04u << 24 : 0));
    vramWrite32(ring + offset + 12, (word3 & ~0xff00u) | ratio << 8);
}

