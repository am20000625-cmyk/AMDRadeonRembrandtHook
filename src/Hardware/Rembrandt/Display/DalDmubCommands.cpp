#include "Hardware/Rembrandt/Display/DalDmubCommands.hpp"
#include "Hardware/Rembrandt/Display/DmubInbox.hpp"
#include "Hardware/Rembrandt/Display/DisplayClock.hpp"
#include "Hardware/Rembrandt/Display/PanelBacklight.hpp"
#include "Diagnostics/KernelLog.hpp"
#include <IOKit/IOLib.h>

extern uint32_t mmioRead(uint32_t reg);
extern void mmioWrite(uint32_t reg, uint32_t value);
extern uint64_t vramRead64(uint64_t offset);
extern bool vramWrite32(uint64_t offset, uint32_t value);

namespace {

constexpr uint32_t segmentDword = 0x34c0;
constexpr uint32_t writePointerDword = segmentDword + 0x1d6;
constexpr uint32_t readPointerDword = segmentDword + 0x1d7;
constexpr uint32_t pwmPeriodDword = segmentDword + 0x2f1b;
constexpr uint32_t ringBytes = 0x2000;
constexpr uint32_t commandBytes = 64;

uint64_t panelOffCommands[2][8];
uint64_t panelOnCommands[2][8];
uint64_t pllCommands[5][8];
uint32_t panelOffSaved, panelOnSaved, pendingPll;
bool panelPowered, panelOffDone, panelOffAllowed, displayCold, clockManagerSent;

bool isOtgActive() {
    bool active = false;
    for (uint32_t i = 0; i < 4; ++i) active |= mmioRead(segmentDword + 0x1b41 + i * 0x80) & 1;
    return active;
}

void saveCommand(uint64_t destination[8], uint64_t ring, uint32_t offset) {
    for (uint32_t i = 0; i < 8; ++i) destination[i] = vramRead64(ring + offset + i * 8);
}

void writeCommand(uint64_t ring, uint32_t offset, const uint64_t source[8]) {
    for (uint32_t i = 1; i < 16; ++i)
        vramWrite32(ring + offset + i * 4, static_cast<uint32_t>(source[i / 2] >> (i % 2 * 32)));
    vramWrite32(ring + offset, static_cast<uint32_t>(source[0]));
}

bool sendDeferredClockCommand(uint64_t ring, uint32_t offset) {
    if (!getExternalDisplayStream().pixelClockKhz || !isOtgActive()) return false;
    if (!clockManagerSent) {
        clockManagerSent = true;
        const uint32_t command[] {0x49, 600000, 100000, 0};
        for (uint32_t i = 15; i >= 1; --i) vramWrite32(ring + offset + i * 4, i < 4 ? command[i] : 0);
        vramWrite32(ring + offset, command[0]);
        return true;
    }
    if (!pendingPll || getExternalDisplayStream().link >= 2) return false;
    uint32_t index = static_cast<uint32_t>(__builtin_ctz(pendingPll));
    pendingPll &= ~(1u << index);
    writeCommand(ring, offset, pllCommands[index]);
    return true;
}


}

void setPanelPowerOffAllowed(bool allowed) {
    panelOffAllowed = allowed;
}

void markPanelPowerLost() {
    panelPowered = false;
    displayCold = true;
}

void powerOffPanel() {
    if (panelOffDone) return;
    if (panelOffSaved != 3) {
        if (panelOnSaved != 3) return;
        for (uint32_t k = 0; k < 2; ++k)
            for (uint32_t i = 0; i < 8; ++i) panelOffCommands[k][i] = panelOnCommands[k][i];
        panelOffCommands[0][0] = (panelOffCommands[0][0] & ~(0xffull << 32)) | (0x0dull << 32);
        panelOffCommands[1][0] = (panelOffCommands[1][0] & ~(0xffull << 40)) | (0x0dull << 40);
    }
    uint64_t ring = getDmubInboxOffset();
    uint32_t hardwareWrite = mmioRead(writePointerDword);
    uint32_t write = gDalWritePointer != ~0u && !gDmubAhead ? gDalWritePointer : hardwareWrite;
    for (uint32_t k = 0; k < 2; ++k, write = (write + commandBytes) % ringBytes)
        for (uint32_t i = 0; i < 16; ++i)
            vramWrite32(ring + write + i * 4, static_cast<uint32_t>(panelOffCommands[k][i / 2] >> (i % 2 * 32)));
    mmioWrite(writePointerDword, write);
    if (gDmubAhead) gDmubAhead += 2;
    else gDalWritePointer = write;
    uint32_t milliseconds = 0;
    while (mmioRead(readPointerDword) != write && milliseconds++ < 500) IODelay(1000);
    panelPowered = false;
    panelOffDone = true;
}

uint32_t filterDalDmubCommands(uint32_t oldPointer, uint32_t newPointer) {
    uint64_t ring = getDmubInboxOffset();
    if (mmioRead(pwmPeriodDword) & 0xffff) savePanelPowerSequence();
    uint32_t real = 0;
    for (uint32_t offset = oldPointer; offset != newPointer; offset = (offset + commandBytes) % ringBytes) {
        uint64_t command = vramRead64(ring + offset);
        uint32_t header = static_cast<uint32_t>(command);
        bool panelOff = ((header & 0xffff) == 0x0f80 && ((command >> 32) & 0xff) == 0x0d) ||
            ((header & 0xffff) == 0x0180 && ((command >> 40) & 0xff) == 0x0d);
        bool panelOn = ((header & 0xffff) == 0x0f80 && ((command >> 32) & 0xff) == 0x0c) ||
            ((header & 0xffff) == 0x0180 && ((command >> 40) & 0xff) == 0x0c);
        if (panelOn) {
            if (panelPowered) {
                vramWrite32(ring + offset, header & ~0xffffu);
                if (sendDeferredClockCommand(ring, offset)) ++real;
                continue;
            }
            uint32_t index = (header & 0xffff) == 0x0f80 ? 0 : 1;
            saveCommand(panelOnCommands[index], ring, offset);
            panelOnSaved |= 1u << index;
            if (index == 1) panelPowered = true;
            panelOffDone = false;
            panelOffAllowed = false;
        }
        if ((header & 0xff) == 0x4c) {
            vramWrite32(ring + offset, header & ~0xffffu);
            if (sendDeferredClockCommand(ring, offset)) ++real;
            continue;
        }
        if ((header & 0xff) == 0x42) {
            bool firmwarePwm = !(mmioRead(pwmPeriodDword) & 0xffff);
            if (!firmwarePwm && (header & 0xffff) == 0x0242) {
                vramWrite32(ring + offset, header & ~0xffffu);
                setBacklightFromDal(static_cast<uint32_t>(vramRead64(ring + offset + 8)));
                if (sendDeferredClockCommand(ring, offset)) ++real;
                continue;
            }
            if (!firmwarePwm && (header & 0xffff) == 0x0142) {
                vramWrite32(ring + offset, header & ~0xffffu);
                if (sendDeferredClockCommand(ring, offset)) ++real;
                continue;
            }
        }
        if ((header & 0xffff) == 0x0280) {
            static uint32_t reports;
            if (__atomic_load_n(&reports, __ATOMIC_RELAXED) < 24 && __atomic_fetch_add(&reports, 1, __ATOMIC_RELAXED) < 24) {
                const auto &stream = getExternalDisplayStream();
                KernelLog::logDiagnosticInfo(KernelLog::Category::Hardware, "HDMI", "DAL command",
                    "0x0280 clock_100hz=%u external_clock_khz=%u external_link=%u otg_active=%u",
                    static_cast<uint32_t>(command >> 32), stream.pixelClockKhz, stream.link, static_cast<unsigned>(isOtgActive()));
            }
        }
        if ((header & 0xffff) == 0x0280 && (command >> 32)) configureHdmiPixelClock(ring, offset, command >> 32);
        if ((header & 0xffff) == 0x0280 && !(command >> 32)) {
            uint32_t pll = vramRead64(ring + offset + 8) & 0xff;
            bool active = isOtgActive();
            bool usbPll = pll == 22 || pll == 23;
            if (pll == 20 || usbPll || !active) {
                if (!usbPll && pll > 20 && pll <= 25) {
                    saveCommand(pllCommands[pll - 21], ring, offset);
                    pendingPll |= 1u << (pll - 21);
                }
                vramWrite32(ring + offset, header & ~0xffffu);
                continue;
            }
        }
        if (panelOff && panelOffAllowed) {
            panelPowered = false;
            panelOffDone = true;
            ++real;
            continue;
        }
        if (panelOff) {
            uint32_t index = (header & 0xffff) == 0x0f80 ? 0 : 1;
            saveCommand(panelOffCommands[index], ring, offset);
            panelOffSaved |= 1u << index;
            vramWrite32(ring + offset, header & ~0xffffu);
            continue;
        }
        if ((header & 0xffff) == 0x0180 && displayCold) displayCold = false;
        if ((header & 0xffff) == 0x0380 && !displayCold) {
            vramWrite32(ring + offset, header & ~0xffffu);
            continue;
        }
        if (header & 0xff) ++real;
    }
    return real;
}
