#include "Hardware/Rembrandt/Display/DmubInbox.hpp"
#include "Diagnostics/KernelLog.hpp"
#include <IOKit/IOLib.h>
#include <kern/thread.h>

extern bool mmioAvailable();
extern uint32_t mmioRead(uint32_t reg);
extern void mmioWrite(uint32_t reg, uint32_t value);

namespace {

constexpr uint32_t segment = 0x34c0;
constexpr uint32_t control = segment + 0x1f6;
constexpr uint32_t control2 = segment + 0x200;
constexpr uint32_t interruptCommand = segment + 0x1f8;
constexpr uint32_t scratch0 = segment + 0x1e3;
constexpr uint32_t scratch7 = segment + 0x1ea;
constexpr uint32_t bootOptions = segment + 0x1f1;
constexpr uint32_t softReset = segment + 0x342;
constexpr uint32_t inboxWrite = segment + 0x1d6;
constexpr uint32_t inboxRead = segment + 0x1d7;
constexpr uint32_t phyStatus = 0x5df0;
constexpr uint32_t phyClock = 0x5df1;

uint32_t sendDisplayClockMessage(uint32_t message, uint32_t argument) {
    constexpr uint32_t messageRegister = 0x16283, argumentRegister = 0x16293, responseRegister = 0x1629b;
    for (uint32_t i = 0; i < 1000 && !mmioRead(responseRegister); ++i) IODelay(10);
    mmioWrite(responseRegister, 0);
    mmioWrite(argumentRegister, argument);
    mmioWrite(messageRegister, message);
    uint32_t response = 0;
    for (uint32_t i = 0; i < 20000 && !(response = mmioRead(responseRegister)); ++i) IODelay(10);
    return response;
}

void watchDmub(void *, wait_result_t) {
    uint32_t released = 0;
    uint32_t lastState = ~0u;
    for (uint32_t elapsed = 0;; elapsed += elapsed < 60000 ? 1 : 10) {
        IOSleep(elapsed < 60000 ? 1 : 10);
        if (!mmioAvailable()) continue;
        if (elapsed % 2000 == 0 && mmioRead(inboxRead) == mmioRead(inboxWrite) && mmioRead(phyStatus)) {
            uint32_t clock = mmioRead(phyClock);
            if (clock && (clock & 0x101100) != 0x101100) mmioWrite(phyClock, clock | 0x101100);
        }
        uint32_t current = mmioRead(control), reset = mmioRead(control2);
        bool running = (current & (1u << 16)) && !(current & (1u << 17));
        uint32_t state = (static_cast<uint32_t>(running) << 1) | (reset & 1);
        if (state != lastState) {
            lastState = state;
            KernelLog::logDiagnosticInfo(KernelLog::Category::Hardware, "DMUB", "state",
                "reset=%u running=%u inbox_read=0x%x inbox_write=0x%x",
                reset & 1, static_cast<unsigned>(running), mmioRead(inboxRead), mmioRead(inboxWrite));
        }
        if (running && (reset & 1)) {
            mmioWrite(bootOptions, mmioRead(bootOptions) | (1u << 10) | (1u << 6));
            mmioWrite(softReset, mmioRead(softReset) & ~0x100u);
            mmioWrite(control2, reset & ~1u);
            ++released;
            IOSleep(5);
            sendDisplayClockMessage(0x16, 0);
            sendDisplayClockMessage(0x12, 0);
            constexpr uint32_t votes[][2] {{0x7, 600}, {0x8, 400}, {0x4, 600}, {0x6, 600}, {0x9, 810}};
            for (const auto &vote : votes) sendDisplayClockMessage(vote[0], vote[1]);
        } else if (!running && !(reset & 1) && released) {
            mmioWrite(control2, reset | 1u);
        }
    }
}

}

uint32_t vbiosSmcSend(uint32_t message, uint32_t argument) {
    return sendDisplayClockMessage(message, argument);
}

void dmubResetForLoad() {
    if (!mmioAvailable()) return;
    uint32_t reset = mmioRead(control2);
    if (!(reset & 1)) {
        mmioWrite(interruptCommand, (1u << 28) | (2u << 16));
        for (uint32_t i = 0; i < 100 && ((mmioRead(interruptCommand) >> 28) & 1); ++i) IODelay(1);
        for (uint32_t i = 0; i < 100 && mmioRead(scratch7) != 0xdeaddead; ++i) IODelay(1);
        for (uint32_t i = 0; i < 100 && !(mmioRead(control) & (1u << 20)); ++i) IODelay(1);
    }
    mmioWrite(control2, reset | 1u);
    mmioWrite(control, mmioRead(control) & ~(1u << 16));
    mmioWrite(softReset, mmioRead(softReset) | 0x100u);
    constexpr uint32_t registers[] {inboxRead, inboxWrite, segment + 0x1df, segment + 0x1de,
        segment + 0x1db, segment + 0x1da, scratch0};
    for (uint32_t reg : registers) mmioWrite(reg, 0);
    mmioWrite(interruptCommand, 0);
    resetDmubInbox();
}

void dmubPollerStart() {
    static bool started;
    if (started) return;
    thread_t thread;
    if (kernel_thread_start(watchDmub, nullptr, &thread) == KERN_SUCCESS) {
        started = true;
        thread_deallocate(thread);
        KernelLog::logInfo(KernelLog::Category::Hardware, "DMUB", "start", "Controller watch started");
    } else KernelLog::logError(KernelLog::Category::Hardware, "DMUB", "start", "Unable to start controller watch");
}
