#include "Kexts/AMDRadeonX6000Framebuffer/FramebufferHooks.hpp"
#include "Diagnostics/KernelLog.hpp"
#include "Diagnostics/Diagnostics.hpp"
#include <IOKit/IOLib.h>
#include <IOKit/IOService.h>
#include <libkern/c++/OSNumber.h>
#include <kern/thread.h>
#include <mach/mach_time.h>
#include <mach-o/loader.h>

extern uint32_t mmioRead(uint32_t reg);
extern void removeVideoCapabilities(void *, wait_result_t);

namespace {

using ControllerFunction = uint64_t (*)(uint64_t, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t);

constexpr mach_vm_address_t windowServerReadyOffset = 0x6914;
mach_vm_address_t originalEstablishBootDisplay;
mach_vm_address_t detectDisplays;
mach_vm_address_t getLink;
mach_vm_address_t agdcAddress;
mach_vm_address_t framebufferOpen;
uint64_t savedController;
bool wakeRecoveryArmed;


// Producer callbacks only publish scalar snapshots. The DMUB watcher drains them.
struct DisplaySample {
    const char *kind;
    uint64_t values[6];
};
struct SampleSlot {
    unsigned state;
    DisplaySample sample;
};
SampleSlot samples[64];
unsigned producerIndex;
bool traceLayoutVerified;
bool matchesFramebufferBuild(mach_vm_address_t address, size_t size) {
    static constexpr uint8_t expected[] {0xf5,0x27,0xf5,0x01,0xcd,0x46,0x34,0xd0,0xae,0xb3,0xc8,0xe5,0xc8,0xad,0xf2,0xc8};
    if (size < sizeof(mach_header_64)) return false;
    auto header = reinterpret_cast<const mach_header_64 *>(address);
    if (header->magic != MH_MAGIC_64 || header->sizeofcmds > size - sizeof(*header)) return false;
    auto cursor = reinterpret_cast<const uint8_t *>(header + 1);
    size_t remaining = header->sizeofcmds;
    for (unsigned i = 0; i < header->ncmds && remaining >= sizeof(load_command); ++i) {
        auto command = reinterpret_cast<const load_command *>(cursor);
        if (command->cmdsize < sizeof(*command) || command->cmdsize > remaining) return false;
        if (command->cmd == LC_UUID && command->cmdsize >= sizeof(uuid_command))
            return !memcmp(reinterpret_cast<const uuid_command *>(cursor)->uuid, expected, sizeof(expected));
        remaining -= command->cmdsize;
        cursor += command->cmdsize;
    }
    return false;
}

void capture(const DisplaySample &sample) {
    unsigned index = __atomic_fetch_add(&producerIndex, 1u, __ATOMIC_RELAXED) % 64;
    unsigned empty = 0;
    if (!__atomic_compare_exchange_n(&samples[index].state, &empty, 1u, false,
            __ATOMIC_ACQUIRE, __ATOMIC_RELAXED)) return;
    samples[index].sample = sample;
    __atomic_store_n(&samples[index].state, 2u, __ATOMIC_RELEASE);
}

template <typename T> T field(uint64_t object, unsigned offset) {
    return *reinterpret_cast<const T *>(object + offset);
}
mach_vm_address_t originalFixed, originalOnline, originalStartup;
uint64_t lastFb0Path;
mach_vm_address_t originalGetLink;
// Snapshot only values observed by the native lookup; replay never follows saved link pointers.
uint32_t linkSnapshots[4];
uint64_t traceGetLink(uint64_t controller, uint8_t index) {
    auto link = reinterpret_cast<uint64_t (*)(uint64_t, uint8_t)>(originalGetLink)(controller, index);
    if (index < 4) {
        uint32_t value = 1;
        if (link) value |= 2u | ((field<uint8_t>(link, 0x54) & 1u) << 2) |
            (uint32_t(field<uint8_t>(link, 0x494)) << 8);
        __atomic_store_n(&linkSnapshots[index], value, __ATOMIC_RELEASE);
    }
    return link;
}
uint32_t startupCalls, fixedCalls, onlineCalls;
uint64_t lastOnlineResult;

uint32_t traceStartup(uint64_t parser, uint64_t path) {
    unsigned call = __atomic_add_fetch(&startupCalls, 1u, __ATOMIC_RELAXED);
    auto result = reinterpret_cast<uint32_t (*)(uint64_t, uint64_t)>(originalStartup)(parser, path);
    if (call <= 5) capture({"getStartUpMode call/result", {call, result}});
    return result;
}

uint32_t traceFixed(uint64_t fb) {
    auto result = reinterpret_cast<uint32_t (*)(uint64_t)>(originalFixed)(fb);
    if (field<uint8_t>(fb, 0x1e0) == 0 &&
        __atomic_fetch_add(&fixedCalls, 1u, __ATOMIC_RELAXED) < 20) {
        auto active = field<uint64_t>(fb, 0x81c8);
        auto staged = field<uint64_t>(fb, 0x81d0);
        __atomic_store_n(&lastFb0Path, active, __ATOMIC_RELEASE);
        capture({"FixedDisplay active/staged/index/onlineA/onlineS/bound",
            {active, staged, active ? field<uint32_t>(active, 0xec) : ~0u,
             active ? unsigned(field<uint8_t>(active, 0x719)) : ~0u,
             staged ? unsigned(field<uint8_t>(staged, 0x719)) : ~0u,
             unsigned(field<uint8_t>(fb, 0x89a4) & 0x20)}});
    }
    return result;
}

uint32_t traceOnline(uint64_t service, const uint32_t *request, size_t length) {
    unsigned target = ~0u, online = ~0u;
    if (request && length == 8) { target = request[0]; online = request[1]; }
    auto result = reinterpret_cast<uint32_t (*)(uint64_t, const uint32_t *, size_t)>(originalOnline)(service, request, length);
    uint64_t pair = (uint64_t(online) << 32) | result;
    auto previous = __atomic_exchange_n(&lastOnlineResult, pair, __ATOMIC_RELAXED);
    if (__atomic_fetch_add(&onlineCalls, 1u, __ATOMIC_RELAXED) < 20 || previous != pair) {
        auto controller = field<uint64_t>(service, 0x118);
        uint64_t path = controller && target < field<uint32_t>(service, 0x170) && target < field<uint8_t>(controller, 0xbd) && target <= 3
            ? controller + 0x3068 + target * 0x7c8 : 0;
        capture({"setFbOnline target/request/result/computedPath/lastFB0Active",
            {target, online, result, path, target == 0 ? __atomic_load_n(&lastFb0Path, __ATOMIC_ACQUIRE) : 0}});
    }
    return result;
}

void openBootFramebuffer(void *, wait_result_t) {
    constexpr uint64_t timeout = 120ULL * 1000 * 1000 * 1000;
    auto matching = IOService::serviceMatching("AMDRadeonX6000_AMDNavi23GraphicsAccelerator");
    auto accelerator = matching ? IOService::waitForMatchingService(matching, timeout) : nullptr;
    OSSafeReleaseNULL(matching);
    if (!accelerator) {
        KernelLog::logWarning(KernelLog::Category::Hook, "DisplayRecovery", "open boot framebuffer", "Accelerator not found within timeout");
        return;
    }
    accelerator->release();
    IOSleep(1000);
    matching = IOService::serviceMatching("AMDRadeonX6000_AmdRadeonFramebuffer");
    auto iterator = matching ? IOService::getMatchingServices(matching) : nullptr;
    OSSafeReleaseNULL(matching);
    IOService *framebuffer = nullptr;
    while (auto service = iterator ? OSDynamicCast(IOService, iterator->getNextObject()) : nullptr) {
        const char *location = service->getLocation();
        if (!framebuffer && location && !strcmp(location, "0")) {
            framebuffer = service;
            framebuffer->retain();
        }
        auto online = service->getProperty("IOFBOnline");
        auto modeProperty = service->copyProperty("IOFBCurrentMode");
        auto mode = OSDynamicCast(OSNumber, modeProperty);
        KernelLog::logDiagnosticInfo(KernelLog::Category::Hook, "DisplayRecovery", "framebuffer candidate",
            "registry_id=0x%llx location=%s online=%s mode_known=%u mode=0x%x config_present=%u",
            static_cast<unsigned long long>(service->getRegistryEntryID()), location ? location : "(none)",
            online == kOSBooleanTrue ? "yes" : online == kOSBooleanFalse ? "no" : "unknown",
            static_cast<unsigned>(mode != nullptr), mode ? mode->unsigned32BitValue() : 0u,
            static_cast<unsigned>(service->getProperty("IOFBConfig") != nullptr));
        OSSafeReleaseNULL(modeProperty);
    }
    OSSafeReleaseNULL(iterator);
    if (!framebuffer) {
        KernelLog::logWarning(KernelLog::Category::Hook, "DisplayRecovery", "open boot framebuffer", "Internal framebuffer not found");
        return;
    }
    KernelLog::logDiagnosticInfo(KernelLog::Category::Hook, "DisplayRecovery", "open boot framebuffer",
        "Scanning vtable slots 200..420");
    auto table = *reinterpret_cast<mach_vm_address_t **>(framebuffer);
    bool foundOpen = false;
    for (size_t slot = 200; table && framebufferOpen && slot <= 420; ++slot) {
        if (table[slot] != framebufferOpen) continue;
        foundOpen = true;
        KernelLog::logDiagnosticInfo(KernelLog::Category::Hook, "DisplayRecovery", "open boot framebuffer",
            "Calling open slot=%lu", static_cast<unsigned long>(slot));
        IOReturn result = reinterpret_cast<IOReturn (*)(IOService *)>(table[slot])(framebuffer);
        KernelLog::logDiagnosticInfo(KernelLog::Category::Hook, "DisplayRecovery", "open boot framebuffer",
            "vtable match slot=%lu result=0x%x", static_cast<unsigned long>(slot), static_cast<unsigned>(result));
        break;
    }
    if (!foundOpen) KernelLog::logDiagnosticWarning(KernelLog::Category::Hook, "DisplayRecovery", "open boot framebuffer", "No open slot match in 200..420");
    framebuffer->release();
}

uint64_t getTimeMicroseconds() {
    uint64_t nanoseconds;
    absolutetime_to_nanoseconds(mach_absolute_time(), &nanoseconds);
    return nanoseconds / 1000;
}

bool isAnyOtgActive() {
    bool active = false;
    for (uint32_t i = 0; i < 4; ++i) active |= mmioRead(0x34c0 + 0x1b41 + i * 0x80) & 1;
    return active;
}

void watchPanelWake(void *, wait_result_t) {
    auto wrangler = IOService::copyMatchingService(IOService::serviceMatching("IODisplayWrangler"));
    uint32_t darkSeconds = 0, replays = 0, noops = 0;
    for (uint32_t seconds = 0; wrangler && seconds < 120 && replays < 2; ++seconds) {
        IOSleep(1000);
        darkSeconds = wrangler->getPowerState() >= 3 && !isAnyOtgActive() ? darkSeconds + 1 : 0;
        if (darkSeconds < 4) continue;
        uint64_t link = reinterpret_cast<ControllerFunction>(getLink)(savedController, 0, 0, 0, 0, 0);
        if (!link) break;
        ++replays;
        KernelLog::logInfo(KernelLog::Category::Hook, "DisplayRecovery", "wake", "Replaying display detection: attempt=%u", replays);
        *reinterpret_cast<uint8_t *>(link + 0x494) = 0;
        memset(reinterpret_cast<void *>(link + 0x4a0), 0, 0x400);
        uint64_t started = getTimeMicroseconds();
        uint64_t result = reinterpret_cast<ControllerFunction>(detectDisplays)(savedController, 1, 0, 0, 0, 0);
        uint64_t elapsed = getTimeMicroseconds() - started;
        logDisplayRecoveryDiagnostics("wake detection", replays, result, elapsed);
        if (elapsed < 100000 && noops < 5) {
            ++noops;
            --replays;
            darkSeconds = 3;
        } else {
            darkSeconds = 0;
        }
    }
    OSSafeReleaseNULL(wrangler);
}

void replayPanelConnection(void *argument, wait_result_t) {
    uint64_t controller = reinterpret_cast<uint64_t>(argument);
    IOSleep(15000);
    if (agdcAddress) {
        auto ready = reinterpret_cast<volatile int32_t *>(agdcAddress + windowServerReadyOffset);
        if (*ready == 0) *ready = 1;
        IOSleep(40000);
    }
    uint64_t link = reinterpret_cast<ControllerFunction>(getLink)(controller, 0, 0, 0, 0, 0);
    if (traceLayoutVerified) {
        for (unsigned i = 0; i < 4; ++i) {
            auto value = __atomic_load_n(&linkSnapshots[i], __ATOMIC_ACQUIRE);
            capture({"DisplayRecovery replay index/observed/present/native54/cache494",
                {i, value & 1u, (value >> 1) & 1u, (value >> 2) & 1u, value >> 8}});
        }
    }
    uint8_t connection = link ? *reinterpret_cast<const uint8_t *>(link + 0x494) : 0;
    KernelLog::logDiagnosticInfo(KernelLog::Category::Hook, "DisplayRecovery", "boot replay",
        "getLink(0) present=%u connection=0x%02x", static_cast<unsigned>(link != 0), static_cast<unsigned>(connection));
    if (!link || connection) return;
    *reinterpret_cast<uint8_t *>(link + 0x494) = 0;
    memset(reinterpret_cast<void *>(link + 0x4a0), 0, 0x400);
    uint64_t started = getTimeMicroseconds();
    uint64_t result = reinterpret_cast<ControllerFunction>(detectDisplays)(controller, 1, 0, 0, 0, 0);
    logDisplayRecoveryDiagnostics("boot detection", 1, result, getTimeMicroseconds() - started);
}

uint64_t wrapEstablishBootDisplay(uint64_t controller, uint64_t b, uint64_t c, uint64_t d, uint64_t e, uint64_t f) {
    if (traceLayoutVerified) {
        auto features = field<uint64_t>(controller, 0x78e8);
        auto parser = field<uint64_t>(controller, 0x7910);
        capture({"DisplayRecovery CFG_NO_BOOT_PWDN/parserValid",
            {features ? ((field<uint32_t>(features, 0x28) >> 17) & 1u) : ~0u,
             parser ? (field<uint8_t>(parser, 0x328) & 1u) : ~0u}});
    }
    uint64_t result = reinterpret_cast<ControllerFunction>(originalEstablishBootDisplay)(controller, b, c, d, e, f);
    static uint32_t reports;
    if (__atomic_load_n(&reports, __ATOMIC_RELAXED) < 8 && __atomic_fetch_add(&reports, 1, __ATOMIC_RELAXED) < 8)
        KernelLog::logDiagnosticInfo(KernelLog::Category::Hook, "DisplayRecovery", "establish boot display",
            "result=0x%llx", static_cast<unsigned long long>(result));
    if (static_cast<uint32_t>(result) == 0xe00002f0 && detectDisplays && getLink) {
        savedController = controller;
        thread_t thread;
        if (kernel_thread_start(replayPanelConnection, reinterpret_cast<void *>(controller), &thread) == KERN_SUCCESS)
            thread_deallocate(thread);
    }
    return result;
}

}

void drainDisplaySamples() {
    for (auto &slot : samples) {
        if (__atomic_load_n(&slot.state, __ATOMIC_ACQUIRE) != 2) continue;
        const auto copy = slot.sample;
        __atomic_store_n(&slot.state, 0u, __ATOMIC_RELEASE);
        KernelLog::logDiagnosticInfo(KernelLog::Category::Hook, "DisplayRecovery", copy.kind,
            "%llx %llx %llx %llx %llx %llx", copy.values[0], copy.values[1], copy.values[2],
            copy.values[3], copy.values[4], copy.values[5]);
    }
}

void armPanelWakeRecovery() {
    wakeRecoveryArmed = true;
}

void startPanelWakeRecovery() {
    if (!wakeRecoveryArmed || !savedController || !getLink || !detectDisplays) return;
    wakeRecoveryArmed = false;
    thread_t thread;
    if (kernel_thread_start(watchPanelWake, nullptr, &thread) == KERN_SUCCESS) {
        KernelLog::logInfo(KernelLog::Category::Hook, "DisplayRecovery", "wake", "Panel wake watch started");
        thread_deallocate(thread);
    } else KernelLog::logError(KernelLog::Category::Hook, "DisplayRecovery", "wake", "Unable to start panel wake watch");
}

void setDisplayRecoveryAgdcAddress(mach_vm_address_t address) {
    agdcAddress = address;
}

void prepareDisplayRecovery(KernelPatcher &patcher, size_t index, mach_vm_address_t address, size_t size) {
    framebufferOpen = patcher.solveSymbol(index, "__ZN35AMDRadeonX6000_AmdRadeonFramebuffer4openEv", address, size);
    if (!framebufferOpen) KernelLog::logWarning(KernelLog::Category::Hook, "DisplayRecovery", "resolve", "Framebuffer open unavailable: error=%d", static_cast<int>(patcher.getError()));
    patcher.clearError();
    thread_t thread;
    if (kernel_thread_start(openBootFramebuffer, nullptr, &thread) == KERN_SUCCESS) thread_deallocate(thread);
    else KernelLog::logError(KernelLog::Category::Hook, "DisplayRecovery", "start", "Unable to start framebuffer open worker");
    if (kernel_thread_start(removeVideoCapabilities, nullptr, &thread) == KERN_SUCCESS) thread_deallocate(thread);
    else KernelLog::logError(KernelLog::Category::Hook, "DisplayRecovery", "start", "Unable to start video capability worker");
    detectDisplays = patcher.solveSymbol(index, "__ZN34AMDRadeonX6000_AmdRadeonController14detectDisplaysEj", address, size);
    if (!detectDisplays) KernelLog::logWarning(KernelLog::Category::Hook, "DisplayRecovery", "resolve", "Display detection unavailable: error=%d", static_cast<int>(patcher.getError()));
    patcher.clearError();
    getLink = patcher.solveSymbol(index, "__ZNK34AMDRadeonX6000_AmdRadeonController7getLinkEh", address, size);
    if (!getLink) KernelLog::logWarning(KernelLog::Category::Hook, "DisplayRecovery", "resolve", "Display link lookup unavailable: error=%d", static_cast<int>(patcher.getError()));
    patcher.clearError();
}

void installDisplayRecoveryHook(KernelPatcher &patcher, size_t index, mach_vm_address_t address, size_t size) {
    KernelPatcher::RouteRequest traces[] {
        {"__ZNK34AMDRadeonX6000_AmdRadeonController7getLinkEh", traceGetLink, originalGetLink},
        {"__ZN35AMDRadeonX6000_AmdRadeonFramebuffer29initializeFixedDisplayBindingEv", traceFixed, originalFixed},
        {"__ZN30AMDRadeonX6000_AmdAgdcServices11setFbOnlineEP14AGDCFBOnline_tm", traceOnline, originalOnline},
        {"__ZNK32AMDRadeonX6000_AmdBootModeParser14getStartUpModeEP16AmdFbDisplayPath", traceStartup, originalStartup}
    };
    traceLayoutVerified = matchesFramebufferBuild(address, size);
    for (auto &trace : traces) {
        if (!traceLayoutVerified) break;
        bool ok = patcher.routeMultiple(index, &trace, 1, address, size);
        KernelLog::logDiagnosticInfo(KernelLog::Category::Hook, "DisplayRecovery", "route",
            "%s installed=%u", trace.symbol, static_cast<unsigned>(ok));
        patcher.clearError();
    }
    KernelPatcher::RouteRequest request {
        "__ZN34AMDRadeonX6000_AmdRadeonController20establishBootDisplayEv",
        wrapEstablishBootDisplay, originalEstablishBootDisplay
    };
    if (patcher.routeMultiple(index, &request, 1, address, size)) KernelLog::logInfo(KernelLog::Category::Hook, "DisplayRecovery", "install", "Boot display hook installed");
    else KernelLog::logError(KernelLog::Category::Hook, "DisplayRecovery", "install", "Boot display hook failed: error=%d", static_cast<int>(patcher.getError()));
    patcher.clearError();
}
