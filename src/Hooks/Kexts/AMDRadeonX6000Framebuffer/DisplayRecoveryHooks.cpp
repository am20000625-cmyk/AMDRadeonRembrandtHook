#include "Kexts/AMDRadeonX6000Framebuffer/FramebufferHooks.hpp"
#include "Diagnostics/KernelLog.hpp"
#include "Diagnostics/Diagnostics.hpp"
#include <IOKit/IOLib.h>
#include <IOKit/IOService.h>
#include <libkern/c++/OSNumber.h>
#include <kern/thread.h>
#include <mach/mach_time.h>

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
    KernelPatcher::RouteRequest request {
        "__ZN34AMDRadeonX6000_AmdRadeonController20establishBootDisplayEv",
        wrapEstablishBootDisplay, originalEstablishBootDisplay
    };
    if (patcher.routeMultiple(index, &request, 1, address, size)) KernelLog::logInfo(KernelLog::Category::Hook, "DisplayRecovery", "install", "Boot display hook installed");
    else KernelLog::logError(KernelLog::Category::Hook, "DisplayRecovery", "install", "Boot display hook failed: error=%d", static_cast<int>(patcher.getError()));
    patcher.clearError();
}
