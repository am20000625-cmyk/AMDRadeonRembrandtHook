#include "Kexts/AMDRadeonX6000Framebuffer/FramebufferHooks.hpp"
#include "Diagnostics/KernelLog.hpp"
#include <IOKit/IOLib.h>
#include <kern/thread_call.h>
#include <mach/mach_time.h>
#include <Headers/kern_mach.hpp>

extern uint32_t mmioRead(uint32_t reg);

namespace {

using HookFunction = uint64_t (*)(uint64_t, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t);

mach_vm_address_t originalPopulateDeviceInfo;
mach_vm_address_t originalPopulateXgmiConfig;
mach_vm_address_t originalMessageAccelerator;
mach_vm_address_t originalDoesAcceleratorExist;
mach_vm_address_t originalCreateControllerCoreServices;
thread_call_t pplibReportCall;
constexpr uint32_t pplibInitialReports = 5;
uint32_t pplibCalls;
uint64_t pplibLastError;
uint64_t pplibReports[pplibInitialReports + 1];

void reportPplib(thread_call_param_t, thread_call_param_t) {
    for (auto &slot : pplibReports) {
        uint64_t sample = __atomic_exchange_n(&slot, 0, __ATOMIC_ACQUIRE);
        if (!sample) continue;
        KernelLog::logDiagnosticInfo(KernelLog::Category::Hook, "PPLIB", "messageAccelerator",
            "selector=3 call=%u ready_before_dispatch=%u result=0x%08x",
            static_cast<unsigned>((sample >> 32) & 0x3fffffff),
            static_cast<unsigned>((sample >> 62) & 1), static_cast<unsigned>(sample));
    }
}

void queuePplibReport(uint32_t call, bool ready, uint32_t result) {
    if (call > pplibInitialReports) {
        if (!result) return;
        uint64_t now;
        absolutetime_to_nanoseconds(mach_absolute_time(), &now);
        auto last = __atomic_load_n(&pplibLastError, __ATOMIC_RELAXED);
        if (last && (now <= last || now - last < 1000000000ULL)) return;
        if (!__atomic_compare_exchange_n(&pplibLastError, &last, now, false, __ATOMIC_RELAXED, __ATOMIC_RELAXED)) return;
    }
    // The hook may inherit a caller's lock. Format the report on a thread call.
    size_t slot = call <= pplibInitialReports ? call - 1 : pplibInitialReports;
    uint64_t sample = (1ULL << 63) | (static_cast<uint64_t>(ready) << 62) |
        (static_cast<uint64_t>(call & 0x3fffffff) << 32) | result;
    __atomic_store_n(&pplibReports[slot], sample, __ATOMIC_RELEASE);
    thread_call_enter(pplibReportCall);
}

uint64_t wrapCreateControllerCoreServices(uint64_t controller, uint64_t b, uint64_t c, uint64_t d, uint64_t e, uint64_t f) {
    KernelLog::logDiagnosticInfo(KernelLog::Category::Hook, "Framebuffer", "create core services", "Enter: controller=%u",
                                 static_cast<unsigned int>(controller != 0));
    auto result = reinterpret_cast<HookFunction>(originalCreateControllerCoreServices)(controller, b, c, d, e, f);
    KernelLog::logDiagnosticInfo(KernelLog::Category::Hook, "Framebuffer", "create core services", "Return: result=0x%llx",
                                 static_cast<unsigned long long>(result));
    return result;
}

uint64_t wrapPopulateDeviceInfo(uint64_t object, uint64_t b, uint64_t c, uint64_t d, uint64_t e, uint64_t f) {
    auto result = reinterpret_cast<HookFunction>(originalPopulateDeviceInfo)(object, b, c, d, e, f);
    auto info = reinterpret_cast<uint8_t *>(object);
    auto &device = *reinterpret_cast<uint32_t *>(info + 0x64);
    auto &revision = *reinterpret_cast<uint32_t *>(info + 0x68);
    auto &emulationRevision = *reinterpret_cast<uint32_t *>(info + 0x6c);
    emulationRevision -= revision;
    device = 0x73ff;
    revision = 0;
    return result;
}

uint64_t wrapPopulateXgmiConfig(uint64_t object, uint64_t b, uint64_t c, uint64_t d, uint64_t e, uint64_t f) {
    auto result = reinterpret_cast<HookFunction>(originalPopulateXgmiConfig)(object, b, c, d, e, f);
    constexpr uint32_t gcBaseDword = 0x1260;
    auto field = [object](uint32_t offset) -> uint32_t & {
        return *reinterpret_cast<uint32_t *>(object + offset);
    };
    uint32_t base = mmioRead(gcBaseDword + 0x16fc) & 0xffffff;
    uint32_t top = mmioRead(gcBaseDword + 0x16fd) & 0xffffff;
    uint32_t offset = mmioRead(gcBaseDword + 0x16e7) & 0xffffff;
    field(0x138) = 0;
    field(0x13c) = 1;
    field(0x140) = base;
    field(0x144) = top;
    field(0x14c) = offset;
    return result;
}

uint64_t wrapMessageAccelerator(void *controller, uint64_t type, uint64_t a, uint64_t b, uint64_t c, uint64_t d) {
    auto exists = reinterpret_cast<bool (*)(void *)>(originalDoesAcceleratorExist);
    uint32_t call = pplibReportCall && static_cast<uint32_t>(type) == 3 ?
        __atomic_add_fetch(&pplibCalls, 1, __ATOMIC_RELAXED) : 0;
    bool ready = exists(controller);
    if (!ready) {
        for (int i = 0; i < 200 && !(ready = exists(controller)); ++i) IOSleep(100);
    }
    auto result = reinterpret_cast<HookFunction>(originalMessageAccelerator)(
        reinterpret_cast<uint64_t>(controller), type, a, b, c, d);
    if (call) queuePplibReport(call, ready, static_cast<uint32_t>(result));
    return result;
}

}

void patchFramebufferCapabilities(mach_vm_address_t address, size_t size) {
    constexpr uint8_t hdmiPattern[] {0x0f, 0, 0, 0, 0x04, 0, 0, 0, 0xc0, 0x27, 0x09, 0x00, 0x01};
    constexpr uint32_t pixelClockKhz = 340000;
    constexpr uint32_t capabilityPattern[] {5, 5, 5, 5, 5, 0, 1, 5, 16, 5};
    constexpr uint32_t pipeCount = 4;
    constexpr uint8_t pipeCode[] {0x48, 0xb9, 0x05, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff,
        0x49, 0x89, 0x8d, 0xa8, 0x02, 0x00, 0x00, 0x41, 0xc7, 0x85, 0x78, 0x03, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00};
    auto bytes = reinterpret_cast<uint8_t *>(address);
    if (MachInfo::setKernelWriting(true, KernelPatcher::kernelWriteLock) != KERN_SUCCESS) {
        KernelLog::logError(KernelLog::Category::Hook, "Framebuffer", "patch capabilities", "Unable to enable kernel writing");
        return;
    }
    for (size_t i = 0; i + sizeof(hdmiPattern) <= size; ++i) {
        if (!memcmp(bytes + i, hdmiPattern, sizeof(hdmiPattern))) memcpy(bytes + i + 8, &pixelClockKhz, sizeof(pixelClockKhz));
    }
    for (size_t i = 0; i + sizeof(capabilityPattern) <= size; i += 4) {
        if (!memcmp(bytes + i, capabilityPattern, sizeof(capabilityPattern))) {
            for (size_t field = 0; field < 3; ++field) memcpy(bytes + i + field * 4, &pipeCount, sizeof(pipeCount));
        }
    }
    for (size_t i = 0; i + sizeof(pipeCode) <= size; ++i) {
        if (!memcmp(bytes + i, pipeCode, sizeof(pipeCode))) {
            bytes[i + 2] = pipeCount;
            bytes[i + 24] = pipeCount;
        }
    }
    MachInfo::setKernelWriting(false, KernelPatcher::kernelWriteLock);
    KernelLog::logInfo(KernelLog::Category::Hook, "Framebuffer", "patch capabilities", "Capability scan complete");
}

void installFramebufferIdentityHooks(KernelPatcher &patcher, size_t index, mach_vm_address_t address, size_t size) {
    KernelPatcher::RouteRequest requests[] {
        {"__ZN31AMDRadeonX6000_AmdAsicInfoNavi218populateDeviceInfoEv", wrapPopulateDeviceInfo, originalPopulateDeviceInfo},
        {"__ZN31AMDRadeonX6000_AmdAsicInfoNavi218populateXGmiConfigEv", wrapPopulateXgmiConfig, originalPopulateXgmiConfig},
    };
    for (auto &request : requests) {
        if (patcher.routeMultiple(index, &request, 1, address, size)) KernelLog::logInfo(KernelLog::Category::Hook, "Framebuffer", "install identity", "Hook installed: %s", request.symbol);
        else KernelLog::logError(KernelLog::Category::Hook, "Framebuffer", "install identity", "Hook failed: %s error=%d", request.symbol, static_cast<int>(patcher.getError()));
        patcher.clearError();
    }
    KernelPatcher::RouteRequest coreServicesRequest {
        "__ZN40AMDRadeonX6000_AmdRadeonControllerNavi2328createControllerCoreServicesEv",
        wrapCreateControllerCoreServices, originalCreateControllerCoreServices
    };
    if (patcher.routeMultiple(index, &coreServicesRequest, 1, address, size))
        KernelLog::logDiagnosticInfo(KernelLog::Category::Hook, "Framebuffer", "install core services", "Hook installed");
    else
        KernelLog::logDiagnosticWarning(KernelLog::Category::Hook, "Framebuffer", "install core services", "Hook failed: error=%d", static_cast<int>(patcher.getError()));
    patcher.clearError();
    originalDoesAcceleratorExist = patcher.solveSymbol(index,
        "__ZNK34AMDRadeonX6000_AmdRadeonController20doesAcceleratorExistEv", address, size);
    if (!originalDoesAcceleratorExist) KernelLog::logWarning(KernelLog::Category::Hook, "Framebuffer", "resolve", "Accelerator existence function unavailable: error=%d", static_cast<int>(patcher.getError()));
    patcher.clearError();
    KernelPatcher::RouteRequest request {
        "__ZNK34AMDRadeonX6000_AmdRadeonController18messageAcceleratorE25_eAMDAccelIOFBRequestTypePvS1_S1_",
        wrapMessageAccelerator, originalMessageAccelerator
    };
    if (originalDoesAcceleratorExist) {
        pplibReportCall = thread_call_allocate(reportPplib, nullptr);
        if (!pplibReportCall)
            KernelLog::logWarning(KernelLog::Category::Hook, "PPLIB", "install", "Unable to allocate messageAccelerator reporter");
        if (patcher.routeMultiple(index, &request, 1, address, size)) KernelLog::logInfo(KernelLog::Category::Hook, "Framebuffer", "install identity", "Hook installed: %s", request.symbol);
        else KernelLog::logError(KernelLog::Category::Hook, "Framebuffer", "install identity", "Hook failed: %s error=%d", request.symbol, static_cast<int>(patcher.getError()));
    }
    patcher.clearError();
}
