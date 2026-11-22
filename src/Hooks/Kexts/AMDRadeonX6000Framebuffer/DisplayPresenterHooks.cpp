#include "Kexts/AMDRadeonX6000Framebuffer/FramebufferHooks.hpp"
#include "Diagnostics/KernelLog.hpp"
#include <Headers/kern_util.hpp>
#include <IOKit/IOLib.h>
#include <IOKit/IOInterruptEventSource.h>
#include <libkern/c++/OSObject.h>
#include <kern/thread.h>
#include <mach/mach_time.h>

namespace {

constexpr uint32_t presenterCount = 6;
constexpr uint32_t timingCaptureSeconds = 900;
mach_vm_address_t originalVblank;
mach_vm_address_t originalVupdate;
mach_vm_address_t originalEnqueue;
mach_vm_address_t originalProcessFlip;
mach_vm_address_t originalDeferredComplete;
mach_vm_address_t originalEnable;
mach_vm_address_t setPipeInterruptState;
const OSMetaClass *presenterMetaClass;
struct PresenterState {
    uint64_t presenter;
    uint32_t vblankCount;
    uint32_t vupdateCount;
    uint32_t enqueueCount;
};
struct EventTiming {
    uint64_t lastEvent;
    uint64_t maxGap;
    uint32_t count;
};
struct PresenterTiming {
    EventTiming vblank;
    EventTiming vupdate;
    EventTiming enqueue;
    EventTiming flip;
    EventTiming deferredComplete;
};
PresenterState presenters[presenterCount];
PresenterTiming presenterTimings[presenterCount];
bool presenterEnabled[presenterCount];
IOSimpleLock *presenterLock;
bool presenterTracking;
bool presenterTimingEnabled;
bool flipTimingInstalled;
bool deferredCompleteTimingInstalled;
bool presenterEntryDiagnostics;
uint32_t rawEvents[5];
uint32_t rejectedClass, rejectedIndex, rejectedUnregistered;
const char *firstRejectedClass;
int32_t firstRejectedIndex = -1;

uint64_t readPresenterField(uint64_t presenter, uint32_t offset) {
    return *reinterpret_cast<volatile uint64_t *>(presenter + offset);
}

int32_t recordPresenter(uint64_t presenter, EventTiming PresenterTiming::*event, bool timingOnly = false) {
    bool collect = __atomic_load_n(&presenterEntryDiagnostics, __ATOMIC_RELAXED);
    if (collect) {
        size_t index = event == &PresenterTiming::vblank ? 0 : event == &PresenterTiming::vupdate ? 1 :
            event == &PresenterTiming::enqueue ? 2 : event == &PresenterTiming::flip ? 3 : 4;
        __atomic_fetch_add(&rawEvents[index], 1, __ATOMIC_RELAXED);
    }
    const auto *meta = presenter ? reinterpret_cast<OSObject *>(presenter)->getMetaClass() : nullptr;
    if (!presenter || !presenterMetaClass || meta != presenterMetaClass) {
        if (collect) {
            __atomic_fetch_add(&rejectedClass, 1, __ATOMIC_RELAXED);
            const char *empty = nullptr;
            const char *name = meta ? meta->getClassName() : "(none)";
            __atomic_compare_exchange_n(&firstRejectedClass, &empty, name, false, __ATOMIC_RELAXED, __ATOMIC_RELAXED);
        }
        return -1;
    }
    uint8_t index = *reinterpret_cast<volatile uint8_t *>(presenter + 0xa0);
    if (index >= presenterCount) {
        if (collect) {
            __atomic_fetch_add(&rejectedIndex, 1, __ATOMIC_RELAXED);
            int32_t unset = -1;
            __atomic_compare_exchange_n(&firstRejectedIndex, &unset, static_cast<int32_t>(index),
                false, __ATOMIC_RELAXED, __ATOMIC_RELAXED);
        }
        return -1;
    }
    OSObject *previous = nullptr;
    auto interruptState = IOSimpleLockLockDisableInterrupt(presenterLock);
    if (!presenterTracking || (timingOnly && (!presenterTimingEnabled || presenters[index].presenter != presenter))) {
        if (collect && presenterTracking && timingOnly && presenterTimingEnabled && presenters[index].presenter != presenter)
            __atomic_fetch_add(&rejectedUnregistered, 1, __ATOMIC_RELAXED);
        IOSimpleLockUnlockEnableInterrupt(presenterLock, interruptState);
        return -1;
    }
    if (presenters[index].presenter != presenter) {
        reinterpret_cast<OSObject *>(presenter)->retain();
        previous = reinterpret_cast<OSObject *>(presenters[index].presenter);
        presenters[index] = {presenter, 0, 0, 0};
        presenterTimings[index] = {};
        presenterEnabled[index] = false;
    }
    if (presenterTimingEnabled) {
        auto &timing = presenterTimings[index].*event;
        auto now = mach_absolute_time();
        if (timing.lastEvent) {
            auto gap = now - timing.lastEvent;
            if (gap > timing.maxGap) timing.maxGap = gap;
        }
        timing.lastEvent = now;
        ++timing.count;
    }
    IOSimpleLockUnlockEnableInterrupt(presenterLock, interruptState);
    if (previous) previous->release();
    return index;
}

void incrementPresenterCount(int32_t index, uint64_t presenter, uint32_t PresenterState::*field) {
    if (index < 0) return;
    auto interruptState = IOSimpleLockLockDisableInterrupt(presenterLock);
    if (presenters[index].presenter == presenter) ++(presenters[index].*field);
    IOSimpleLockUnlockEnableInterrupt(presenterLock, interruptState);
}

void wrapVblank(uint64_t owner, uint64_t argument) {
    auto index = recordPresenter(owner, &PresenterTiming::vblank);
    reinterpret_cast<void (*)(uint64_t, uint64_t)>(originalVblank)(owner, argument);
    incrementPresenterCount(index, owner, &PresenterState::vblankCount);
}

void wrapVupdate(uint64_t owner, uint64_t argument) {
    auto index = recordPresenter(owner, &PresenterTiming::vupdate);
    reinterpret_cast<void (*)(uint64_t, uint64_t)>(originalVupdate)(owner, argument);
    incrementPresenterCount(index, owner, &PresenterState::vupdateCount);
}

uint64_t wrapEnqueue(uint64_t presenter, uint64_t mask) {
    auto index = recordPresenter(presenter, &PresenterTiming::enqueue);
    uint64_t result = reinterpret_cast<uint64_t (*)(uint64_t, uint64_t)>(originalEnqueue)(presenter, mask);
    incrementPresenterCount(index, presenter, &PresenterState::enqueueCount);
    return result;
}

IOReturn wrapProcessFlip(uint64_t presenter, uint32_t flags) {
    recordPresenter(presenter, &PresenterTiming::flip, true);
    return reinterpret_cast<IOReturn (*)(uint64_t, uint32_t)>(originalProcessFlip)(presenter, flags);
}

void wrapDeferredComplete(OSObject *owner, IOInterruptEventSource *sender, int count) {
    recordPresenter(reinterpret_cast<uint64_t>(owner), &PresenterTiming::deferredComplete, true);
    reinterpret_cast<IOInterruptEventSource::Action>(originalDeferredComplete)(owner, sender, count);
}

uint32_t wrapEnable(uint64_t presenter) {
    return reinterpret_cast<uint32_t (*)(uint64_t)>(originalEnable)(presenter);
}

unsigned long long ticksToMicroseconds(uint64_t ticks) {
    uint64_t nanoseconds;
    absolutetime_to_nanoseconds(ticks, &nanoseconds);
    return static_cast<unsigned long long>(nanoseconds / 1000);
}

unsigned long long eventAgeMicroseconds(const EventTiming &timing, uint64_t now) {
    return timing.lastEvent ? ticksToMicroseconds(now - timing.lastEvent) : 0;
}

EventTiming takeEventTiming(EventTiming &timing) {
    EventTiming snapshot = timing;
    timing.maxGap = 0;
    timing.count = 0;
    return snapshot;
}

void reportPresenterTiming(void *, wait_result_t) {
    KernelLog::logDiagnosticInfo(KernelLog::Category::Hook, "PresenterTiming", "start",
        "Up to %u samples, fields=count/max_gap_us/age_us flip_hook=%u deferred_hook=%u",
        timingCaptureSeconds, static_cast<uint32_t>(flipTimingInstalled), static_cast<uint32_t>(deferredCompleteTimingInstalled));
    bool wasActive[presenterCount] {};
    auto previous = mach_absolute_time();
    for (uint32_t sample = 0; sample < timingCaptureSeconds; ++sample) {
        IOSleep(1000);
        PresenterTiming snapshots[presenterCount] {};
        auto interruptState = IOSimpleLockLockDisableInterrupt(presenterLock);
        auto now = mach_absolute_time();
        bool running = presenterTracking;
        for (uint32_t i = 0; i < presenterCount; ++i) {
            auto &timing = presenterTimings[i];
            snapshots[i] = {takeEventTiming(timing.vblank), takeEventTiming(timing.vupdate), takeEventTiming(timing.enqueue),
                            takeEventTiming(timing.flip), takeEventTiming(timing.deferredComplete)};
        }
        IOSimpleLockUnlockEnableInterrupt(presenterLock, interruptState);
        if (!running) break;
        if ((sample + 1) <= 120 && (sample + 1) % 10 == 0) {
            auto name = __atomic_load_n(&firstRejectedClass, __ATOMIC_RELAXED);
            KernelLog::logDiagnosticInfo(KernelLog::Category::Hook, "PresenterTiming", "heartbeat",
                "seconds=%u cumulative vblank=%u vupdate=%u enqueue=%u flip=%u deferred=%u reject_class=%u first_class=%s reject_index=%u first_index=%d reject_unregistered=%u",
                sample + 1, __atomic_load_n(&rawEvents[0], __ATOMIC_RELAXED),
                __atomic_load_n(&rawEvents[1], __ATOMIC_RELAXED), __atomic_load_n(&rawEvents[2], __ATOMIC_RELAXED),
                __atomic_load_n(&rawEvents[3], __ATOMIC_RELAXED), __atomic_load_n(&rawEvents[4], __ATOMIC_RELAXED),
                __atomic_load_n(&rejectedClass, __ATOMIC_RELAXED), name ? name : "(none)",
                __atomic_load_n(&rejectedIndex, __ATOMIC_RELAXED), __atomic_load_n(&firstRejectedIndex, __ATOMIC_RELAXED),
                __atomic_load_n(&rejectedUnregistered, __ATOMIC_RELAXED));
            if (sample + 1 == 120) __atomic_store_n(&presenterEntryDiagnostics, false, __ATOMIC_RELAXED);
        }
        for (uint32_t i = 0; i < presenterCount; ++i) {
            const auto &timing = snapshots[i];
            bool active = timing.vblank.count || timing.vupdate.count || timing.enqueue.count ||
                timing.flip.count || timing.deferredComplete.count;
            if (active || wasActive[i]) {
                KernelLog::logDiagnosticInfo(KernelLog::Category::Hook, "PresenterTiming", "sample",
                    "pipe=%u uptime_us=%llu window_us=%llu vblank=%u/%llu/%llu vupdate=%u/%llu/%llu "
                    "enqueue=%u/%llu/%llu flip=%u/%llu/%llu deferred=%u/%llu/%llu",
                    i, ticksToMicroseconds(now), ticksToMicroseconds(now - previous),
                    timing.vblank.count, ticksToMicroseconds(timing.vblank.maxGap), eventAgeMicroseconds(timing.vblank, now),
                    timing.vupdate.count, ticksToMicroseconds(timing.vupdate.maxGap), eventAgeMicroseconds(timing.vupdate, now),
                    timing.enqueue.count, ticksToMicroseconds(timing.enqueue.maxGap), eventAgeMicroseconds(timing.enqueue, now),
                    timing.flip.count, ticksToMicroseconds(timing.flip.maxGap), eventAgeMicroseconds(timing.flip, now),
                    timing.deferredComplete.count, ticksToMicroseconds(timing.deferredComplete.maxGap),
                    eventAgeMicroseconds(timing.deferredComplete, now));
            }
            wasActive[i] = active;
        }
        previous = now;
    }
    auto interruptState = IOSimpleLockLockDisableInterrupt(presenterLock);
    presenterTimingEnabled = false;
    IOSimpleLockUnlockEnableInterrupt(presenterLock, interruptState);
    __atomic_store_n(&presenterEntryDiagnostics, false, __ATOMIC_RELAXED);
    KernelLog::logDiagnosticInfo(KernelLog::Category::Hook, "PresenterTiming", "stop", "Capture ended");
}

void stopPresenterTracking() {
    OSObject *objects[presenterCount] {};
    auto interruptState = IOSimpleLockLockDisableInterrupt(presenterLock);
    presenterTracking = false;
    presenterTimingEnabled = false;
    for (uint32_t i = 0; i < presenterCount; ++i) {
        objects[i] = reinterpret_cast<OSObject *>(presenters[i].presenter);
        presenters[i].presenter = 0;
    }
    IOSimpleLockUnlockEnableInterrupt(presenterLock, interruptState);
    for (auto object : objects) if (object) object->release();
}

void watchPresenters(void *, wait_result_t) {
    uint32_t quiet[presenterCount] {};
    uint64_t observed[presenterCount] {};
    PresenterState previous[presenterCount] {};
    uint32_t progress = 0;
    while (progress < 3000) {
        IOSleep(5000);
        ++progress;
        for (uint32_t i = 0; i < presenterCount; ++i) {
            auto interruptState = IOSimpleLockLockDisableInterrupt(presenterLock);
            uint64_t presenter = presenters[i].presenter;
            bool enabled = presenterEnabled[i];
            if (presenter) reinterpret_cast<OSObject *>(presenter)->retain();
            IOSimpleLockUnlockEnableInterrupt(presenterLock, interruptState);
            if (!presenter) continue;
            if (observed[i] != presenter) {
                observed[i] = presenter;
                quiet[i] = 0;
            }
            if (enabled || !originalEnable) {
                reinterpret_cast<OSObject *>(presenter)->release();
                continue;
            }
            if (!readPresenterField(presenter, 0xd8) || readPresenterField(presenter, 0x158)) {
                quiet[i] = 0;
                reinterpret_cast<OSObject *>(presenter)->release();
                continue;
            }
            if (++quiet[i] < 3) {
                reinterpret_cast<OSObject *>(presenter)->release();
                continue;
            }
            interruptState = IOSimpleLockLockDisableInterrupt(presenterLock);
            bool current = presenters[i].presenter == presenter && !presenterEnabled[i];
            if (current) presenterEnabled[i] = true;
            IOSimpleLockUnlockEnableInterrupt(presenterLock, interruptState);
            if (!current) {
                reinterpret_cast<OSObject *>(presenter)->release();
                continue;
            }
            uint64_t framebuffer = readPresenterField(presenter, 0x1650);
            uint32_t result = reinterpret_cast<uint32_t (*)(uint64_t)>(originalEnable)(presenter);
            if (!result && framebuffer && setPipeInterruptState)
                reinterpret_cast<uint32_t (*)(uint64_t, bool)>(setPipeInterruptState)(framebuffer, true);
            reinterpret_cast<OSObject *>(presenter)->release();
        }
        for (uint32_t i = 0; i < presenterCount; ++i) {
            auto interruptState = IOSimpleLockLockDisableInterrupt(presenterLock);
            PresenterState state = presenters[i];
            IOSimpleLockUnlockEnableInterrupt(presenterLock, interruptState);
            if (!state.presenter || (state.vblankCount == previous[i].vblankCount &&
                state.vupdateCount == previous[i].vupdateCount && state.enqueueCount == previous[i].enqueueCount)) continue;
            previous[i] = state;
            ++progress;
        }
    }
    stopPresenterTracking();
}

void startPresenterTiming(KernelPatcher &patcher, size_t index, mach_vm_address_t address, size_t size) {
    KernelPatcher::RouteRequest flipRequest {
        "__ZN29AMDRadeonX6000_AmdFbPresenter22processFlipTransactionEj", wrapProcessFlip, originalProcessFlip};
    flipTimingInstalled = patcher.routeMultiple(index, &flipRequest, 1, address, size);
    patcher.clearError();
    KernelPatcher::RouteRequest completeRequest {
        "__ZN29AMDRadeonX6000_AmdFbPresenter27deferredTransactionCompleteEP8OSObjectP22IOInterruptEventSourcei",
        wrapDeferredComplete, originalDeferredComplete};
    deferredCompleteTimingInstalled = patcher.routeMultiple(index, &completeRequest, 1, address, size);
    patcher.clearError();

    thread_t thread;
    if (kernel_thread_start(reportPresenterTiming, nullptr, &thread) == KERN_SUCCESS) {
        thread_deallocate(thread);
        return;
    }
    auto interruptState = IOSimpleLockLockDisableInterrupt(presenterLock);
    presenterTimingEnabled = false;
    IOSimpleLockUnlockEnableInterrupt(presenterLock, interruptState);
    KernelLog::logDiagnosticWarning(KernelLog::Category::Hook, "PresenterTiming", "start", "Unable to start timing report");
}

}

void installDisplayPresenterHooks(KernelPatcher &patcher, size_t index, mach_vm_address_t address, size_t size) {
    if (!presenterLock) presenterLock = IOSimpleLockAlloc();
    if (!presenterLock) {
        KernelLog::logError(KernelLog::Category::Hook, "Presenter", "initialize", "Unable to allocate tracking lock");
        return;
    }
    bool captureTiming = checkKernelArgument("-rmbpresenttiming");
    __atomic_store_n(&presenterEntryDiagnostics, captureTiming, __ATOMIC_RELAXED);
    auto interruptState = IOSimpleLockLockDisableInterrupt(presenterLock);
    presenterTracking = true;
    IOSimpleLockUnlockEnableInterrupt(presenterLock, interruptState);
    presenterMetaClass = reinterpret_cast<const OSMetaClass *>(
        patcher.solveSymbol(index, "__ZN29AMDRadeonX6000_AmdFbPresenter10gMetaClassE", address, size));
    if (!presenterMetaClass) KernelLog::logWarning(KernelLog::Category::Hook, "Presenter", "resolve", "Presenter metaclass unavailable: error=%d", static_cast<int>(patcher.getError()));
    patcher.clearError();
    interruptState = IOSimpleLockLockDisableInterrupt(presenterLock);
    presenterTimingEnabled = captureTiming && presenterMetaClass;
    IOSimpleLockUnlockEnableInterrupt(presenterLock, interruptState);
    setPipeInterruptState = patcher.solveSymbol(index,
        "__ZN35AMDRadeonX6000_AmdRadeonFramebuffer21setPipeInterruptStateEb", address, size);
    if (!setPipeInterruptState) KernelLog::logWarning(KernelLog::Category::Hook, "Presenter", "resolve", "Pipe interrupt function unavailable: error=%d", static_cast<int>(patcher.getError()));
    patcher.clearError();
    KernelPatcher::RouteRequest requests[] {
        {"__ZN29AMDRadeonX6000_AmdFbPresenter24transactionVBlankHandlerEP8OSObjectPv", wrapVblank, originalVblank},
        {"__ZN29AMDRadeonX6000_AmdFbPresenter25transactionVUpdateHandlerEP8OSObjectPv", wrapVupdate, originalVupdate},
        {"__ZN29AMDRadeonX6000_AmdFbPresenter19enqueueTransactionsEy", wrapEnqueue, originalEnqueue},
        {"__ZN29AMDRadeonX6000_AmdFbPresenter18enableTransactionsEv", wrapEnable, originalEnable},
    };
    if (patcher.routeMultiple(index, requests, address, size)) {
        KernelLog::logInfo(KernelLog::Category::Hook, "Presenter", "install", "Presenter hooks installed");
        thread_t thread;
        if (kernel_thread_start(watchPresenters, nullptr, &thread) == KERN_SUCCESS) {
            thread_deallocate(thread);
            if (captureTiming && presenterMetaClass) startPresenterTiming(patcher, index, address, size);
        } else {
            KernelLog::logError(KernelLog::Category::Hook, "Presenter", "start", "Unable to start presenter watch");
            stopPresenterTracking();
        }
    } else {
        KernelLog::logError(KernelLog::Category::Hook, "Presenter", "install", "Presenter hooks failed: error=%d", static_cast<int>(patcher.getError()));
        stopPresenterTracking();
    }
    patcher.clearError();
}
