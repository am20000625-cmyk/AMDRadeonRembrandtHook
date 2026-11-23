#include "Kexts/AMDRadeonX6000Framebuffer/FramebufferHooks.hpp"
#include "Diagnostics/KernelLog.hpp"
#include "Kexts/AMDRadeonX6000Framebuffer/DcnRegisterMap.hpp"
#include "Hardware/Rembrandt/Display/DalDmubCommands.hpp"
#include "Hardware/Rembrandt/Display/DalRegisterCompatibility.hpp"
#include "Hardware/Rembrandt/Display/DpAux.hpp"
#include "Hardware/Rembrandt/Display/DmubInbox.hpp"
#include "Hardware/Rembrandt/Display/UsbCDpLink.hpp"
#include <kern/thread.h>
#include <mach/mach_time.h>
#include "Hardware/Rembrandt/Display/DisplayClock.hpp"

extern uint32_t mmioRead(uint32_t reg);
extern void mmioWrite(uint32_t reg, uint32_t value);

namespace {

using RegisterCallback = uint64_t (*)(uint64_t, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t);
using RawReadCallback = uint64_t (*)(uint64_t, uint64_t);
using RawWriteCallback = uint64_t (*)(uint64_t, uint64_t, uint64_t);

constexpr uint32_t segmentDword = 0x34c0;
constexpr uint32_t inboxWritePointer = segmentDword + 0x1d6;
constexpr uint32_t inboxReadPointer = segmentDword + 0x1d7;
constexpr uint32_t disabledPipes = segmentDword + 0xca;
constexpr mach_vm_address_t framebufferTextBase = 0xbfe6000;
constexpr uint32_t bootWatermarksSize = 0x36e;

mach_vm_address_t originalReadRegister;
mach_vm_address_t originalWriteRegister;
mach_vm_address_t originalExecuteTable;
mach_vm_address_t originalHardwareRead;
mach_vm_address_t originalValidatedRead;
mach_vm_address_t originalRawWrite;
mach_vm_address_t framebufferAddress;
uint32_t bootWatermarksFirst;
IOSimpleLock *dalAccessLock;

struct DalAccess {
    thread_t thread;
    DalAccess *next;
};

DalAccess *activeDalAccess;

class DalAccessGuard {
public:
    DalAccessGuard() : access {current_thread(), nullptr} {
        auto interruptState = IOSimpleLockLockDisableInterrupt(dalAccessLock);
        access.next = activeDalAccess;
        activeDalAccess = &access;
        IOSimpleLockUnlockEnableInterrupt(dalAccessLock, interruptState);
    }

    ~DalAccessGuard() {
        auto interruptState = IOSimpleLockLockDisableInterrupt(dalAccessLock);
        DalAccess **entry = &activeDalAccess;
        while (*entry && *entry != &access) entry = &(*entry)->next;
        if (*entry) *entry = access.next;
        IOSimpleLockUnlockEnableInterrupt(dalAccessLock, interruptState);
    }

private:
    DalAccess access;
};

bool isCurrentThreadInDalAccess() {
    thread_t thread = current_thread();
    auto interruptState = IOSimpleLockLockDisableInterrupt(dalAccessLock);
    bool active = false;
    for (auto entry = activeDalAccess; entry; entry = entry->next) {
        if (entry->thread == thread) {
            active = true;
            break;
        }
    }
    IOSimpleLockUnlockEnableInterrupt(dalAccessLock, interruptState);
    return active;
}

bool isDisplayRegister(uint32_t reg) {
    return (reg >= 0xc0 && reg < 0x1260) || (reg >= segmentDword && reg < 0x8000);
}

bool isBootWatermarksCaller(void *address) {
    uint32_t caller = static_cast<uint32_t>(reinterpret_cast<mach_vm_address_t>(address) - framebufferAddress + framebufferTextBase);
    return bootWatermarksFirst && caller >= bootWatermarksFirst && caller <= bootWatermarksFirst + bootWatermarksSize;
}

bool hasTranslatedFields(uint32_t reg) {
    return (reg >= 0x4f8b && reg <= 0x4fbb && (reg & 0xf) == 0xb) ||
        (reg >= 0x3b2c && reg <= 0x3b2c + 3 * 0xdc && (reg - 0x3b2c) % 0xdc == 0);
}

uint64_t wrapReadRegister(uint64_t helper, uint64_t reg, uint64_t c, uint64_t d, uint64_t e, uint64_t f) {
    uint32_t requested = static_cast<uint32_t>(reg);
    if (isDalPowerGatingRegister(requested)) return readDalPowerGatingRegister(requested);
    if ((requested == inboxWritePointer || requested == inboxReadPointer) && isDmubInboxHeld()) {
        uint32_t hardwareWrite = mmioRead(inboxWritePointer);
        if (requested == inboxWritePointer || mmioRead(inboxReadPointer) == hardwareWrite) return gDalWritePointer;
    }
    uint32_t hardware = translateDcnRegister(requested);
    if (hardware == 0x16274) return 0xff;
    recordDalAuxAccess(hardware);
    if (isUnavailableDalPhyRegister(hardware)) return 0;
    uint64_t value;
    {
        DalAccessGuard guard;
        value = reinterpret_cast<RegisterCallback>(originalReadRegister)(helper, hardware, c, d, e, f);
    }
    if (hasTranslatedFields(hardware))
        value = translateDalRegisterRead(hardware, static_cast<uint32_t>(value));
    value = (value & ~0xffffffffull) | adjustUsbCDpRead(hardware, static_cast<uint32_t>(value));
    if (requested == disabledPipes) value |= 0x10;
    return value;
}

void reportHdmiClock(const HdmiClockDiagnostic &diagnostic) {
    if (!diagnostic.commandCount) return;
    static uint64_t previous;
    uint64_t now;
    absolutetime_to_nanoseconds(mach_absolute_time(), &now);
    auto last = __atomic_load_n(&previous, __ATOMIC_RELAXED);
    if (last && (now <= last || now - last < 1000000000ULL)) return;
    if (!__atomic_compare_exchange_n(&previous, &last, now, false, __ATOMIC_RELAXED, __ATOMIC_RELAXED)) return;
    const auto &stream = diagnostic.stream;
    KernelLog::logDiagnosticInfo(KernelLog::Category::Hardware, "HDMI", "DAL command",
        "0x0280 count=%u crtc=%d clock_100hz=%u external_clock_khz=%u external_link=%u encoding=%u depth=%u otg_locked=%d",
        diagnostic.commandCount, diagnostic.crtc, diagnostic.clock100Hz, stream.pixelClockKhz, stream.link,
        stream.encoding, stream.depth, diagnostic.otgLocked);
}

uint64_t wrapWriteRegister(uint64_t helper, uint64_t reg, uint64_t value, uint64_t d, uint64_t e, uint64_t f) {
    uint32_t requested = static_cast<uint32_t>(reg);
    if (isDalPowerGatingRegister(requested)) {
        writeDalPowerGatingRegister(requested, static_cast<uint32_t>(value));
        return 0;
    }
    bool locked = false;
    HdmiClockDiagnostic diagnostic {0, 0, -1, -1, {}};
    auto finish = [&] {
        if (locked) IOLockUnlock(gDmubInboxLock);
        reportHdmiClock(diagnostic);
    };
    if (requested == inboxWritePointer) {
        if (gDmubInboxLock) {
            IOLockLock(gDmubInboxLock);
            locked = true;
        }
        uint32_t next = static_cast<uint32_t>(value);
        uint32_t real = filterDalDmubCommands(gDmubAhead ? gDalWritePointer : mmioRead(inboxWritePointer), next, diagnostic);
        if (gDmubAhead) {
            uint32_t hardwareWrite = realignDmubInbox(next);
            gDalWritePointer = next;
            if (hardwareWrite != ~0u) mmioWrite(inboxWritePointer, hardwareWrite);
            finish();
            return 0;
        }
        gDalWritePointer = next;
        if (!real) {
            finish();
            return 0;
        }
        prepareUsbCDpCommands(next);
    }
    uint32_t hardware = translateDcnRegister(requested);
    recordDalAuxAccess(hardware);
    if (isDalPhyRegister(hardware)) {
        finish();
        return 0;
    }
    if (hasTranslatedFields(hardware))
        value = translateDalRegisterWrite(hardware, static_cast<uint32_t>(value), mmioRead(hardware));
    value = (value & ~0xffffffffull) | prepareDalRegisterWrite(hardware, static_cast<uint32_t>(value));
    value = adjustUsbCDpWrite(hardware, value);
    prepareUsbCDpWrite(hardware, static_cast<uint32_t>(value));
    uint64_t result;
    {
        DalAccessGuard guard;
        result = reinterpret_cast<RegisterCallback>(originalWriteRegister)(helper, hardware, value, d, e, f);
    }
    finish();
    return result;
}

template <mach_vm_address_t &original>
uint64_t wrapRawRead(uint64_t object, uint64_t reg) {
    uint32_t hardware = static_cast<uint32_t>(reg);
    if (!isCurrentThreadInDalAccess() && isDisplayRegister(hardware)) {
        if (isBootWatermarksCaller(__builtin_return_address(0))) reg = hardware = translateDcnRegister(hardware);
        if (isUnavailableDalPhyRegister(hardware)) return 0;
    }
    return reinterpret_cast<RawReadCallback>(original)(object, reg);
}

uint64_t wrapRawWrite(uint64_t object, uint64_t reg, uint64_t value) {
    uint32_t hardware = static_cast<uint32_t>(reg);
    if (!isCurrentThreadInDalAccess() && isDisplayRegister(hardware)) {
        if (isBootWatermarksCaller(__builtin_return_address(0))) reg = hardware = translateDcnRegister(hardware);
        if (isDalPhyRegister(hardware)) return 0;
    }
    return reinterpret_cast<RawWriteCallback>(originalRawWrite)(object, reg, value);
}

uint64_t wrapExecuteTable(uint64_t helper, uint64_t index, uint64_t parameters, uint64_t d, uint64_t e, uint64_t f) {
    auto bytes = reinterpret_cast<const uint8_t *>(parameters);
    if (static_cast<uint32_t>(index) == 77 && bytes && bytes[1] == 7) return 1;
    return reinterpret_cast<RegisterCallback>(originalExecuteTable)(helper, index, parameters, d, e, f);
}

}

bool initializeDalRegisterHooks() {
    if (!initializeDmubInbox()) {
        KernelLog::logError(KernelLog::Category::Hook, "DAL", "initialize", "DMUB inbox initialization failed");
        return false;
    }
    if (!dalAccessLock) dalAccessLock = IOSimpleLockAlloc();
    if (!dalAccessLock) {
        KernelLog::logError(KernelLog::Category::Hook, "DAL", "initialize", "Unable to allocate access lock");
        return false;
    }
    if (!startPanelLinkWatch()) KernelLog::logWarning(KernelLog::Category::Hook, "DAL", "initialize", "Panel link watch could not start");
    KernelLog::logInfo(KernelLog::Category::Hook, "DAL", "initialize", "Register access initialized");
    return true;
}

void installDalRegisterHooks(KernelPatcher &patcher, size_t index, mach_vm_address_t address, size_t size) {
    if (!dalAccessLock) return;
    framebufferAddress = address;
    auto watermarks = patcher.solveSymbol(index, "__ZN39AMDRadeonX6000_AmdRadeonControllerNavi219setupBootWatermarksEv", address, size);
    if (!watermarks) KernelLog::logWarning(KernelLog::Category::Hook, "DAL", "resolve", "Boot watermark function unavailable: error=%d", static_cast<int>(patcher.getError()));
    if (watermarks) bootWatermarksFirst = static_cast<uint32_t>(watermarks - address + framebufferTextBase);
    patcher.clearError();
    KernelPatcher::RouteRequest requests[] {
        {"__ZN27AMDRadeonX6000_AmdDalHelper21dalAtomReadRegister32EPvj", wrapReadRegister, originalReadRegister},
        {"__ZN27AMDRadeonX6000_AmdDalHelper22dalAtomWriteRegister32EPvjj", wrapWriteRegister, originalWriteRegister},
        {"__ZN27AMDRadeonX6000_AmdDalHelper19dalAtomExecCmdTableEPvjS0_", wrapExecuteTable, originalExecuteTable},
    };
    if (patcher.routeMultiple(index, requests, address, size)) KernelLog::logInfo(KernelLog::Category::Hook, "DAL", "install", "DAL register hooks installed");
    else KernelLog::logError(KernelLog::Category::Hook, "DAL", "install", "DAL register hooks failed: error=%d", static_cast<int>(patcher.getError()));
    patcher.clearError();
}

void installRawRegisterHooks(KernelPatcher &patcher, size_t index, mach_vm_address_t address, size_t size) {
    if (!dalAccessLock) return;
    KernelPatcher::RouteRequest requests[] {
        {"__ZN32AMDRadeonX6000_AmdRegisterAccess11hwReadReg32Ej", wrapRawRead<originalHardwareRead>, originalHardwareRead},
        {"__ZN32AMDRadeonX6000_AmdRegisterAccess17readValidateReg32Ej", wrapRawRead<originalValidatedRead>, originalValidatedRead},
        {"__ZN32AMDRadeonX6000_AmdRegisterAccess10writeReg32Ejj", wrapRawWrite, originalRawWrite},
    };
    if (patcher.routeMultiple(index, requests, address, size)) KernelLog::logInfo(KernelLog::Category::Hook, "DAL", "install raw", "Raw register hooks installed");
    else KernelLog::logError(KernelLog::Category::Hook, "DAL", "install raw", "Raw register hooks failed: error=%d", static_cast<int>(patcher.getError()));
    patcher.clearError();
}
