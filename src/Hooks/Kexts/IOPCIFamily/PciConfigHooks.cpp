#include "Kexts/IOPCIFamily/PciConfigHooks.hpp"
#include "Hardware/Rembrandt/PciIdentity.hpp"
#include "Kernel/GpuInterruptStartup.hpp"
#include "Diagnostics/KernelLog.hpp"
#include <IOKit/pci/IOPCIDevice.h>

namespace {

using Read8Function = uint8_t (*)(void *, uint64_t);
using Read16Function = uint16_t (*)(void *, uint64_t);
using Read32Function = uint32_t (*)(void *, uint64_t);

mach_vm_address_t originalRead8;
mach_vm_address_t originalRead16;
mach_vm_address_t originalRead32;
mach_vm_address_t originalGetResources;
bool pciConfigHooksInstalled;
bool deviceIdLogged;
bool revisionLogged;

IOReturn wrapGetResources(IOPCIDevice *device) {
    auto result = reinterpret_cast<IOReturn (*)(IOPCIDevice *)>(originalGetResources)(device);
    if (result == kIOReturnSuccess) prepareRembrandtGpuResources(device);
    return result;
}

uint32_t adjustPciRevision(void *device, uint64_t offset, uint32_t value) {
    uint32_t revision = value & 0xff;
    if (offset != kIOPCIConfigRevisionID || (revision != 0x02 && revision != 0x06 && revision != 0x0b && revision != 0xc8)) return value;
    auto read16 = reinterpret_cast<Read16Function>(originalRead16);
    if (read16(device, kIOPCIConfigVendorID) != 0x1002 || read16(device, kIOPCIConfigDeviceID) != 0x1681) return value;
    if (!__atomic_exchange_n(&revisionLogged, true, __ATOMIC_RELAXED))
        KernelLog::logInfo(KernelLog::Category::Hook, "IOPCI", "map revision", "Rembrandt PCI revision %02x -> c7", revision);
    return (value & ~0xffu) | 0xc7;
}

uint8_t wrapReadConfig8(void *device, uint64_t offset) {
    uint8_t value = reinterpret_cast<Read8Function>(originalRead8)(device, offset);
    return static_cast<uint8_t>(adjustPciRevision(device, offset, value));
}

uint16_t wrapReadConfig16(void *device, uint64_t offset) {
    auto read16 = reinterpret_cast<Read16Function>(originalRead16);
    uint16_t value = read16(device, offset);
    if (offset == kIOPCIConfigDeviceID && isRembrandtDeviceId(value) && read16(device, kIOPCIConfigVendorID) == 0x1002) {
        if (!__atomic_exchange_n(&deviceIdLogged, true, __ATOMIC_RELAXED))
            KernelLog::logInfo(KernelLog::Category::Hook, "IOPCI", "map device", "Rembrandt PCI device %04x -> 73ff", static_cast<unsigned int>(value));
        value = 0x73ff;
    }
    return static_cast<uint16_t>(adjustPciRevision(device, offset, value));
}

uint32_t wrapReadConfig32(void *device, uint64_t offset) {
    uint32_t value = reinterpret_cast<Read32Function>(originalRead32)(device, offset);
    if (offset == kIOPCIConfigVendorID && (value & 0xffffu) == 0x1002 && isRembrandtDeviceId(value >> 16)) {
        if (!__atomic_exchange_n(&deviceIdLogged, true, __ATOMIC_RELAXED))
            KernelLog::logInfo(KernelLog::Category::Hook, "IOPCI", "map device", "Rembrandt PCI device %04x -> 73ff", value >> 16);
        value = 0x73ff1002;
    }
    return adjustPciRevision(device, offset, value);
}

}

bool arePciConfigHooksInstalled() {
    return __atomic_load_n(&pciConfigHooksInstalled, __ATOMIC_ACQUIRE);
}

bool installPciConfigHooks(KernelPatcher &patcher, size_t index, mach_vm_address_t address, size_t size) {
    KernelPatcher::RouteRequest requests[] {
        {"__ZN11IOPCIDevice20extendedConfigRead16Ey", wrapReadConfig16, originalRead16},
        {"__ZN11IOPCIDevice20extendedConfigRead32Ey", wrapReadConfig32, originalRead32},
        {"__ZN11IOPCIDevice19extendedConfigRead8Ey", wrapReadConfig8, originalRead8},
    };
    bool installed = patcher.routeMultiple(index, requests, address, size);
    if (installed) {
        __atomic_store_n(&pciConfigHooksInstalled, true, __ATOMIC_RELEASE);
        KernelLog::logInfo(KernelLog::Category::Hook, "IOPCI", "install", "PCI configuration hooks installed");
    }
    else KernelLog::logError(KernelLog::Category::Hook, "IOPCI", "install", "PCI configuration hooks failed: error=%d", static_cast<int>(patcher.getError()));
    patcher.clearError();
    KernelPatcher::RouteRequest resourcesRequest {"__ZN11IOPCIDevice12getResourcesEv", wrapGetResources, originalGetResources};
    if (patcher.routeMultiple(index, &resourcesRequest, 1, address, size))
        KernelLog::logInfo(KernelLog::Category::Hook, "IOPCI", "install", "GPU resource hook installed");
    else KernelLog::logError(KernelLog::Category::Hook, "IOPCI", "install", "GPU resource hook failed: error=%d", static_cast<int>(patcher.getError()));
    patcher.clearError();
    return installed;
}
