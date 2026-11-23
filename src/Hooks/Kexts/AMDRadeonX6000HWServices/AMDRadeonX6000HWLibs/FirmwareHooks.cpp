#include "Kexts/AMDRadeonX6000HWServices/AMDRadeonX6000HWLibs/HWLibsHooks.hpp"
#include "Kexts/AMDRadeonX6000Framebuffer/FramebufferHooks.hpp"
#include "Hardware/Rembrandt/Display/DalDmubCommands.hpp"
#include "Hardware/Rembrandt/Display/DpAux.hpp"
#include "Hardware/Rembrandt/Firmware/FirmwareImages.hpp"
#include "Hardware/Rembrandt/Smu13Mailbox.hpp"
#include "Diagnostics/KernelLog.hpp"
#include <IOKit/IOMessage.h>
#include <IOKit/IOService.h>
#include <IOKit/pwr_mgt/IOPM.h>
#include <IOKit/pwr_mgt/RootDomain.h>

extern void dmubResetForLoad();
extern void gartSave();
extern void quiesceIh(const char *reason);
extern volatile bool gKiqSleepSkip;

namespace {

using HookFunction = uint64_t (*)(uint64_t, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t);
struct FirmwareSubstitution {
    const char *symbol;
    const uint8_t *data;
    const uint8_t *end;
};

mach_vm_address_t originalInitializePspTmr;
mach_vm_address_t originalCheckFirmwareCapability;
mach_vm_address_t originalGetDcnFirmwareConstraint;
mach_vm_address_t originalPspPowerDown;
mach_vm_address_t unloadPspTmr;
mach_vm_address_t stopPspRing;
mach_vm_address_t loadDmubFirmware;
mach_vm_address_t teardownFunctions[6];
uint64_t activePsp;
IONotifier *powerNotifier;
bool firmwareHooksReady;

const char *const teardownSymbols[] {
    "_psp_hdcp_unload", "_psp_hc_unload", "_psp_ras_unload", "_psp_dtm_unload",
    "_psp_asd_unload", "_psp_tmr_unload"
};

bool isCpuCapabilityChange(UInt32 type, void *argument, bool gain) {
    auto change = static_cast<const IOPMSystemCapabilityChangeParameters *>(argument);
    if (type != kIOMessageSystemCapabilityChange || !change) return false;
    if (!(change->changeFlags & (gain ? kIOPMSystemCapabilityDidChange : kIOPMSystemCapabilityWillChange))) return false;
    bool from = change->fromCapabilities & kIOPMSystemCapabilityCPU;
    bool to = change->toCapabilities & kIOPMSystemCapabilityCPU;
    return gain ? !from && to : from && !to;
}

void teardownPsp(uint64_t psp) {
    for (auto function : teardownFunctions)
        if (function) reinterpret_cast<uint64_t (*)(uint64_t)>(function)(psp);
    for (uint64_t ring = 1; ring <= 2; ++ring)
        reinterpret_cast<uint64_t (*)(uint64_t, uint64_t)>(stopPspRing)(psp, ring);
}

IOReturn handlePowerEvent(void *, void *, UInt32 type, IOService *, void *argument, vm_size_t) {
    if (type == kIOMessageSystemWillPowerOff || type == kIOMessageSystemWillRestart) {
        KernelLog::logInfo(KernelLog::Category::Hook, "Power", "shutdown", "Power event=0x%x", static_cast<unsigned int>(type));
        setPanelPowerOffAllowed(true);
        powerOffPanel();
    } else if (type == kIOMessageSystemWillSleep || isCpuCapabilityChange(type, argument, false)) {
        KernelLog::logInfo(KernelLog::Category::Hook, "Power", "sleep", "Preparing display and PSP for sleep");
        gKiqSleepSkip = true;
        setPanelPowerOffAllowed(true);
        armPanelWakeRecovery();
        armHwLibsWakePost();
        setPanelLinkWatchPaused(true);
    } else if (type == kIOMessageSystemHasPoweredOn) {
        KernelLog::logInfo(KernelLog::Category::Hook, "Power", "wake", "Starting display recovery");
        gKiqSleepSkip = false;
        setPanelPowerOffAllowed(false);
        startPanelWakeRecovery();
        setPanelLinkWatchPaused(false);
        requestPanelLinkRetraining();
    }
    if (type == kIOMessageSystemWillPowerOff && activePsp) activePsp = 0;
    if ((type == kIOMessageSystemWillPowerOff || type == kIOMessageSystemWillRestart) && activePsp && stopPspRing) {
        uint64_t psp = activePsp;
        activePsp = 0;
        haltGraphicsEngines();
        teardownPsp(psp);
        resetGraphicsEngines();
        quiesceIh("shutdown");
    }
    return kIOReturnSuccess;
}

uint64_t wrapInitializePspTmr(uint64_t psp, uint64_t b, uint64_t c, uint64_t d, uint64_t e, uint64_t f) {
    if (!__atomic_load_n(&firmwareHooksReady, __ATOMIC_ACQUIRE))
        return reinterpret_cast<HookFunction>(originalInitializePspTmr)(psp, b, c, d, e, f);
    auto object = reinterpret_cast<uint8_t *>(psp);
    *reinterpret_cast<const uint8_t **>(object + 0x3588) = fw_yc_toc;
    *reinterpret_cast<uint32_t *>(object + 0x3580) = static_cast<uint32_t>(fw_yc_toc_end - fw_yc_toc);
    uint64_t result = reinterpret_cast<HookFunction>(originalInitializePspTmr)(psp, b, c, d, e, f);
    KernelLog::logInfo(KernelLog::Category::Hook, "PSP", "TMR", "initial result=0x%x toc bytes=%u TMR bytes=0x%llx", static_cast<uint32_t>(result), *reinterpret_cast<uint32_t *>(object + 0x3580), *reinterpret_cast<uint64_t *>(object + 0xaf8));
    if (static_cast<uint32_t>(result) && unloadPspTmr) {
        KernelLog::logWarning(KernelLog::Category::Hook, "Firmware", "initialize PSP", "TMR initialization failed: status=0x%x; retrying", static_cast<uint32_t>(result));
        reinterpret_cast<uint64_t (*)(uint64_t)>(unloadPspTmr)(psp);
        result = reinterpret_cast<HookFunction>(originalInitializePspTmr)(psp, b, c, d, e, f);
    }
    if (!static_cast<uint32_t>(result)) {
        KernelLog::logInfo(KernelLog::Category::Hook, "Firmware", "initialize PSP", "TMR initialized with Rembrandt firmware");
        activePsp = psp;
        if (!powerNotifier) {
            powerNotifier = registerPrioritySleepWakeInterest(handlePowerEvent, nullptr, nullptr);
            if (!powerNotifier) KernelLog::logError(KernelLog::Category::Hook, "Firmware", "register power", "Power notification registration failed");
        }
    }
    else KernelLog::logError(KernelLog::Category::Hook, "Firmware", "initialize PSP", "TMR initialization failed: status=0x%x", static_cast<uint32_t>(result));
    return result;
}

uint64_t wrapCheckFirmwareCapability(uint64_t psp, uint64_t type, uint64_t c, uint64_t d, uint64_t e, uint64_t f) {
    if (!__atomic_load_n(&firmwareHooksReady, __ATOMIC_ACQUIRE))
        return reinterpret_cast<HookFunction>(originalCheckFirmwareCapability)(psp, type, c, d, e, f);
    uint32_t firmwareType = static_cast<uint32_t>(type);
    uint64_t result = firmwareType == 30 || firmwareType == 31 || firmwareType == 32 ? 0 :
        reinterpret_cast<HookFunction>(originalCheckFirmwareCapability)(psp, type, c, d, e, f) & 0xff;
    KernelLog::logDiagnosticInfo(KernelLog::Category::Hook, "PSP", "firmware capability", "type=%u load=%u", firmwareType, static_cast<unsigned int>(result));
    if (result && firmwareType == 35) dmubResetForLoad();
    return result;
}

uint64_t wrapGetDcnFirmwareConstraint(uint64_t version, uint64_t dmcub, uint64_t list, uint64_t d, uint64_t e, uint64_t f) {
    if (!__atomic_load_n(&firmwareHooksReady, __ATOMIC_ACQUIRE))
        return reinterpret_cast<HookFunction>(originalGetDcnFirmwareConstraint)(version, dmcub, list, d, e, f);
    uint64_t result = reinterpret_cast<HookFunction>(originalGetDcnFirmwareConstraint)(version, dmcub, list, d, e, f);
    if (loadDmubFirmware) {
        auto loadResult = reinterpret_cast<HookFunction>(loadDmubFirmware)(dmcub, reinterpret_cast<uint64_t>("atidmcub_0.dat"),
            reinterpret_cast<uint64_t>(fw_yc_dmcub), static_cast<uint32_t>(fw_yc_dmcub_end - fw_yc_dmcub), list, 0);
        static uint32_t reports;
        if (__atomic_load_n(&reports, __ATOMIC_RELAXED) < 8 && __atomic_fetch_add(&reports, 1, __ATOMIC_RELAXED) < 8)
            KernelLog::logDiagnosticInfo(KernelLog::Category::Hook, "DMUB", "firmware load",
                "_dmcub_load_fw success=%u raw_eax=0x%08x",
                static_cast<unsigned>(static_cast<uint8_t>(loadResult) != 0), static_cast<unsigned>(loadResult));
    }
    return result;
}

uint64_t wrapPspPowerDown(uint64_t psp, uint64_t flags, uint64_t c, uint64_t d, uint64_t e, uint64_t f) {
    if (!__atomic_load_n(&firmwareHooksReady, __ATOMIC_ACQUIRE))
        return reinterpret_cast<HookFunction>(originalPspPowerDown)(psp, flags, c, d, e, f);
    bool light = !(*reinterpret_cast<uint64_t *>(psp + 0x28) & 0x20) && (*reinterpret_cast<uint8_t *>(flags) & 1);
    uint64_t result = reinterpret_cast<HookFunction>(originalPspPowerDown)(psp, flags, c, d, e, f);
    markPanelPowerLost();
    if (light && stopPspRing) {
        teardownPsp(psp);
        gartSave();
        haltGraphicsEngines();
        resetGraphicsEngines();
    } else {
        gartSave();
    }
    return result;
}

}

void installFirmwareHooks(KernelPatcher &patcher, size_t index, mach_vm_address_t address, size_t size) {
    const FirmwareSubstitution substitutions[] {
        {"_gc_10_3_4_me_ucode", fw_yc_me, fw_yc_me_end},
        {"_gc_10_3_4_pfp_ucode", fw_yc_pfp, fw_yc_pfp_end},
        {"_gc_10_3_4_ce_ucode", fw_yc_ce, fw_yc_ce_end},
        {"_gc_10_3_4_mec_ucode", fw_yc_mec, fw_yc_mec_end},
        {"_gc_10_3_4_mec_jt_ucode", fw_yc_mec_jt, fw_yc_mec_jt_end},
        {"_gc_10_3_4_rlc_ucode", fw_yc_rlc, fw_yc_rlc_end},
        {"_gc_10_3_4_rlc_srlist_cntl", fw_yc_srlist_cntl, fw_yc_srlist_cntl_end},
        {"_gc_10_3_4_rlc_srlist_gpm_mem", fw_yc_srlist_gpm, fw_yc_srlist_gpm_end},
        {"_gc_10_3_4_rlc_srlist_srm_mem", fw_yc_srlist_srm, fw_yc_srlist_srm_end},
        {"_gc_10_3_4_rlc_lx6_iram_ucode", fw_yc_rlc_iram, fw_yc_rlc_iram_end},
        {"_gc_10_3_4_rlc_lx6_dram_ucode", fw_yc_rlc_dram, fw_yc_rlc_dram_end},
    };
    KernelPatcher::RouteRequest requests[] {
        {"_psp_tmr_init", wrapInitializePspTmr, originalInitializePspTmr},
        {"_psp_np_fw_load_capability_check", wrapCheckFirmwareCapability, originalCheckFirmwareCapability},
        {"_dmcub_dcn31_get_fw_constrant", wrapGetDcnFirmwareConstraint, originalGetDcnFirmwareConstraint},
        {"_psp_power_down_event", wrapPspPowerDown, originalPspPowerDown},
    };
    loadDmubFirmware = patcher.solveSymbol(index, "_dmcub_load_fw", address, size);
    if (!loadDmubFirmware) KernelLog::logError(KernelLog::Category::Hook, "Firmware", "resolve", "_dmcub_load_fw unavailable: error=%d", static_cast<int>(patcher.getError()));
    patcher.clearError();
    unloadPspTmr = patcher.solveSymbol(index, "_psp_tmr_unload", address, size);
    if (!unloadPspTmr) KernelLog::logWarning(KernelLog::Category::Hook, "Firmware", "resolve", "_psp_tmr_unload unavailable: error=%d", static_cast<int>(patcher.getError()));
    patcher.clearError();
    stopPspRing = patcher.solveSymbol(index, "_psp_ring_stop", address, size);
    if (!stopPspRing) KernelLog::logWarning(KernelLog::Category::Hook, "Firmware", "resolve", "_psp_ring_stop unavailable: error=%d", static_cast<int>(patcher.getError()));
    patcher.clearError();
    for (size_t i = 0; i < 6; ++i) {
        teardownFunctions[i] = patcher.solveSymbol(index, teardownSymbols[i], address, size);
        if (!teardownFunctions[i]) KernelLog::logWarning(KernelLog::Category::Hook, "Firmware", "resolve", "%s unavailable: error=%d", teardownSymbols[i], static_cast<int>(patcher.getError()));
        patcher.clearError();
    }
    uint8_t *descriptors[sizeof(substitutions) / sizeof(substitutions[0])] {};
    for (size_t i = 0; i < sizeof(substitutions) / sizeof(substitutions[0]); ++i) {
        descriptors[i] = patcher.solveSymbol<uint8_t *>(index, substitutions[i].symbol, address, size);
        if (!descriptors[i]) KernelLog::logError(KernelLog::Category::Hook, "Firmware", "resolve", "Descriptor %s unavailable: error=%d", substitutions[i].symbol, static_cast<int>(patcher.getError()));
        patcher.clearError();
        if (!descriptors[i]) return;
    }
    if (!loadDmubFirmware) return;
    bool routed = patcher.routeMultiple(index, requests, address, size);
    if (!routed) KernelLog::logError(KernelLog::Category::Hook, "Firmware", "install", "Firmware hooks failed: error=%d", static_cast<int>(patcher.getError()));
    patcher.clearError();
    if (!routed) return;
    for (size_t i = 0; i < sizeof(substitutions) / sizeof(substitutions[0]); ++i) {
        const auto &substitution = substitutions[i];
        auto descriptor = descriptors[i];
        *reinterpret_cast<uint32_t *>(descriptor + 0x0c) = static_cast<uint32_t>(substitution.end - substitution.data);
        *reinterpret_cast<const uint8_t **>(descriptor + 0x20) = substitution.data;
        KernelLog::logInfo(KernelLog::Category::Hook, "Firmware", "substitute", "%s size=%u", substitution.symbol, static_cast<unsigned int>(substitution.end - substitution.data));
    }
    __atomic_store_n(&firmwareHooksReady, true, __ATOMIC_RELEASE);
    KernelLog::logInfo(KernelLog::Category::Hook, "Firmware", "install", "Firmware hooks and descriptors ready");
}
