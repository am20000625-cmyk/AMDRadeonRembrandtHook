#include "Kexts/AMDRadeonX6000Framebuffer/FramebufferHooks.hpp"
#include "Diagnostics/KernelLog.hpp"
#include "Hardware/Rembrandt/Display/DisplayClock.hpp"
#include <stdarg.h>
#include <libkern/libkern.h>
#include <string.h>
#include <IOKit/IOLib.h>

namespace {

DisplayStream pendingStream;

uint32_t readStreamField(const char *message, const char *key) {
    const char *field = strstr(message, key);
    return field ? static_cast<uint32_t>(strtoul(field + strlen(key), nullptr, 10)) : 0;
}

void wrapDisplayMessage(void *, int, const char *format, ...) {
    if (!format) return;
    bool stream = strstr(format, "pixelencoder") != nullptr;
    bool link = !stream && strstr(format, "link: %d") != nullptr;
    if (!stream && !link) return;
    char message[400];
    va_list arguments;
    va_start(arguments, format);
    vsnprintf(message, sizeof(message), format, arguments);
    va_end(arguments);
    if (stream) {
        pendingStream.pixelClockKhz = readStreamField(message, "pix_clk_khz: ");
        pendingStream.encoding = readStreamField(message, "pixelencoder:");
        pendingStream.depth = readStreamField(message, "displaycolorDepth:");
    } else {
        uint32_t linkIndex = readStreamField(message, "link: ");
        if (linkIndex && pendingStream.pixelClockKhz)
            setExternalDisplayStream(pendingStream.pixelClockKhz, pendingStream.encoding, pendingStream.depth, linkIndex);
    }
}

}

void installDisplayStreamHook(KernelPatcher &patcher, mach_vm_address_t address, size_t size) {
    constexpr uint8_t prologue[] {0x55, 0x48, 0x89, 0xe5, 0x41, 0x57, 0x41, 0x56,
        0x41, 0x55, 0x41, 0x54, 0x53, 0x48, 0x81, 0xec};
    constexpr mach_vm_address_t offsets[] {0x1697b3, 0x169b83};
    for (auto offset : offsets) {
        if (size < sizeof(prologue) || offset > size - sizeof(prologue)) continue;
        mach_vm_address_t function = address + offset;
        if (memcmp(reinterpret_cast<const void *>(function), prologue, sizeof(prologue))) continue;
        patcher.routeFunction(function, reinterpret_cast<mach_vm_address_t>(wrapDisplayMessage), false);
        if (patcher.getError() != KernelPatcher::Error::NoError)
            KernelLog::logError(KernelLog::Category::Hook, "DisplayStream", "install", "Hook failed: error=%d", static_cast<int>(patcher.getError()));
        else
            KernelLog::logInfo(KernelLog::Category::Hook, "DisplayStream", "install",
                "Display stream hook installed: offset=0x%llx address=0x%llx",
                static_cast<unsigned long long>(offset), static_cast<unsigned long long>(function));
        patcher.clearError();
        return;
    }
    KernelLog::logWarning(KernelLog::Category::Hook, "DisplayStream", "install", "Expected function prologue not found");
}
