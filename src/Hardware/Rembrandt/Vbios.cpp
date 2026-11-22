#include "Hardware/Rembrandt/Vbios.hpp"
#include "Diagnostics/KernelLog.hpp"
#include <IOKit/IOLib.h>
#include <IOKit/IODeviceMemory.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOPlatformExpert.h>
#include <IOKit/acpi/IOACPIPlatformExpert.h>
#include <IOKit/pci/IOPCIDevice.h>
#include <libkern/c++/OSData.h>
#include <libkern/libkern.h>

namespace {

uint16_t readUint16(const uint8_t *bytes) {
    return static_cast<uint16_t>(static_cast<uint16_t>(bytes[0]) | static_cast<uint16_t>(bytes[1]) << 8);
}

uint32_t readUint32(const uint8_t *bytes) {
    return static_cast<uint32_t>(readUint16(bytes)) | static_cast<uint32_t>(readUint16(bytes + 2)) << 16;
}

class AcpiPlatformAccess : public IOACPIPlatformExpert {
public:
    const OSData *getVfct() { return getACPITableData("VFCT", 0); }
};

OSData *copyVfctVbios(IOPCIDevice *device) {
    auto platform = IOService::getPlatform();
    if (!platform || !platform->metaCast("IOACPIPlatformExpert")) {
        KernelLog::logError(KernelLog::Category::Hardware, "VBIOS", "read VFCT", "ACPI platform expert unavailable");
        return nullptr;
    }
    auto table = static_cast<AcpiPlatformAccess *>(platform)->getVfct();
    if (!table) {
        KernelLog::logError(KernelLog::Category::Hardware, "VBIOS", "read VFCT", "ACPI VFCT table unavailable");
        return nullptr;
    }
    auto bytes = static_cast<const uint8_t *>(table->getBytesNoCopy());
    uint32_t size = table->getLength();
    if (!bytes || size < 0x4c || size > 512 * 1024 || memcmp(bytes, "VFCT", 4)) {
        KernelLog::logError(KernelLog::Category::Hardware, "VBIOS", "read VFCT", "Invalid VFCT header or buffer length=%u", size);
        return nullptr;
    }
    uint32_t length = readUint32(bytes + 4);
    uint32_t offset = readUint32(bytes + 0x34);
    if (length < 0x4c || length > size || offset < 0x4c || offset >= length) {
        KernelLog::logError(KernelLog::Category::Hardware, "VBIOS", "read VFCT", "Invalid VFCT length=%u image offset=0x%x", length, offset);
        return nullptr;
    }
    uint8_t checksum = 0;
    for (uint32_t i = 0; i < length; ++i) checksum += bytes[i];
    if (checksum) {
        KernelLog::logError(KernelLog::Category::Hardware, "VBIOS", "read VFCT", "Invalid ACPI checksum=0x%02x", static_cast<unsigned int>(checksum));
        return nullptr;
    }
    auto reg = OSDynamicCast(OSData, device->getProperty("reg"));
    auto deviceId = OSDynamicCast(OSData, device->getProperty("device-id"));
    if (!reg || reg->getLength() < 4 || !deviceId || deviceId->getLength() < 4) {
        KernelLog::logError(KernelLog::Category::Hardware, "VBIOS", "read VFCT", "GPU PCI address or device identity unavailable");
        return nullptr;
    }
    uint32_t address = readUint32(static_cast<const uint8_t *>(reg->getBytesNoCopy()));
    uint32_t id = readUint32(static_cast<const uint8_t *>(deviceId->getBytesNoCopy())) & 0xffff;
    uint32_t bus = (address >> 16) & 0xff;
    uint32_t slot = (address >> 11) & 0x1f;
    uint32_t function = (address >> 8) & 7;
    while (offset < length) {
        if (length - offset < 28) {
            KernelLog::logError(KernelLog::Category::Hardware, "VBIOS", "read VFCT", "Truncated VFCT image header at 0x%x", offset);
            return nullptr;
        }
        uint32_t imageSize = readUint32(bytes + offset + 24);
        uint32_t imageOffset = offset + 28;
        if (!imageSize || imageSize > length - imageOffset) {
            KernelLog::logError(KernelLog::Category::Hardware, "VBIOS", "read VFCT", "Invalid VFCT image size=%u at 0x%x", imageSize, offset);
            return nullptr;
        }
        if (readUint32(bytes + offset) == bus && readUint32(bytes + offset + 4) == slot &&
            readUint32(bytes + offset + 8) == function && readUint16(bytes + offset + 12) == 0x1002 &&
            readUint16(bytes + offset + 14) == id) {
            auto image = OSData::withBytes(bytes + imageOffset, imageSize);
            if (!image) KernelLog::logError(KernelLog::Category::Hardware, "VBIOS", "read VFCT", "Unable to copy %u-byte VBIOS", imageSize);
            else KernelLog::logInfo(KernelLog::Category::Hardware, "VBIOS", "read VFCT", "Found GPU %02x:%02x.%u device=%04x image size=%u", bus, slot, function, id, imageSize);
            return image;
        }
        offset = imageOffset + imageSize;
    }
    KernelLog::logError(KernelLog::Category::Hardware, "VBIOS", "read VFCT", "No VBIOS for GPU %02x:%02x.%u device=%04x", bus, slot, function, id);
    return nullptr;
}

void writeUint16(uint8_t *bytes, uint16_t value) {
    bytes[0] = static_cast<uint8_t>(value);
    bytes[1] = static_cast<uint8_t>(value >> 8);
}

void writeUint32(uint8_t *bytes, uint32_t value) {
    writeUint16(bytes, static_cast<uint16_t>(value));
    writeUint16(bytes + 2, static_cast<uint16_t>(value >> 16));
}

uint32_t readVramSize(IOPCIDevice *device) {
    constexpr uint32_t offset = 0xde3 * sizeof(uint32_t);
    auto memory = device->getDeviceMemoryWithRegister(kIOPCIConfigBaseAddress5);
    if (!memory || memory->getLength() < offset + sizeof(uint32_t)) return 0;
    auto map = memory->map(kIOMapInhibitCache);
    if (!map) return 0;
    auto base = reinterpret_cast<volatile uint32_t *>(map->getVirtualAddress());
    uint32_t size = base ? base[offset / sizeof(uint32_t)] : 0;
    map->release();
    return size >= 16 && size <= 65536 ? size : 0;
}

uint32_t readMemoryWidth(const uint8_t *image, uint32_t size, uint32_t master, uint32_t masterSize) {
    if (masterSize < 0x42) return 0;
    uint32_t table = readUint16(image + master + 0x40);
    if (!table || table > size - 28 || readUint16(image + table) < 28 ||
        readUint16(image + table) > size - table || image[table + 2] != 2 || image[table + 3] != 2) return 0;
    uint32_t type = image[table + 26];
    uint32_t channels = image[table + 27];
    if (!channels || channels > 4) return 0;
    if (type == 34) return channels * 64;  // DDR5
    if (type == 35) return channels * 32;  // LPDDR5
    return 0;
}

bool isValidTable(const uint8_t *image, uint32_t size, uint32_t offset) {
    return !offset || (offset <= size - 4 && readUint16(image + offset) >= 4 &&
        readUint16(image + offset) <= size - offset);
}

bool publishVbios(IOPCIDevice *device, OSData *data) {
    auto image = static_cast<const uint8_t *>(data->getBytesNoCopy());
    uint32_t size = data->getLength();
    if (!image || size < 0x4a || size > 0xfe00 || size % 512 || image[0] != 0x55 || image[1] != 0xaa ||
        image[2] * 512U != size) {
        KernelLog::logError(KernelLog::Category::Hardware, "VBIOS", "validate", "Invalid ROM signature or length=%u", size);
        return false;
    }
    uint32_t pci = readUint16(image + 0x18);
    uint32_t header = readUint16(image + 0x48);
    if (pci > size - 0x18 || header > size - 0x22 || memcmp(image + pci, "PCIR", 4) ||
        readUint16(image + pci + 4) != 0x1002 || readUint16(image + pci + 0x10) * 512U != size ||
        !(image[pci + 0x15] & 0x80) || memcmp(image + header + 4, "ATOM", 4) ||
        readUint16(image + header) < 0x22 || readUint16(image + header) > size - header ||
        image[header + 2] != 2 || (image[header + 3] != 2 && image[header + 3] != 3)) {
        KernelLog::logError(KernelLog::Category::Hardware, "VBIOS", "validate", "Invalid PCIR or ATOM header");
        return false;
    }
    uint32_t master = readUint16(image + header + 0x20);
    if (master > size - 0x3e) {
        KernelLog::logError(KernelLog::Category::Hardware, "VBIOS", "validate", "Master table offset out of bounds: 0x%x", master);
        return false;
    }
    uint32_t masterSize = readUint16(image + master);
    if (masterSize < 0x3e || masterSize > size - master || image[master + 2] != 2 || image[master + 3] != 1) {
        KernelLog::logError(KernelLog::Category::Hardware, "VBIOS", "validate", "Invalid master table size or revision: size=%u", masterSize);
        return false;
    }
    uint8_t checksum = 0;
    for (uint32_t i = 0; i < size; ++i) checksum += image[i];
    if (checksum) {
        KernelLog::logError(KernelLog::Category::Hardware, "VBIOS", "validate", "Invalid ROM checksum=0x%02x", static_cast<unsigned int>(checksum));
        return false;
    }
    uint32_t pspOffset = readUint16(image + master + 0x16);
    uint32_t vramOffset = readUint16(image + master + 0x3c);
    if (!isValidTable(image, size, pspOffset) || !isValidTable(image, size, vramOffset)) {
        KernelLog::logError(KernelLog::Category::Hardware, "VBIOS", "validate", "Invalid PSP or VRAM table: PSP=0x%x VRAM=0x%x", pspOffset, vramOffset);
        return false;
    }
    bool addPsp = !pspOffset;
    bool addVram = !vramOffset;
    uint32_t width = 0;
    uint32_t vramSize = 0;
    if (!vramOffset) {
        width = readMemoryWidth(image, size, master, masterSize);
        vramSize = readVramSize(device);
        if (!width || !vramSize) {
            KernelLog::logError(KernelLog::Category::Hardware, "VBIOS", "prepare", "Memory parameters unavailable: size=%u MiB width=%u bits", vramSize, width);
            return false;
        }
    }
    uint32_t end = size;
    if (!pspOffset) {
        pspOffset = (end + 15) & ~15U;
        end = pspOffset + 0x414;
    }
    if (!vramOffset) {
        vramOffset = (end + 15) & ~15U;
        end = vramOffset + 0x358;
    }
    uint32_t outputSize = (end + 511) & ~511U;
    if (end > 0x10000 || outputSize > 0xfe00) {
        KernelLog::logError(KernelLog::Category::Hardware, "VBIOS", "prepare", "Augmented ROM exceeds size limit: size=%u", outputSize);
        return false;
    }
    auto output = static_cast<uint8_t *>(IOMalloc(outputSize));
    if (!output) {
        KernelLog::logError(KernelLog::Category::Hardware, "VBIOS", "prepare", "Unable to allocate %u-byte ROM buffer", outputSize);
        return false;
    }
    bzero(output, outputSize);
    memcpy(output, image, size);
    writeUint16(output + pci + 6, 0x73ff);
    if (addPsp) {
        writeUint16(output + master + 0x16, static_cast<uint16_t>(pspOffset));
        writeUint16(output + pspOffset, 0x414);
        output[pspOffset + 2] = 2;
        output[pspOffset + 3] = 1;
        memcpy(output + pspOffset + 4, "$PSP", 4);
    }
    if (addVram) {
        writeUint16(output + master + 0x3c, static_cast<uint16_t>(vramOffset));
        writeUint16(output + vramOffset, 0x358);
        output[vramOffset + 2] = 2;
        output[vramOffset + 3] = 6;
        output[vramOffset + 20] = 1;
        auto module = output + vramOffset + 24;
        uint32_t channels = width / 16;
        writeUint32(module, vramSize);
        writeUint32(module + 4, (1U << channels) - 1);
        writeUint16(module + 20, 0x34);
        module[23] = 0x70;
        module[24] = static_cast<uint8_t>(channels);
        module[25] = 4;
        memcpy(module + 32, "RMB-UMA", 8);
    }
    output[2] = static_cast<uint8_t>(outputSize / 512);
    writeUint16(output + pci + 0x10, static_cast<uint16_t>(outputSize / 512));
    output[0x21] = 0;
    checksum = 0;
    for (uint32_t i = 0; i < outputSize; ++i) checksum += output[i];
    output[0x21] = static_cast<uint8_t>(-checksum);
    auto replacement = OSData::withBytes(output, outputSize);
    IOFree(output, outputSize);
    bool published = replacement && device->setProperty("ATY,bin_image", replacement);
    if (replacement) replacement->release();
    if (!published) {
        KernelLog::logError(KernelLog::Category::Hardware, "VBIOS", "publish", "Unable to create or publish augmented ROM, size=%u", outputSize);
        return false;
    }
    KernelLog::logInfo(KernelLog::Category::Hardware, "VBIOS", "publish", "PSP added=%u VRAM added=%u size=%u MiB width=%u bits published=%u",
        static_cast<unsigned int>(addPsp), static_cast<unsigned int>(addVram),
        vramSize, width, static_cast<unsigned int>(published));
    return published;
}

}

bool prepareRembrandtVbios(IOPCIDevice *device) {
    if (!device) {
        KernelLog::logError(KernelLog::Category::Hardware, "VBIOS", "prepare", "GPU device unavailable");
        return false;
    }
    device->removeProperty("ATY,bin_image");
    auto image = copyVfctVbios(device);
    if (!image) return false;
    bool published = publishVbios(device, image);
    image->release();
    return published;
}
