#ifndef KC_VISION_NT_STRUCTS_H
#define KC_VISION_NT_STRUCTS_H

#include "wpiutil/wpi/SymbolExports.h"
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rectangle2d.h>
#include <wpiutil/wpi/struct/Struct.h>

struct FuelClump {
    frc::Translation2d centroid;
    frc::Rectangle2d bounds;
    int area{};
    uint16_t label{};
};

template <>
struct WPILIB_DLLEXPORT wpi::Struct<FuelClump> final {
    static constexpr size_t BOUNDS_OFFSET = wpi::GetStructSize<frc::Translation2d>();
    static constexpr size_t AREA_OFFSET = BOUNDS_OFFSET + wpi::GetStructSize<frc::Rectangle2d>();
    static constexpr size_t LABEL_OFFSET = AREA_OFFSET + sizeof(int);

    static constexpr std::string_view GetTypeName() {
        return "FuelClump";
    }

    static constexpr size_t GetSize() {
        return wpi::GetStructSize<frc::Translation2d>() + wpi::GetStructSize<frc::Rectangle2d>() + sizeof(int) + sizeof(uint16_t);
    }

    static constexpr std::string_view GetSchema() {
        return "Translation2d centroid; Rectangle2d bounds; int32 area; uint16 label";
    }

    static FuelClump Unpack(std::span<const uint8_t> data);

    static void Pack(std::span<uint8_t> data, const FuelClump& value);

    static void ForEachNested(std::invocable<std::string_view, std::string_view> auto fn) {
        wpi::ForEachStructSchema<frc::Translation2d>(fn);
        wpi::ForEachStructSchema<frc::Rectangle2d>(fn);
    }
};

static_assert(wpi::StructSerializable<FuelClump>);
static_assert(wpi::HasNestedStruct<FuelClump>);



#endif //KC_VISION_NT_STRUCTS_H