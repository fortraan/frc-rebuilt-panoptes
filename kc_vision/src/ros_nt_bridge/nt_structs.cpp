#include "nt_structs.h"

#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rectangle2d.h>
#include <wpiutil/wpi/struct/Struct.h>

void wpi::Struct<FuelClump>::Pack(std::span<uint8_t> data, const FuelClump &value) {
    wpi::PackStruct<0>(data, value.centroid);
    wpi::PackStruct<BOUNDS_OFFSET>(data, value.bounds);
    wpi::PackStruct<AREA_OFFSET>(data, value.area);
    wpi::PackStruct<LABEL_OFFSET>(data, value.label);
}

FuelClump wpi::Struct<FuelClump>::Unpack(std::span<const uint8_t> data) {
    return FuelClump {
        wpi::UnpackStruct<frc::Translation2d, 0>(data),
        wpi::UnpackStruct<frc::Rectangle2d, BOUNDS_OFFSET>(data),
        wpi::UnpackStruct<int, AREA_OFFSET>(data),
        wpi::UnpackStruct<uint16_t, LABEL_OFFSET>(data)
    };
}