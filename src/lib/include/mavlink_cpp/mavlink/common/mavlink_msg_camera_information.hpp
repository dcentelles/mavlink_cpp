// MESSAGE CAMERA_INFORMATION support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief CAMERA_INFORMATION message
 *
 * WIP: Information about a camera
 */
struct CAMERA_INFORMATION : mavlink::Message {
    static constexpr msgid_t MSG_ID = 259;
    static constexpr size_t LENGTH = 91;
    static constexpr size_t MIN_LENGTH = 91;
    static constexpr uint8_t CRC_EXTRA = 49;
    static constexpr auto NAME = "CAMERA_INFORMATION";


    uint32_t time_boot_ms; /*< Timestamp (milliseconds since system boot) */
    uint8_t camera_id; /*< Camera ID (1 for first, 2 for second, etc.) */
    uint8_t camera_count; /*< Number of cameras */
    std::array<uint8_t, 32> vendor_name; /*< Name of the camera vendor */
    std::array<uint8_t, 32> model_name; /*< Name of the camera model */
    uint32_t firmware_version; /*< Version of the camera firmware */
    float focal_length; /*< Focal length in mm */
    float sensor_size_h; /*< Image sensor size horizontal in mm */
    float sensor_size_v; /*< Image sensor size vertical in mm */
    uint16_t resolution_h; /*< Image resolution in pixels horizontal */
    uint16_t resolution_v; /*< Image resolution in pixels vertical */
    uint8_t lens_id; /*< Reserved for a lens ID */


    inline std::string get_name(void) const override
    {
            return NAME;
    }

    inline Info get_message_info(void) const override
    {
            return { MSG_ID, LENGTH, MIN_LENGTH, CRC_EXTRA };
    }

    inline std::string to_yaml(void) const override
    {
        std::stringstream ss;

        ss << NAME << ":" << std::endl;
        ss << "  time_boot_ms: " << time_boot_ms << std::endl;
        ss << "  camera_id: " << +camera_id << std::endl;
        ss << "  camera_count: " << +camera_count << std::endl;
        ss << "  vendor_name: [" << to_string(vendor_name) << "]" << std::endl;
        ss << "  model_name: [" << to_string(model_name) << "]" << std::endl;
        ss << "  firmware_version: " << firmware_version << std::endl;
        ss << "  focal_length: " << focal_length << std::endl;
        ss << "  sensor_size_h: " << sensor_size_h << std::endl;
        ss << "  sensor_size_v: " << sensor_size_v << std::endl;
        ss << "  resolution_h: " << resolution_h << std::endl;
        ss << "  resolution_v: " << resolution_v << std::endl;
        ss << "  lens_id: " << +lens_id << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << time_boot_ms;                  // offset: 0
        map << firmware_version;              // offset: 4
        map << focal_length;                  // offset: 8
        map << sensor_size_h;                 // offset: 12
        map << sensor_size_v;                 // offset: 16
        map << resolution_h;                  // offset: 20
        map << resolution_v;                  // offset: 22
        map << camera_id;                     // offset: 24
        map << camera_count;                  // offset: 25
        map << vendor_name;                   // offset: 26
        map << model_name;                    // offset: 58
        map << lens_id;                       // offset: 90
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_boot_ms;                  // offset: 0
        map >> firmware_version;              // offset: 4
        map >> focal_length;                  // offset: 8
        map >> sensor_size_h;                 // offset: 12
        map >> sensor_size_v;                 // offset: 16
        map >> resolution_h;                  // offset: 20
        map >> resolution_v;                  // offset: 22
        map >> camera_id;                     // offset: 24
        map >> camera_count;                  // offset: 25
        map >> vendor_name;                   // offset: 26
        map >> model_name;                    // offset: 58
        map >> lens_id;                       // offset: 90
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
