// MESSAGE CAMERA_SETTINGS support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief CAMERA_SETTINGS message
 *
 * WIP: Settings of a camera, can be requested using MAV_CMD_REQUEST_CAMERA_SETTINGS and written using MAV_CMD_SET_CAMERA_SETTINGS
 */
struct CAMERA_SETTINGS : mavlink::Message {
    static constexpr msgid_t MSG_ID = 260;
    static constexpr size_t LENGTH = 33;
    static constexpr size_t MIN_LENGTH = 33;
    static constexpr uint8_t CRC_EXTRA = 175;
    static constexpr auto NAME = "CAMERA_SETTINGS";


    uint32_t time_boot_ms; /*< Timestamp (milliseconds since system boot) */
    uint8_t camera_id; /*< Camera ID (1 for first, 2 for second, etc.) */
    uint8_t exposure_mode; /*< 0: full auto 1: full manual 2: aperture priority 3: shutter priority */
    float aperture; /*< Aperture is 1/value */
    float shutter_speed; /*< Shutter speed in seconds */
    float iso_sensitivity; /*< ISO sensitivity */
    float ev; /*< Exposure Value */
    float white_balance; /*< Color temperature in degrees Kelvin. (0: Auto WB) */
    uint8_t mode_id; /*< Reserved for a camera mode ID. (0: Photo 1: Video) */
    uint8_t audio_recording; /*< Audio recording enabled (0: off 1: on) */
    uint8_t color_mode_id; /*< Reserved for a color mode ID (Neutral, Vivid, etc.) */
    uint8_t image_format_id; /*< Reserved for image format ID (Jpeg/Raw/Jpeg+Raw) */
    uint8_t image_quality_id; /*< Reserved for image quality ID (Compression) */
    uint8_t metering_mode_id; /*< Reserved for metering mode ID (Average, Center, Spot, etc.) */
    uint8_t flicker_mode_id; /*< Reserved for flicker mode ID (Auto, 60Hz, 50Hz, etc.) */


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
        ss << "  exposure_mode: " << +exposure_mode << std::endl;
        ss << "  aperture: " << aperture << std::endl;
        ss << "  shutter_speed: " << shutter_speed << std::endl;
        ss << "  iso_sensitivity: " << iso_sensitivity << std::endl;
        ss << "  ev: " << ev << std::endl;
        ss << "  white_balance: " << white_balance << std::endl;
        ss << "  mode_id: " << +mode_id << std::endl;
        ss << "  audio_recording: " << +audio_recording << std::endl;
        ss << "  color_mode_id: " << +color_mode_id << std::endl;
        ss << "  image_format_id: " << +image_format_id << std::endl;
        ss << "  image_quality_id: " << +image_quality_id << std::endl;
        ss << "  metering_mode_id: " << +metering_mode_id << std::endl;
        ss << "  flicker_mode_id: " << +flicker_mode_id << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << time_boot_ms;                  // offset: 0
        map << aperture;                      // offset: 4
        map << shutter_speed;                 // offset: 8
        map << iso_sensitivity;               // offset: 12
        map << ev;                            // offset: 16
        map << white_balance;                 // offset: 20
        map << camera_id;                     // offset: 24
        map << exposure_mode;                 // offset: 25
        map << mode_id;                       // offset: 26
        map << audio_recording;               // offset: 27
        map << color_mode_id;                 // offset: 28
        map << image_format_id;               // offset: 29
        map << image_quality_id;              // offset: 30
        map << metering_mode_id;              // offset: 31
        map << flicker_mode_id;               // offset: 32
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_boot_ms;                  // offset: 0
        map >> aperture;                      // offset: 4
        map >> shutter_speed;                 // offset: 8
        map >> iso_sensitivity;               // offset: 12
        map >> ev;                            // offset: 16
        map >> white_balance;                 // offset: 20
        map >> camera_id;                     // offset: 24
        map >> exposure_mode;                 // offset: 25
        map >> mode_id;                       // offset: 26
        map >> audio_recording;               // offset: 27
        map >> color_mode_id;                 // offset: 28
        map >> image_format_id;               // offset: 29
        map >> image_quality_id;              // offset: 30
        map >> metering_mode_id;              // offset: 31
        map >> flicker_mode_id;               // offset: 32
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
