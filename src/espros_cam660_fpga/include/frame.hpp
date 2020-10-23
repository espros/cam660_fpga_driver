#ifndef __ESPROS_FRAME_H__
#define __ESPROS_FRAME_H__

#include <cstdint>
#include <vector>

namespace espros {

    typedef std::vector<uint8_t> Packet;

    struct Frame
    {
        enum DataType { GRAYSCALE, DISTANCE, AMPLITUDE, DCS };

        static const int UDP_HEADER_OFFSET = 20;

        uint8_t stride;        
        uint16_t dataType;
        uint16_t width;
        uint16_t height;
        uint16_t payloadHeaderOffset;
        uint32_t px_size;        
        uint64_t frame_id;        
        std::vector<uint8_t> distData;
        std::vector<uint8_t> amplData;
        std::vector<uint8_t> dcsData;


        Frame(uint16_t, uint64_t, uint16_t, uint16_t, uint16_t);
        void sortData(const Packet&);
    };

} //end namespace espros

#endif // __ESPROS_FRAME_H__
