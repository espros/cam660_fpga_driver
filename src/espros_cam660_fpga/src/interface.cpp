#include <ros/ros.h>
#include <iostream>
#include "frame.hpp"
#include "interface.hpp"

namespace espros {

Interface::Interface() : tcpConnection(ioService),
    udpServer(ioService),
    isStreaming(false),
    dataType(0),
    currentFrame_id(0),
    data(Packet(25+320*240*4*2))
{
    serverThread.reset(new boost::thread(boost::bind(&boost::asio::io_service::run, &ioService)));

    udpServer.subscribe([&](const Packet& p) -> void
    {
        uint32_t packetNum  = (p[16] << 24) + (p[17] << 16) + (p[18] << 8) + p[19];
        uint32_t offset = (p[8] << 24) + (p[9] << 16) + (p[10] << 8) + p[11];
        uint16_t payloadSize = (p[6] << 8) + p[7];

        if(packetNum == 0) { // new frame
            uint16_t width  = (p[23] << 8) + p[24];
            uint16_t height = (p[25] << 8) + p[26];
            int payloadHeaderOffset = (p[43] << 8) + p[44];

            currentFrame = std::shared_ptr<Frame>(new Frame(dataType, currentFrame_id++, width, height, payloadHeaderOffset));
            memcpy(&data[offset], &p[Frame::UDP_HEADER_OFFSET], payloadSize);
            cameraInfoReady(getCameraInfo(p));

        }else{
            uint32_t numPackets = (p[12] << 24) + (p[13] << 16) + (p[14] << 8) + p[15];
            memcpy(&data[offset], &p[Frame::UDP_HEADER_OFFSET], payloadSize);

            if (packetNum == numPackets - 1) { //last frame                                
                currentFrame->sortData(data);  //copy data -> dist, ampl, dcs
                frameReady(currentFrame);
            }
        }

    }); //end lambda function

}

Interface::~Interface() {
    stopStream();
    serverThread->interrupt();
    ioService.stop();
}

void Interface::stopStream() {
    if (!isStreaming) { return; }
    std::vector<uint8_t> command({0x00, COMMAND_STOP_STREAM});
    tcpConnection.sendCommand(command);
    isStreaming = false;

}

void Interface::streamDCS()
{
    setDataType(Frame::DCS);
    streamMeasurement(COMMAND_GET_DCS);
}

void Interface::streamDistanceAmplitude() {
    setDataType(Frame::AMPLITUDE);
    streamMeasurement(COMMAND_GET_DIST_AND_AMP);
}

void Interface::streamDistance() {
    setDataType(Frame::DISTANCE);
    streamMeasurement(COMMAND_GET_DISTANCE);
}

void Interface::streamGrayscale() {
    setDataType(Frame::GRAYSCALE);
    streamMeasurement(COMMAND_GET_GRAYSCALE);
}


void Interface::setOffset(int16_t offset){
    std::vector<uint8_t> payload = {
        0x00, 0x14,
        static_cast<uint8_t>(offset >> 8),
        static_cast<uint8_t>(offset & 0x00ff)
    };

    tcpConnection.sendCommand(payload);
}

void Interface::setMinAmplitude(uint16_t minAmplitude){
    std::vector<uint8_t> payload = {
        0x00, 0x15,
        static_cast<uint8_t>(minAmplitude >> 8),
        static_cast<uint8_t>(minAmplitude & 0x00ff)
    };

    tcpConnection.sendCommand(payload);
}

void Interface::setBinning(const bool vertical, const bool horizontal)
{
    uint8_t  byte = 0;
    if(vertical && horizontal){
        byte = 3;
    }else if(vertical){
        byte = 1;
    }else if(horizontal){
        byte = 2;
    }

    std::vector<uint8_t> payload = {
        0x00, 0x18, byte
    };

    tcpConnection.sendCommand(payload);
}


void Interface::setRoi(const uint16_t x0, const uint16_t y0, const uint16_t x1, const uint16_t y1)
{
    std::vector<uint8_t> payload = {
        0x00, 0x00,
        static_cast<uint8_t>(x0 >> 8), static_cast<uint8_t>(x0 & 0x00ff),
        static_cast<uint8_t>(y0 >> 8), static_cast<uint8_t>(y0 & 0x00ff),
        static_cast<uint8_t>(x1 >> 8), static_cast<uint8_t>(x1 & 0x00ff),
        static_cast<uint8_t>(y1 >> 8), static_cast<uint8_t>(y1 & 0x00ff)};
    tcpConnection.sendCommand(payload);
}


void Interface::setIntegrationTime(uint16_t low, uint16_t mid, uint16_t high, uint16_t gray)
{
    std::vector<uint8_t> payload = {
        0x00, 0x01,
        static_cast<uint8_t>(low >> 8), static_cast<uint8_t>(low & 0x00ff),
        static_cast<uint8_t>(mid >> 8), static_cast<uint8_t>(mid & 0x00ff),
        static_cast<uint8_t>(high >> 8), static_cast<uint8_t>(high & 0x00ff),
        static_cast<uint8_t>(gray >> 8), static_cast<uint8_t>(gray & 0x00ff)};
    tcpConnection.sendCommand(payload);
}


void Interface::setFilter(const bool medianFilter, const bool averageFilter, const uint16_t temporalFactor, const uint16_t temporalThreshold, const uint16_t edgeThreshold, const uint16_t temporalEdgeThresholdLow, const uint16_t temporalEdgeThresholdHigh, const uint16_t interferenceDetectionLimit, const bool interferenceDetectionUseLastValue)
{
    std::vector<uint8_t> payload;
    uint16_t command = COMMAND_SET_FILTER;

    //Insert the 16Bit command
    insertValue(payload, command);

    //Insert temporal filter factor
    insertValue(payload, temporalFactor);

    //Insert temporal filter threshold
    insertValue(payload, temporalThreshold);

    //Insert median filter
    insertValue8(payload, boolToUint8(medianFilter));

    //Insert average filter
    insertValue8(payload, boolToUint8(averageFilter));

    //Insert edge filter threshold
    insertValue(payload, edgeThreshold);

    //Insert interference detection use last value flag
    insertValue8(payload, boolToUint8(interferenceDetectionUseLastValue));

    //Insert edge filter interference detection limit
    insertValue(payload, interferenceDetectionLimit);

    //Insert edge filter threshold low
    insertValue(payload, temporalEdgeThresholdLow);

    //Insert edge filter threshold high
    insertValue(payload, temporalEdgeThresholdHigh);

    tcpConnection.sendCommand(payload);
}


void Interface::setModulation(const uint8_t index, const uint8_t channel){

    std::vector<uint8_t> payload;
    uint16_t command = COMMAND_SET_MODULATION;

    insertValue(payload, command);

    uint8_t data = static_cast<char>(index);
    payload.push_back(data);

    data = static_cast<char>(channel);
    payload.push_back(data);

    data = 0; //AutoChannel reserved
    payload.push_back(data);

    tcpConnection.sendCommand(payload);
}


void Interface::insertValue8(std::vector<uint8_t> &output, const uint8_t value){
    output.push_back(value);
}

void Interface::insertValue(std::vector<uint8_t> &output, const int16_t value)
{
    output.push_back(value >> 8);
    output.push_back(static_cast<uint8_t>(value & 0xFF));
}

uint8_t Interface::boolToUint8(const bool value)
{
    if (value)  return 1;
    else        return 0;
}

boost::signals2::connection Interface::subscribeFrame(std::function<void (std::shared_ptr<Frame>)> onFrameReady)
{
    frameReady.connect(onFrameReady);
}

boost::signals2::connection Interface::subscribeCameraInfo(std::function<void (std::shared_ptr<CameraInfo>)> onCameraInfoReady)
{
    cameraInfoReady.connect(onCameraInfoReady);
}

std::shared_ptr<CameraInfo> Interface::getCameraInfo(const Packet& p) {
    std::shared_ptr<CameraInfo> camInfo(new CameraInfo);

    int offset = 23;
    camInfo->width  = (p[offset++] << 8) + p[offset++];
    camInfo->height = (p[offset++] << 8) + p[offset++];
    camInfo->roiX0  = (p[offset++] << 8) + p[offset++];
    camInfo->roiY0  = (p[offset++] << 8) + p[offset++];
    camInfo->roiX1  = (p[offset++] << 8) + p[offset++];
    camInfo->roiY1  = (p[offset++] << 8) + p[offset++];

    return camInfo;
}

void Interface::setDataType(uint8_t d) {
    dataType = d;
}

void Interface::streamMeasurement(uint8_t cmd) {
    tcpConnection.sendCommand(std::vector<uint8_t>({0x00, cmd, 0x01}));
    isStreaming = true;
}

} //end namespace espros
