#ifndef __ESPROS_INTERFACE_H__
#define __ESPROS_INTERFACE_H__

#include <boost/thread.hpp>
#include "frame.hpp"
#include "camera_info.hpp"
#include "tcp_connection.hpp"
#include "udp_server.hpp"


namespace espros {

typedef std::vector<uint8_t> Packet;

class Interface {
public:
  Interface();
  ~Interface();

  void stopStream();  
  void streamDCS();
  void streamGrayscale();
  void streamDistance();
  void streamDistanceAmplitude();
  void setOffset(int16_t offset);
  void setMinAmplitude(uint16_t minAmplitude);
  void setBinning(const bool vertical, const bool horizontal);
  void setRoi(const uint16_t x0, const uint16_t y0, const uint16_t x1, const uint16_t y1);
  void setIntegrationTime(uint16_t, uint16_t, uint16_t, uint16_t);
  void setModulation(const uint8_t index, const uint8_t channel);
  void setFilter(const bool medianFilter, const bool averageFilter, const uint16_t temporalFactor, const uint16_t temporalThreshold, const uint16_t edgeThreshold,
                 const uint16_t temporalEdgeThresholdLow, const uint16_t temporalEdgeThresholdHigh, const uint16_t interferenceDetectionLimit, const bool interferenceDetectionUseLastValue);


  boost::signals2::connection subscribeFrame(std::function<void (std::shared_ptr<Frame>)>);
  boost::signals2::connection subscribeCameraInfo(std::function<void (std::shared_ptr<CameraInfo>)>);
  std::shared_ptr<CameraInfo> getCameraInfo(const Packet &);

private:
  bool isStreaming;
  uint8_t dataType;
  uint64_t currentFrame_id;  

  const static uint8_t COMMAND_SET_INT_TIMES = 1;
  const static uint8_t COMMAND_GET_DIST_AND_AMP = 2;
  const static uint8_t COMMAND_GET_DISTANCE = 3;
  const static uint8_t COMMAND_GET_GRAYSCALE = 5;
  const static uint8_t COMMAND_STOP_STREAM = 6;
  const static uint8_t COMMAND_GET_DCS = 7;
  const static uint8_t COMMAND_CALIBRATE = 30;
  const static uint8_t COMMAND_SET_FILTER = 22;
  const static uint8_t COMMAND_SET_MODULATION = 23;
  const static uint8_t COMMAND_SET_BINNING = 24;

  Packet data;
  std::shared_ptr<Frame> currentFrame;
  boost::asio::io_service ioService;
  boost::scoped_ptr<boost::thread> serverThread;
  boost::signals2::signal<void (std::shared_ptr<Frame>)> frameReady;
  boost::signals2::signal<void (std::shared_ptr<CameraInfo>)> cameraInfoReady;
  TcpConnection tcpConnection;
  UdpServer udpServer;

  void setDataType(uint8_t);  
  void streamMeasurement(uint8_t);
  void insertValue8(std::vector<uint8_t> &output, const uint8_t value);
  void insertValue(std::vector<uint8_t> &output, const int16_t value);
  uint8_t boolToUint8(const bool value);
};

} //end namespace espros

#endif
