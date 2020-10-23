#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <dynamic_reconfigure/server.h>
#include <espros_cam660_fpga/espros_cam660_fpgaConfig.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "cartesian_transform.hpp"
#include "interface.hpp"

using namespace espros;

int imageType; //image and aquisition type: 0 - grayscale, 1 - distance, 2 - distance_amplitude
int lensType;  //0- wide field, 1- standard field, 2 - narrow field
int old_lensType;
bool medianFilter;
bool averageFilter;
double temporalFilterFactor;
int temporalFilterThreshold;
int edgeThreshold;
int temporalEdgeThresholdLow;
int temporalEdgeThresholdHigh;
int interferenceDetectionLimit;
bool startStream;
bool useLastValue;
bool publishPointCloud;
bool cartesian;
int channel;
int frequencyModulation;
int int0, int1, intGr;
int offset, minAmplitude;
int lensCenterOffsetX = 0;
int lensCenterOffsetY = 0;
int old_lensCenterOffsetX = 0;
int old_lensCenterOffsetY = 0;

const int width   = 320;
const int width2  = 160;
const int height  = 240;
const int height2 = 120;
const double sensorPixelSizeMM = 0.02; //camera sensor pixel size 20x20 um

uint32_t frameSeq;
boost::signals2::connection connectionFrames;
boost::signals2::connection connectionCameraInfo;

ros::Publisher distanceImagePublisher;
ros::Publisher amplitudeImagePublisher;
ros::Publisher dcsImagePublisher;

ros::Publisher cameraInfoPublisher;
ros::Publisher pointCloud2Publisher;
ros::ServiceServer cameraInfoService;

Interface interface;
CartesianTransform cartesianTransform;
sensor_msgs::CameraInfo cameraInfo;

//=======================================================================

void setParameters()
{
    interface.stopStream();
    interface.setOffset(offset);
    interface.setMinAmplitude(minAmplitude);
    interface.setIntegrationTime(int0, int1, 0, intGr);
    interface.setFilter(medianFilter, averageFilter, static_cast<uint16_t>(temporalFilterFactor * 1000), temporalFilterThreshold, edgeThreshold,
                        temporalEdgeThresholdLow, temporalEdgeThresholdHigh, interferenceDetectionLimit, useLastValue);

    uint8_t modIndex;
    if(frequencyModulation == 0) modIndex = 1;
    else if(frequencyModulation == 1)  modIndex = 0;
    else    modIndex = frequencyModulation;

    interface.setModulation(modIndex, channel);

    if(startStream){

        if(imageType == Frame::GRAYSCALE) interface.streamGrayscale();
        else if(imageType == Frame::DISTANCE) interface.streamDistance();
        else if(imageType == Frame::AMPLITUDE) interface.streamDistanceAmplitude();
        else interface.streamDCS();

    }else{
        interface.stopStream();
    }

    if(old_lensCenterOffsetX != lensCenterOffsetX || old_lensCenterOffsetY != lensCenterOffsetY || old_lensType != lensType){
        cartesianTransform.initLensTransform(sensorPixelSizeMM, width, height, lensCenterOffsetX, lensCenterOffsetY, lensType);
        old_lensCenterOffsetX = lensCenterOffsetX;
        old_lensCenterOffsetY = lensCenterOffsetY;
        old_lensType = lensType;
    }

    ROS_INFO("set parameters...");
    ROS_DEBUG("lens_type %d", lensType);
    ROS_DEBUG("lens_center_offset_x %d", lensCenterOffsetX);
    ROS_DEBUG("lens_center_offset_y %d", lensCenterOffsetY);
    ROS_DEBUG("image_type %d", imageType);
    ROS_DEBUG("start_stream %d", startStream);
    ROS_DEBUG("integration_time0 %d", int0);
    ROS_DEBUG("integration_time1 %d", int1);
    ROS_DEBUG("integration_time_gray %d", intGr);
    ROS_DEBUG("min_amplitude %d", minAmplitude);
    ROS_DEBUG("offset %d", offset);
    ROS_DEBUG("frequency_modulation %d", frequencyModulation);
    ROS_DEBUG("channel %d ", channel);
    ROS_DEBUG("median_filter %d ", medianFilter);
    ROS_DEBUG("average_filter %d", averageFilter);
    ROS_DEBUG("temporal_filter_factor %f", temporalFilterFactor);
    ROS_DEBUG("temporal_filter_threshold %d ", temporalFilterThreshold);
    ROS_DEBUG("edge_filter_threshold %d", edgeThreshold);
    ROS_DEBUG("temporal_edge_threshold_low %d ", temporalEdgeThresholdLow);
    ROS_DEBUG("temporal_edge_threshold_high %d", temporalEdgeThresholdHigh);
    ROS_DEBUG("interference_detection_limit %d ", interferenceDetectionLimit);
    ROS_DEBUG("use_last_value %d", useLastValue);
    ROS_DEBUG("cartesian %d", cartesian);
    ROS_DEBUG("publish_point_cloud %d", publishPointCloud);

}

void updateConfig(espros_cam660_fpga::espros_cam660_fpgaConfig &config, uint32_t level)
{
    startStream = config.start_stream;
    lensType = config.lens_type;
    lensCenterOffsetX = config.lens_center_offset_x;
    lensCenterOffsetY = config.lens_center_offset_y;
    imageType = config.image_type;
    offset = config.offset;
    minAmplitude = config.min_amplitude;
    int0 = config.integration_time_tof_1;
    int1 = config.integration_time_tof_2;
    intGr = config.integration_time_gray;
    frequencyModulation = config.frequency_modulation;
    channel = config.channel;
    medianFilter = config.median_filter;
    averageFilter = config.average_filter;
    temporalFilterFactor = config.temporal_filter_factor;
    temporalFilterThreshold = static_cast<uint16_t>(config.temporal_filter_threshold);
    edgeThreshold = static_cast<uint16_t>(config.edge_filter_threshold);
    temporalEdgeThresholdLow = static_cast<uint16_t>(config.temporal_edge_filter_threshold_low);
    temporalEdgeThresholdHigh = static_cast<uint16_t>(config.temporal_edge_filter_threshold_high);
    interferenceDetectionLimit = static_cast<uint16_t>(config.interference_detection_limit);
    useLastValue = config.use_last_value;
    cartesian = config.enable_cartesian;
    publishPointCloud = config.enable_point_cloud;

    setParameters();
}


bool setCameraInfo(sensor_msgs::SetCameraInfo::Request& req, sensor_msgs::SetCameraInfo::Response& res)
{
    req.camera_info.width  = cameraInfo.width;
    req.camera_info.height = cameraInfo.height;
    req.camera_info.roi    = cameraInfo.roi;

    cameraInfoPublisher.publish(req.camera_info);

    res.success = true;
    res.status_message = "";
    return true;
}

void startStreaming()
{
    switch(imageType) {
    case Frame::GRAYSCALE:
        interface.streamGrayscale();
        ROS_INFO("streaming grayscale");
        break;
    case Frame::DISTANCE:
        interface.streamDistance();
        ROS_INFO("streaming distance");
        break;
    case Frame::AMPLITUDE:
        interface.streamDistanceAmplitude();
        ROS_INFO("streaming distance-amplitude");
        break;
    case Frame::DCS:
        interface.streamDCS();
        ROS_INFO("streaming dcs");
        break;
    default:
        break;
    }
}


void updateCameraInfo(std::shared_ptr<CameraInfo> ci)
{
    cameraInfo.width = ci->width;
    cameraInfo.height = ci->height;
    cameraInfo.roi.x_offset = ci->roiX0;
    cameraInfo.roi.y_offset = ci->roiY0;
    cameraInfo.roi.width = ci->roiX1 - ci->roiX0;
    cameraInfo.roi.height = ci->roiY1 - ci->roiY0;
}

void updateFrame(std::shared_ptr<Frame> frame)
{
    int x, y, k, l;
    if(frame->dataType == Frame::DISTANCE || frame->dataType == Frame::AMPLITUDE ){
        sensor_msgs::Image imgDistance;
        imgDistance.header.seq = frameSeq++;
        imgDistance.header.stamp = ros::Time::now();
        imgDistance.header.frame_id = std::to_string(frame->frame_id);
        imgDistance.height = static_cast<uint32_t>(frame->height);
        imgDistance.width = static_cast<uint32_t>(frame->width);
        imgDistance.encoding = sensor_msgs::image_encodings::MONO16;
        imgDistance.step = imgDistance.width * frame->px_size;
        imgDistance.is_bigendian = 0;
        imgDistance.data = frame->distData;
        distanceImagePublisher.publish(imgDistance);
    }

    if(frame->dataType == Frame::AMPLITUDE || frame->dataType == Frame::GRAYSCALE){
        sensor_msgs::Image imgAmpl;
        imgAmpl.header.seq = frameSeq;
        imgAmpl.header.stamp = ros::Time::now();
        imgAmpl.header.frame_id = std::to_string(frame->frame_id);
        imgAmpl.height = static_cast<uint32_t>(frame->height);
        imgAmpl.width = static_cast<uint32_t>(frame->width);
        imgAmpl.encoding = sensor_msgs::image_encodings::MONO16;
        imgAmpl.step = imgAmpl.width * frame->px_size;
        imgAmpl.is_bigendian = 0;
        imgAmpl.data = frame->amplData;
        amplitudeImagePublisher.publish(imgAmpl);
    }

    if(frame->dataType == Frame::DCS){
        sensor_msgs::Image imgDCS;
        imgDCS.header.seq = frameSeq;
        imgDCS.header.stamp = ros::Time::now();
        imgDCS.header.frame_id = std::to_string(frame->frame_id);
        imgDCS.height = static_cast<uint32_t>(frame->height) * 4;
        imgDCS.width = static_cast<uint32_t>(frame->width);
        imgDCS.encoding = sensor_msgs::image_encodings::MONO16;
        imgDCS.step = imgDCS.width * frame->px_size;
        imgDCS.is_bigendian = 0;
        imgDCS.data = frame->dcsData;
        dcsImagePublisher.publish(imgDCS);
    }

    if(publishPointCloud && frame->dataType != Frame::GRAYSCALE){

        const size_t nPixel = frame->width * frame->height;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        cloud->header.frame_id = "sensor_frame";
        cloud->header.stamp = pcl_conversions::toPCL(ros::Time::now());
        cloud->width = static_cast<uint32_t>(frame->width);
        cloud->height = static_cast<uint32_t>(frame->height);
        cloud->is_dense = false;
        cloud->points.resize(nPixel);

        uint16_t distance, amplitude;
        double px, py, pz;

        for(k=0, l=0, y=0; y< frame->height; y++){
            for(x=0; x< frame->width; x++, k++, l+=2){
                pcl::PointXYZI &p = cloud->points[k];
                distance = (frame->distData[l+1] << 8) + frame->distData[l];

                if(frame->dataType == Frame::AMPLITUDE)
                    amplitude = (frame->amplData[l+1] << 8)  + frame->amplData[l];


                if (distance > 0 && distance < 65000){

                    if(cartesian){
                        cartesianTransform.transformPixel(x, y, distance, px, py, pz);
                        p.x = static_cast<float>(px / 1000.0); //mm -> m
                        p.y = static_cast<float>(py / 1000.0);
                        p.z = static_cast<float>(pz / 1000.0);

                        if(frame->dataType == Frame::AMPLITUDE) p.intensity = static_cast<float>(amplitude);
                        else p.intensity = static_cast<float>(pz / 1000.0);

                    }else{
                        p.x = x / 100.0;
                        p.y = y / 100.0;
                        p.z = distance / 1000.0;
                        if(frame->dataType == Frame::AMPLITUDE) p.intensity =  static_cast<float>(amplitude);
                        else p.intensity = static_cast<float>(distance / 1000.0);
                    }
                }else{
                    p.x = std::numeric_limits<float>::quiet_NaN();
                    p.y = std::numeric_limits<float>::quiet_NaN();
                    p.z = std::numeric_limits<float>::quiet_NaN();
                }
            }
        }

        pointCloud2Publisher.publish(cloud);
    }
}


//===================================================

void initialise()
{
    frameSeq = 0;
    ros::NodeHandle nh("~");

    nh.getParam("lens_Type", lensType);
    nh.getParam("lens_center_offset_x", lensCenterOffsetX);
    nh.getParam("lens_center_offset_y", lensCenterOffsetY);
    nh.getParam("start_stream", startStream);
    nh.getParam("image_type", imageType);
    nh.getParam("int0", int0);
    nh.getParam("int1", int1);
    nh.getParam("int_gray", intGr);
    nh.getParam("frequency_modulation", frequencyModulation);
    nh.getParam("channel", channel);
    nh.getParam("offset", offset);
    nh.getParam("min_amplitude", minAmplitude);
    nh.getParam("median_filter", medianFilter);
    nh.getParam("average_filter", averageFilter);
    nh.getParam("temporal_filter_factor", temporalFilterFactor);
    nh.getParam("temporal_filter_threshold", temporalFilterThreshold);
    nh.getParam("edge_threshold", edgeThreshold);
    nh.getParam("temporal_edge_threshold_low", temporalEdgeThresholdLow);
    nh.getParam("temporal_edge_threshold_high", temporalEdgeThresholdHigh);
    nh.getParam("interference_detection_limit", interferenceDetectionLimit);
    nh.getParam("use_last_value", useLastValue);
    nh.getParam("cartesian", cartesian);
    nh.getParam("publish_point_cloud", publishPointCloud);

    //advertise publishers
    distanceImagePublisher = nh.advertise<sensor_msgs::Image>("distance_image_raw", 1000);
    amplitudeImagePublisher = nh.advertise<sensor_msgs::Image>("amplitude_image_raw", 1000);
    dcsImagePublisher = nh.advertise<sensor_msgs::Image>("dcs_image_raw", 1000);
    pointCloud2Publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZI> > ("points", 100);
    cameraInfoPublisher = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1000);

    //advertise services
    cameraInfoService = nh.advertiseService("set_camera_info", setCameraInfo);

    //connect to interface
    connectionCameraInfo = interface.subscribeCameraInfo([&](std::shared_ptr<CameraInfo> ci) -> void { updateCameraInfo(ci); });
    connectionFrames = interface.subscribeFrame([&](std::shared_ptr<Frame> f) -> void {  updateFrame(f); });

    cartesianTransform.initLensTransform(sensorPixelSizeMM, width, height, lensCenterOffsetX, lensCenterOffsetY, lensType); //0.02 mm - sensor pixel size
    old_lensCenterOffsetX = lensCenterOffsetX;
    old_lensCenterOffsetY = lensCenterOffsetY;
    old_lensType = lensType;

    ROS_INFO("camera 660 fpga driver version 1.6.0");
}


//==========================================================================

int main(int argc, char **argv)
{   
    //if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) ros::console::notifyLoggerLevelsChanged();

    ros::init(argc, argv, "cam660_fpga_node");
    dynamic_reconfigure::Server<espros_cam660_fpga::espros_cam660_fpgaConfig> server;
    dynamic_reconfigure::Server<espros_cam660_fpga::espros_cam660_fpgaConfig>::CallbackType f;
    f = boost::bind(&updateConfig, _1, _2);
    server.setCallback(f);

    initialise();
    setParameters();
    startStreaming();
    ros::spin();
}
