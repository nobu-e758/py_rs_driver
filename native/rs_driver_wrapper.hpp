#include <iostream>
#include <functional>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <rs_driver/msg/point_cloud_msg.hpp>
#include <rs_driver/api/lidar_driver.hpp>
#include <rs_driver/driver/driver_param.hpp>
#include <rs_driver/common/error_code.hpp>

using namespace robosense::lidar;
using namespace std;
namespace py = pybind11;

typedef PointXYZI PointT;
typedef PointCloudT<PointT> PointCloudMsg;

typedef std::function<void(uint32_t, uint32_t, uint32_t, double, std::vector<std::vector<double>>)> PointCloudCallback;
typedef std::function<void(uint8_t, py::str)> ExceptionCallback;

bool native_msg = true; // Display native message or not

std::shared_ptr<PointCloudMsg> pointCloudGetCallback(void)
{
    return std::make_shared<PointCloudMsg>();
}

PointCloudCallback pyPointCloudCallback;
void pointCloudPutCallback(std::shared_ptr<PointCloudMsg> msg)
{
    std::vector<std::vector<double>> data;
    for (PointT &element : msg->points)
    {
        std::vector<double> xyzi;
        xyzi.push_back(element.x);
        xyzi.push_back(element.y);
        xyzi.push_back(element.z);
        xyzi.push_back(element.intensity);
        data.push_back(xyzi);
    }

    if (pyPointCloudCallback)
    {
        pyPointCloudCallback(
            msg->seq,
            msg->height,
            msg->width,
            msg->timestamp,
            data);
    }
    if (native_msg)
        RS_MSG << "msg: " << msg->seq << " point cloud size: " << msg->points.size() << RS_REND;
}

ExceptionCallback pyExceptionCallback;
void exceptionCallback(const Error &code)
{
    if (pyExceptionCallback)
        pyExceptionCallback(int(code.error_code_type), code.toString());
    if (native_msg)
        RS_WARNING << code.toString() << RS_REND;
}

class RSDriverWrapper
{
public:
    RSDriverWrapper() : lidar_driver(LidarDriver<PointCloudMsg>()), param(RSDriverParam())
    {
    }
    ~RSDriverWrapper()
    {
        stop();
    }

    void displayNativeMsg(bool flag)
    {
        native_msg = flag;
    }

    bool init(py::object py_param)
    {
        // lidar_type
        param.lidar_type = (LidarType)(py_param.attr("lidar_type").cast<uint32_t>());
        // input_type
        param.input_type = (InputType)(py_param.attr("input_type").cast<uint32_t>());
        // input_param
        param.input_param.msop_port = py_param.attr("input_param").attr("msop_port").cast<uint16_t>();
        param.input_param.difop_port = py_param.attr("input_param").attr("difop_port").cast<uint16_t>();
        param.input_param.host_address = py_param.attr("input_param").attr("host_address").cast<string>();
        param.input_param.group_address = py_param.attr("input_param").attr("group_address").cast<string>();
        param.input_param.pcap_path = py_param.attr("input_param").attr("pcap_path").cast<string>();
        param.input_param.pcap_repeat = py_param.attr("input_param").attr("pcap_repeat").cast<bool>();
        param.input_param.pcap_rate = py_param.attr("input_param").attr("pcap_rate").cast<float>();
        param.input_param.use_vlan = py_param.attr("input_param").attr("use_vlan").cast<bool>();
        param.input_param.user_layer_bytes = py_param.attr("input_param").attr("user_layer_bytes").cast<uint16_t>();
        param.input_param.tail_layer_bytes = py_param.attr("input_param").attr("tail_layer_bytes").cast<uint16_t>();
        // decoder_param
        param.decoder_param.config_from_file = py_param.attr("decoder_param").attr("config_from_file").cast<bool>();
        param.decoder_param.angle_path = py_param.attr("decoder_param").attr("angle_path").cast<string>();
        param.decoder_param.wait_for_difop = py_param.attr("decoder_param").attr("wait_for_difop").cast<bool>();
        param.decoder_param.min_distance = py_param.attr("decoder_param").attr("min_distance").cast<float>();
        param.decoder_param.max_distance = py_param.attr("decoder_param").attr("max_distance").cast<float>();
        param.decoder_param.start_angle = py_param.attr("decoder_param").attr("start_angle").cast<float>();
        param.decoder_param.end_angle = py_param.attr("decoder_param").attr("end_angle").cast<float>();
        param.decoder_param.split_frame_mode = (SplitFrameMode)(py_param.attr("decoder_param").attr("split_frame_mode").cast<uint32_t>());
        param.decoder_param.split_angle = py_param.attr("decoder_param").attr("split_angle").cast<float>();
        param.decoder_param.num_blks_split = py_param.attr("decoder_param").attr("num_blks_split").cast<uint16_t>();
        param.decoder_param.use_lidar_clock = py_param.attr("decoder_param").attr("use_lidar_clock").cast<bool>();
        param.decoder_param.dense_points = py_param.attr("decoder_param").attr("dense_points").cast<bool>();
        param.decoder_param.ts_first_point = py_param.attr("decoder_param").attr("ts_first_point").cast<bool>();
        // decoder_param.transform_param
        param.decoder_param.transform_param.x = py_param.attr("decoder_param").attr("transform_param").attr("x").cast<float>();
        param.decoder_param.transform_param.y = py_param.attr("decoder_param").attr("transform_param").attr("y").cast<float>();
        param.decoder_param.transform_param.z = py_param.attr("decoder_param").attr("transform_param").attr("z").cast<float>();
        param.decoder_param.transform_param.roll = py_param.attr("decoder_param").attr("transform_param").attr("roll").cast<float>();
        param.decoder_param.transform_param.pitch = py_param.attr("decoder_param").attr("transform_param").attr("pitch").cast<float>();
        param.decoder_param.transform_param.yaw = py_param.attr("decoder_param").attr("transform_param").attr("yaw").cast<float>();

        lidar_driver.regPointCloudCallback(pointCloudGetCallback, pointCloudPutCallback);
        lidar_driver.regExceptionCallback(exceptionCallback);

        if (native_msg)
        {
            RS_TITLE << "------------------------------------------------------" << RS_REND;
            RS_TITLE << "            RS_Driver Core Version: v" << getDriverVersion() << RS_REND;
            RS_TITLE << "------------------------------------------------------" << RS_REND;
            param.print();
        }

        if (!lidar_driver.init(param))
        {
            if (pyExceptionCallback)
                pyExceptionCallback(int(ErrCodeType::ERROR_CODE), "Driver Initialize Error");
            if (native_msg)
                RS_ERROR << "Driver Initialize Error" << RS_REND;
            return false;
        }
        return true;
    }

    bool start()
    {
        return lidar_driver.start();
    }
    void stop()
    {
        pyExceptionCallback = NULL;
        pyPointCloudCallback = NULL;
        lidar_driver.stop();
    }

    float getLidarTemperature()
    {
        float temp;
        lidar_driver.getTemperature(temp);
        return temp;
    }

    void regExceptionCallback(ExceptionCallback fn)
    {
        pyExceptionCallback = fn;
    }
    void regPointCloudCallback(PointCloudCallback fn)
    {
        pyPointCloudCallback = fn;
    }

    // Not yet implemented...
    //
    // void initDecoderOnly(const RSDriverParam& param);
    // void regPacketCallback(const std::function<void(const Packet&)>& cb_put_pkt);
    // bool decodeMsopScan(const ScanMsg& scan_msg, PointCloudMsg<T_Point>& point_cloud_msg);
    // void decodeDifopPkt(const PacketMsg& msg);

private:
    LidarDriver<PointCloudMsg> lidar_driver;
    RSDriverParam param;
};
