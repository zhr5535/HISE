#ifndef _MV_CAMERA_H_
#define _MV_CAMERA_H_

#include "sdk/MvCameraControl.h"
#include <string.h>

#ifndef MV_NULL
#define MV_NULL nullptr
#endif

class HikCamera
{
public:
    HikCamera();
    ~HikCamera();

    // ch:获取SDK版本号 | en:Get SDK Version
    static int get_sdk_version();

    // ch:枚举设备| en:Enumerate Device
    static int enum_devices(MV_CC_DEVICE_INFO_LIST *device_list);

    // ch:判断设备是否可达 | en:Is the device accessible
    static bool is_device_accessible(MV_CC_DEVICE_INFO *device_info, unsigned int access_tag);

    // ch:打开设备 | en:Open Device
    int open(MV_CC_DEVICE_INFO *device_info);

    // ch:关闭设备 | en:Close Device
    int close();

    // ch:判断相机是否处于连接状态 | en:Is The Device Connected
    bool is_device_connected();

    // ch:注册图像数据回调| en:Register Image Data CallBack
    int register_image_callback(void(__stdcall *output)(unsigned char *data, MV_FRAME_OUT_INFO_EX *frame_info, void *user), void *user);

    // ch:开启抓图 | en:Start Grabbing
    int start_grabbing();

    // ch:停止抓图 | en:Stop Grabbing
    int stop_grabbing();

    // ch:主动获取一帧图像数据 | en:Get one frame initiatively
    int get_image_buffer(int msec = 1000);

    // ch:释放图像缓存 | en:Free image buffer
    int free_image_buffer();

    // ch:显示一帧图像 | en:Display one frame image
    int display_one_frame(MV_DISPLAY_FRAME_INFO *display_info);

    // ch:设置SDK内部图像缓存节点个数 | en:Set the number of the internal image cache nodes in SDK
    int set_image_nodenum(unsigned int num);

    // ch:获取设备信息 | en:Get device information
    int get_device_info(MV_CC_DEVICE_INFO *device_info);

    // ch:获取GEV相机的统计信息| en:Get detect info of GEV camera
    int get_GEV_all_match_info(MV_MATCH_INFO_NET_DETECT *match_info_net_detect);

    // ch:获取U3V相机的统计信息 | en:Get detect info of U3V camera
    int get_U3V_all_match_info(MV_MATCH_INFO_USB_DETECT *match_info_usb_detect);

    // ch:获取和设置Int型参数，如 Width和Height，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
    // en:Get Int type parameters, such as Width and Height, for details please refer to MvCameraNode.xlsx file under SDK installation directory
    int get_int_value(IN const char *strkey, OUT MVCC_INTVALUE_EX *int_value);
    int set_int_value(IN const char *strkey, IN int64_t value);

    // ch:获取和设置Enum型参数，如 PixelFormat，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
    // en:Get Enum type parameters, such as PixelFormat, for details please refer to MvCameraNode.xlsx file under SDK installation directory
    int get_enum_value(IN const char *strkey, OUT MVCC_ENUMVALUE *enum_value);
    int set_enum_value(IN const char *strkey, IN unsigned int value);
    int set_enum_value_by_string(IN const char *strkey, IN const char *value);

    // ch:ch:获取和设置Float型参数，如 ExposureTime和Gain，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
    // en:Get Float type parameters, such as ExposureTime and Gain, for details please refer to MvCameraNode.xlsx file under SDK installation directory
    int get_float_value(IN const char *strkey, OUT MVCC_FLOATVALUE *float_value);
    int set_float_value(IN const char *strkey, IN float value);

    // ch:获取和设置Bool型参数，如 ReverseX，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
    // en:Get Bool type parameters, such as ReverseX, for details please refer to MvCameraNode.xlsx file under SDK installation directory
    int get_bool_value(IN const char *strkey, OUT bool *value);
    int set_bool_value(IN const char *strkey, IN bool value);

    // ch:获取和设置String型参数，如 DeviceUserID，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件UserSetSave
    // en:Get String type parameters, such as DeviceUserID, for details please refer to MvCameraNode.xlsx file under SDK installation directory
    int get_string_value(IN const char *strkey, MVCC_STRINGVALUE *string_value);
    int set_string_value(IN const char *strkey, IN const char *strvalue);

    // ch:执行一次Command型命令，如 UserSetSave，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
    // en:Execute Command once, such as UserSetSave, for details please refer to MvCameraNode.xlsx file under SDK installation directory
    int command_execute(IN const char *strkey);

    // ch:探测网络最佳包大小(只对GigE相机有效)  | en:Detection network optimal package size(It only works for the GigE camera)
    int get_optimal_packet_size(unsigned int *optimal_packet_size);

    // ch:注册消息异常回调 | en:Register Message Exception CallBack
    int register_exception_callback(void(__stdcall *exception)(unsigned int msg_type, void *user), void *user);

    // ch:注册单个事件回调 | en:Register Event CallBack
    int register_event_callback(const char *event_name, void(__stdcall *event)(MV_EVENT_OUT_INFO *event_info, void *user), void *user);

    // ch:强制IP | en:Force IP
    int force_ip(unsigned int ip, unsigned int sub_net_mask, unsigned int default_gate_way);

    // ch:配置IP方式 | en:IP configuration method
    int set_ip_config(unsigned int type);

    // ch:设置网络传输模式 | en:Set Net Transfer Mode
    int set_net_trans_mode(unsigned int type);

    // ch:像素格式转换 | en:Pixel format conversion
    int convert_pixel_type(MV_CC_PIXEL_CONVERT_PARAM *cvt_param);

    // ch:保存图片 | en:save image
    int save_image(MV_SAVE_IMAGE_PARAM_EX3 *param); // 输出Param，不直接存成文件，方便后续处理

    int save_image_to_file(MV_SAVE_IMAGE_TO_FILE_PARAM_EX *save_file_param);

    int get_frame_timeout(IN OUT unsigned char *data, IN unsigned int data_size, IN OUT MV_FRAME_OUT_INFO_EX *frame_info, IN unsigned int msec);

    int get_payload_size(OUT unsigned int payloadsize);

    int get_valid_image_num(unsigned int *valid_image_num);

    void *handle() const
    {
        return m_handle;
    }

    const MV_FRAME_OUT &outframe() const
    {
        return m_outframe;
    }

    void set_outframe(MV_FRAME_OUT frame)
    {
        m_outframe = frame;
    }

private:
    void *m_handle; // 相机句柄

    MV_FRAME_OUT m_outframe = {0}; // 输出帧数据与参数，需要使用get_image_buffer获取或手动写入
};

#endif //_MV_CAMERA_H_