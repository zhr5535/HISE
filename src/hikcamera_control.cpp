#include "hikcamera/hikcamera_control.h"

HikCamera::HikCamera()
{
    m_handle = MV_NULL;
}

HikCamera::~HikCamera()
{
    if (m_handle)
    {
        // MV_CC_CloseDevice(m_handle);
        MV_CC_DestroyHandle(m_handle);
        m_handle = MV_NULL;
    }
}

// ch:获取SDK版本号 | en:Get SDK Version
int HikCamera::get_sdk_version()
{
    return MV_CC_GetSDKVersion();
}

// ch:枚举设备 | en:Enumerate Device
int HikCamera::enum_devices(MV_CC_DEVICE_INFO_LIST *device_list)
{
    return MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, device_list);
}

// ch:判断设备是否可达 | en:Is the device accessible
bool HikCamera::is_device_accessible(MV_CC_DEVICE_INFO *device_info, unsigned int access_tag)
{
    return MV_CC_IsDeviceAccessible(device_info, access_tag);
}

int HikCamera::open(MV_CC_DEVICE_INFO *device_info)
{
    if (MV_NULL == device_info)
    {
        return MV_E_PARAMETER;
    }

    if (m_handle)
    {
        return MV_E_CALLORDER;
    }

    int Ret = MV_CC_CreateHandle(&m_handle, device_info);
    if (MV_OK != Ret)
    {
        return Ret;
    }

    Ret = MV_CC_OpenDevice(m_handle);
    if (MV_OK != Ret)
    {
        MV_CC_DestroyHandle(m_handle);
        m_handle = MV_NULL;
    }

    // 获取帧信息
    /* Ret = start_grabbing();
    Ret = get_image_buffer(&m_outframe, 1000);
    Ret = free_image_buffer(&m_outframe);
    Ret = stop_grabbing(); */

    return Ret;
}

// ch:关闭设备 | en:Close Device
int HikCamera::close()
{
    if (MV_NULL == m_handle)
    {
        return MV_E_HANDLE;
    }

    MV_CC_CloseDevice(m_handle);

    int Ret = MV_CC_DestroyHandle(m_handle);
    m_handle = MV_NULL;

    memset(&m_outframe, 0, sizeof(MV_FRAME_OUT));

    return Ret;
}

// ch : 判断相机是否处于连接状态 | en : Is The Device Connected
bool HikCamera::is_device_connected()
{
    return MV_CC_IsDeviceConnected(m_handle);
}

// ch:注册图像数据回调 | en:Register Image Data CallBack
int HikCamera::register_image_callback(void(__stdcall *output)(unsigned char *data, MV_FRAME_OUT_INFO_EX *frame_info, void *user), void *user)
{
    return MV_CC_RegisterImageCallBackEx(m_handle, output, user);
}

// ch:开启抓图 | en:Start Grabbing
int HikCamera::start_grabbing()
{
    return MV_CC_StartGrabbing(m_handle);
}

// ch:停止抓图 | en:Stop Grabbing
int HikCamera::stop_grabbing()
{
    return MV_CC_StopGrabbing(m_handle);
}

// ch:主动获取一帧图像数据 | en:Get one frame initiatively
int HikCamera::get_image_buffer(int msec)
{
    return MV_CC_GetImageBuffer(m_handle, &m_outframe, msec);
}

// ch:释放图像缓存 | en:Free image buffer
int HikCamera::free_image_buffer()
{
    return MV_CC_FreeImageBuffer(m_handle, &m_outframe);
}

// ch:设置显示窗口句柄 | en:Set Display Window Handle
int HikCamera::display_one_frame(MV_DISPLAY_FRAME_INFO *display_info)
{
    return MV_CC_DisplayOneFrame(m_handle, display_info);
}

// ch:设置SDK内部图像缓存节点个数 | en:Set the number of the internal image cache nodes in SDK
int HikCamera::set_image_nodenum(unsigned int num)
{
    return MV_CC_SetImageNodeNum(m_handle, num);
}

// ch:获取设备信息 | en:Get device information
int HikCamera::get_device_info(MV_CC_DEVICE_INFO *device_info)
{
    return MV_CC_GetDeviceInfo(m_handle, device_info);
}

// ch:获取GEV相机的统计信息 | en:Get detect info of GEV camera
int HikCamera::get_GEV_all_match_info(MV_MATCH_INFO_NET_DETECT *match_info_net_detect)
{
    if (MV_NULL == match_info_net_detect)
    {
        return MV_E_PARAMETER;
    }

    MV_CC_DEVICE_INFO stDevInfo = {0};
    get_device_info(&stDevInfo);
    if (stDevInfo.nTLayerType != MV_GIGE_DEVICE)
    {
        return MV_E_SUPPORT;
    }

    MV_ALL_MATCH_INFO struMatchInfo = {0};

    struMatchInfo.nType = MV_MATCH_TYPE_NET_DETECT;
    struMatchInfo.pInfo = match_info_net_detect;
    struMatchInfo.nInfoSize = sizeof(MV_MATCH_INFO_NET_DETECT);
    memset(struMatchInfo.pInfo, 0, sizeof(MV_MATCH_INFO_NET_DETECT));

    return MV_CC_GetAllMatchInfo(m_handle, &struMatchInfo);
}

// ch:获取U3V相机的统计信息 | en:Get detect info of U3V camera
int HikCamera::get_U3V_all_match_info(MV_MATCH_INFO_USB_DETECT *match_info_usb_detect)
{
    if (MV_NULL == match_info_usb_detect)
    {
        return MV_E_PARAMETER;
    }

    MV_CC_DEVICE_INFO stDevInfo = {0};
    get_device_info(&stDevInfo);
    if (stDevInfo.nTLayerType != MV_USB_DEVICE)
    {
        return MV_E_SUPPORT;
    }

    MV_ALL_MATCH_INFO struMatchInfo = {0};

    struMatchInfo.nType = MV_MATCH_TYPE_USB_DETECT;
    struMatchInfo.pInfo = match_info_usb_detect;
    struMatchInfo.nInfoSize = sizeof(MV_MATCH_INFO_USB_DETECT);
    memset(struMatchInfo.pInfo, 0, sizeof(MV_MATCH_INFO_USB_DETECT));

    return MV_CC_GetAllMatchInfo(m_handle, &struMatchInfo);
}

// ch:获取和设置Int型参数，如 Width和Height，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
// en:Get Int type parameters, such as Width and Height, for details please refer to MvCameraNode.xlsx file under SDK installation directory
int HikCamera::get_int_value(IN const char *strkey, OUT MVCC_INTVALUE_EX *int_value)
{
    return MV_CC_GetIntValueEx(m_handle, strkey, int_value);
}

int HikCamera::set_int_value(IN const char *strkey, IN int64_t value)
{
    return MV_CC_SetIntValueEx(m_handle, strkey, value);
}

// ch:获取和设置Enum型参数，如 PixelFormat，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
// en:Get Enum type parameters, such as PixelFormat, for details please refer to MvCameraNode.xlsx file under SDK installation directory
int HikCamera::get_enum_value(IN const char *strkey, OUT MVCC_ENUMVALUE *enum_value)
{
    return MV_CC_GetEnumValue(m_handle, strkey, enum_value);
}

int HikCamera::set_enum_value(IN const char *strkey, IN unsigned int value)
{
    return MV_CC_SetEnumValue(m_handle, strkey, value);
}

int HikCamera::set_enum_value_by_string(IN const char *strkey, IN const char *value)
{
    return MV_CC_SetEnumValueByString(m_handle, strkey, value);
}

// ch:获取和设置Float型参数，如 ExposureTime和Gain，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
// en:Get Float type parameters, such as ExposureTime and Gain, for details please refer to MvCameraNode.xlsx file under SDK installation directory
int HikCamera::get_float_value(IN const char *strkey, OUT MVCC_FLOATVALUE *float_value)
{
    return MV_CC_GetFloatValue(m_handle, strkey, float_value);
}

int HikCamera::set_float_value(IN const char *strkey, IN float value)
{
    return MV_CC_SetFloatValue(m_handle, strkey, value);
}

// ch:获取和设置Bool型参数，如 ReverseX，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
// en:Get Bool type parameters, such as ReverseX, for details please refer to MvCameraNode.xlsx file under SDK installation directory
int HikCamera::get_bool_value(IN const char *strkey, OUT bool *value)
{
    return MV_CC_GetBoolValue(m_handle, strkey, value);
}

int HikCamera::set_bool_value(IN const char *strkey, IN bool value)
{
    return MV_CC_SetBoolValue(m_handle, strkey, value);
}

// ch:获取和设置String型参数，如 DeviceUserID，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件UserSetSave
// en:Get String type parameters, such as DeviceUserID, for details please refer to MvCameraNode.xlsx file under SDK installation directory
int HikCamera::get_string_value(IN const char *strkey, MVCC_STRINGVALUE *string_value)
{
    return MV_CC_GetStringValue(m_handle, strkey, string_value);
}

int HikCamera::set_string_value(IN const char *strkey, IN const char *strvalue)
{
    return MV_CC_SetStringValue(m_handle, strkey, strvalue);
}

// ch:执行一次Command型命令，如 UserSetSave，详细内容参考SDK安装目录下的 MvCameraNode.xlsx 文件
// en:Execute Command once, such as UserSetSave, for details please refer to MvCameraNode.xlsx file under SDK installation directory
int HikCamera::command_execute(IN const char *strkey)
{
    return MV_CC_SetCommandValue(m_handle, strkey);
}

// ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
int HikCamera::get_optimal_packet_size(unsigned int *optimal_packet_size)
{
    if (MV_NULL == optimal_packet_size)
    {
        return MV_E_PARAMETER;
    }

    int Ret = MV_CC_GetOptimalPacketSize(m_handle);
    if (Ret < MV_OK)
    {
        return Ret;
    }

    *optimal_packet_size = (unsigned int)Ret;

    return MV_OK;
}

// ch:注册消息异常回调 | en:Register Message Exception CallBack
int HikCamera::register_exception_callback(void(__stdcall *exception)(unsigned int msg_type, void *user), void *user)
{
    return MV_CC_RegisterExceptionCallBack(m_handle, exception, user);
}

// ch:注册单个事件回调 | en:Register Event CallBack
int HikCamera::register_event_callback(const char *event_name, void(__stdcall *event)(MV_EVENT_OUT_INFO *event_info, void *user), void *user)
{
    return MV_CC_RegisterEventCallBackEx(m_handle, event_name, event, user);
}

// ch:强制IP | en:Force IP
int HikCamera::force_ip(unsigned int ip, unsigned int sub_net_mask, unsigned int default_gate_way)
{
    return MV_GIGE_ForceIpEx(m_handle, ip, sub_net_mask, default_gate_way);
}

// ch:配置IP方式 | en:IP configuration method
int HikCamera::set_ip_config(unsigned int type)
{
    return MV_GIGE_SetIpConfig(m_handle, type);
}

// ch:设置网络传输模式 | en:Set Net Transfer Mode
int HikCamera::set_net_trans_mode(unsigned int type)
{
    return MV_GIGE_SetNetTransMode(m_handle, type);
}

// ch:像素格式转换 | en:Pixel format conversion
int HikCamera::convert_pixel_type(MV_CC_PIXEL_CONVERT_PARAM *cvt_param)
{
    return MV_CC_ConvertPixelType(m_handle, cvt_param);
}

// ch:保存图片 | en:save image
int HikCamera::save_image(MV_SAVE_IMAGE_PARAM_EX3 *param)
{
    return MV_CC_SaveImageEx3(m_handle, param);
}

// 保存图片到文件地址
int HikCamera::save_image_to_file(MV_SAVE_IMAGE_TO_FILE_PARAM_EX *save_file_param)
{
    return MV_CC_SaveImageToFileEx(m_handle, save_file_param);
}

// get frame timeout
int HikCamera::get_frame_timeout(IN OUT unsigned char *data, IN unsigned int data_size, IN OUT MV_FRAME_OUT_INFO_EX *frame_info, IN unsigned int msec)
{
    return MV_CC_GetOneFrameTimeout(m_handle, data, data_size, frame_info, msec);
}

// 获取数据包（一帧图像）大小
int HikCamera::get_payload_size(OUT unsigned int payloadsize)
{
    int Ret = MV_OK;
    MVCC_INTVALUE_EX stParam;
    memset(&stParam, 0, sizeof(MVCC_INTVALUE_EX));
    // 使用camera新的接口
    Ret = MV_CC_GetIntValueEx(m_handle, "PayloadSize", &stParam);
    payloadsize = stParam.nCurValue;
    return Ret;
}

// 获取当前图像缓存区的有效图像个数
int HikCamera::get_valid_image_num(unsigned int *valid_image_num)
{
    return MV_CC_GetValidImageNum(m_handle, valid_image_num);
}
