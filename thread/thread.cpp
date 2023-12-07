#include "thread.hpp"

////
bool producer(Factory<TaskData> &factory, MessageFilter<MCUData> &receive_factory,
              std::chrono::_V2::steady_clock::time_point time_start){
    GX_STATUS status = GX_STATUS_SUCCESS;
    /*
     *First init: Implementation of GxCamera and SetInitialArmor it
    */
    GxCamera gxCam;
    status = gxCam.initLib();
    GX_VERIFY(status);
    /*
     *Second SetInitialArmor: Open Camera by SN/Index
    */
    //status = gxCam.openDeviceBySN(SN_sentry_below);	//By SN
    status = gxCam.openDeviceByIndex("1");		//By Index
    GX_VERIFY(status);
    /*
     *Third SetInitialArmor: Set Camera Params: ROI, Exposure, Gain, WhiteBalance
    */
    gxCam.setRoiParam(1280, 1024, 0, 0);				// ROI
    gxCam.setExposureParam(20000, false, 10000, 30000);	// Exposure
    gxCam.setGainParam(0, false, 0, 10);				// Gain
    gxCam.setWhiteBalanceOn(true);						// WhiteBalance
    status = gxCam.startAcquiring();					// Send Start Acquisition Command
    GX_VERIFY(status);
    while(true)
    {
        TaskData src;
        auto time_cap=std::chrono::_V2::steady_clock::now();
        status = gxCam.snapCvMat(src.img);
        if(src.img.empty())
        {
            continue;
        }
        src.timestamp = (int) (std::chrono::duration<double, std::milli>(time_cap - time_start).count());
#ifdef USING_SERIAL
        //获取下位机数据
        MCUData mcu_status;
        if (!receive_factory.consume(mcu_status, src.timestamp))
            continue;
        src.quat = mcu_status.quat;
//        src.mode = mcu_status.mode;
        src.mode = 3;
        src.bullet_speed = mcu_status.bullet_speed;
#endif
        factory.produce(src);

    }
    return true;
}

bool consumer(Factory<TaskData> &task_factory, Factory<VisionData> &transmit_factory)
{
//    Autoaim autoaim;
//    Buff buff;
    auto mode = -1;
    auto last_mode = -1;
    while (1) {
        TaskData dst;
        VisionData data;

        task_factory.consume(dst);
        mode = dst.mode;
#ifndef USING_SERIAL
        mode = 1;
        // dst.mode = mode;
#endif // DEBUG_WITHOUT_COM
        if (mode == 1 || mode == 2) {
//            autoaim.run(dst, data);
            transmit_factory.produce(data);
        } else if (mode == 3 || mode == 4) {
//            buff.run(dst, data);
            transmit_factory.produce(data);
        }
    }
    return true;
}

/**
 * @brief 数据发送线程
 *
 * @param serial SerialPort类
 * @param transmit_factory Factory类
 * @return true
 * @return false
 */
bool dataTransmitter(SerialPort &serial, Factory<VisionData> &transmit_factory) {
    while (1) {
        VisionData transmit;
        transmit_factory.consume(transmit);
        //若串口离线则跳过数据发送
        //TODO:使用无串口的模式时会导致此线程死循环，浪费CPU性能
        if (serial.need_init == true) {
            // cout<<"offline..."<<endl;
#ifndef DEBUG_WITHOUT_COM
#ifdef SAVE_LOG_ALL
            LOG(ERROR) << "[TRANSMITTER] Serial offline, trying to reconnect...";
#endif //SAVE_LOG_ALL
#endif //DEBUG_WITHOUT_COM
            usleep(5000);
            continue;
        }
        serial.TransformData(transmit);
        serial.send();
        // cout<<"transmitting..."<<endl;
    }
    return true;
}



///////串口接收 此处进入debug
/**
 * @brief 数据接收线程
 *
 * @param serial
 * @param receive_factory
 * @param time_start
 * @return true
 * @return false
 */
bool dataReceiver(SerialPort &serial, MessageFilter<MCUData> &receive_factory,
                  std::chrono::_V2::steady_clock::time_point time_start) {

    while (1) {
        //若串口离线则跳过数据发送
        if (serial.need_init == true) {
//             cout<<"offline..."<<endl;
            usleep(5000);
            continue;
        }
        //数据读取不成功进行循环
        while (!serial.get_Mode());
        auto time_cap = std::chrono::steady_clock::now();
        auto timestamp = (int) (std::chrono::duration<double, std::milli>(time_cap - time_start).count());
//        cout << "Quat: " << serial.quat[0] << " " << serial.quat[1] << " " << serial.quat[2] << " " << serial.quat[3]
//             << " " << endl;
        int mode = serial.mode;
//        fmt::print(fmt::fg(fmt::color(fmt::color::pale_golden_rod)), "mode: {} \n", mode);
        float bullet_speed = serial.bullet_speed;
        int color = serial.color;
        // int mode = 2;
        Eigen::Quaterniond quat = {serial.quat[0], serial.quat[1], serial.quat[2], serial.quat[3]};
//        Eigen::Vector3d acc = {serial.acc[0], serial.acc[1], serial.acc[2]};;
//        Eigen::Vector3d gyro = {serial.gyro[0], serial.gyro[1], serial.gyro[2]};;
        MCUData mcu_status = {mode, quat, bullet_speed, color, timestamp};
        receive_factory.produce(mcu_status, timestamp);
//         Eigen::Vector3d vec = quat.toRotationMatrix().eulerAngles(2,1,0);
//         cout<<"Euler : "<<vec[0] * 180.f / CV_PI<<" "<<vec[1] * 180.f / CV_PI<<" "<<vec[2] * 180.f / CV_PI<<endl;
        // cout<<"transmitting..."<<endl;
    }
    return true;
}

/**
 * @brief 串口监视线程
 *
 * @param serial
 * @return true
 * @return false
 */
bool serialWatcher(SerialPort &serial) {
    int last = 0;
#ifdef DEBUG_WITHOUT_COM
    #ifdef SAVE_TRANSMIT_LOG
    LOG(WARNING)<<"[SERIAL] Warning: You are not using Serial port";
#endif //SAVE_TRANSMIT_LOG
#endif // DEBUG_WITHOUT_COM

    while (1) {
        sleep(1);
        //检测文件夹是否存在或串口需要初始化
        if (access(serial.device.path.c_str(), F_OK) == -1 || serial.need_init) {
            serial.need_init = true;
#ifdef DEBUG_WITHOUT_COM
            int now = clock() / CLOCKS_PER_SEC;
            if (now - last > 10) {
                last = now;
                fmt::print(fmt::fg(fmt::color::orange), "[SERIAL] Warning: You are not using Serial port\n");
            }
            serial.withoutSerialPort();
#else
            bool success = serial.initSerialPort();
            if (!success) {
                std::cerr << "Failed to initialize serial port." << std::endl;
                return -1;
            }
#endif //DEBUG_WITHOUT_COM
        }
    }
}




