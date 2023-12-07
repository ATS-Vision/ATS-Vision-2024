#include "thread/thread.hpp"
#include "serial/seial.hpp"
//#include "tool/tool.hpp"
int BAUD=115200;
const string SERIAL_ID="1a86/7523/264";
int main(int argc,char* argv[])
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = false;  //除了日志文件之外是否需要标准输出
    FLAGS_colorlogtostderr = true;  //是否启用不同颜色显示
//    google::SetLogDestination(google::GLOG_INFO,"../log/info/");  //设置日志级别
//    google::SetLogDestination(google::GLOG_WARNING,"../log/warning/");
    google::SetLogDestination(google::GLOG_ERROR,"../log/error/");
    auto time_start = std::chrono::steady_clock::now();
    Factory<TaskData> task_factory(3);
    Factory<VisionData> data_transmit_factory(5);
    MessageFilter<MCUData> data_receiver(100);
    SerialPort serial(SERIAL_ID, BAUD);

    /*****
    **优化方向 join方法放在线程下方使其确保使用的是一帧数据
    **/////
    /////C板线程
    std::thread serial_watcher(&serialWatcher, ref(serial));
    std::thread receiver(&dataReceiver, ref(serial), ref(data_receiver), time_start);
    ////不使用串口
///    std::thread serial_watcher(&serialWatcher, ref(serial));

    std::thread task_producer(&producer, ref(task_factory), ref(data_receiver), time_start);
    std::thread task_consumer(&consumer, ref(task_factory), ref(data_transmit_factory));
    std::thread transmitter(&dataTransmitter, ref(serial), ref(data_transmit_factory));

    task_producer.join();
    task_consumer.join();
    transmitter.join();
    receiver.join();


}