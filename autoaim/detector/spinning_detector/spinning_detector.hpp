#ifndef SPINNING_DETECTOR_HPP_
#define SPINNING_DETECTOR_HPP_


#include "../tracker/tracker.hpp"
#include "../param_struct/param_struct.hpp"
#include "../param_struct/enum.hpp"
#include "../../../tool/tool.hpp"
#include "../../../tool/param_struct.hpp"

class SpinningDetector
{
public:
    SpinningDetector();
    SpinningDetector(int color, GyroParam gyro_params);
    ~SpinningDetector();

    void createArmorTracker(std::multimap<std::string, ArmorTracker>& trackers_map,
                            std::vector<Armor>& armors, std::map<std::string, int>& new_armors_cnt_map, int64_t now);
    bool isSpinning(std::multimap<std::string, ArmorTracker>& trackers_map, int64_t now);
    bool isSpinning(std::multimap<std::string, ArmorTracker>& trackers_map, std::map<std::string, int>& new_armors_cnt_map, int64_t now);

    double max_hop_period_;
    double last_timestamp_;
    int xyz_axis_[3] = {1, 2, 0};
    GyroParam gyro_params_;
    SpinningMap spinning_map_;

private:
    Armor last_armor_;
    int detect_color_;
    DetectorInfo detector_info_;
};

#endif