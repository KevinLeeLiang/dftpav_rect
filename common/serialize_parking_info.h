#pragma once

#include "parking_info.h"
#include "planning_info.h"

PLANNING_NAMESPACE_START

class PlanningLogHelper {
  public:
    DEFINE_SINGLETON(PlanningLogHelper);

    PlanningLogHelper() = default;
    virtual ~PlanningLogHelper() = default;

    void log_to_file(const std::string &data);

    void init(const std::string &log_file_name);

    void reset();

    void set_N(int N);

  private:
    std::string log_file_name_;
    bool init_ = false;
    int N_ = 1;
};

class SerializeParkingInfo {
  public:
    SerializeParkingInfo();

    virtual ~SerializeParkingInfo() = default;

    void add_points(std::string topic_name, const Vec2d point);

    void add_points(std::string topic_name, double x, double y, double v);

    void add_extra_info(std::string key, std::string value);

    void clear();

    void show_json_string(ParkingInfo::Ptr parking_info);

    void show_json_string(PlanningInfo::Ptr planning_info);

  private:
    // topic命名规则: 坐标系_topic名字
    // x,y,v
    std::map<std::string, std::vector<std::tuple<double, double, double>>> _points;
    std::vector<std::pair<std::string, std::string>> _extra_info;
};

PLANNING_NAMESPACE_END
