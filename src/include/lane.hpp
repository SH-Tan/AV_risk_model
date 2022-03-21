#ifndef __Lane_STRUCT_H
#define __Lane_STRUCT_H

#include <cmath>
#include <vector>
#include <string>

using namespace std;

namespace riskfield {

class Lane {
    public:

        Lane(const float &start_l, const float &l, const float &start_w, const float &w, const float &yaw_, const float &start, string type_ = "")
                : sl(start_l), length(l), sw(start_w), width(w), yaw(yaw_), st(start), type(type_) {}  // 构造函数

        ~Lane() {}

        float get_sl() const { return sl; }

        float get_l() const { return length; }

        float get_sw() const { return sw; }

        float get_w() const { return width; }
        
        float get_yaw() const { return yaw; }

        float get_st() const { return st; }

        string get_type() const { return type; }


    private:
        Lane(const Lane&) = delete;              // copy cotr 明确拒绝
        Lane& operator=(const Lane&) = delete;   // 明确拒绝 

        float sl;  // 长度开始坐标,图像坐标系
        float length;  // 长度
        float sw;  // 宽度开始坐标,图像坐标系
        float width;  // 宽度
        float yaw; // angle
        float st;  // 该车道线在图像坐标系上开始的坐标
        string type;

};

}  // riskfield

#endif  // __Lane_STRUCT_H
