#include "road_risk_model.h"
#include "car_risk_model.h"
#include "draw.hpp"
#include "ego.h"

#include <string>
#include <cmath>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <assert.h>

using namespace riskfield;
using namespace cv;

/** ego **/
vector<float> ego_dimension{4, 2.5, 2};  // l w h
vector<float> ego_location{105, 50, 1};  // x,y,z
float ego_y = 0*M_PI/180;
float ego_v = 20;
float ego_a =0.2;

/** obs **/
vector<float> o1_dimension{4, 2.5, 2};
vector<float> o1_location{113, 60, 1};
float o1_y = 30*M_PI/180;
float o1_v = 15;
float o1_a = 3;
string o1_type = "Car";


vector<float> o2_dimension{4, 2.5, 2};
vector<float> o2_location{105, 100, 1};
float o2_y = 0*M_PI/180;
float o2_v = 25;
float o2_a = 5;
string o2_type = "Car";

vector<vector<string>> readF() {
    cout << "in readF" << endl;
    vector<vector<string>> user_arr;
    ifstream fp("/home/tan/AV_risk_model/src/US-101.csv"); //定义声明一个ifstream对象，指定文件路径
    string line;
    getline(fp,line); //跳过列名，第一行不做处理
    // cout << line << endl;
    while (getline(fp,line)){ //循环读取每行数据
        // cout << line << endl;
        vector<string> data_line;
        string number;
		// 存成二维表结构
		stringstream ss(line);//string数据流化
        //将一行数据按'，'分割
        while (getline(ss, number, ',')) { 
            data_line.push_back(number);
        }
        user_arr.push_back(data_line); //插入到vector中
    }
    return user_arr;
}

void print(vector<vector<string>> &v) {
    for (int i = 0; i < (int)v.size(); i++) {
        for (int j = 0; j < (int)v[i].size(); j++) {
            cout << v[i][j] << " ";
        }
        cout << endl;
    }
}

int main() {
    // vector<vector<string>> car_data = readF();
    // assert(car_data.size() > 0 && car_data[0].size() > 0);
    // cout << car_data.size() << " " << car_data[0].size() << endl;
    // print(car_data);

    EgoCar::refresh_ego(ego_dimension, ego_location, ego_y, ego_v, ego_a);
    ObsCar* o1 = new ObsCar(o1_dimension, o1_location, o1_y, o1_v, o1_a, o1_type);
    ObsCar* o2 = new ObsCar(o2_dimension, o2_location, o2_y, o2_v, o2_a, o2_type);

    Lane* l1 = new Lane(0,100,100,40,0,100,"boundary");
    Lane* l2 = new Lane(100,100,100,40,30*M_PI/180,100,"boundary");
    Lane* l3 = new Lane(0,100,100,40,0,110,"white lane");
    Lane* l4 = new Lane(100,100,100,40,30*M_PI/180,110,"white lane");
    Lane* l5 = new Lane(0,100,100,40,0,120,"yellow lane");
    Lane* l6 = new Lane(100,100,100,40,30*M_PI/180,120,"yellow lane");
    vector<Lane*> lane_list = {l1, l2, l3, l4, l5, l6};

    Mat mapRoad = Mat::zeros(320, 480, CV_32FC3);
    Mat mapCar = Mat::zeros(320, 480, CV_32FC3);
    Mat mapT = Mat::zeros(320, 480, CV_32FC3);
    Mat map = Mat::zeros(320, 480, CV_8UC3);
    // imshow("ori map", map);
    // waitKey();

    /** draw **/
    Draw *draw_pen = new Draw();

    /** road model **/
    RoadModel* road_model = new RoadModel(lane_list);  // the width of the road model

    /** car model **/
    vector<ObsCar*> obslist = {o1, o2};
    CarModel* car_model = new CarModel(obslist);

    /** Lane **/
    // road_model->buildLaneModel(mapRoad);
    // draw_pen->norm2draw(mapRoad);
    // draw_pen->printMap(mapRode);

    /** car **/
    car_model->carModel(mapCar);
    draw_pen->norm2draw(mapCar);
    // draw_pen->printMap(mapCar);

    mapT = mapRoad + mapCar;
 
    // draw_pen->norm2draw(mapT);
    draw_pen->convert(mapT, map);
    // draw_pen->printMap(map);

    cv::circle(map, Point(50,105), 3, Scalar(10,10, 200), FILLED);

    imshow("risk distribution", map);
    waitKey();

    delete road_model;
    delete car_model;
    delete draw_pen;


    return 0;
}