#include "road_risk_model.h"
#include "car_risk_model.h"
#include "draw.hpp"

#include <string>
#include <cmath>
#include <vector>

using namespace riskfield;
using namespace cv;

/** ego **/
vector<float> ego_dimension{4, 2.5, 2};  // l w h
vector<float> ego_location{202, 200, 1};
float ego_y = 0*M_PI/180;
float ego_v = 20;
float ego_a = 5;

/** obs **/
vector<float> o1_dimension{4, 2.5, 2};
vector<float> o1_location{205, 192, 1};
float o1_y = 30*M_PI/180;
float o1_v = 25;
float o1_a = 10;
string o1_type = "Car";


vector<float> o2_dimension{4, 2.5, 2};
vector<float> o2_location{202, 190, 1};
float o2_y = 0*M_PI/180;
float o2_v = 15;
float o2_a = 10;
string o2_type = "Car";



int main() {
    EgoCar::refresh_ego(ego_dimension, ego_location, ego_y, ego_v, ego_a);
    ObsCar* o1 = new ObsCar(o1_dimension, o1_location, o1_y, o1_v, o1_a, o1_type);
    ObsCar* o2 = new ObsCar(o2_dimension, o2_location, o2_y, o2_v, o2_a, o2_type);

    Mat mapRode = Mat::zeros(320, 480, CV_32FC3);
    Mat mapCar = Mat::zeros(320, 480, CV_32FC3);
    Mat map = Mat::zeros(320, 480, CV_8UC3);
    // imshow("ori map", map);
    // waitKey();

    /** draw **/
    Draw *draw_pen = new Draw();

    /** road model **/
    RoadModel* road_model = new RoadModel(40, 100);  // the width of the road model

    vector<float> y;  // yellow lane set
    vector<float> w;  // white lane set

    /** car model **/
    vector<ObsCar*> obslist = {o1, o2};
    CarModel* car_model = new CarModel(obslist);

    /** Lane **/
    road_model->LaneModel(mapRode, y, w);
    // draw_pen->printMap(mapRode);

    road_model->BoarderModel(mapRode);
    // draw_pen->printMap(mapRode);

    /** car **/
    // car_model->carModel(mapCar);
    // draw_pen->printMap(mapCar);

    // draw_pen->norm2draw(mapRode);
    draw_pen->convert(mapRode, map);
    // draw_pen->printMap(map);

    imshow("risk distribution", map);
    waitKey();

    delete road_model;
    delete car_model;
    delete draw_pen;

    return 0;
}