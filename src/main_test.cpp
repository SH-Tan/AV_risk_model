#include "road_risk_model.h"
#include "car_risk_model.h"

#include <string>
#include <cmath>
#include <vector>

using namespace riskfield;
using namespace cv;

/** ego **/
vector<float> ego_dimension{12,2,2};
vector<float> ego_location{4,2,2};
float ego_y = 0*M_PI/180;
float ego_v = 20;
float ego_a = 5;

/** obs **/
vector<float> o1_dimension{7,6,2};
vector<float> o1_location{4,2,2};
float o1_y = -30*M_PI/180;
float o1_v = 25;
float o1_a = 10;
string o1_type = "Car";


vector<float> o2_dimension{16,2,2};
vector<float> o2_location{4,2,2};
float o2_y = 0*M_PI/180;
float o2_v = 15;
float o2_a = 10;
string o2_type = "Car";



int main() {
    EgoCar::refresh_ego(ego_dimension, ego_location, ego_y, ego_v, ego_a);
    ObsCar* o1 = new ObsCar(o1_dimension, o1_location, o1_y, o1_v, o1_a, o1_type);
    ObsCar* o2 = new ObsCar(o2_dimension, o2_location, o2_y, o2_v, o2_a, o2_type);

    Mat map = Mat::zeros(1200, 800, CV_8UC3);

    /** road model **/
    RoadModel* road_model = new RoadModel(16);  // the width of the road model

    /** car model **/
    vector<ObsCar*> obslist = {o1, o2};
    CarModel* car_model = new CarModel(obslist);

    road_model->LaneModel(map);
    road_model->BoarderModel(map);

    car_model->carModel(map);

    imshow("risk distribution", map);
    waitKey();

    return 0;
}