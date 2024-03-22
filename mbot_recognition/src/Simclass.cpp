#include "Simclass.h"

void Sim::Initialization(inParam_sim inParam) {
    this->sensor_type = inParam.sensor_type;
    this->Q_search = inParam.Q_search;
    this->Q_tracking = inParam.Q_tracking;
}
