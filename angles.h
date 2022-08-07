#include "sructs.h"

double angle(Vec2D v1, Vec2D v2) {
    double phi;
    if (v1.len()*v2.len() != 0 ) {
        double res = (v1.x * v2.x + v2.y*v1.y) / (v1.len() * v2.len());
        phi = acos(res);
    }
    else{
        phi = 5;
    }
    return phi;
}

int side(Vec2D v1, Vec2D v2){
    double tmp = v1.x*v2.y - v1.y*v2.x;
    if (tmp >= 0)
        tmp = 1;
    if  (tmp < 0)
        tmp= -1;
    return int(tmp);
}

