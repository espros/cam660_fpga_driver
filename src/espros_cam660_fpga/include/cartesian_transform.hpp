#ifndef __ESPROS_CARTESIAN_TRANSFORM_H__
#define __ESPROS_CARTESIAN_TRANSFORM_H__

#include <cstdint>
#include <vector>

namespace espros {

typedef unsigned int uint;

class CartesianTransform
{
public:

    enum LensType { WIDE_FIELD = 0, STANDARD_FIELD, NARROW_FIELD };

    CartesianTransform();
    ~CartesianTransform();    
    void transformPixel(uint srcX, uint srcY, double srcZ, double &destX, double &destY, double &destZ);
    void initLensTransform(double sensorPointSizeMM, int width, int height, int offsetX, int offsetY, int lensType);


private:

    int distortionTableSize;
    int numCols;
    int numRows;
    double angle[101];
    double rp[101];
    double xUA[320][240];
    double yUA[320][240];
    double zUA[320][240];

    double getAngle(double x, double y, double sensorPointSizeMM);
    double interpolate(double x_in, double x0, double y0, double x1, double y1);
    void initLensDistortionTable(LensType lensType);

};


} //end namespace espros

#endif // __ESPROS_CARTESIAN_TRANSFORM_H__
