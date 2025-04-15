#ifndef TERAIN_SYSTEM_H
#define TERAIN_SYSTEM_H

#include "chrono/core/ChVector3.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector2.h"

using namespace chrono;

class TerrainSystemCoordinates
{
public:
    double sizeX;
    double sizeY;
    double delta;
    ChVector2d bottom_right;
    double z_offset;

    bool visualization_mesh;
    TerrainSystemCoordinates(
        double sizeX = 0,
        double sizeY = 0,
        double z_offset = 0,
        ChVector2d bottom_right = ChVector2d(0, 0))
    {
        this->sizeX = sizeX;
        this->sizeY = sizeY;
        this->bottom_right = bottom_right;
        this->z_offset = z_offset;
    };

    void update(double sizeX, double sizeY, double z_offset, ChVector2d bottom_right)
    {
        this->sizeX = sizeX;
        this->sizeY = sizeY;
        this->bottom_right = bottom_right;
        this->z_offset = z_offset;
    }

    ChVector3d convertUEToChrono(ChVector3d point)
    {
        double x = (point.x() - bottom_right.x()) / 100 - sizeX / 2;
        double y = (bottom_right.y() - point.y()) / 100 + sizeY / 2;
        double z = point.z() / 100 + z_offset;

        return ChVector3d(x, y, z);
    }

    ChVector3d convertRosToChronoDirection(ChVector3d point)
    {
        double x = point.x();
        double y = -point.y();
        double z = point.z();
        return ChVector3d(x, y, z);
    }

    ChVector3d convertChronoToROSDirection(ChVector3d point)
    {
        double x = point.x();
        double y = -point.y();
        double z = point.z();
        return ChVector3d(x, y, z);
    }

    ChVector3d convertRosToChrono(ChVector3d point)
    {
        ChVector3d new_point = convertRosToChronoDirection(point);
        double x = (new_point.x() - (bottom_right.x()/ 100)) - sizeX / 2;
        double y = (bottom_right.y()/ 100 + new_point.y())  + sizeY / 2;
        double z = new_point.z() + z_offset;
        return ChVector3d(x, y, z);

    }

    ChVector3d convertChronoToUE(ChVector3d point)
    {
        //std::cout << "Converting Chrono to UE: " << point.x() << " " << point.y() << " " << point.z() << std::endl;
        double x = (point.x() + sizeX / 2) * 100 + bottom_right.x();
        double y = bottom_right.y() - (point.y() - sizeY / 2) * 100;
        double z = (point.z() - z_offset) * 100;
        //std::cout << "Converted Result: " << x << " " << y << " " << z << std::endl;
        return ChVector3d(x, y, z);
    }

    ChVector3d convertChronoToROS(ChVector3d point)
    {
        ChVector3d new_point = convertChronoToUE(point);
        double x = new_point.x()/100;
        double y = -new_point.y()/100;
        double z = new_point.z()/100;
        return ChVector3d(x, y, z);
    }

    ChVector3d convertChronotToROSRotation(ChVector3d point)
    {
        ChVector3d new_rotation = convertChronoToUERotation(point);
        double x = new_rotation.x();
        double y = new_rotation.z();
        double z = -new_rotation.y();
        return ChVector3d(x, y, z);
    }

    ChVector3d convertChronoToUERotation(ChVector3d point)
    {
        //std::cout << "Converting Chrono to UE Rotation: " << point.x() << " " << point.y() << " " << point.z() << std::endl;
        double x = point.x();
        double y = -point.y();
        double z = -point.z();
        //std::cout << "Converted Result: " << x << " " << y << " " << z << std::endl;
        return ChVector3d(x, y, z);
    }

    ~TerrainSystemCoordinates() {}
};
#endif