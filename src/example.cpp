#include "example_data.h"

#include <geodesy/utm.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <iostream>
#include <optional>

geodesy::UTMPoint get_utm_point( const Field& field )
{
    geographic_msgs::GeoPoint geoPoint;
    geoPoint.latitude = field.latitude;
    geoPoint.longitude = field.longitude;
    geoPoint.altitude = field.altitude;
    return geodesy::UTMPoint( geoPoint );
}

geometry_msgs::Point get_point( const Field& field )
{
    return geodesy::toGeometry( get_utm_point( field ) );
}

double get_heading( geodesy::UTMPoint from, geodesy::UTMPoint to )
{
    const double eastingDelta = to.easting - from.easting;
    const double northingDelta = to.northing - from.northing;

    return std::atan2( northingDelta, eastingDelta );
}

geometry_msgs::Quaternion get_heading_quaternion( geodesy::UTMPoint from, geodesy::UTMPoint to )
{
    tf2::Quaternion tf2Quat;
    tf2Quat.setRPY( 0, 0, get_heading( from, to ) );

    return tf2::toMsg( tf2Quat );
}

int main()
{
    std::optional< geodesy::UTMPoint > origin;
    std::optional< geodesy::UTMPose > originPose;

    for( const auto& field : data )
    {
        auto utmPoint = get_utm_point( field );

        // set initial position
        if( !origin ) origin = utmPoint;
        else if( !originPose )
        {
            originPose = geodesy::UTMPose( *origin, get_heading_quaternion( *origin, utmPoint ) );

            std::cout << *originPose << std::endl;
        }

        //std::cout << utmPoint << std::endl;

        auto originPoint = geodesy::toGeometry( *origin );
        auto point = geodesy::toGeometry( utmPoint );

        //std::cout << "origin: " << originPoint << std::endl;
        //std::cout << "point: " << point << std::endl;

        auto deltaPoint = point;
        deltaPoint.x -= originPoint.x;
        deltaPoint.y -= originPoint.y;
        deltaPoint.z -= originPoint.z;

        //std::cout << "delta: " << deltaPoint << std::endl;
        std::cout << std::fixed << std::setprecision( 8 ) << deltaPoint.x << ", " << deltaPoint.y << ", " << deltaPoint.z << std::endl;

    }

    return 0;
}