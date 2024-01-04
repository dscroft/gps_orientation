#include <string>
#include <list>

#include <ros/ros.h>
#include <ros/console.h> 

#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/NavSatFix.h>
#include <geodesy/utm.h>
#include <sensor_msgs/Imu.h>

static const size_t maxHistory = 100;
static const std::string nodeName = "gps_pose_estimate" ;

class GpsCallback
{
private:
    ros::NodeHandle nh;
    ros::Subscriber _sub;
    ros::Publisher _pub;

    std::list<geodesy::UTMPoint> _historicPoints;

    auto  _find_match( const geodesy::UTMPoint& point,
            const float distThreshSqrd = 1.0 )
    {
        auto matchPred = [&]( const geodesy::UTMPoint& p ) -> bool
        {
            return geodesy::sameGridZone( point, p ) &&
                    std::pow( p.easting - point.easting, 2 ) + 
                    std::pow( p.northing - point.northing, 2 ) < distThreshSqrd;
        };

        return std::find_if( std::rbegin(this->_historicPoints), 
                std::rend(this->_historicPoints),
                matchPred );
    }

public:

    static float seperation2( geodesy::UTMPoint a, 
                        geodesy::UTMPoint b )
    {
        if( !geodesy::sameGridZone( a, b ) )
        {
            return std::numeric_limits<float>::max();
        }

        return std::pow( a.easting - b.easting, 2 ) + 
                std::pow( a.northing - b.northing, 2 );
    }

    static geometry_msgs::Quaternion quat_from_yaw( const float yaw )
    {
        tf2::Quaternion tf2Quat;
        tf2Quat.setRPY( 0, 0, yaw );
    
        return tf2::toMsg( tf2Quat );
    }

    static float get_theta( geodesy::UTMPoint from, 
                        geodesy::UTMPoint to )
    {
        return std::atan2( to.northing - from.northing, 
                        to.easting - from.easting );
    }

    // paramater ros navsatfix
    void gps_callback( const sensor_msgs::NavSatFix &msg )
    {
        //ROS_INFO_STREAM( msg );

        geographic_msgs::GeoPoint geoPoint;
        geoPoint.latitude = msg.latitude;
        geoPoint.longitude = msg.longitude;
        geoPoint.altitude = msg.altitude;

        geodesy::UTMPoint utmPoint( geoPoint );

        const float nearThresh = std::pow( 1, 2 );
        const float farThresh = nearThresh * 2;
        auto matchPred = [&]( const geodesy::UTMPoint& p ) -> bool
        {
            if( !geodesy::sameGridZone( utmPoint, p ) )
                return false;

            const float dist = std::pow( p.easting - utmPoint.easting, 2 ) + 
                    std::pow( p.northing - utmPoint.northing, 2 );

            return dist >= nearThresh;
        }; 

        ROS_INFO_STREAM( "historicPoints: " << this->_historicPoints.size() );

        auto matchIt = std::find_if( std::begin(this->_historicPoints), 
                std::end(this->_historicPoints),
                matchPred );

        if( matchIt != std::end(this->_historicPoints) )
        {
            ROS_INFO_STREAM( "Match" );
            const float yaw = get_theta( *matchIt, utmPoint );
            ROS_INFO_STREAM( "theta: " << yaw );

            sensor_msgs::Imu out;
            out.header = msg.header;
            out.orientation = quat_from_yaw( yaw );

            this->_pub.publish( out );

            this->_historicPoints.erase( std::next(matchIt), std::end(this->_historicPoints) );
        }

        const int maxHistory = 1000;
        if( this->_historicPoints.size() >= maxHistory )
        {
            this->_historicPoints.erase( 
                    std::next(std::begin(this->_historicPoints), maxHistory), 
                    std::end(this->_historicPoints) );
        }

        this->_historicPoints.emplace_front( utmPoint );
    }

    GpsCallback( const std::string topic ) : nh( "~" )
    {
        this->_sub = this->nh.subscribe( topic, 10, &GpsCallback::gps_callback, this );
        this->_pub = this->nh.advertise<sensor_msgs::Imu>( "/gps/heading", 10 );
    }
};

int main( int argc, char* argv[] )
{
    /* Initialisation */
    ros::init( argc, argv, nodeName );
    ROS_INFO_STREAM( nodeName << " running") ;
    ros::NodeHandle nh( "~" ) ;

    GpsCallback gpsCb( "/gps/fix" );
    
    //ros::Subscriber sub = nh.subscribe( "/odom", 10, &odom_callback );

    //pub = nh.advertise<nav_msgs::Odometry>( "/odom/pose", 10 );

    /*while( ros::ok() )
    {
       ros::sp
    }*/
    ros::spin();

    return 0 ;
}
