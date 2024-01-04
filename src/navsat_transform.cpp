#include <string>
#include <list>
#include <optional>

#include <ros/ros.h>
#include <ros/console.h> 

#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/NavSatFix.h>
#include <geodesy/utm.h>
#include <sensor_msgs/Imu.h>

static const size_t maxHistory = 100;
static const std::string nodeName = "navsat_transform" ;

class GpsCallback
{
private:
    ros::NodeHandle nh;
    ros::Subscriber _sub;
    ros::Publisher _pub;

    std::optional< geometry_msgs::Pose > _origin;
    std::list<geometry_msgs::Point> _historicPoints;

    decltype(_historicPoints)::iterator
    _find_match( const geometry_msgs::Point& point )
    {
        const float nearThresh = std::pow( 1, 2 );
        const float farThresh = nearThresh * 2;
        auto matchPred = [&]( const geometry_msgs::Point& p ) -> bool
        {
            const float dist = std::pow( p.x - point.x, 2 ) + 
                    std::pow( p.y - point.y, 2 );

            return dist >= nearThresh;
        }; 

        return std::find_if( std::begin(this->_historicPoints), 
                std::end(this->_historicPoints),
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

    static float get_theta( geometry_msgs::Point from, 
                        geometry_msgs::Point to )
    {
        return std::atan2( to.y - from.y, 
                        to.x - from.x );
    }

    static geodesy::UTMPoint get_utm_point( const sensor_msgs::NavSatFix& msg )
    {
        geographic_msgs::GeoPoint geoPoint;
        geoPoint.latitude = msg.latitude;
        geoPoint.longitude = msg.longitude;
        geoPoint.altitude = msg.altitude;
        return geodesy::UTMPoint( geoPoint );
    }

    

    // paramater ros navsatfix
    void gps_callback( const sensor_msgs::NavSatFix &msg )
    {
        //ROS_INFO_STREAM( msg );
        /* Convert the gps location to a UTM point 
            This code should be fine as long as you are not 
            on a UTM boundary */
        geodesy::UTMPoint utmPoint = get_utm_point( msg );
        geometry_msgs::Point point = geodesy::toGeometry( utmPoint );

        nav_msgs::Odometry odom;
        odom.header = msg.header;
        
        odom.pose.pose.position = point;

        this->_pub.publish( odom );

        /*ROS_INFO_STREAM( "historicPoints: " << this->_historicPoints.size() );

        auto matchIt = this->_find_match( point );

        if( matchIt != std::end(this->_historicPoints) )
        {
            ROS_INFO_STREAM( "Match" );

            // calculate heading
            const float yaw = get_theta( *matchIt, point );
            const auto quaternion = quat_from_yaw( yaw );

            // origin is unknown
            if( !this->_origin )
            {
                ROS_INFO_STREAM( "Setting origin" );
                this->_origin = geometry_msgs::Pose();
                
                this->_origin->position = *matchIt;
                this->_origin->orientation = quaternion;
            }

            // publish odom
            nav_msgs::Odometry odom;
            odom.header = msg.header;
            
            odom.pose.pose.position = point;
            odom.pose.pose.position.x -= this->_origin->position.x;
            odom.pose.pose.position.y -= this->_origin->position.y;
            odom.pose.pose.position.z -= this->_origin->position.z;

            auto get_yaw = []( const geometry_msgs::Quaternion& quat ) -> float
            {
                tf::Quaternion tfQuat;
                tf::quaternionMsgToTF( quat, tfQuat );

                tf::Matrix3x3 m( tfQuat );
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);

                return yaw;
            };

            const float originYaw = get_yaw( this->_origin->orientation );
            const float currentYaw = get_yaw( quaternion );

            const float yawDiff = currentYaw - originYaw;
            ROS_INFO_STREAM( "Yaw diff: " << yawDiff );

            const float x = odom.pose.pose.position.x, y = odom.pose.pose.position.y;

            odom.pose.pose.position.x = x * cos(yawDiff) + 
                                        y * sin(yawDiff);
            odom.pose.pose.position.y = x * -sin(yawDiff) + 
                                        y * cos(yawDiff);

            //odom.pose.pose.orientation = quaternion;

            this->_pub.publish( odom );

            this->_historicPoints.erase( std::next(matchIt), std::end(this->_historicPoints) );*/
        /*}  

        const int maxHistory = 1000;
        if( this->_historicPoints.size() >= maxHistory )
        {
            this->_historicPoints.erase( 
                    std::next(std::begin(this->_historicPoints), maxHistory), 
                    std::end(this->_historicPoints) );
        }

        this->_historicPoints.emplace_front( point );*/
    }

    GpsCallback( const std::string topic ) : nh( "~" )
    {
        this->_sub = this->nh.subscribe( topic, 10, &GpsCallback::gps_callback, this );
        this->_pub = this->nh.advertise<nav_msgs::Odometry>( "/gps/odom", 10 );
    }
};

void test()
{
    geographic_msgs::GeoPoint geoPoint;
    geoPoint.latitude = 52;
    geoPoint.longitude = 1;
    geoPoint.altitude = 100;

    geodesy::UTMPoint point( geoPoint );
    point.easting = 1000;
    point.northing = 1000;

    std::cout << point << std::endl;

    geodesy::UTMPoint origin = point;
    origin.easting = 100;
    origin.northing = 100;

    auto geometry = geodesy::toGeometry( point );

    std::cout << geometry << std::endl;
}

void test2()
{
    geometry_msgs::TransformStamped transform;
    transform.transform.translation.x = -10;
    transform.transform.translation.y = -10;
    transform.transform.translation.z = 0;

    tf2::Quaternion tf2Quat;
    tf2Quat.setRPY( 0, 0, -1.5708 );  

    transform.transform.rotation = tf2::toMsg( tf2Quat );

    geometry_msgs::Pose pose;
    pose.position.x = 100;
    pose.position.y = 200;
    pose.position.z = 0;
    
    tf2Quat.setRPY( 0, 0, 0 );
    pose.orientation = tf2::toMsg( tf2Quat );

    geometry_msgs::Pose out;
    tf2::doTransform( pose, pose, transform );

    std::cout << pose << std::endl;
}

int main( int argc, char* argv[] )
{
   // test2();
   // return 0;

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
