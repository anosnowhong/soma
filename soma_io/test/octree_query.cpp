#include <ros/ros.h>
#include <mongodb_store/message_store.h>
#include <boost/foreach.hpp>
#include <octomap_msgs/Octomap.h>

using namespace std;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "octree_query_test");
    ros::NodeHandle nh;

    mongodb_store::MessageStoreProxy messageStore(nh);

    vector<boost::shared_ptr<octomap_msgs::Octomap> > results;

    messageStore.query<octomap_msgs::Octomap>(results);
    BOOST_FOREACH(boost::shared_ptr<octomap_msgs::Octomap> oct, results)
    {
        ROS_INFO_STREAM("Got:" << *oct);
    }
    
    return 0;
}
