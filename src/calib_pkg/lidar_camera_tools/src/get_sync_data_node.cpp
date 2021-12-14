#include "get_sync_data/get_sync_data.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_sync_data");
    ros::NodeHandle n;
    GetSyncData get_sync_data(n);
    ros::spin();

    /* code */
    return 0;
}
