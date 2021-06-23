#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
STRICT_MODE_ON

#include <iostream>
#include <vector>
#include <string>
#include <cstdlib>
#include <pthread.h>
#include <opencv2/opencv.hpp>
#include <rpc/rpc_error.h>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "ImageHandler.hpp"
#include "SensorHandler.hpp"
void * getStereoAndDepthImages(void *args) {

    ImageHandler ihandler;
    ihandler.initialize();
    SensorHandler shandler;
    while (1) {
       // ihandler.gotogoal();
        shandler.getlocation();
        shandler.lidarprocess();
    }

    return 0;
}



int main()
{
    using namespace std;
    msr::airlib::MultirotorRpcLibClient client;
    pthread_t tid;
    cout << "Press Enter to enable API control" << endl; cin.get();
    client.enableApiControl(true);

    cout << "Press Enter to arm the drone" << endl; cin.get();
    client.armDisarm(true);

    

    cout << "Press Enter to takeoff" << endl; cin.get();
    client.takeoffAsync(5)->waitOnLastTask();
    int rec = pthread_create(&tid, NULL, getStereoAndDepthImages, NULL);

    auto position = client.getMultirotorState().getPosition(); 
    client.moveToPositionAsync(position.x() , position.y() , position.z() - 60, 5)->waitOnLastTask();
  // client.moveToPositionAsync(position.x() +200, position.y() + 60, position.z() -30, 5)->waitOnLastTask();
   
  //  getStereoAndDepthImages();
    cout << "Press Enter to land" << endl; cin.get();
    client.landAsync()->waitOnLastTask();
    pthread_exit(NULL);
    return 0;
}