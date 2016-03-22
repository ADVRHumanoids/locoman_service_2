#include <yarp/os/all.h>
#include <GYM/control_module.hpp>
#include <cstdlib>

#include "locoman_service_2_module.hpp"

// default module period
#define MODULE_PERIOD 1000 //[millisec]

int main(int argc, char* argv[])
{
    // yarp network declaration and check
    yarp::os::Network yarp;
    if(!yarp.checkNetwork()){
        std::cerr <<"yarpserver not running - run yarpserver"<< std::endl;
        exit(EXIT_FAILURE);
    }
    // yarp network initialization
    yarp.init();

    // create rf
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    // set locoman_service_2_initial_config.ini as default
    // to specify another config file, run with this arg: --from your_config_file.ini 
    rf.setDefaultConfigFile( "locoman_service_2_initial_config.ini" ); 
    rf.setDefaultContext( "locoman_service_2" );  
    rf.configure(argc, argv);
    // create my module
    locoman_service_2_module locoman_service_2_mod = locoman_service_2_module( argc, argv, "locoman_service_2", MODULE_PERIOD, rf );
    
    // run the module
    locoman_service_2_mod.runModule( rf );
    
    // yarp network deinitialization
    yarp.fini();
    
    exit(EXIT_SUCCESS);
}
