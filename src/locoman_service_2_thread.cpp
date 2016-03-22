#include <yarp/os/all.h>

#include "locoman_service_2_thread.h"
#include "locoman_service_2_constants.h"

locoman_service_2_thread::locoman_service_2_thread( std::string module_prefix, 
                             			yarp::os::ResourceFinder rf, 
                             			std::shared_ptr< paramHelp::ParamHelperServer > ph) :
    control_thread( module_prefix, rf, ph )
{
    // TODO: skeleton constructor
}

bool locoman_service_2_thread::custom_init()
{
    // TODO: skeleton function   
    return true;
}

void locoman_service_2_thread::run()
{   
    // TODO: skeleton function
}    
