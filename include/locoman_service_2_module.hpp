#ifndef locoman_service_2_MODULE_HPP_
#define locoman_service_2_MODULE_HPP_

#include <GYM/control_module.hpp>

#include "locoman_service_2_thread.h"
#include "locoman_service_2_constants.h"

/**
 * @brief locoman_service_2 module derived from control_module
 * 
 * @author 
 */
class locoman_service_2_module : public control_module<locoman_service_2_thread> {
public:
    
    /**
     * @brief constructor: do nothing but construct the superclass
     * 
     */
    locoman_service_2_module(    int argc, 
                               char* argv[],
                               std::string module_prefix, 
                               int module_period, 
                               yarp::os::ResourceFinder rf ) : control_module<locoman_service_2_thread>(  argc, 
                                                                                            		argv, 
                                                                                            		module_prefix, 
                                                                                            		module_period,
                                                                                            		rf )
    {
    }
    
    /**
     * @brief overriden function to specify the custom params for the param helper
     * 
     * @return a vector of the custom params for the param helper
     */
    virtual std::vector< paramHelp::ParamProxyInterface* > custom_get_ph_parameters() 
    {
	// TODO: function skeleton
        std::vector<paramHelp::ParamProxyInterface *> custom_params;
        return custom_params;
    }
    
    
};

#endif
