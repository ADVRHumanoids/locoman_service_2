#ifndef locoman_service_2_THREAD_H_
#define locoman_service_2_THREAD_H_

#include <GYM/control_thread.hpp>

/**
 * @brief locoman_service_2 control thread
 * 
 **/
class locoman_service_2_thread : public control_thread
{
private:   
    
public:
    
    /**
     * @brief constructor
     * 
     * @param module_prefix the prefix of the module
     * @param rf resource finderce
     * @param ph param helper
     */
     locoman_service_2_thread( std::string module_prefix, yarp::os::ResourceFinder rf, std::shared_ptr<paramHelp::ParamHelperServer> ph );
    
    
    /**
     * @brief locoman_service_2 control thread initialization
     * 
     * @return true on succes, false otherwise
     */
    virtual bool custom_init();
    
    /**
     * @brief locoman_service_2 control thread main loop
     * 
     */
    virtual void run();
    
};

#endif
