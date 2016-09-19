#include <yarp/os/all.h>

#include "locoman_service_2_thread.h"
#include "locoman_service_2_constants.h"

#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <GYM/yarp_command_interface.hpp>
#include <fstream>
#include <unistd.h>
#include <limits>


// 
#include <iCub/iDynTree/yarp_kdl.h>
#include <locoman/utils/screws.hpp>
#include <locoman/utils/kinematics.hpp>
#include <locoman/utils/kinetostatics.hpp>
#include <locoman/utils/locoman_utils.hpp>
#include <locoman/utils/algebra.hpp>

locoman_service_2_thread::locoman_service_2_thread( 
                                        std::string module_prefix, 
                             			yarp::os::ResourceFinder rf, 
                             			std::shared_ptr< paramHelp::ParamHelperServer > ph) :
                                        control_thread( module_prefix, rf, ph ),
    size_q(locoman::utils::getNumberOfKinematicJoints(robot)),
    q_sensed(size_q, 0.0) ,
    
    CoM_w_cmd(3, 0.0) ,
    CoM_w_up(3, 0.0) ,
    CoM_w_dw(3, 0.0) ,
    FC_size(24)  ,
    FC_HANDS_size(24) ,
    WINDOW_size(15) , //30 //50  // 15
    FC_DES(FC_size, 0.0) , 
    FC_DES_LEFT_sensor(6, 0.0) ,
    FC_DES_RIGHT_sensor(6,0.0),
    FC_FILTERED(FC_size),
    FC_WINDOW(FC_size, WINDOW_size ) ,
    //
    FC_HANDS_DES(FC_HANDS_size, 0.0) ,
  
    fc_received(48, 0.0),
    mg_vect(3,0.0) ,

  //---------------------------------
    // number of fc components on the feet
    zero_3(3, 0.0) ,
    Zeros_6_6(6,6) ,
    Eye_6(6,6) ,
    Eye_3(3,3) ,
    Eye_4(4,4) ,
    B(6,3) ,
    Kc( 24 , 24) ,
    Kc_f_h( 48 , 48) ,
    Kc_f_rh( 36 ,36 ) ,
    ft_l_ankle(6,0.0) ,
    ft_r_ankle(6,0.0) ,
    ft_l_wrist(6,0.0) ,
    ft_r_wrist(6,0.0) ,
    fc_offset_left(12, 0.0) ,
    fc_offset_right(12, 0.0) ,
  //  fc_offset_left_hand(12, 0.0) ,
  //  fc_offset_right_hand(12, 0.0),
    
  fc_l_foot(12, 0.0), // ; //= map_l_fcToSens_PINV*fc_received.subVector(  0,5  ) ;
  fc_r_foot(12, 0.0), // ; //= map_r_fcToSens_PINV*fc_received.subVector(  6,11  ) ;
  fc_feet(24, 0.0),
  fc_hands(24, 0.0),
    
  fc_l_hand(12, 0.0), // ; //= map_l_hand_fcToSens*fc_received.subVector( 12,17  ) ;
  fc_r_hand(12, 0.0), // ;
    
    //
    map_l_fcToSens(6,12) ,
    map_r_fcToSens(6,12) ,  
    map_r_hand_fcToSens(6,12) ,
    map_l_hand_fcToSens(6,12) ,
    map_l_fcToSens_PINV(12,6) ,
    map_r_fcToSens_PINV(12,6) ,
    map_l_hand_fcToSens_PINV(12,6) ,
    map_r_hand_fcToSens_PINV(12,6) ,
    //
  fc_l1_foot_filt(3, 0.0) , // = FC_FILTERED.subVector(0,2)  ;  // Applied from the robot to the world
  fc_l2_foot_filt(3, 0.0) , // = FC_FILTERED.subVector(3,5)  ;
  fc_l3_foot_filt(3, 0.0) , // = FC_FILTERED.subVector(6,8)  ;
  fc_l4_foot_filt(3, 0.0) , // = FC_FILTERED.subVector(9,11)  ;

  fc_r1_foot_filt(3, 0.0) , // = FC_FILTERED.subVector(12,14)  ; 
  fc_r2_foot_filt(3, 0.0) , // = FC_FILTERED.subVector(15,17)  ; 
  fc_r3_foot_filt(3, 0.0) , // = FC_FILTERED.subVector(18,20)  ; 
  fc_r4_foot_filt(3, 0.0) , // = FC_FILTERED.subVector(21,23)  ; 
  
  fc_l1_hand_filt(3, 0.0) , // = FC_HANDS_FILTERED.subVector(0,2)  ;  // Applied from the robot to the world
  fc_l2_hand_filt(3, 0.0) , // = FC_HANDS_FILTERED.subVector(3,5)  ;
  fc_l3_hand_filt(3, 0.0) , // = FC_HANDS_FILTERED.subVector(6,8)  ;
  fc_l4_hand_filt(3, 0.0) , // = FC_HANDS_FILTERED.subVector(9,11)  ;

  fc_r1_hand_filt(3, 0.0) , // = FC_HANDS_FILTERED.subVector(12,14)  ; 
  fc_r2_hand_filt(3, 0.0) , // = FC_HANDS_FILTERED.subVector(15,17)  ; 
  fc_r3_hand_filt(3, 0.0) , // = FC_HANDS_FILTERED.subVector(18,20)  ; 
  fc_r4_hand_filt(3, 0.0) , // = FC_HANDS_FILTERED.subVector(21,23)  ;   
//     
  d_EE_r_des(6, 0.0 ) ,
  d_EE_l_des(6 , 0.0) ,  
    
  T_w_aw_0(4,4) , // = locoman::utils::AW_world_posture(model, robot) ;
  T_aw_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_aw_0) ;    

  T_w_waist_0(4,4) , //    = model.iDyn3_model.getPosition(waist_index) ;  
  T_w_l_ankle_0(4,4) , //  = model.iDyn3_model.getPosition(l_ankle_index) ;
  T_w_l_c1_0(4,4) , //     = model.iDyn3_model.getPosition(l_c1_index)    ;    
  T_w_l_c2_0(4,4) , //     = model.iDyn3_model.getPosition(l_c2_index)    ;  
  T_w_l_c3_0(4,4) , //     = model.iDyn3_model.getPosition(l_c3_index)    ;
  T_w_l_c4_0(4,4) , //     = model.iDyn3_model.getPosition(l_c4_index)    ;    
    
  T_w_r_ankle_0(4,4) , //  = model.iDyn3_model.getPosition(r_ankle_index) ;
  T_w_r_c1_0(4,4) , //     = model.iDyn3_model.getPosition(r_c1_index)    ;    
  T_w_r_c2_0(4,4) , //     = model.iDyn3_model.getPosition(r_c2_index)    ;  
  T_w_r_c3_0(4,4) , //     = model.iDyn3_model.getPosition(r_c3_index)    ;
  T_w_r_c4_0(4,4) , //     = model.iDyn3_model.getPosition(r_c4_index)    ;   
    
  T_w_l_hand_0(4,4) , //   = model.iDyn3_model.getPosition( l_hand_index ) ;
  T_w_r_hand_0(4,4) , //   = model.iDyn3_model.getPosition( r_hand_index ) ;   

  T_w_l_wrist_0(4,4) , //  = model.iDyn3_model.getPosition(l_wrist_index) ;
  T_w_l1_hand_0(4,4) , //  = model.iDyn3_model.getPosition(l_hand_c1_index)    ;    
  T_w_l2_hand_0(4,4) , //  = model.iDyn3_model.getPosition(l_hand_c2_index)    ;  
  T_w_l3_hand_0(4,4) , //  = model.iDyn3_model.getPosition(l_hand_c3_index)    ;
  T_w_l4_hand_0(4,4) , //  = model.iDyn3_model.getPosition(l_hand_c4_index)    ;    
    
  T_w_r_wrist_0(4,4) , //  = model.iDyn3_model.getPosition(r_wrist_index) ;
  T_w_r1_hand_0(4,4) , //  = model.iDyn3_model.getPosition(r_hand_c1_index)    ;    
  T_w_r2_hand_0(4,4) , //  = model.iDyn3_model.getPosition(r_hand_c2_index)    ;  
  T_w_r3_hand_0(4,4) , //  = model.iDyn3_model.getPosition(r_hand_c3_index)    ;
  T_w_r4_hand_0(4,4) , //  = model.iDyn3_model.getPosition(r_hand_c4_index)    ;     
  
  T_waist_w_0(4,4) , //    = locoman::utils::iHomogeneous(T_w_waist_0)  ;
  T_l_ankle_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_l_ankle_0) ;
  T_l_c1_w_0(4,4) , //     = locoman::utils::iHomogeneous(T_w_l_c1_0) ;    
  T_l_c2_w_0(4,4) , //     = locoman::utils::iHomogeneous(T_w_l_c2_0) ;  
  T_l_c3_w_0(4,4) , //     = locoman::utils::iHomogeneous(T_w_l_c3_0) ;
  T_l_c4_w_0(4,4) , //     = locoman::utils::iHomogeneous(T_w_l_c4_0) ;    
    
  T_r_ankle_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_r_ankle_0) ;
  T_r_c1_w_0(4,4) , //     = locoman::utils::iHomogeneous(T_w_r_c1_0) ;    
  T_r_c2_w_0(4,4) , //     = locoman::utils::iHomogeneous(T_w_r_c2_0) ;  
  T_r_c3_w_0(4,4) , //     = locoman::utils::iHomogeneous(T_w_r_c3_0) ;
  T_r_c4_w_0(4,4) , //     = locoman::utils::iHomogeneous(T_w_r_c4_0) ;    

  T_l_wrist_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_l_wrist_0)  ;
  T_l1_hand_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_l1_hand_0) ;    
  T_l2_hand_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_l2_hand_0) ;  
  T_l3_hand_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_l3_hand_0) ;
  T_l4_hand_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_l4_hand_0) ;    
    
  T_r_wrist_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_r_wrist_0) ;
  T_r1_hand_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_r1_hand_0) ;    
  T_r2_hand_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_r2_hand_0) ;  
  T_r3_hand_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_r3_hand_0) ;
  T_r4_hand_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_r4_hand_0) ;    

  T_l_hand_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_l_hand_0) ;
  T_r_hand_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_r_hand_0) ;   
  
  T_aw_l_c1_0(4,4) , //  = T_aw_w_0 * T_w_l_c1_0 ;  // {AW} is fixed in a loop
  T_aw_l_c2_0(4,4) , //  = T_aw_w_0 * T_w_l_c2_0 ;  // in every loop the floating base is re-initialized 
  T_aw_l_c3_0(4,4) , //  = T_aw_w_0 * T_w_l_c3_0 ;  // coincident with {AW}
  T_aw_l_c4_0(4,4) , //  = T_aw_w_0 * T_w_l_c4_0 ;

  T_aw_r_c1_0(4,4) , //  = T_aw_w_0 * T_w_r_c1_0 ;
  T_aw_r_c2_0(4,4) , //  = T_aw_w_0 * T_w_r_c2_0 ;
  T_aw_r_c3_0(4,4) , //  = T_aw_w_0 * T_w_r_c3_0 ;
  T_aw_r_c4_0(4,4) , //  = T_aw_w_0 * T_w_r_c4_0 ; 

  T_aw_l1_hand_0(4,4) , //  = T_aw_w_0 * T_w_l1_hand_0 ;  // {AW} is fixed in a loop
  T_aw_l2_hand_0(4,4) , //  = T_aw_w_0 * T_w_l2_hand_0 ;  // in every loop the floating base is re-initialized 
  T_aw_l3_hand_0(4,4) , //  = T_aw_w_0 * T_w_l3_hand_0 ;  // coincident with {AW}
  T_aw_l4_hand_0(4,4) , //  = T_aw_w_0 * T_w_l4_hand_0 ;

  T_aw_r1_hand_0(4,4) , //  = T_aw_w_0 * T_w_r1_hand_0 ;
  T_aw_r2_hand_0(4,4) , //  = T_aw_w_0 * T_w_r2_hand_0 ;
  T_aw_r3_hand_0(4,4) , //  = T_aw_w_0 * T_w_r3_hand_0 ;
  T_aw_r4_hand_0(4,4) , //  = T_aw_w_0 * T_w_r4_hand_0 ;   
    
  //-----------------------------------------------------
  Complete_Jac_feet( 24 , size_q + 6) ,  //( 8*B.cols() , size_q + 6) ;
  Complete_Jac_f_h( 48 , size_q + 6) , // ( 16*B.cols() , size_q + 6) ;
  //J_feet( 24 , size_q + 6)  ,
  J_c_feet( 24 , size_q ) ,
  S_c_feet_T( 24 , 6 ) , 
  S_c_feet( 6 , 24 )   , // = S_c_T.transposed() ;  
  
  J_c_f_h( 48 , size_q )    , //  = Complete_Jac_f_h.submatrix( 0,  Complete_Jac_f_h.rows()-1 , 6, Complete_Jac_f_h.cols()-1 ) ;
  S_c_f_h_T( 48 , 6 ) , // = Complete_Jac_f_h.submatrix( 0,  Complete_Jac_f_h.rows()-1 , 0, 5 ) ;
  S_c_f_h( 6 , 48 )   ,
  
  J_l_c1_mix_0( 6, ( size_q + 6 ) ) , //
  J_l_c2_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  J_l_c3_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  J_l_c4_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ;
  
  J_r_c1_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  J_r_c2_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  J_r_c3_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  J_r_c4_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ;
  
  J_l_hand_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  J_r_hand_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ;
  
  J_l1_hand_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  J_l2_hand_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  J_l3_hand_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  J_l4_hand_mix_0( 6, ( size_q + 6 ) ) ,//robot.getNumberOfKinematicJoints() + 6 ) ) ;

  J_r1_hand_mix_0( 6, ( size_q + 6 ) ) , // robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  J_r2_hand_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  J_r3_hand_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  J_r4_hand_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ;

  //-------------------------------------------
  
  J_l_c1_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_c1_w_0), zero_3 ))* J_l_c1_mix_0 ;
  J_l_c2_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_c2_w_0), zero_3 ))* J_l_c2_mix_0 ;
  J_l_c3_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_c3_w_0), zero_3 ))* J_l_c3_mix_0 ;
  J_l_c4_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_c4_w_0), zero_3 ))* J_l_c4_mix_0 ;

  J_r_c1_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_c1_w_0), zero_3 ))* J_r_c1_mix_0 ;
  J_r_c2_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_c2_w_0), zero_3 ))* J_r_c2_mix_0 ;
  J_r_c3_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_c3_w_0), zero_3 ))* J_r_c3_mix_0 ;
  J_r_c4_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_c4_w_0), zero_3 ))* J_r_c4_mix_0 ;

  J_l_hand_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_hand_w_0), zero_3 ))* J_l_hand_mix_0 ;
  J_r_hand_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_hand_w_0), zero_3 ))* J_r_hand_mix_0 ;

  J_l1_hand_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l1_hand_w_0), zero_3 ))* J_l1_hand_mix_0 ;
  J_l2_hand_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l2_hand_w_0), zero_3 ))* J_l2_hand_mix_0 ;
  J_l3_hand_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l3_hand_w_0), zero_3 ))* J_l3_hand_mix_0 ;
  J_l4_hand_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l4_hand_w_0), zero_3 ))* J_l4_hand_mix_0 ;

  J_r1_hand_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r1_hand_w_0), zero_3 ))* J_r1_hand_mix_0 ;
  J_r2_hand_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r2_hand_w_0), zero_3 ))* J_r2_hand_mix_0 ;
  J_r3_hand_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r3_hand_w_0), zero_3 ))* J_r3_hand_mix_0 ;
  J_r4_hand_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r4_hand_w_0), zero_3 ))* J_r4_hand_mix_0 ;
  
  //---------------------------------------------------------------------------------------------------------------------------------------------------------------  
  J_aw_l_c1_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_l_c1_0)* J_l_c1_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c1_spa_0 ;// locoman::utils::Adjoint(T_aw_l_c1_0)* J_l_c1_body_0
  J_aw_l_c2_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_l_c2_0)* J_l_c2_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c2_spa_0 ;
  J_aw_l_c3_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_l_c3_0)* J_l_c3_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c3_spa_0 ;
  J_aw_l_c4_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_l_c4_0)* J_l_c4_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c4_spa_0 ;

  J_aw_r_c1_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_r_c1_0)* J_r_c1_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c1_spa_0 ;
  J_aw_r_c2_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_r_c2_0)* J_r_c2_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c2_spa_0 ;
  J_aw_r_c3_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_r_c3_0)* J_r_c3_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c3_spa_0 ;
  J_aw_r_c4_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_r_c4_0)* J_r_c4_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c4_spa_0 ;

  J_aw_l1_hand_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_l1_hand_0)* J_l1_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c1_spa_0 ;// locoman::utils::Adjoint(T_aw_l_c1_0)* J_l_c1_body_0
  J_aw_l2_hand_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_l2_hand_0)* J_l2_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c2_spa_0 ;
  J_aw_l3_hand_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_l3_hand_0)* J_l3_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c3_spa_0 ;
  J_aw_l4_hand_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_l4_hand_0)* J_l4_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c4_spa_0 ;
 
  J_aw_r1_hand_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_r1_hand_0)* J_r1_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c1_spa_0 ;
  J_aw_r2_hand_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_r2_hand_0)* J_r2_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c2_spa_0 ;
  J_aw_r3_hand_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_r3_hand_0)* J_r3_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c3_spa_0 ;
  J_aw_r4_hand_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_r4_hand_0)* J_r4_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c4_spa_0 ;
  
  //------------------------------------------------
  Q_mg(size_q+ 6, size_q + 6)   , 
  
  Q_aw_l_c1(size_q+ 6, size_q + 6)   , //= Q_ci(J_aw_l_c1_spa_0, T_aw_l_c1_0, fc_l_c1_filt ) ;
  Q_aw_l_c2(size_q+ 6, size_q + 6)   , // = Q_ci(J_aw_l_c2_spa_0, T_aw_l_c2_0, fc_l_c2_filt ) ; // (size_q+ 6, size_q + 6) ;
  Q_aw_l_c3(size_q+ 6, size_q + 6)   , // = Q_ci(J_aw_l_c3_spa_0, T_aw_l_c3_0, fc_l_c3_filt ) ; //(size_q+ 6, size_q + 6) ; 
  Q_aw_l_c4(size_q+ 6, size_q + 6)   , // = Q_ci(J_aw_l_c4_spa_0, T_aw_l_c4_0, fc_l_c4_filt ) ; //(size_q+ 6, size_q + 6) ;

  Q_aw_r_c1(size_q+ 6, size_q + 6)   , // = Q_ci(J_aw_r_c1_spa_0, T_aw_r_c1_0, fc_r_c1_filt ) ; //(size_q+ 6, size_q + 6) ;
  Q_aw_r_c2(size_q+ 6, size_q + 6)   , // = Q_ci(J_aw_r_c2_spa_0, T_aw_r_c2_0, fc_r_c2_filt ) ; //(size_q+ 6, size_q + 6) ;
  Q_aw_r_c3(size_q+ 6, size_q + 6)   , // = Q_ci(J_aw_r_c3_spa_0, T_aw_r_c3_0, fc_r_c3_filt ) ; //(size_q+ 6, size_q + 6) ; 
  Q_aw_r_c4(size_q+ 6, size_q + 6)   , // = Q_ci(J_aw_r_c4_spa_0, T_aw_r_c4_0, fc_r_c4_filt ) ; //(size_q+ 6, size_q + 6) ;

  Q_aw_l1_hand(size_q+ 6, size_q + 6)   , // = Q_ci(J_aw_r_c1_spa_0, T_aw_r_c1_0, fc_r_c1_filt ) ; //(size_q+ 6, size_q + 6) ;
  Q_aw_l2_hand(size_q+ 6, size_q + 6)   , // = Q_ci(J_aw_r_c2_spa_0, T_aw_r_c2_0, fc_r_c2_filt ) ; //(size_q+ 6, size_q + 6) ;
  Q_aw_l3_hand(size_q+ 6, size_q + 6)   , // = Q_ci(J_aw_r_c3_spa_0, T_aw_r_c3_0, fc_r_c3_filt ) ; //(size_q+ 6, size_q + 6) ; 
  Q_aw_l4_hand(size_q+ 6, size_q + 6)   ,
  
  Q_aw_r1_hand(size_q+ 6, size_q + 6)   , // = Q_ci(J_aw_r_c1_spa_0, T_aw_r_c1_0, fc_r_c1_filt ) ; //(size_q+ 6, size_q + 6) ;
  Q_aw_r2_hand(size_q+ 6, size_q + 6)   , // = Q_ci(J_aw_r_c2_spa_0, T_aw_r_c2_0, fc_r_c2_filt ) ; //(size_q+ 6, size_q + 6) ;
  Q_aw_r3_hand(size_q+ 6, size_q + 6)   , // = Q_ci(J_aw_r_c3_spa_0, T_aw_r_c3_0, fc_r_c3_filt ) ; //(size_q+ 6, size_q + 6) ; 
  Q_aw_r4_hand(size_q+ 6, size_q + 6)   ,

  Q_aw_l_tot(size_q+ 6, size_q + 6)   , // = Q_aw_l_c1 + Q_aw_l_c2 + Q_aw_l_c3 + Q_aw_l_c4;
  Q_aw_r_tot(size_q+ 6, size_q + 6)   , // = Q_aw_r_c1 + Q_aw_r_c2 + Q_aw_r_c3 + Q_aw_r_c4;
  Q_aw_l_hand_tot(size_q+ 6, size_q + 6)   , // = Q_aw_r_c1 + Q_aw_r_c2 + Q_aw_r_c3 + Q_aw_r_c4;
  Q_aw_r_hand_tot(size_q+ 6, size_q + 6)   , // = Q_aw_r_c1 + Q_aw_r_c2 + Q_aw_r_c3 + Q_aw_r_c4;
  
  Q_aw_c(size_q+ 6, size_q + 6)   ,  // =  Q_aw_l_tot + Q_aw_r_tot ;  
  U_aw_s_cont( 6 , 6) ,              // = Q_aw_c.submatrix( 0 ,  5 , 0, 5) ;     
  Q_aw_s_cont( 6 , size_q ) ,        //  = Q_aw_c.submatrix( 0  , 5,  6,  (Q_aw_c.cols()-1)  ) ;
  
  Q_aw_c_f_rh(size_q+ 6, size_q + 6)   , // =  Q_aw_l_tot + Q_aw_r_tot ;  
  U_aw_s_c_f_rh( 6 , 6) ,                // = Q_aw_c.submatrix( 0 ,  5 , 0, 5) ;     
  Q_aw_s_c_f_rh( 6 , size_q ) ,          //  = Q_aw_c.submatrix( 0  , 5,  6,
  
  Q_aw_c_f_h(size_q+ 6, size_q + 6) ,    // = Q_aw_c + Q_aw_l_hand_tot + Q_aw_r_hand_tot ;
  U_aw_s_f_h( 6,  6) ,                   // = Q_aw_c_f_h.submatrix( 0 ,  5 , 0, 5) ;     
  Q_aw_s_f_h(6, size_q ) ,
  
  FLMM(30, size_q + 30)  ,  //= locoman::utils::FLMM_redu(J_c, S_c, Q_aw_s_cont, U_aw_s_cont, Kc ) ;
  cFLMM( 30, size_q + 30 ) ,
  FLMM_old( 30, size_q + 30 ) , 
  cFLMM_old( 30, size_q + 30 ) ,
  Rf(24, size_q)   ,
  Rf_filt(24, size_q) ,
  Rf_filt_pinv( size_q, 24) ,
  Rf_feet_old( size_q, 24)  , 
  Rf_feet_old_filt( size_q, 24) ,
  
  Rf_old_locoman( size_q, 24) ,
    
  Rf_filt_f_h(48, size_q) ,  
    
  Big_J_new(27, size_q+6),  
    
  Big_Rf_new(48, size_q),    
  
  //------------------------------------------------------------------
  d_fc_des_to_world(size_fc, 0.0)  ,

  T_l_c1_r_c1_loop(4,4) ,
  T_r_c1_l_c1_loop(4,4) ,
//   T_l_c1_r_c1_loop.zero() ,
//   T_r_c1_l_c1_loop.zero() ,  
  J_com_w( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ;
  J_com_w_redu( 3,  ( size_q + 6 )  ) , //( robot.getNumberOfKinematicJoints() + 6 ))   ;
  J_com_aw( 3,  ( size_q + 6 ) ) , //( robot.getNumberOfKinematicJoints() + 6 ))   ;
  J_com_waist( 3,  ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ))   ;
 
  J_com_w_q_i( 6, ( size_q + 6 ) ) ,  //( 6, ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ) ) ;
  J_com_w_redu_q_i( 3,  ( size_q + 6 )  ) ,  //( 3,  ( size_q + 6 )  ) ; //( robot.getNumberOfKinematicJoints() + 6 ))   ;
  J_com_aw_q_i( 3,  ( size_q + 6 ) ) ,
 
  J_r_c1_aw( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ;
  J_l_c1_aw( 6, ( size_q + 6 ) )   //robot.getNumberOfKinematicJoints() + 6 ) ) ;
  
  //new_Robot("new_Robot",get_urdf_path(),get_srdf_path())  
                                    
{
    // TODO: skeleton constructor
      //------------ 
    CoM_waist_cmd = locoman::utils::getRot( locoman::utils::iHomogeneous(model.iDyn3_model.getPosition( model.iDyn3_model.getLinkIndex("Waist"))))*
                    model.iDyn3_model.getCOM() ;
    T_waist_l1_foot_cmd = locoman::utils::iHomogeneous(model.iDyn3_model.getPosition( model.iDyn3_model.getLinkIndex("Waist")))*
                    model.iDyn3_model.getPosition( (model.iDyn3_model.getLinkIndex("l_foot_upper_left_link"))) ; 
    T_waist_r1_foot_cmd = locoman::utils::iHomogeneous(model.iDyn3_model.getPosition( model.iDyn3_model.getLinkIndex("Waist")))*
                    model.iDyn3_model.getPosition( (model.iDyn3_model.getLinkIndex("r_foot_upper_left_link"))) ; 
    T_waist_l_hand_cmd = locoman::utils::iHomogeneous(model.iDyn3_model.getPosition( model.iDyn3_model.getLinkIndex("Waist")))*
                    model.iDyn3_model.getPosition( (model.iDyn3_model.getLinkIndex("LSoftHand")))  ;    
    T_waist_r_hand_cmd = locoman::utils::iHomogeneous(model.iDyn3_model.getPosition( model.iDyn3_model.getLinkIndex("Waist"))) *
                    model.iDyn3_model.getPosition( (model.iDyn3_model.getLinkIndex("RSoftHand")))  ;      
    R_waist_aw_cmd = locoman::utils::getRot( locoman::utils::iHomogeneous(model.iDyn3_model.getPosition( model.iDyn3_model.getLinkIndex("Waist")))) ; //just initialization
    //------------ 
    CoM_w_cmd  = model.iDyn3_model.getCOM()  ;
    CoM_w_up = CoM_w_cmd ;
    CoM_w_dw = CoM_w_cmd ;
    T_l1_r1_up = locoman::utils::iHomogeneous(model.iDyn3_model.getPosition( (model.iDyn3_model.getLinkIndex("l_foot_upper_left_link")))) *
                     model.iDyn3_model.getPosition( (model.iDyn3_model.getLinkIndex("r_foot_upper_left_link")) ) ;  
    T_l1_r1_fw = locoman::utils::iHomogeneous(model.iDyn3_model.getPosition( (model.iDyn3_model.getLinkIndex("l_foot_upper_left_link")))) *
                     model.iDyn3_model.getPosition( (model.iDyn3_model.getLinkIndex("r_foot_upper_left_link"))) ;       
    T_l1_r1_dw = locoman::utils::iHomogeneous(model.iDyn3_model.getPosition( (model.iDyn3_model.getLinkIndex("l_foot_upper_left_link")))) *
                     model.iDyn3_model.getPosition( (model.iDyn3_model.getLinkIndex("r_foot_upper_left_link"))) ;       
 
    T_w_l1_cmd = model.iDyn3_model.getPosition( (model.iDyn3_model.getLinkIndex("l_foot_upper_left_link"))) ;     
    T_w_r1_cmd = model.iDyn3_model.getPosition( (model.iDyn3_model.getLinkIndex("r_foot_upper_left_link"))) ;    
     
    T_r1_l1_up = locoman::utils::iHomogeneous( model.iDyn3_model.getPosition( (model.iDyn3_model.getLinkIndex("r_foot_upper_left_link"))) ) *
                     model.iDyn3_model.getPosition( (model.iDyn3_model.getLinkIndex("l_foot_upper_left_link")))   ;
    T_r1_l1_fw = locoman::utils::iHomogeneous( model.iDyn3_model.getPosition( (model.iDyn3_model.getLinkIndex("r_foot_upper_left_link"))) ) *
                     model.iDyn3_model.getPosition( (model.iDyn3_model.getLinkIndex("l_foot_upper_left_link")))   ;
    T_r1_l1_dw = locoman::utils::iHomogeneous( model.iDyn3_model.getPosition( (model.iDyn3_model.getLinkIndex("r_foot_upper_left_link"))) ) *
                     model.iDyn3_model.getPosition( (model.iDyn3_model.getLinkIndex("l_foot_upper_left_link")))   ;        
}

bool locoman_service_2_thread::custom_init()
{
    //--------------------------------------------------------
    // YARP Port Section
    // from_locoman_thread.open(std::string("/" + get_module_prefix() + "/test"));   
   
    if(!receiving_q.open(std::string("/" + get_module_prefix() + "/receiving_q")) ) 
     { std::cout << "ERROR: cannot open YARP port " << std::string(get_module_prefix() + "/receiving_q") << std::endl;
        return false; }
        
    if(!receiving_fc.open(std::string("/" + get_module_prefix() + "/receiving_fc")) ) 
     { std::cout << "ERROR: cannot open YARP port " << std::string(get_module_prefix() + "/receiving_fc") << std::endl;
        return false; }    
    
    //protorype
   // to_locoman_thread.open(std::string("/" + get_module_prefix() + "/test_output")); 

    
    to_locoman_Matrix.open(std::string("/" + get_module_prefix() + "/sending_Matrix"));   

    to_locoman_Big_J.open(std::string("/" + get_module_prefix() + "/sending_Big_J"));   

    to_locoman_Big_Rf.open(std::string("/" + get_module_prefix() + "/sending_Big_Rf"));   

    to_locoman_Rf_old.open(std::string("/" + get_module_prefix() + "/sending_Rf_old"));   

    to_multicontact_Big_Rf.open(std::string("/" + get_module_prefix() + "/sending_multicontact_Big_Rf"));   

    
    // end of YARP Port Section
    //---------------------------------------------------------
   
    
    struct sched_param thread_param; 
    thread_param.sched_priority = 99;
    
    model.setFloatingBaseLink("Waist");
    yarp::sig::Vector q_current(locoman::utils::getNumberOfKinematicJoints(robot),0.0) ; // = robot.sensePosition();
    robot.idynutils.updateiDyn3Model(q_current, true);
    
    
  //------------------------------------------------------------------------------------------------------------------

  // Defining Various Parameters
  //yarp::sig::Vector zero_3(3, 0.0) ;
  //yarp::sig::Matrix Zeros_6_6(6,6) ;
  Zeros_6_6.zero();
  //yarp::sig::Matrix Eye_6(6,6) ;
  Eye_6.eye() ;
  //yarp::sig::Matrix Eye_3(3,3) ;
  Eye_3.eye() ;
  Eye_4.eye() ;
  //yarp::sig::Matrix B( 6 , 3 ) ;
  B.zero();
  B.setSubmatrix( Eye_3 , 0 , 0 ) ;
  
  /*size_q = robot.getNumberOfKinematicJoints()*/ ; // getNumberOfKinematicJoints = 31 (29 + 2 for the hands)
  size_u = 6 ;
  size_fc = 24 ; // number of fc components on the feet
  kc = 1E6 ;
  Kq = locoman::utils::getKq(robot) ;
  Kc.eye() ;
  Kc = kc*Kc ;    //   ;// 1E6*Kc ;    1E8*Kc ;  
  Kc_f_h.eye() ;
  Kc_f_h = kc*Kc_f_h ;  
  Kc_f_rh.eye() ;
  Kc_f_rh = (kc/10)*Kc_f_rh ;
  
  waist_index   =  model.iDyn3_model.getLinkIndex("Waist");  
  l_ankle_index = model.iDyn3_model.getLinkIndex("l_leg_ft") ; // sensors are placed in *_ankle in the model

  l_c1_index    = model.iDyn3_model.getLinkIndex("l_foot_upper_left_link") ;
  l_c2_index    = model.iDyn3_model.getLinkIndex("l_foot_upper_right_link");
  l_c3_index    = model.iDyn3_model.getLinkIndex("l_foot_lower_left_link") ;
  l_c4_index    = model.iDyn3_model.getLinkIndex("l_foot_lower_right_link");

  r_ankle_index = model.iDyn3_model.getLinkIndex("r_leg_ft") ;

  r_c1_index    = model.iDyn3_model.getLinkIndex("r_foot_upper_left_link");
  r_c2_index    = model.iDyn3_model.getLinkIndex("r_foot_upper_right_link");
  r_c3_index    = model.iDyn3_model.getLinkIndex("r_foot_lower_left_link");
  r_c4_index    = model.iDyn3_model.getLinkIndex("r_foot_lower_right_link");

  l_hand_index  = model.iDyn3_model.getLinkIndex("LSoftHand");
  r_hand_index  = model.iDyn3_model.getLinkIndex("RSoftHand");    

  l_wrist_index  = model.iDyn3_model.getLinkIndex("l_arm_ft") ;
  l_hand_c1_index = model.iDyn3_model.getLinkIndex("l_hand_upper_right_link"); 
  l_hand_c2_index = model.iDyn3_model.getLinkIndex("l_hand_lower_right_link"); 
  l_hand_c3_index = model.iDyn3_model.getLinkIndex("l_hand_upper_left_link");  
  l_hand_c4_index = model.iDyn3_model.getLinkIndex("l_hand_lower_left_link");  
//     
  r_wrist_index   = model.iDyn3_model.getLinkIndex("r_arm_ft") ;
  r_hand_c1_index = model.iDyn3_model.getLinkIndex("r_hand_upper_right_link"); 
  r_hand_c2_index = model.iDyn3_model.getLinkIndex("r_hand_lower_right_link"); 
  r_hand_c3_index = model.iDyn3_model.getLinkIndex("r_hand_upper_left_link");  
  r_hand_c4_index = model.iDyn3_model.getLinkIndex("r_hand_lower_left_link");  
// //    
  
   
  map_l_fcToSens =  locoman::utils::fConToSens( l_ankle_index, 
                                                l_c1_index  , 
                                                l_c2_index  ,                                
                                                l_c3_index  , 
                                                l_c4_index,
                                                model
                                                ) ;
                            
  map_r_fcToSens =  locoman::utils::fConToSens( r_ankle_index, 
                                                r_c1_index, 
                                                r_c2_index,
                                                r_c3_index, 
                                                r_c4_index,
                                                model
                                                ) ;
                      // 
  map_l_hand_fcToSens = locoman::utils::fConToSens( l_wrist_index, 
                                                    l_hand_c1_index, 
                                                    l_hand_c2_index,
                                                    l_hand_c3_index, 
                                                    l_hand_c4_index,
                                                    model
                                                    ) ;
    
  map_r_hand_fcToSens = locoman::utils::fConToSens( r_wrist_index, 
                                                    r_hand_c1_index, 
                                                    r_hand_c2_index,
                                                    r_hand_c3_index, 
                                                    r_hand_c4_index ,
                                                    model
                                                    ) ;
                                                                     
  map_l_fcToSens_PINV = locoman::utils::Pinv_trunc_SVD( map_l_fcToSens, 1E-10 ) ; //*  ft_l_ankle  ;  // yarp::math::pinv( map_l_fcToSens, 1E-6)  *  ft_l_ankle     ;
  map_r_fcToSens_PINV = locoman::utils::Pinv_trunc_SVD( map_r_fcToSens, 1E-10 ) ; // *  ft_r_ankle  ;  */// yarp::math::pinv( map_r_fcToSens, 1E-6)  *  ft_r_ankle     ;
  map_l_hand_fcToSens_PINV = locoman::utils::Pinv_trunc_SVD( map_l_hand_fcToSens, 1E-10 ) ; //*  ft_l_wrist  ;  // yarp::math::pinv( map_l_fcToSens, 1E-6)  *  ft_l_ankle     ;
  map_r_hand_fcToSens_PINV = locoman::utils::Pinv_trunc_SVD( map_r_hand_fcToSens, 1E-10 ) ; // 
    
  T_w_aw_0.zero()     ; //= locoman::utils::AW_world_posture(model, robot) ;
  T_aw_w_0.zero()  ; //= locoman::utils::iHomogeneous(T_w_aw_0) ;    

  //-------------------------------------------------------------------------------------------------------------    
  // Defining Useful Transformations
  T_w_waist_0.zero()   ; // = model.iDyn3_model.getPosition(waist_index) ;  
  T_w_l_ankle_0.zero()   ; // = model.iDyn3_model.getPosition(l_ankle_index) ;
  T_w_l_c1_0.zero()    ; //= model.iDyn3_model.getPosition(l_c1_index)    ;    
  T_w_l_c2_0.zero()    ; // = model.iDyn3_model.getPosition(l_c2_index)    ;  
  T_w_l_c3_0.zero()   ; // = model.iDyn3_model.getPosition(l_c3_index)    ;
  T_w_l_c4_0.zero()    ; // = model.iDyn3_model.getPosition(l_c4_index)    ;    
    
  T_w_r_ankle_0.zero()  ; //= model.iDyn3_model.getPosition(r_ankle_index) ;
  T_w_r_c1_0.zero()    ; // = model.iDyn3_model.getPosition(r_c1_index)    ;    
  T_w_r_c2_0.zero()    ; // = model.iDyn3_model.getPosition(r_c2_index)    ;  
  T_w_r_c3_0.zero()   ; // = model.iDyn3_model.getPosition(r_c3_index)    ;
  T_w_r_c4_0.zero()    ; // = model.iDyn3_model.getPosition(r_c4_index)    ;   
    
  T_w_l_hand_0.zero()  ; //= model.iDyn3_model.getPosition( l_hand_index ) ;
  T_w_r_hand_0.zero() ;   

  T_w_l_wrist_0.zero()  ;
  T_w_l1_hand_0.zero()  ;
  T_w_l2_hand_0.zero()  ;
  T_w_l3_hand_0.zero()  ;
  T_w_l4_hand_0.zero()  ;
    
  T_w_r_wrist_0.zero()  ;
  T_w_r1_hand_0.zero()  ;
  T_w_r2_hand_0.zero()  ;
  T_w_r3_hand_0.zero()  ;
  T_w_r4_hand_0.zero()  ;
  
  // -----------------------------------------------------------------------
  T_waist_w_0.zero()  ;
  T_l_ankle_w_0.zero()  ;
  T_l_c1_w_0.zero()  ;
  T_l_c2_w_0.zero()  ;
  T_l_c3_w_0.zero()  ;
  T_l_c4_w_0.zero()  ;
    
  T_r_ankle_w_0.zero()  ;
  T_r_c1_w_0.zero()  ;
  T_r_c2_w_0.zero()  ;
  T_r_c3_w_0.zero()  ;
  T_r_c4_w_0.zero()  ;

  T_l_wrist_w_0.zero()  ;
  T_l1_hand_w_0.zero()  ;
  T_l2_hand_w_0.zero()  ;
  T_l3_hand_w_0.zero()  ;
  T_l4_hand_w_0.zero()  ;
      
  T_r_wrist_w_0.zero()  ;
  T_r1_hand_w_0.zero()  ;
  T_r2_hand_w_0.zero()  ;
  T_r3_hand_w_0.zero()  ;
  T_r4_hand_w_0.zero()  ;

  //---------------------------------------------------------------------
  
  T_l_hand_w_0.zero()  ;
  T_r_hand_w_0.zero()  ;
  
  T_aw_l_c1_0.zero()  ;
  T_aw_l_c2_0.zero()  ;
  T_aw_l_c3_0.zero()  ;
  T_aw_l_c4_0.zero()  ;

  T_aw_r_c1_0.zero()  ;
  T_aw_r_c2_0.zero()  ;
  T_aw_r_c3_0.zero()  ;
  T_aw_r_c4_0.zero()  ;

  T_aw_l1_hand_0.zero()  ;
  T_aw_l2_hand_0.zero()  ;
  T_aw_l3_hand_0.zero()  ;
  T_aw_l4_hand_0.zero()  ;

  T_aw_r1_hand_0.zero()  ;
  T_aw_r2_hand_0.zero()  ;
  T_aw_r3_hand_0.zero()  ;
  T_aw_r4_hand_0.zero()  ;
    
   //  Jacobian Matrices 
  // J_feet.zero() ; 
  Complete_Jac_feet.zero(); 
  Complete_Jac_f_h.zero(); ;
  J_c_feet.zero()    ; 
  S_c_feet_T.zero()  ;
  S_c_feet.zero()    ;
  
  J_c_f_h.zero()   ; //  = Complete_Jac_f_h.submatrix( 0,  Complete_Jac_f_h.rows()-1 , 6, Complete_Jac_f_h.cols()-1 ) ;
  S_c_f_h_T.zero() ; // = Complete_Jac_f_h.submatrix( 0,  Complete_Jac_f_h.rows()-1 , 0, 5 ) ;
  S_c_f_h.zero()   ;
  
  J_l_c1_mix_0.zero()  ;
  J_l_c2_mix_0.zero()  ;
  J_l_c3_mix_0.zero()  ;
  J_l_c4_mix_0.zero()  ;
  
  J_r_c1_mix_0.zero()  ;
  J_r_c2_mix_0.zero()  ;
  J_r_c3_mix_0.zero()  ;
  J_r_c4_mix_0.zero()  ;
  
  J_l_hand_mix_0.zero()  ;
  J_r_hand_mix_0.zero()  ;  
  J_l1_hand_mix_0.zero()  ;
  J_l2_hand_mix_0.zero()  ;
  J_l3_hand_mix_0.zero()  ;
  J_l4_hand_mix_0.zero()  ;

  J_r1_hand_mix_0.zero()  ;
  J_r2_hand_mix_0.zero()  ;
  J_r3_hand_mix_0.zero()  ;
  J_r4_hand_mix_0.zero()  ;

 // -------------------------------------------
  
  J_l_c1_body_0.zero()  ;
  J_l_c2_body_0.zero()  ;
  J_l_c3_body_0.zero()  ;
  J_l_c4_body_0.zero()  ;

  J_r_c1_body_0.zero()  ;
  J_r_c2_body_0.zero()  ;
  J_r_c3_body_0.zero()  ;
  J_r_c4_body_0.zero()  ;

  J_l_hand_body_0.zero()  ;
  J_r_hand_body_0.zero()  ;

  J_l1_hand_body_0.zero()  ;
  J_l2_hand_body_0.zero()  ;
  J_l3_hand_body_0.zero()  ;
  J_l4_hand_body_0.zero()  ;

  J_r1_hand_body_0.zero()  ;
  J_r2_hand_body_0.zero()  ;
  J_r3_hand_body_0.zero()  ;
  J_r4_hand_body_0.zero()  ;
  
  ////---------------------------------------------------------------------------------------------------------------------------------------------------------------  
  J_aw_l_c1_spa_0.zero()  ;
  J_aw_l_c2_spa_0.zero()  ;
  J_aw_l_c3_spa_0.zero()  ;
  J_aw_l_c4_spa_0.zero()  ;

  J_aw_r_c1_spa_0.zero()  ;
  J_aw_r_c2_spa_0.zero()  ;
  J_aw_r_c3_spa_0.zero()  ;
  J_aw_r_c4_spa_0.zero()  ;

  J_aw_l1_hand_spa_0.zero()  ;
  J_aw_l2_hand_spa_0.zero()  ;
  J_aw_l3_hand_spa_0.zero()  ;
  J_aw_l4_hand_spa_0.zero()  ;
 
  J_aw_r1_hand_spa_0.zero()  ;
  J_aw_r2_hand_spa_0.zero()  ;
  J_aw_r3_hand_spa_0.zero()  ;
  J_aw_r4_hand_spa_0.zero()  ;
  
  ////------------------------------------------------
  Q_mg.zero();
  
  Q_aw_l_c1.zero()  ;
  Q_aw_l_c2.zero()  ;
  Q_aw_l_c3.zero()  ;
  Q_aw_l_c4.zero()  ;

  Q_aw_r_c1.zero()  ;
  Q_aw_r_c2.zero()  ;
  Q_aw_r_c3.zero()  ;
  Q_aw_r_c4.zero()  ;

  Q_aw_l1_hand.zero()  ;
  Q_aw_l2_hand.zero()  ;
  Q_aw_l3_hand.zero()  ;
  Q_aw_l4_hand.zero()  ;
  
  Q_aw_r1_hand.zero()  ;
  Q_aw_r2_hand.zero()  ;
  Q_aw_r3_hand.zero()  ;
  Q_aw_r4_hand.zero()  ;
    
  Q_aw_l_tot.zero()  ;
  Q_aw_r_tot.zero()  ;
  Q_aw_l_hand_tot.zero()  ;
  Q_aw_r_hand_tot.zero()  ;
  Q_aw_c.zero()  ;
  U_aw_s_cont.zero()  ;
  Q_aw_s_cont.zero()  ;
  Q_aw_c_f_rh.zero()  ;
  U_aw_s_c_f_rh.zero()  ;
  Q_aw_s_c_f_rh.zero()  ;
  
  Q_aw_c_f_h.zero() ;    // = Q_aw_c + Q_aw_l_hand_tot + Q_aw_r_hand_tot ;
  U_aw_s_f_h.zero() ;                  // = Q_aw_c_f_h.submatrix( 0 ,  5 , 0, 5) ;     
  Q_aw_s_f_h.zero() ;
  
  // ---------------------------------------------------------
  FLMM.zero()  ;   //   (30, size_q + 30)  ,  //= locoman::utils::FLMM_redu(J_c, S_c, Q_aw_s_cont, U_aw_s_cont, Kc ) ;
  cFLMM.zero()  ;   //   ( 30, size_q + 30 ) ,
  Rf.zero()  ;   //   (24, size_q)   ,
  Rf_filt.zero()  ;   //   (24, size_q) ,
  Rf_filt_pinv.zero()  ;   //   ( size_q, 24) ,
  Rf_filt_f_h.zero() ;
  Rf_feet_old.zero()  ; 
  Rf_feet_old_filt.zero(); 
  Rf_old_locoman.zero() ;
  Big_J_new.zero() ;
  Big_Rf_new.zero() ;
  // 
  d_fc_des_to_world.zero();  ;
// 
  T_l_c1_r_c1_loop.zero()  ;
  T_r_c1_l_c1_loop.zero()  ;
  T_l_c1_r_c1_loop.zero() ;
  T_r_c1_l_c1_loop.zero() ;  
  J_com_w.zero()  ;
  J_com_w_redu.zero()  ;
  J_com_aw.zero()  ;
  J_com_waist.zero()  ;
  
  J_com_w_q_i.zero()   ; //( 6, ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ) ) ;
  J_com_w_redu_q_i.zero() ;  //( 3,  ( size_q + 6 )  ) ; //( robot.getNumberOfKinematicJoints() + 6 ))   ;
  J_com_aw_q_i.zero() ;
  
  J_r_c1_aw.zero()  ;
  J_l_c1_aw.zero()  ;
  
  
  //--------------------------------------------
  // A new Robot
  
  //---------------------------------------
  
  
  return true;

}





//--------------------------------------------------------------------------
//--------------                      --------------------------------------
//--------------    !!!   RUN !!!     --------------------------------------
//--------------------------------------------------------------------------


void locoman_service_2_thread::run()
{   
 
    cout_print = 1 ;
    tic = locoman::utils::Tic() ;
    // Receiving form yarp port
   // v_from_locoman_thread = from_locoman_thread.read();
    receiving_q_vect = receiving_q.read() ;             // 
    q_sensed = *receiving_q_vect ;    // TODO predefine the vector

    robot.idynutils.updateiDyn3Model( q_sensed, true ); 
   
    //
    receiving_fc_vect = receiving_fc.read() ;            // 
    fc_received = *receiving_fc_vect ; // sensor measures (after filetering and offset);  
    
     if(cout_print){ std::cout << "q_sensed = "  << q_sensed.toString() << std::endl ; 
		     std::cout << "fc_received = "  << fc_received.toString() << std::endl ;  }
  
    fc_feet  = fc_received.subVector(  0,23   )  ;
    fc_hands = fc_received.subVector(  24,47  ) ;;
    
    fc_l_foot = fc_feet.subVector(  0,11  ) ;
    fc_r_foot = fc_feet.subVector( 12,23  ) ;
    
    fc_l_hand = fc_hands.subVector(  0,11  ) ;
    fc_r_hand = fc_hands.subVector( 12,23  ) ;
    
    fc_l1_foot_filt = fc_l_foot.subVector( 0,2 )  ;
    fc_l2_foot_filt = fc_l_foot.subVector( 3,5 )  ;
    fc_l3_foot_filt = fc_l_foot.subVector( 6,8 )  ;
    fc_l4_foot_filt = fc_l_foot.subVector( 9,11)  ;

    fc_r1_foot_filt = fc_r_foot.subVector( 0,2 )  ;
    fc_r2_foot_filt = fc_r_foot.subVector( 3,5 )  ;
    fc_r3_foot_filt = fc_r_foot.subVector( 6,8 )  ;
    fc_r4_foot_filt = fc_r_foot.subVector( 9,11)  ;
    
    fc_l1_hand_filt = fc_l_hand.subVector( 0,2 )  ;
    fc_l2_hand_filt = fc_l_hand.subVector( 0,2 )  ;
    fc_l3_hand_filt = fc_l_hand.subVector( 0,2 )  ;
    fc_l4_hand_filt = fc_l_hand.subVector( 0,2 )  ;
    
    fc_r1_hand_filt = fc_r_hand.subVector( 0,2 )  ;
    fc_r2_hand_filt = fc_r_hand.subVector( 0,2 )  ;
    fc_r3_hand_filt = fc_r_hand.subVector( 0,2 )  ;
    fc_r4_hand_filt = fc_r_hand.subVector( 0,2 )  ;    
  
    
    int l_sole_index = model.iDyn3_model.getLinkIndex("l_sole") ;
    int r_sole_index = model.iDyn3_model.getLinkIndex("r_sole") ;  // TODO: remove
  //-------------------------------------------------------------------------------------------------------------    
  // Using the old one
  // Defining Useful Transformations

  T_w_aw_0  = locoman::utils::AW_world_posture(model, robot) ;  
  T_aw_w_0 = locoman::utils::iHomogeneous(T_w_aw_0) ;    

  T_w_waist_0   = model.iDyn3_model.getPosition(waist_index) ;  
  T_w_l_ankle_0 = model.iDyn3_model.getPosition(l_ankle_index) ;
  T_w_l_c1_0    = model.iDyn3_model.getPosition(l_c1_index)    ;    
  T_w_l_c2_0    = model.iDyn3_model.getPosition(l_c2_index)    ;  
  T_w_l_c3_0    = model.iDyn3_model.getPosition(l_c3_index)    ;
  T_w_l_c4_0    = model.iDyn3_model.getPosition(l_c4_index)    ;    
    
  T_w_r_ankle_0 = model.iDyn3_model.getPosition(r_ankle_index) ;
  T_w_r_c1_0    = model.iDyn3_model.getPosition(r_c1_index)    ;    
  T_w_r_c2_0    = model.iDyn3_model.getPosition(r_c2_index)    ;  
  T_w_r_c3_0    = model.iDyn3_model.getPosition(r_c3_index)    ;
  T_w_r_c4_0    = model.iDyn3_model.getPosition(r_c4_index)    ;   
    
  T_w_l_hand_0  = model.iDyn3_model.getPosition( l_hand_index ) ;
  T_w_r_hand_0  = model.iDyn3_model.getPosition( r_hand_index ) ;   

  T_w_l_wrist_0 = model.iDyn3_model.getPosition(l_wrist_index) ;
  T_w_l1_hand_0 = model.iDyn3_model.getPosition(l_hand_c1_index)    ;    
  T_w_l2_hand_0 = model.iDyn3_model.getPosition(l_hand_c2_index)    ;  
  T_w_l3_hand_0 = model.iDyn3_model.getPosition(l_hand_c3_index)    ;
  T_w_l4_hand_0 = model.iDyn3_model.getPosition(l_hand_c4_index)    ;    
    
  T_w_r_wrist_0 = model.iDyn3_model.getPosition(r_wrist_index) ;
  T_w_r1_hand_0 = model.iDyn3_model.getPosition(r_hand_c1_index)    ;    
  T_w_r2_hand_0 = model.iDyn3_model.getPosition(r_hand_c2_index)    ;  
  T_w_r3_hand_0 = model.iDyn3_model.getPosition(r_hand_c3_index)    ;
  T_w_r4_hand_0 = model.iDyn3_model.getPosition(r_hand_c4_index)    ;     
  
  yarp::sig::Matrix T_w_l_sole_0 =  model.iDyn3_model.getPosition(l_sole_index)    ; 
  yarp::sig::Matrix T_w_r_sole_0 =  model.iDyn3_model.getPosition(r_sole_index)    ; // TODO: remove
  //-----------------------------------------------------------------------
  T_waist_w_0   = locoman::utils::iHomogeneous(T_w_waist_0)  ;
  T_l_ankle_w_0 = locoman::utils::iHomogeneous(T_w_l_ankle_0) ;
  T_l_c1_w_0    = locoman::utils::iHomogeneous(T_w_l_c1_0) ;    
  T_l_c2_w_0    = locoman::utils::iHomogeneous(T_w_l_c2_0) ;  
  T_l_c3_w_0    = locoman::utils::iHomogeneous(T_w_l_c3_0) ;
  T_l_c4_w_0    = locoman::utils::iHomogeneous(T_w_l_c4_0) ;   
    
  T_r_ankle_w_0 = locoman::utils::iHomogeneous(T_w_r_ankle_0) ;
  T_r_c1_w_0    = locoman::utils::iHomogeneous(T_w_r_c1_0) ;    
  T_r_c2_w_0    = locoman::utils::iHomogeneous(T_w_r_c2_0) ;  
  T_r_c3_w_0    = locoman::utils::iHomogeneous(T_w_r_c3_0) ;
  T_r_c4_w_0    = locoman::utils::iHomogeneous(T_w_r_c4_0) ;    

  T_l_wrist_w_0 = locoman::utils::iHomogeneous(T_w_l_wrist_0) ;
  T_l1_hand_w_0 = locoman::utils::iHomogeneous(T_w_l1_hand_0) ;    
  T_l2_hand_w_0 = locoman::utils::iHomogeneous(T_w_l2_hand_0) ;  
  T_l3_hand_w_0 = locoman::utils::iHomogeneous(T_w_l3_hand_0) ;
  T_l4_hand_w_0 = locoman::utils::iHomogeneous(T_w_l4_hand_0) ;    
    
  T_r_wrist_w_0 = locoman::utils::iHomogeneous(T_w_r_wrist_0) ;
  T_r1_hand_w_0 = locoman::utils::iHomogeneous(T_w_r1_hand_0) ;    
  T_r2_hand_w_0 = locoman::utils::iHomogeneous(T_w_r2_hand_0) ;  
  T_r3_hand_w_0 = locoman::utils::iHomogeneous(T_w_r3_hand_0) ;
  T_r4_hand_w_0 = locoman::utils::iHomogeneous(T_w_r4_hand_0) ;    

  
  yarp::sig::Matrix T_l_sole_w_0 = locoman::utils::iHomogeneous(T_w_l_sole_0) ; 
  yarp::sig::Matrix T_r_sole_w_0 = locoman::utils::iHomogeneous(T_w_r_sole_0) ; // TODO: remove

  
  // ---------------------------------------------------------------------
  
  
  T_l_hand_w_0 = locoman::utils::iHomogeneous(T_w_l_hand_0) ;
  T_r_hand_w_0 = locoman::utils::iHomogeneous(T_w_r_hand_0) ;   
   
  T_aw_l_c1_0 = T_aw_w_0 * T_w_l_c1_0 ;  // {AW} is fixed in a loop
  T_aw_l_c2_0 = T_aw_w_0 * T_w_l_c2_0 ;  // in every loop the floating base is re-initialized 
  T_aw_l_c3_0 = T_aw_w_0 * T_w_l_c3_0 ;  // coincident with {AW}
  T_aw_l_c4_0 = T_aw_w_0 * T_w_l_c4_0 ;

  T_aw_r_c1_0 = T_aw_w_0 * T_w_r_c1_0 ;
  T_aw_r_c2_0 = T_aw_w_0 * T_w_r_c2_0 ;
  T_aw_r_c3_0 = T_aw_w_0 * T_w_r_c3_0 ;
  T_aw_r_c4_0 = T_aw_w_0 * T_w_r_c4_0 ; 

  T_aw_l1_hand_0 = T_aw_w_0 * T_w_l1_hand_0 ;  // {AW} is fixed in a loop
  T_aw_l2_hand_0 = T_aw_w_0 * T_w_l2_hand_0 ;  // in every loop the floating base is re-initialized 
  T_aw_l3_hand_0 = T_aw_w_0 * T_w_l3_hand_0 ;  // coincident with {AW}
  T_aw_l4_hand_0 = T_aw_w_0 * T_w_l4_hand_0 ;

  T_aw_r1_hand_0 = T_aw_w_0 * T_w_r1_hand_0 ;
  T_aw_r2_hand_0 = T_aw_w_0 * T_w_r2_hand_0 ;
  T_aw_r3_hand_0 = T_aw_w_0 * T_w_r3_hand_0 ;
  T_aw_r4_hand_0 = T_aw_w_0 * T_w_r4_hand_0 ; 
  
  yarp::sig::Matrix T_aw_l_sole_0 = T_aw_w_0*T_w_l_sole_0 ; 
  yarp::sig::Matrix T_aw_r_sole_0 = T_aw_w_0*T_w_r_sole_0 ;  // TODO: remove

  //-----------------------------------------------------
  model.iDyn3_model.getJacobian( l_c1_index, J_l_c1_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
  model.iDyn3_model.getJacobian( l_c2_index, J_l_c2_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
  model.iDyn3_model.getJacobian( l_c3_index, J_l_c3_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
  model.iDyn3_model.getJacobian( l_c4_index, J_l_c4_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian

  model.iDyn3_model.getJacobian( r_c1_index, J_r_c1_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
  model.iDyn3_model.getJacobian( r_c2_index, J_r_c2_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
  model.iDyn3_model.getJacobian( r_c3_index, J_r_c3_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
  model.iDyn3_model.getJacobian( r_c4_index, J_r_c4_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
 
  model.iDyn3_model.getJacobian( l_hand_c1_index, J_l1_hand_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
  model.iDyn3_model.getJacobian( l_hand_c2_index, J_l2_hand_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
  model.iDyn3_model.getJacobian( l_hand_c3_index, J_l3_hand_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
  model.iDyn3_model.getJacobian( l_hand_c4_index, J_l4_hand_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian

  model.iDyn3_model.getJacobian( r_hand_c1_index, J_r1_hand_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
  model.iDyn3_model.getJacobian( r_hand_c2_index, J_r2_hand_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
  model.iDyn3_model.getJacobian( r_hand_c3_index, J_r3_hand_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
  model.iDyn3_model.getJacobian( r_hand_c4_index, J_r4_hand_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
    
  model.iDyn3_model.getJacobian( l_hand_index, J_l_hand_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
  model.iDyn3_model.getJacobian( r_hand_index, J_r_hand_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian

  yarp::sig::Matrix J_l_sole_mix_0(6,size_q+6 ) ;
  yarp::sig::Matrix J_r_sole_mix_0(6,size_q+6 ) ;
  model.iDyn3_model.getJacobian( l_sole_index, J_l_sole_mix_0, false  ) ; 
  model.iDyn3_model.getJacobian( r_sole_index, J_r_sole_mix_0, false  ) ; // TODO: remove

    
  
  //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  
  J_l_c1_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_c1_w_0), zero_3 ))* J_l_c1_mix_0 ;
  J_l_c2_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_c2_w_0), zero_3 ))* J_l_c2_mix_0 ;
  J_l_c3_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_c3_w_0), zero_3 ))* J_l_c3_mix_0 ;
  J_l_c4_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_c4_w_0), zero_3 ))* J_l_c4_mix_0 ;

  J_r_c1_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_c1_w_0), zero_3 ))* J_r_c1_mix_0 ;
  J_r_c2_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_c2_w_0), zero_3 ))* J_r_c2_mix_0 ;
  J_r_c3_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_c3_w_0), zero_3 ))* J_r_c3_mix_0 ;
  J_r_c4_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_c4_w_0), zero_3 ))* J_r_c4_mix_0 ;

  J_l_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_hand_w_0), zero_3 ))* J_l_hand_mix_0 ;
  J_r_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_hand_w_0), zero_3 ))* J_r_hand_mix_0 ;

  J_l1_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l1_hand_w_0), zero_3 ))* J_l1_hand_mix_0 ;
  J_l2_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l2_hand_w_0), zero_3 ))* J_l2_hand_mix_0 ;
  J_l3_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l3_hand_w_0), zero_3 ))* J_l3_hand_mix_0 ;
  J_l4_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l4_hand_w_0), zero_3 ))* J_l4_hand_mix_0 ;

  J_r1_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r1_hand_w_0), zero_3 ))* J_r1_hand_mix_0 ;
  J_r2_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r2_hand_w_0), zero_3 ))* J_r2_hand_mix_0 ;
  J_r3_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r3_hand_w_0), zero_3 ))* J_r3_hand_mix_0 ;
  J_r4_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r4_hand_w_0), zero_3 ))* J_r4_hand_mix_0 ;
  
  yarp::sig::Matrix J_l_sole_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_sole_w_0), zero_3 ))* J_l_sole_mix_0  ;
  yarp::sig::Matrix J_r_sole_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_sole_w_0), zero_3 ))* J_r_sole_mix_0  ;
  //TOO: remov
  
  //---------------------------------------------------------------------------------------------------------------------------------------------------------------
  //Introducing Spatial Jacobian terms: Fixed base in {AW}
  
  J_aw_l_c1_spa_0 = locoman::utils::Adjoint(T_aw_l_c1_0)* J_l_c1_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c1_spa_0 ;// locoman::utils::Adjoint(T_aw_l_c1_0)* J_l_c1_body_0
  J_aw_l_c2_spa_0 = locoman::utils::Adjoint(T_aw_l_c2_0)* J_l_c2_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c2_spa_0 ;
  J_aw_l_c3_spa_0 = locoman::utils::Adjoint(T_aw_l_c3_0)* J_l_c3_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c3_spa_0 ;
  J_aw_l_c4_spa_0 = locoman::utils::Adjoint(T_aw_l_c4_0)* J_l_c4_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c4_spa_0 ;

  J_aw_r_c1_spa_0 = locoman::utils::Adjoint(T_aw_r_c1_0)* J_r_c1_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c1_spa_0 ;
  J_aw_r_c2_spa_0 = locoman::utils::Adjoint(T_aw_r_c2_0)* J_r_c2_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c2_spa_0 ;
  J_aw_r_c3_spa_0 = locoman::utils::Adjoint(T_aw_r_c3_0)* J_r_c3_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c3_spa_0 ;
  J_aw_r_c4_spa_0 = locoman::utils::Adjoint(T_aw_r_c4_0)* J_r_c4_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c4_spa_0 ;

  J_aw_l1_hand_spa_0 = locoman::utils::Adjoint(T_aw_l1_hand_0)* J_l1_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c1_spa_0 ;// locoman::utils::Adjoint(T_aw_l_c1_0)* J_l_c1_body_0
  J_aw_l2_hand_spa_0 = locoman::utils::Adjoint(T_aw_l2_hand_0)* J_l2_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c2_spa_0 ;
  J_aw_l3_hand_spa_0 = locoman::utils::Adjoint(T_aw_l3_hand_0)* J_l3_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c3_spa_0 ;
  J_aw_l4_hand_spa_0 = locoman::utils::Adjoint(T_aw_l4_hand_0)* J_l4_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c4_spa_0 ;

  J_aw_r1_hand_spa_0 = locoman::utils::Adjoint(T_aw_r1_hand_0)* J_r1_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c1_spa_0 ;
  J_aw_r2_hand_spa_0 = locoman::utils::Adjoint(T_aw_r2_hand_0)* J_r2_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c2_spa_0 ;
  J_aw_r3_hand_spa_0 = locoman::utils::Adjoint(T_aw_r3_hand_0)* J_r3_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c3_spa_0 ;
  J_aw_r4_hand_spa_0 = locoman::utils::Adjoint(T_aw_r4_hand_0)* J_r4_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c4_spa_0 ; 
 
  yarp::sig::Matrix J_aw_l_sole_spa_0 = locoman::utils::Adjoint(T_aw_l_sole_0)* J_l_sole_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c3_spa_0 ;
  yarp::sig::Matrix J_aw_r_sole_spa_0 = locoman::utils::Adjoint(T_aw_r_sole_0)* J_r_sole_mix_0 ;
  //TODO: remove
  //---------------------------------------------------------
  
  J_aw_l_c1_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
  J_aw_l_c2_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
  J_aw_l_c3_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
  J_aw_l_c4_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
 
  J_aw_r_c1_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
  J_aw_r_c2_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
  J_aw_r_c3_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
  J_aw_r_c4_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
 
  J_aw_l1_hand_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
  J_aw_l2_hand_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
  J_aw_l3_hand_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
  J_aw_l4_hand_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
 
  J_aw_r1_hand_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
  J_aw_r2_hand_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
  J_aw_r3_hand_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
  J_aw_r4_hand_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
  
  J_aw_l_sole_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;  // TODO : remove
  J_aw_r_sole_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
  
  //Recomputing body Jacobian

  J_l_c1_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_l_c1_0) ) * J_aw_l_c1_spa_0 ;
  J_l_c2_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_l_c2_0) ) * J_aw_l_c2_spa_0 ;
  J_l_c3_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_l_c3_0) ) * J_aw_l_c3_spa_0 ;
  J_l_c4_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_l_c4_0) ) * J_aw_l_c4_spa_0 ;

  J_r_c1_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_r_c1_0) ) * J_aw_r_c1_spa_0 ;
  J_r_c2_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_r_c2_0) ) * J_aw_r_c2_spa_0 ;
  J_r_c3_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_r_c3_0) ) * J_aw_r_c3_spa_0 ;
  J_r_c4_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_r_c4_0) ) * J_aw_r_c4_spa_0 ;

  J_l1_hand_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_l1_hand_0) ) * J_aw_l1_hand_spa_0 ;
  J_l2_hand_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_l2_hand_0) ) * J_aw_l2_hand_spa_0 ;
  J_l3_hand_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_l3_hand_0) ) * J_aw_l3_hand_spa_0 ;
  J_l4_hand_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_l4_hand_0) ) * J_aw_l4_hand_spa_0 ;

  J_r1_hand_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_r1_hand_0) ) * J_aw_r1_hand_spa_0 ;
  J_r2_hand_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_r2_hand_0) ) * J_aw_r2_hand_spa_0 ;
  J_r3_hand_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_r3_hand_0) ) * J_aw_r3_hand_spa_0 ;
  J_r4_hand_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_r4_hand_0) ) * J_aw_r4_hand_spa_0 ;
  
  J_l_sole_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_l_sole_0) ) * J_aw_l_sole_spa_0 ;
  J_r_sole_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_r_sole_0) ) * J_aw_r_sole_spa_0 ;
 //------------------------------------------------------------------------------------------------------------
   
  //Stance and Jacobian Matrices
       
  Complete_Jac_feet.setSubmatrix( B.transposed()*J_l_c1_body_0 , 0 ,0 )  ;
  Complete_Jac_feet.setSubmatrix( B.transposed()*J_l_c2_body_0 , B.cols() ,0 )  ;
  Complete_Jac_feet.setSubmatrix( B.transposed()*J_l_c3_body_0 , 2*B.cols() ,0 )  ;
  Complete_Jac_feet.setSubmatrix( B.transposed()*J_l_c4_body_0 , 3*B.cols() ,0 )  ;

  Complete_Jac_feet.setSubmatrix( B.transposed()*J_r_c1_body_0 , 4*B.cols() ,0 )  ;
  Complete_Jac_feet.setSubmatrix( B.transposed()*J_r_c2_body_0 , 5*B.cols() ,0 )  ;
  Complete_Jac_feet.setSubmatrix( B.transposed()*J_r_c3_body_0 , 6*B.cols() ,0 )  ;
  Complete_Jac_feet.setSubmatrix( B.transposed()*J_r_c4_body_0 , 7*B.cols() ,0 )  ;

  J_c_feet   = Complete_Jac_feet.submatrix( 0,  Complete_Jac_feet.rows()-1 , 6, Complete_Jac_feet.cols()-1 ) ;
  S_c_feet_T = Complete_Jac_feet.submatrix( 0,  Complete_Jac_feet.rows()-1 , 0, 5 ) ;
  S_c_feet   = S_c_feet_T.transposed() ;
    
  Complete_Jac_f_h( 16*B.cols() , size_q + 6) ;
//   Complete_Jac_f_h.setSubmatrix( B.transposed()*J_l_c1_body_0 , 0 ,0 )  ;
//   Complete_Jac_f_h.setSubmatrix( B.transposed()*J_l_c2_body_0 , B.cols() ,0 )  ;
//   Complete_Jac_f_h.setSubmatrix( B.transposed()*J_l_c3_body_0 , 2*B.cols() ,0 )  ;
//   Complete_Jac_f_h.setSubmatrix( B.transposed()*J_l_c4_body_0 , 3*B.cols() ,0 )  ;
// 
//   Complete_Jac_f_h.setSubmatrix( B.transposed()*J_r_c1_body_0 , 4*B.cols() ,0 )  ;
//   Complete_Jac_f_h.setSubmatrix( B.transposed()*J_r_c2_body_0 , 5*B.cols() ,0 )  ;
//   Complete_Jac_f_h.setSubmatrix( B.transposed()*J_r_c3_body_0 , 6*B.cols() ,0 )  ;
//   Complete_Jac_f_h.setSubmatrix( B.transposed()*J_r_c4_body_0 , 7*B.cols() ,0 )  ;

  
  Complete_Jac_f_h.setSubmatrix( Complete_Jac_feet , 0 ,0 )  ;

  Complete_Jac_f_h.setSubmatrix( B.transposed()*J_l1_hand_body_0 , 8*B.cols() ,0 )  ;
  Complete_Jac_f_h.setSubmatrix( B.transposed()*J_l2_hand_body_0 , 9*B.cols() ,0 )  ;
  Complete_Jac_f_h.setSubmatrix( B.transposed()*J_l3_hand_body_0 , 10*B.cols() ,0 )  ;
  Complete_Jac_f_h.setSubmatrix( B.transposed()*J_l4_hand_body_0 , 11*B.cols() ,0 )  ; 
  
  Complete_Jac_f_h.setSubmatrix( B.transposed()*J_r1_hand_body_0 , 12*B.cols() ,0 )  ;
  Complete_Jac_f_h.setSubmatrix( B.transposed()*J_r2_hand_body_0 , 13*B.cols() ,0 )  ;
  Complete_Jac_f_h.setSubmatrix( B.transposed()*J_r3_hand_body_0 , 14*B.cols() ,0 )  ;
  Complete_Jac_f_h.setSubmatrix( B.transposed()*J_r4_hand_body_0 , 15*B.cols() ,0 )  ; 
  
  J_c_f_h   = Complete_Jac_f_h.submatrix( 0,  Complete_Jac_f_h.rows()-1 , 6, Complete_Jac_f_h.cols()-1 ) ;
  S_c_f_h_T = Complete_Jac_f_h.submatrix( 0,  Complete_Jac_f_h.rows()-1 , 0, 5 ) ;
  S_c_f_h   = S_c_f_h_T.transposed() ;

  //-------------------------------------------------------------------------------------------------------------
 // Defining derivative Terms
  

 // Computing Derivative Terms
  Q_aw_l_c1 = locoman::utils::Q_ci(J_aw_l_c1_spa_0, T_aw_l_c1_0, fc_l1_foot_filt ) ;
  Q_aw_l_c2 = locoman::utils::Q_ci(J_aw_l_c2_spa_0, T_aw_l_c2_0, fc_l2_foot_filt ) ; // (size_q+ 6, size_q + 6) ;
  Q_aw_l_c3 = locoman::utils::Q_ci(J_aw_l_c3_spa_0, T_aw_l_c3_0, fc_l3_foot_filt ) ; //(size_q+ 6, size_q + 6) ; 
  Q_aw_l_c4 = locoman::utils::Q_ci(J_aw_l_c4_spa_0, T_aw_l_c4_0, fc_l4_foot_filt ) ; //(size_q+ 6, size_q + 6) ;
  
  Q_aw_r_c1 = locoman::utils::Q_ci(J_aw_r_c1_spa_0, T_aw_r_c1_0, fc_r1_foot_filt ) ; //(size_q+ 6, size_q + 6) ;
  Q_aw_r_c2 = locoman::utils::Q_ci(J_aw_r_c2_spa_0, T_aw_r_c2_0, fc_r2_foot_filt ) ; //(size_q+ 6, size_q + 6) ;
  Q_aw_r_c3 = locoman::utils::Q_ci(J_aw_r_c3_spa_0, T_aw_r_c3_0, fc_r3_foot_filt ) ; //(size_q+ 6, size_q + 6) ; 
  Q_aw_r_c4 = locoman::utils::Q_ci(J_aw_r_c4_spa_0, T_aw_r_c4_0, fc_r4_foot_filt ) ; //(size_q+ 6, size_q + 6) ;
  
  Q_aw_l1_hand = locoman::utils::Q_ci(J_aw_l1_hand_spa_0, T_aw_l1_hand_0, fc_l1_hand_filt ) ; //(size_q+ 6, size_q + 6) ;  
  Q_aw_l2_hand = locoman::utils::Q_ci(J_aw_l2_hand_spa_0, T_aw_l2_hand_0, fc_l2_hand_filt ) ; //(size_q+ 6, size_q + 6) ;
  Q_aw_l3_hand = locoman::utils::Q_ci(J_aw_l3_hand_spa_0, T_aw_l3_hand_0, fc_l3_hand_filt ) ; //(size_q+ 6, size_q + 6) ; 
  Q_aw_l4_hand = locoman::utils::Q_ci(J_aw_l4_hand_spa_0, T_aw_l4_hand_0, fc_l4_hand_filt ) ; //(size_q+ 6, size_q + 6) ;
  
  Q_aw_r1_hand = locoman::utils::Q_ci(J_aw_r1_hand_spa_0, T_aw_r1_hand_0, fc_r1_hand_filt ) ; //(size_q+ 6, size_q + 6) ;  
  Q_aw_r2_hand = locoman::utils::Q_ci(J_aw_r2_hand_spa_0, T_aw_r2_hand_0, fc_r2_hand_filt ) ; //(size_q+ 6, size_q + 6) ;
  Q_aw_r3_hand = locoman::utils::Q_ci(J_aw_r3_hand_spa_0, T_aw_r3_hand_0, fc_r3_hand_filt ) ; //(size_q+ 6, size_q + 6) ; 
  Q_aw_r4_hand = locoman::utils::Q_ci(J_aw_r4_hand_spa_0, T_aw_r4_hand_0, fc_r4_hand_filt ) ; //(size_q+ 6, size_q + 6) ;

  //-----------------------------------------------------------------------------------------------------------------------------
  // Derivative terms of the gravitational term
  
  //yarp::sig::Vector mg_vect(3,0.0) ;
  mg_vect[2] = -mg ;
  Q_mg = 0.0*locoman::utils::Q_mg( q_sensed, mg_vect, T_aw_w_0 , robot) ; // TODO debug this function
    
  //------------------------------------------------------------------------------------------------------
  // FEET 
  Q_aw_l_tot = Q_aw_l_c1 + Q_aw_l_c2 + Q_aw_l_c3 + Q_aw_l_c4;
  Q_aw_r_tot = Q_aw_r_c1 + Q_aw_r_c2 + Q_aw_r_c3 + Q_aw_r_c4;
  
  yarp::sig::Matrix Q_aw_old =  Q_aw_l_tot + Q_aw_r_tot ;  
  yarp::sig::Matrix U_aw_s_old = Q_aw_old.submatrix( 0 ,  5 , 0, 5) ;     
  yarp::sig::Matrix Q_aw_s_old = Q_aw_old.submatrix( 0  , 5,  6,  (Q_aw_old.cols()-1)  ) ;
  
  Q_aw_c =  Q_aw_l_tot + Q_aw_r_tot + Q_mg ;  
 
  U_aw_s_cont = Q_aw_c.submatrix( 0 ,  5 , 0, 5) ;     
  Q_aw_s_cont = Q_aw_c.submatrix( 0  , 5,  6,  (Q_aw_c.cols()-1)  ) ;

  // FEET AND HANDS 
 
  Q_aw_c_f_h = Q_aw_c + Q_aw_l_hand_tot + Q_aw_r_hand_tot ;
  U_aw_s_f_h = Q_aw_c_f_h.submatrix( 0 ,  5 , 0, 5) ;     
  Q_aw_s_f_h = Q_aw_c_f_h.submatrix( 0  , 5,  6,  (Q_aw_c_f_h.cols()-1)  ) ;

  //-----------------------------------------------------------------------------------------------------
  
  //-----------------------------------------------------------------------------
  /*  std::cout << "J_c_feet.toString()  = " <<  std::endl << J_c_feet.toString() << std::endl ;    
   std::cout << "S_c_feet.toString()  = " <<  std::endl << S_c_feet.toString() << std::endl ;    
   std::cout << "Q_aw_s_cont.toString()  = " <<  std::endl << Q_aw_s_cont.toString() << std::endl ;    
   std::cout << "U_aw_s_cont.toString()  = " <<  std::endl << U_aw_s_cont.toString() << std::endl ;    
   std::cout << "Kc.toString()  = " <<  std::endl << Kc.toString() << std::endl ;    
  */
  
  //-------------------------------------------------------------------------------------
  model.iDyn3_model.getCOMJacobian(J_com_w) ;
  J_com_w_redu = J_com_w.submatrix(0,2 , 0 , J_com_w.cols()-1 ) ;  
  J_com_aw     = locoman::utils::getRot(T_aw_w_0) *J_com_w_redu; 
  J_com_waist  = locoman::utils::getRot(T_waist_w_0)*J_com_w_redu;
  
  //-----------------------------------------------------------------------
  
  Rf_filt = locoman::utils::Rf_redu(J_c_feet, S_c_feet, Q_aw_s_cont, U_aw_s_cont, Kc ) ; 
    
  
  FLMM = locoman::utils::FLMM_redu(J_c_feet, S_c_feet, Q_aw_s_cont, U_aw_s_cont, Kc ) ;
  cFLMM = locoman::utils::Pinv_trunc_SVD(FLMM.submatrix(0, FLMM.rows()-1 , 0, FLMM.rows()-1), 1E-10 ) * FLMM ;
  
  Rf_filt = cFLMM.submatrix(0, size_fc-1, cFLMM.cols()-size_q, cFLMM.cols()-1) ;  
  Rf_filt = locoman::utils::filter_SVD( Rf_filt,  1E-10);
  
  
  /*
   FLMM_old  = locoman::utils::FLMM_redu(J_c_feet, S_c_feet, Q_aw_s_old, U_aw_s_old, Kc ) ;
  cFLMM_old = locoman::utils::Pinv_trunc_SVD(FLMM_old.submatrix(0, FLMM_old.rows()-1 , 0, FLMM_old.rows()-1), 1E-10 ) * FLMM_old;
   
  Rf_feet_old = cFLMM_old.submatrix(0, size_fc-1, cFLMM_old.cols()-size_q, cFLMM_old.cols()-1) ;  
  Rf_feet_old_filt = locoman::utils::filter_SVD( Rf_feet_old,  1E-10); */
  
  
  /* if(cout_print){
    std::cout << "Rf_redu = " <<  std::endl << Rf_filt.toString() << std::endl ;    
    std::cout << "Rf_filt.rows() = " << Rf_filt.rows() << std::endl ;
    std::cout << "Rf_filt.cols() = " << Rf_filt.cols() << std::endl ;
    std::cout << "Rf_filt.toString() direct = " <<  std::endl << Rf_filt.toString() << std::endl ;    
    }*/   
  // std::cout << "Rf_filt.toString() FILTERED = " <<  std::endl << Rf_filt.toString() << std::endl ;    

  /*-------------------------------------------------------------------------------
  
  Rf_filt_f_h = locoman::utils::Rf_redu(J_c_f_h, S_c_f_h, Q_aw_s_f_h, U_aw_s_f_h, Kc_f_h ) ; 
  Rf_filt_f_h = locoman::utils::filter_SVD( Rf_filt_f_h , 1E-10); 

  //------------------------------------------------------------------------
/*
  yarp::sig::Vector d_q_dsp_6 = q_sensed ; // a test... adn remove it!
  d_q_dsp_6[0] = 1.0 ;
  d_q_dsp_6[d_q_dsp_6.length()-1] = 1.0 ;  
  
  if(cout_print){ std::cout << " d_q_dsp_6  =  "<< std::endl << d_q_dsp_6.toString() << std::endl  ;  }
  */
  //------------------------------------------------------------------------
  // ... sending back 

  // Prototype of the sending back port
//   yarp::sig::Vector &data = to_locoman_thread.prepare();
//   data.resize(q_sensed.size());
//   data = q_sensed;
//   to_locoman_thread.write();
  

  yarp::sig::Matrix &data_Matrix = to_locoman_Matrix.prepare();
  data_Matrix.resize(Rf_filt.rows(), Rf_filt.cols());
  data_Matrix = Rf_filt ;
  to_locoman_Matrix.write();  
  
  
  // Big_J_new[0][0] =1.0 ;
  Big_J_new.setSubmatrix(J_l_hand_body_0, 0,0 ) ;
  Big_J_new.setSubmatrix(J_r_hand_body_0, 6,0 ) ;
  Big_J_new.setSubmatrix(J_l_sole_body_0,  12,0 ) ;     // J_l_c1_body_0  // TODO : remove
  Big_J_new.setSubmatrix(J_r_sole_body_0,  18,0 ) ;     // J_r_c1_body_0
  Big_J_new.setSubmatrix(J_com_waist,    24,0 ) ;
  
  yarp::sig::Matrix &data_Big_J = to_locoman_Big_J.prepare();
  data_Big_J.resize(Big_J_new.rows(), Big_J_new.cols());
  data_Big_J = Big_J_new ;
  to_locoman_Big_J.write();  
    
//   Big_Rf_new = locoman::utils::Rf_redu(J_c_f_h, S_c_f_h, Q_aw_s_cont, U_aw_s_cont, Kc ) ; 
//   Big_Rf_new.zero() ;
  Big_Rf_new = Rf_filt_f_h ;
  yarp::sig::Matrix &data_Big_Rf = to_locoman_Big_Rf.prepare();
  data_Big_Rf.resize(Big_Rf_new.rows(), Big_Rf_new.cols());
  data_Big_Rf = Big_Rf_new ;
  to_locoman_Big_Rf.write();  
  
 // Big_Rf_new = Rf_filt_f_h ;
  yarp::sig::Matrix &data_multicontact_Big_Rf = to_multicontact_Big_Rf.prepare();
  data_multicontact_Big_Rf.resize(Big_Rf_new.rows(), Big_Rf_new.cols());
  data_multicontact_Big_Rf = Big_Rf_new ;
  to_multicontact_Big_Rf.write();  
  
  
  //--------------------------------------------------------------------
  // Rf_feet_old
  //-----------------------------------------------------------------------------------------------------

  FLMM_old  = locoman::utils::FLMM_redu(J_c_feet, S_c_feet, Q_aw_s_old, U_aw_s_old, Kc ) ;
  cFLMM_old = locoman::utils::Pinv_trunc_SVD(FLMM_old.submatrix(0, FLMM_old.rows()-1 , 0, FLMM_old.rows()-1), 1E-10 ) * FLMM_old;
   
  Rf_feet_old = cFLMM_old.submatrix(0, size_fc-1, cFLMM_old.cols()-size_q, cFLMM_old.cols()-1) ;  
  Rf_feet_old_filt = locoman::utils::filter_SVD( Rf_feet_old,  1E-10); 

  Rf_old_locoman = Rf_feet_old_filt ;
  yarp::sig::Matrix &data_Rf_old = to_locoman_Rf_old.prepare();
  data_Rf_old.resize(Rf_old_locoman.rows(), Rf_old_locoman.cols());
  data_Rf_old = Rf_old_locoman ;
  to_locoman_Rf_old.write();
  
  //----------------------------------------------------------------------
 

  
  
  toc = locoman::utils::Toc(tic) ;
  if(1){std::cout << "tic-toc = " << toc << " seconds" << std::endl ;}

    
    
    //------------------------------------------------------------------------------------------
    // The New Method for the Jacobian And Stance Matrix Computation
      
 //----------------------------------------------------------------------------------------- 
//   // UPDATING FRAMES CONFIGURATION
//   //  
//   // "Auxiliary World" Frame => {AW} // 
//   T_w_aw_0  = locoman::utils::AW_world_posture(model, robot) ;  
//   T_aw_w_0  = locoman::utils::iHomogeneous(T_w_aw_0) ;
//   
//   // Left Foot Frames  
//   T_w_l_c1_0 = model.iDyn3_model.getPosition(l_c1_index) ;    
//   T_w_l_c2_0 = model.iDyn3_model.getPosition(l_c2_index) ;  
//   T_w_l_c3_0 = model.iDyn3_model.getPosition(l_c3_index) ;
//   T_w_l_c4_0 = model.iDyn3_model.getPosition(l_c4_index) ;     
//   T_l_c1_w_0 = locoman::utils::iHomogeneous(T_w_l_c1_0)  ;        
//   T_aw_l_c1_0 =T_aw_w_0*T_w_l_c1_0  ;
//    //  Right Foot Frames
//   T_w_r_c1_0 = model.iDyn3_model.getPosition(r_c1_index) ;    
//   T_w_r_c2_0 = model.iDyn3_model.getPosition(r_c2_index) ;  
//   T_w_r_c3_0 = model.iDyn3_model.getPosition(r_c3_index) ;
//   T_w_r_c4_0 = model.iDyn3_model.getPosition(r_c4_index) ;    
//   T_r_c1_w_0 = locoman::utils::iHomogeneous(T_w_r_c1_0)  ;       
//   T_aw_r_c1_0 = T_aw_w_0*T_w_r_c1_0  ;
//   // -----------------------------------------------------------------------------
//   // Left Hand Frames
//   T_w_l1_hand_0 = model.iDyn3_model.getPosition(l_hand_c1_index)    ;  
//   T_w_l2_hand_0 = model.iDyn3_model.getPosition(l_hand_c2_index)    ;  
//   T_w_l3_hand_0 = model.iDyn3_model.getPosition(l_hand_c3_index)    ;
//   T_w_l4_hand_0 = model.iDyn3_model.getPosition(l_hand_c4_index)    ;  
//   T_aw_l1_hand_0 = T_aw_w_0*T_w_l1_hand_0  ;
//   T_l1_hand_w_0 = locoman::utils::iHomogeneous(T_w_l1_hand_0) ;    
//   // Right Hand
//   T_w_r1_hand_0 = model.iDyn3_model.getPosition(r_hand_c1_index)    ;  
//   T_w_r2_hand_0 = model.iDyn3_model.getPosition(r_hand_c2_index)    ;  
//   T_w_r3_hand_0 = model.iDyn3_model.getPosition(r_hand_c3_index)    ;
//   T_w_r4_hand_0 = model.iDyn3_model.getPosition(r_hand_c4_index)    ;  
//   T_aw_r1_hand_0 = T_aw_w_0*T_w_r1_hand_0  ;
//   T_r1_hand_w_0 = locoman::utils::iHomogeneous(T_w_r1_hand_0) ;  
//   
// //   ----------------------------------------------------------------------------------------- 
// //   UPDATING JACOBIAN MATRICES
// //   Left Foot Jacobians
//   model.iDyn3_model.getJacobian( l_c1_index, J_l_c1_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
//   J_l_c1_body_0   = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_c1_w_0), zero_3 ))* J_l_c1_mix_0 ;
//   J_aw_l_c1_spa_0 = locoman::utils::Adjoint(T_aw_l_c1_0)* J_l_c1_body_0 ; 
//   J_aw_l_c1_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
//   J_l_c1_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_l_c1_0) ) * J_aw_l_c1_spa_0 ;  
//   J_l_c2_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_w_l_c2_0)*T_w_l_c1_0 )* J_l_c1_body_0 ;
//   J_l_c3_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_w_l_c3_0)*T_w_l_c1_0 )* J_l_c1_body_0 ;  
//   J_l_c4_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_w_l_c4_0)*T_w_l_c1_0 )* J_l_c1_body_0 ;  
//   
// //   Right Foot Jacobian  
//   model.iDyn3_model.getJacobian( r_c1_index, J_r_c1_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
//   J_r_c1_body_0   = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_c1_w_0), zero_3 ))* J_r_c1_mix_0 ;
//   J_aw_r_c1_spa_0 = locoman::utils::Adjoint(T_aw_r_c1_0)* J_r_c1_body_0 ; 
//   J_aw_r_c1_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ; 
//   J_r_c1_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_r_c1_0) ) * J_aw_r_c1_spa_0 ;  
//   J_r_c2_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_w_r_c2_0)*T_w_r_c1_0 )* J_r_c1_body_0 ;
//   J_r_c3_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_w_r_c3_0)*T_w_r_c1_0 )* J_r_c1_body_0 ;  
//   J_r_c4_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_w_r_c4_0)*T_w_r_c1_0 )* J_r_c1_body_0 ;  
//    
// //   Left Hand Jacobians
//   model.iDyn3_model.getJacobian( l_hand_c1_index, J_l1_hand_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
//   J_l1_hand_body_0   = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l1_hand_w_0), zero_3 ))* J_l1_hand_mix_0 ;
//   J_aw_l1_hand_spa_0 = locoman::utils::Adjoint(T_aw_l1_hand_0)* J_l1_hand_body_0 ; 
//   J_aw_l1_hand_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
//   J_l1_hand_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_l1_hand_0) ) * J_aw_l1_hand_spa_0 ;  
//   J_l2_hand_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_w_l2_hand_0)*T_w_l1_hand_0 )* J_l1_hand_body_0 ;
//   J_l3_hand_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_w_l3_hand_0)*T_w_l1_hand_0 )* J_l1_hand_body_0 ;  
//   J_l4_hand_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_w_l4_hand_0)*T_w_l1_hand_0 )* J_l1_hand_body_0 ;  
//   
// //   Right Hand Jacobian  
//   model.iDyn3_model.getJacobian( r_hand_c1_index, J_r1_hand_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
//   J_r1_hand_body_0   = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r1_hand_w_0), zero_3 ))* J_r1_hand_mix_0 ;
//   J_aw_r1_hand_spa_0 = locoman::utils::Adjoint(T_aw_r1_hand_0)* J_r1_hand_body_0 ; 
//   J_aw_r1_hand_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
//   J_r1_hand_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_r1_hand_0) ) * J_aw_r1_hand_spa_0 ;  
//   J_r2_hand_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_w_r2_hand_0)*T_w_r1_hand_0 )* J_r1_hand_body_0 ;
//   J_r3_hand_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_w_r3_hand_0)*T_w_r1_hand_0 )* J_r1_hand_body_0 ;  
//   J_r4_hand_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_w_r4_hand_0)*T_w_r1_hand_0 )* J_r1_hand_body_0 ;  
//   
// /*  -------------------------------------------------------------------------------------------------------------------
//   --------------------------------------------------------------------------------------------------------------------
//   IMPOSTO FLMM etc solo per i piedi; 
//   
//   Stance and Jacobian Matrices*/  
//   
//   J_feet.setSubmatrix( B.transposed()*J_l_c1_body_0 , 0 ,0 )  ;
//   J_feet.setSubmatrix( B.transposed()*J_l_c2_body_0 , B.cols() ,0 )    ;
//   J_feet.setSubmatrix( B.transposed()*J_l_c3_body_0 , 2*B.cols() ,0 )  ;
//   J_feet.setSubmatrix( B.transposed()*J_l_c4_body_0 , 3*B.cols() ,0 )  ;
// 
//   J_feet.setSubmatrix( B.transposed()*J_r_c1_body_0 , 4*B.cols() ,0 )  ;
//   J_feet.setSubmatrix( B.transposed()*J_r_c2_body_0 , 5*B.cols() ,0 )  ;
//   J_feet.setSubmatrix( B.transposed()*J_r_c3_body_0 , 6*B.cols() ,0 )  ;
//   J_feet.setSubmatrix( B.transposed()*J_r_c4_body_0 , 7*B.cols() ,0 )  ;
// 
//   J_c   = J_feet.submatrix( 0,  J_feet.rows()-1 , 6, J_feet.cols()-1 ) ;
//   S_c_T = J_feet.submatrix( 0,  J_feet.rows()-1 , 0, 5 ) ;
//   S_c   = S_c_T.transposed() ;
//    
// //   -------------------------------------------------------------------------------------------------------------
// //   Defining derivative Terms
// //   Computing Derivative Terms
//   Q_aw_l_c1 = locoman::utils::Q_ci(J_aw_l_c1_spa_0, T_aw_l_c1_0, fc_l1_foot_filt ) ;
//   Q_aw_l_c2 = locoman::utils::Q_ci(J_aw_l_c2_spa_0, T_aw_l_c2_0, fc_l2_foot_filt ) ; // (size_q+ 6, size_q + 6) ;
//   Q_aw_l_c3 = locoman::utils::Q_ci(J_aw_l_c3_spa_0, T_aw_l_c3_0, fc_l3_foot_filt ) ; //(size_q+ 6, size_q + 6) ; 
//   Q_aw_l_c4 = locoman::utils::Q_ci(J_aw_l_c4_spa_0, T_aw_l_c4_0, fc_l4_foot_filt ) ; //(size_q+ 6, size_q + 6) ;
//   
//   Q_aw_r_c1 = locoman::utils::Q_ci(J_aw_r_c1_spa_0, T_aw_r_c1_0, fc_r1_foot_filt ) ; //(size_q+ 6, size_q + 6) ;
//   Q_aw_r_c2 = locoman::utils::Q_ci(J_aw_r_c2_spa_0, T_aw_r_c2_0, fc_r2_foot_filt ) ; //(size_q+ 6, size_q + 6) ;
//   Q_aw_r_c3 = locoman::utils::Q_ci(J_aw_r_c3_spa_0, T_aw_r_c3_0, fc_r3_foot_filt ) ; //(size_q+ 6, size_q + 6) ; 
//   Q_aw_r_c4 = locoman::utils::Q_ci(J_aw_r_c4_spa_0, T_aw_r_c4_0, fc_r4_foot_filt ) ; //(size_q+ 6, size_q + 6) ;
//   
//   Q_aw_l_tot = Q_aw_l_c1 + Q_aw_l_c2 + Q_aw_l_c3 + Q_aw_l_c4;
//   Q_aw_r_tot = Q_aw_r_c1 + Q_aw_r_c2 + Q_aw_r_c3 + Q_aw_r_c4;
//   Q_aw_c =  Q_aw_l_tot + Q_aw_r_tot ;  
//   
//   U_aw_s_cont = Q_aw_c.submatrix( 0 ,  5 , 0, 5) ;     
//   Q_aw_s_cont = Q_aw_c.submatrix( 0  , 5,  6,  (Q_aw_c.cols()-1)  ) ;
//  
//     FLMM  = locoman::utils::FLMM_redu(J_c_2, S_c_2, Q_aw_s_cont, U_aw_s_cont, Kc ) ;
//   cFLMM = locoman::utils::Pinv_trunc_SVD( FLMM.submatrix(0, FLMM.rows()-1 , 0, FLMM.rows()-1), 1E-10 ) * FLMM;
//   //cFLMM =    yarp::math::luinv(FLMM.submatrix(0, FLMM.rows()-1 , 0, FLMM.rows()-1)) * FLMM;
// 
//   Rf = cFLMM.submatrix(0, size_fc-1, cFLMM.cols()-size_q, cFLMM.cols()-1) ;  
//   Rf_filt = locoman::utils::filter_SVD( Rf ,  1E-10); 
//   
//     if(cout_print){
//     std::cout << "Rf_filt.rows() = " << Rf_filt.rows() << std::endl ;
//     std::cout << "Rf_filt.cols() = " << Rf_filt.cols() << std::endl ;
//     std::cout << "Rf_filt.toString()  OLD STYLE = " <<  std::endl << Rf_filt.toString() << std::endl ;    
//   }      
//     
  
//   ------------------------------------------------------------------------
    
    
    
    
    
    
    
//   //----------------------------------------------------------------------------------------
//   // Q_g  Computation with a -for- loop 
//   
//   
//   // Computing tau_com
//   
//   yarp::sig::Vector q_h = q_sensed ;
//   double h_incremental = std::pow(std::numeric_limits<double>::epsilon(), 1.0/2.0);
// 
//   
//   yarp::sig::Vector mg_vect(3,0.0) ;
//   mg_vect[2] = -mg ;
//   
//   model.iDyn3_model.getCOMJacobian(J_com_w) ;
//   J_com_w_redu = J_com_w.submatrix(0,2 , 0 , J_com_w.cols()-1 ) ;  
//   J_com_aw     = locoman::utils::getRot(T_aw_w_0) *J_com_w_redu; 
// 
//   yarp::sig::Vector tau_mg_0 = J_com_aw.transposed()*mg_vect ;
//   
//   yarp::sig::Vector tau_mg_q_i( tau_mg_0.length(), 0.0 ) ;// = tau_mg_0 ;
//   
//   yarp::sig::Vector tau_mg_differential_i( tau_mg_0.length(), 0.0 )  ;
//   
//   yarp::sig::Matrix Q_mg(tau_mg_0.length(),tau_mg_0.length()) ; 
//   yarp::sig::Matrix Rot_aw_w_0 = locoman::utils::getRot(T_aw_w_0)  ;
//   
//   for(int i=0; i<q_sensed.length() ; i++ ){
//    tau_mg_q_i.zero();
//    q_h = q_sensed ;
//    q_h[i] += h_incremental ;
//    robot.idynutils.updateiDyn3Model( q_h, true );  // model.iDyn3_model.idynutils.updateiDyn3Model ;
//    model.iDyn3_model.getCOMJacobian(J_com_w_q_i) ;
//    J_com_w_redu_q_i = J_com_w_q_i.submatrix(0,2 , 0 , J_com_w_q_i.cols()-1 ) ;  
//    J_com_aw_q_i = Rot_aw_w_0 *J_com_w_redu_q_i ; 
//    tau_mg_q_i = J_com_aw_q_i.transposed() * mg_vect ;
//    tau_mg_differential_i = (tau_mg_q_i - tau_mg_0)/h_incremental ;
//    Q_mg.setCol( i, tau_mg_differential_i ) ;
//    
//   } 
//   robot.idynutils.updateiDyn3Model( q_sensed, true ); 
// 
//     
//   yarp::sig::Matrix Q_mg_diff = Q_mg- Q_mg_2 ;
//   Q_mg_diff.resize(Q_mg_diff.cols()*Q_mg_diff.rows() , 1);
//   yarp::sig::Vector Q_mg_diff_vect = Q_mg_diff.getCol(0)  ;
//   double max_q_mg_diff = yarp::math::findMax(Q_mg_diff_vect) ;
//   double min_q_mg_diff = yarp::math::findMin(Q_mg_diff_vect) ;
//       std::cout << "max_q_mg_diff  = " << max_q_mg_diff << std::endl ;
//       std::cout << "min_q_mg_diff  = " << min_q_mg_diff << std::endl ;
//   
    
    //---------------------------------------------------------------------------------------------------

  //  std::cout << "robot name = "  <<  robot.idynutils.getRobotName() << std::endl ; 
    
//     FC_DES = fc_received.subVector(0,size_fc-1) ; //size_fc
//     FC_FILTERED= fc_received.subVector(size_fc,2*size_fc-1 ) ;
//     std::cout << "FC_DES = "  << FC_DES.toString() << std::endl ; 
//     std::cout << "FC_feet = "  << FC_FILTERED.toString() << std::endl ; 
//     //
//     yarp::sig::Vector fc_l_c_to_world = FC_FILTERED.subVector(0,size_fc/2-1)     ;
//     yarp::sig::Vector fc_r_c_to_world = FC_FILTERED.subVector(size_fc/2, size_fc-1) ;
//     //--------------------------------------------------------
//     //

    //
/*    fc_l_c1_filt = FC_FILTERED.subVector(0,2) ;// - fc_offset_left.subVector(0,2)  ;  // Applied from the robot to the world
    fc_l_c2_filt = FC_FILTERED.subVector(3,5) ;//- fc_offset_left.subVector(3,5)  ;
    fc_l_c3_filt = FC_FILTERED.subVector(6,8) ;//- fc_offset_left.subVector(6,8)  ;
    fc_l_c4_filt = FC_FILTERED.subVector(9,11);// - fc_offset_left.subVector(9,11) ;

    fc_r_c1_filt = FC_FILTERED.subVector(12,14) ;//- fc_offset_right.subVector(0,2) ; 
    fc_r_c2_filt = FC_FILTERED.subVector(15,17) ;//- fc_offset_right.subVector(3,5)  ; 
    fc_r_c3_filt = FC_FILTERED.subVector(18,20) ;//- fc_offset_right.subVector(6,8)  ; 
    fc_r_c4_filt = FC_FILTERED.subVector(21,23) ;  */
    
    //--------------------------------------------------------------------
    

  //   std::cout << "T_w_aw_0 = "  << T_w_aw_0.toString() << std::endl ; 
 
  /*
  //---------------------------------------
  T_aw_w_0 = locoman::utils::iHomogeneous(T_w_aw_0) ;    

  //-------------------------------------------------------------------------------------------------------------    
  // Defining Useful Transformations
  T_w_waist_0   = model.iDyn3_model.getPosition(waist_index) ;  
  T_w_l_ankle_0 = model.iDyn3_model.getPosition(l_ankle_index) ;

    
  T_w_r_ankle_0 = model.iDyn3_model.getPosition(r_ankle_index) ;
 T_r_ankle_w_0 = locoman::utils::iHomogeneous(T_w_r_ankle_0) ;
  T_r_c2_w_0    = locoman::utils::iHomogeneous(T_w_r_c2_0) ;  
  T_r_c3_w_0    = locoman::utils::iHomogeneous(T_w_r_c3_0) ;
  T_r_c4_w_0    = locoman::utils::iHomogeneous(T_w_r_c4_0) ;   
  

    

  
  // -----------------------------------------------------------------------
  T_waist_w_0   = locoman::utils::iHomogeneous(T_w_waist_0)  ;
  T_l_ankle_w_0 = locoman::utils::iHomogeneous(T_w_l_ankle_0) ;
  T_l_c2_w_0    = locoman::utils::iHomogeneous(T_w_l_c2_0) ;  
  T_l_c3_w_0    = locoman::utils::iHomogeneous(T_w_l_c3_0) ;
  T_l_c4_w_0    = locoman::utils::iHomogeneous(T_w_l_c4_0) ;    */
    
  
  /*T_w_l_hand_0  = model.iDyn3_model.getPosition( l_hand_index ) ;
  T_w_r_hand_0  = model.iDyn3_model.getPosition( r_hand_index ) ;*/   
 // T_w_l_wrist_0 = model.iDyn3_model.getPosition(l_wrist_index) ;
 // T_w_r_wrist_0 = model.iDyn3_model.getPosition(r_wrist_index) ;
 

//   T_l_wrist_w_0 = locoman::utils::iHomogeneous(T_w_l_wrist_0)  ;
//   T_l1_hand_w_0 = locoman::utils::iHomogeneous(T_w_l1_hand_0) ;    
//   T_l2_hand_w_0 = locoman::utils::iHomogeneous(T_w_l2_hand_0) ;  
//   T_l3_hand_w_0 = locoman::utils::iHomogeneous(T_w_l3_hand_0) ;
//   T_l4_hand_w_0 = locoman::utils::iHomogeneous(T_w_l4_hand_0) ;    
//     
//   T_r_wrist_w_0 = locoman::utils::iHomogeneous(T_w_r_wrist_0) ;
//   T_r1_hand_w_0 = locoman::utils::iHomogeneous(T_w_r1_hand_0) ;    
//   T_r2_hand_w_0 = locoman::utils::iHomogeneous(T_w_r2_hand_0) ;  
//   T_r3_hand_w_0 = locoman::utils::iHomogeneous(T_w_r3_hand_0) ;
//   T_r4_hand_w_0 = locoman::utils::iHomogeneous(T_w_r4_hand_0) ;    
// 
//   //---------------------------------------------------------------------
//   
//   T_l_hand_w_0 = locoman::utils::iHomogeneous(T_w_l_hand_0) ;
//   T_r_hand_w_0 = locoman::utils::iHomogeneous(T_w_r_hand_0) ;   
//   
//   T_aw_l_c1_0 = T_aw_w_0 * T_w_l_c1_0 ;  // {AW} is fixed in a loop
//   T_aw_l_c2_0 = T_aw_w_0 * T_w_l_c2_0 ;  // in every loop the floating base is re-initialized 
//   T_aw_l_c3_0 = T_aw_w_0 * T_w_l_c3_0 ;  // coincident with {AW}
//   T_aw_l_c4_0 = T_aw_w_0 * T_w_l_c4_0 ;
// 
//   T_aw_r_c1_0 = T_aw_w_0 * T_w_r_c1_0 ;
//   T_aw_r_c2_0 = T_aw_w_0 * T_w_r_c2_0 ;
//   T_aw_r_c3_0 = T_aw_w_0 * T_w_r_c3_0 ;
//   T_aw_r_c4_0 = T_aw_w_0 * T_w_r_c4_0 ; 

//   T_aw_l1_hand_0 = T_aw_w_0 * T_w_l1_hand_0 ;  // {AW} is fixed in a loop
//   T_aw_l2_hand_0 = T_aw_w_0 * T_w_l2_hand_0 ;  // in every loop the floating base is re-initialized 
//   T_aw_l3_hand_0 = T_aw_w_0 * T_w_l3_hand_0 ;  // coincident with {AW}
//   T_aw_l4_hand_0 = T_aw_w_0 * T_w_l4_hand_0 ;
// 
//   T_aw_r1_hand_0 = T_aw_w_0 * T_w_r1_hand_0 ;
//   T_aw_r2_hand_0 = T_aw_w_0 * T_w_r2_hand_0 ;
//   T_aw_r3_hand_0 = T_aw_w_0 * T_w_r3_hand_0 ;
//   T_aw_r4_hand_0 = T_aw_w_0 * T_w_r4_hand_0 ; 
//   

  //-----------------------------------------------------
//   model.iDyn3_model.getJacobian( l_c2_index, J_l_c2_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
//   model.iDyn3_model.getJacobian( l_c3_index, J_l_c3_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
//   model.iDyn3_model.getJacobian( l_c4_index, J_l_c4_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian

 
//   model.iDyn3_model.getJacobian( l_hand_c1_index, J_l1_hand_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
//   model.iDyn3_model.getJacobian( l_hand_c2_index, J_l2_hand_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
//   model.iDyn3_model.getJacobian( l_hand_c3_index, J_l3_hand_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
//   model.iDyn3_model.getJacobian( l_hand_c4_index, J_l4_hand_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
// 
//   model.iDyn3_model.getJacobian( r_hand_c1_index, J_r1_hand_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
//   model.iDyn3_model.getJacobian( r_hand_c2_index, J_r2_hand_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
//   model.iDyn3_model.getJacobian( r_hand_c3_index, J_r3_hand_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
//   model.iDyn3_model.getJacobian( r_hand_c4_index, J_r4_hand_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
//     
//   model.iDyn3_model.getJacobian( l_hand_index, J_l_hand_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
//   model.iDyn3_model.getJacobian( r_hand_index, J_r_hand_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
// 
//   J_l_c2_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_c2_w_0), zero_3 ))* J_l_c2_mix_0 ;
//   J_l_c3_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_c3_w_0), zero_3 ))* J_l_c3_mix_0 ;
//   J_l_c4_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_c4_w_0), zero_3 ))* J_l_c4_mix_0 ;
// 
//   J_r_c1_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_c1_w_0), zero_3 ))* J_r_c1_mix_0 ;
//   J_r_c2_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_c2_w_0), zero_3 ))* J_r_c2_mix_0 ;
//   J_r_c3_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_c3_w_0), zero_3 ))* J_r_c3_mix_0 ;
//   J_r_c4_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_c4_w_0), zero_3 ))* J_r_c4_mix_0 ;
// 
//   J_l_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_hand_w_0), zero_3 ))* J_l_hand_mix_0 ;
//   J_r_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_hand_w_0), zero_3 ))* J_r_hand_mix_0 ;
// 
//   J_l1_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l1_hand_w_0), zero_3 ))* J_l1_hand_mix_0 ;
//   J_l2_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l2_hand_w_0), zero_3 ))* J_l2_hand_mix_0 ;
//   J_l3_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l3_hand_w_0), zero_3 ))* J_l3_hand_mix_0 ;
//   J_l4_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l4_hand_w_0), zero_3 ))* J_l4_hand_mix_0 ;
// 
//   J_r1_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r1_hand_w_0), zero_3 ))* J_r1_hand_mix_0 ;
//   J_r2_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r2_hand_w_0), zero_3 ))* J_r2_hand_mix_0 ;
//   J_r3_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r3_hand_w_0), zero_3 ))* J_r3_hand_mix_0 ;
//   J_r4_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r4_hand_w_0), zero_3 ))* J_r4_hand_mix_0 ;
//   
//   //---------------------------------------------------------------------------------------------------------------------------------------------------------------
//   // Introducing Spatial Jacobian terms: Fixed base in {AW}
//   
//   J_aw_l_c2_spa_0 = locoman::utils::Adjoint(T_aw_l_c2_0)* J_l_c2_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c2_spa_0 ;
//   J_aw_l_c3_spa_0 = locoman::utils::Adjoint(T_aw_l_c3_0)* J_l_c3_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c3_spa_0 ;
//   J_aw_l_c4_spa_0 = locoman::utils::Adjoint(T_aw_l_c4_0)* J_l_c4_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c4_spa_0 ;
// 
//   J_aw_r_c1_spa_0 = locoman::utils::Adjoint(T_aw_r_c1_0)* J_r_c1_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c1_spa_0 ;
//   J_aw_r_c2_spa_0 = locoman::utils::Adjoint(T_aw_r_c2_0)* J_r_c2_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c2_spa_0 ;
//   J_aw_r_c3_spa_0 = locoman::utils::Adjoint(T_aw_r_c3_0)* J_r_c3_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c3_spa_0 ;
//   J_aw_r_c4_spa_0 = locoman::utils::Adjoint(T_aw_r_c4_0)* J_r_c4_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c4_spa_0 ;
// 
//   J_aw_l1_hand_spa_0 = locoman::utils::Adjoint(T_aw_l1_hand_0)* J_l1_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c1_spa_0 ;// locoman::utils::Adjoint(T_aw_l_c1_0)* J_l_c1_body_0
//   J_aw_l2_hand_spa_0 = locoman::utils::Adjoint(T_aw_l2_hand_0)* J_l2_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c2_spa_0 ;
//   J_aw_l3_hand_spa_0 = locoman::utils::Adjoint(T_aw_l3_hand_0)* J_l3_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c3_spa_0 ;
//   J_aw_l4_hand_spa_0 = locoman::utils::Adjoint(T_aw_l4_hand_0)* J_l4_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c4_spa_0 ;
// 
//   J_aw_r1_hand_spa_0 = locoman::utils::Adjoint(T_aw_r1_hand_0)* J_r1_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c1_spa_0 ;
//   J_aw_r2_hand_spa_0 = locoman::utils::Adjoint(T_aw_r2_hand_0)* J_r2_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c2_spa_0 ;
//   J_aw_r3_hand_spa_0 = locoman::utils::Adjoint(T_aw_r3_hand_0)* J_r3_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c3_spa_0 ;
//   J_aw_r4_hand_spa_0 = locoman::utils::Adjoint(T_aw_r4_hand_0)* J_r4_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c4_spa_0 ;
//   
// 
//   J_aw_l_c2_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
//   J_aw_l_c3_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
//   J_aw_l_c4_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
//  
//   J_aw_r_c1_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
//   J_aw_r_c2_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
//   J_aw_r_c3_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
//   J_aw_r_c4_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
//  
//   J_aw_l1_hand_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
//   J_aw_l2_hand_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
//   J_aw_l3_hand_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
//   
//   J_aw_l4_hand_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
//  
//   J_aw_r1_hand_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
//   J_aw_r2_hand_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
//   J_aw_r3_hand_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
//   J_aw_r4_hand_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
//   
//   // Recomputing body Jacobian
// 
//      
//   J_l_c2_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_l_c2_0) ) * J_aw_l_c2_spa_0 ;
//   J_l_c3_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_l_c3_0) ) * J_aw_l_c3_spa_0 ;
//   J_l_c4_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_l_c4_0) ) * J_aw_l_c4_spa_0 ;
// 
//   J_r_c1_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_r_c1_0) ) * J_aw_r_c1_spa_0 ;
//   J_r_c2_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_r_c2_0) ) * J_aw_r_c2_spa_0 ;
//   J_r_c3_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_r_c3_0) ) * J_aw_r_c3_spa_0 ;
//   J_r_c4_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_r_c4_0) ) * J_aw_r_c4_spa_0 ;
// 
//   J_l1_hand_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_l1_hand_0) ) * J_aw_l1_hand_spa_0 ;
//   J_l2_hand_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_l2_hand_0) ) * J_aw_l2_hand_spa_0 ;
//   J_l3_hand_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_l3_hand_0) ) * J_aw_l3_hand_spa_0 ;
//   J_l4_hand_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_l4_hand_0) ) * J_aw_l4_hand_spa_0 ;
// 
//   J_r1_hand_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_r1_hand_0) ) * J_aw_r1_hand_spa_0 ;
//   J_r2_hand_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_r2_hand_0) ) * J_aw_r2_hand_spa_0 ;
//   J_r3_hand_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_r3_hand_0) ) * J_aw_r3_hand_spa_0 ;
//   J_r4_hand_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_r4_hand_0) ) * J_aw_r4_hand_spa_0 ;
  //------------------------------------------------------------------------------------------------------------
  //  
  // Stance and Jacobian Matrices
    
//   yarp::sig::Matrix Complete_Jac( 8*B.cols() , size_q + 6) ;
//   Complete_Jac.setSubmatrix( B.transposed()*J_l_c1_body_0 , 0 ,0 )  ;
//   Complete_Jac.setSubmatrix( B.transposed()*J_l_c2_body_0 , B.cols() ,0 )  ;
//   Complete_Jac.setSubmatrix( B.transposed()*J_l_c3_body_0 , 2*B.cols() ,0 )  ;
//   Complete_Jac.setSubmatrix( B.transposed()*J_l_c4_body_0 , 3*B.cols() ,0 )  ;
// 
//   Complete_Jac.setSubmatrix( B.transposed()*J_r_c1_body_0 , 4*B.cols() ,0 )  ;
//   Complete_Jac.setSubmatrix( B.transposed()*J_r_c2_body_0 , 5*B.cols() ,0 )  ;
//   Complete_Jac.setSubmatrix( B.transposed()*J_r_c3_body_0 , 6*B.cols() ,0 )  ;
//   Complete_Jac.setSubmatrix( B.transposed()*J_r_c4_body_0 , 7*B.cols() ,0 )  ;
// 
//   yarp::sig::Matrix J_c   = Complete_Jac.submatrix( 0,  Complete_Jac.rows()-1 , 6, Complete_Jac.cols()-1 ) ;
//   yarp::sig::Matrix S_c_T = Complete_Jac.submatrix( 0,  Complete_Jac.rows()-1 , 0, 5 ) ;
//   yarp::sig::Matrix S_c   = S_c_T.transposed() ;
    
//   yarp::sig::Matrix Complete_Jac_f_rh( 12*B.cols() , size_q + 6) ;
//   Complete_Jac_f_rh.setSubmatrix( B.transposed()*J_l_c1_body_0 , 0 ,0 )  ;
//   Complete_Jac_f_rh.setSubmatrix( B.transposed()*J_l_c2_body_0 , B.cols() ,0 )  ;
//   Complete_Jac_f_rh.setSubmatrix( B.transposed()*J_l_c3_body_0 , 2*B.cols() ,0 )  ;
//   Complete_Jac_f_rh.setSubmatrix( B.transposed()*J_l_c4_body_0 , 3*B.cols() ,0 )  ;
// 
//   Complete_Jac_f_rh.setSubmatrix( B.transposed()*J_r_c1_body_0 , 4*B.cols() ,0 )  ;
//   Complete_Jac_f_rh.setSubmatrix( B.transposed()*J_r_c2_body_0 , 5*B.cols() ,0 )  ;
//   Complete_Jac_f_rh.setSubmatrix( B.transposed()*J_r_c3_body_0 , 6*B.cols() ,0 )  ;
//   Complete_Jac_f_rh.setSubmatrix( B.transposed()*J_r_c4_body_0 , 7*B.cols() ,0 )  ;
// 
//   Complete_Jac_f_rh.setSubmatrix( B.transposed()*J_r1_hand_body_0 , 8*B.cols() ,0 )  ;
//   Complete_Jac_f_rh.setSubmatrix( B.transposed()*J_r2_hand_body_0 , 9*B.cols() ,0 )  ;
//   Complete_Jac_f_rh.setSubmatrix( B.transposed()*J_r3_hand_body_0 , 10*B.cols() ,0 )  ;
//   Complete_Jac_f_rh.setSubmatrix( B.transposed()*J_r4_hand_body_0 , 11*B.cols() ,0 )  ; 
//   
//   yarp::sig::Matrix J_c_f_rh   = Complete_Jac_f_rh.submatrix( 0,  Complete_Jac_f_rh.rows()-1 , 6, Complete_Jac_f_rh.cols()-1 ) ;
//   yarp::sig::Matrix S_c_f_rh_T = Complete_Jac_f_rh.submatrix( 0,  Complete_Jac_f_rh.rows()-1 , 0, 5 ) ;
//   yarp::sig::Matrix S_c_f_rh   = S_c_f_rh_T.transposed() ;

  // -------------------------------------------------------------------------------------------------------------
  // Defining derivative Terms

  // Computing Derivative Terms
 /* Q_aw_l_c1 = locoman::utils::Q_ci(J_aw_l_c1_spa_0, T_aw_l_c1_0, fc_l1_foot_filt ) ;
  Q_aw_l_c2 = locoman::utils::Q_ci(J_aw_l_c2_spa_0, T_aw_l_c2_0, fc_l2_foot_filt ) ; // (size_q+ 6, size_q + 6) ;
  Q_aw_l_c3 = locoman::utils::Q_ci(J_aw_l_c3_spa_0, T_aw_l_c3_0, fc_l3_foot_filt ) ; //(size_q+ 6, size_q + 6) ; 
  Q_aw_l_c4 = locoman::utils::Q_ci(J_aw_l_c4_spa_0, T_aw_l_c4_0, fc_l4_foot_filt ) ; //(size_q+ 6, size_q + 6) ;
  //

  Q_aw_r_c1 = locoman::utils::Q_ci(J_aw_r_c1_spa_0, T_aw_r_c1_0, fc_r1_foot_filt ) ; //(size_q+ 6, size_q + 6) ;
  Q_aw_r_c2 = locoman::utils::Q_ci(J_aw_r_c2_spa_0, T_aw_r_c2_0, fc_r2_foot_filt ) ; //(size_q+ 6, size_q + 6) ;
  Q_aw_r_c3 = locoman::utils::Q_ci(J_aw_r_c3_spa_0, T_aw_r_c3_0, fc_r3_foot_filt ) ; //(size_q+ 6, size_q + 6) ; 
  Q_aw_r_c4 = locoman::utils::Q_ci(J_aw_r_c4_spa_0, T_aw_r_c4_0, fc_r4_foot_filt ) ;*/ //(size_q+ 6, size_q + 6) ;
  

//   Q_aw_l_tot = Q_aw_l_c1 + Q_aw_l_c2 + Q_aw_l_c3 + Q_aw_l_c4;
//   Q_aw_r_tot = Q_aw_r_c1 + Q_aw_r_c2 + Q_aw_r_c3 + Q_aw_r_c4;
//   // Q_aw_r_hand_tot = Q_aw_r1_hand + Q_aw_r2_hand + Q_aw_r3_hand + Q_aw_r4_hand;
// 
//   Q_aw_c =  Q_aw_l_tot + Q_aw_r_tot ;  
// 
//   U_aw_s_cont = Q_aw_c.submatrix( 0 ,  5 , 0, 5) ;     
//   Q_aw_s_cont = Q_aw_c.submatrix( 0  , 5,  6,  (Q_aw_c.cols()-1)  ) ;

//   
  //---------------------------------------------------------
  
  
    
  // Da qui ci metto i conti per il to_lf, giusto per fare una prova...  
  
  // mg = 1200;  

 
//   d_fc_des_to_world  = FC_DES - FC_FILTERED ; //  
//  
//    std::cout << " d_fc_des_to_world  =  "<< std::endl << d_fc_des_to_world.toString() << std::endl  ;  
// 
//   //-----------------------------------------------------------------------------------------------------
// //     std::cout << " J_c  =  "<< std::endl << J_c.toString() << std::endl  ;  
// //     std::cout << " S_c  =  "<< std::endl << S_c.toString() << std::endl  ;  
// //     std::cout << " Q_aw_s_cont  =  "<< std::endl << Q_aw_s_cont.toString() << std::endl  ;  
// //     std::cout << " U_aw_s_cont  =  "<< std::endl << U_aw_s_cont.toString() << std::endl  ;  
// //    
//    
//   yarp::sig::Matrix FLMM  = locoman::utils::FLMM_redu(J_c, S_c, Q_aw_s_cont, U_aw_s_cont, Kc ) ;
//   yarp::sig::Matrix cFLMM = locoman::utils::Pinv_trunc_SVD(FLMM.submatrix(0, FLMM.rows()-1 , 0, FLMM.rows()-1), 1E-10 ) * FLMM;
//    
//   yarp::sig::Matrix Rf_temp_2 = cFLMM.submatrix(0, size_fc-1, cFLMM.cols()-size_q, cFLMM.cols()-1) ;  
//   yarp::sig::Matrix Rf_temp_2_filt = locoman::utils::filter_SVD( Rf_temp_2,  1E-10); 
// 
//   
//   double err = norm( d_fc_des_to_world )  ;  // d_fc_des_to_world
// 
//   double regu_filter = 1E6 ; 
//   
//   
//   yarp::sig::Vector d_q_dsp_6 = -1.0* locoman::utils::Pinv_Regularized( Rf_temp_2_filt, regu_filter)* d_fc_des_to_world ;
// 
//   std::cout << " d_q_dsp_6  =  "<< std::endl << d_q_dsp_6.toString() << std::endl  ;  

    
    


    
    
    
    
    // HERE START the LAst Working Version before the Complete Refactor, with the V
//     //
//     std::cout << "q_sensed = "  << q_sensed.toString() << std::endl ; 
//     std::cout << "fc_sensed_2 = "  << fc_sensed_2.toString() << std::endl ; 
//   //  std::cout << "robot name = "  <<  robot.idynutils.getRobotName() << std::endl ; 
//     
//     FC_DES = fc_sensed_2.subVector(0,size_fc-1) ; //size_fc
//     FC_FILTERED= fc_sensed_2.subVector(size_fc,2*size_fc-1 ) ;
//     std::cout << "FC_DES = "  << FC_DES.toString() << std::endl ; 
//     std::cout << "FC_feet = "  << FC_FILTERED.toString() << std::endl ; 
//     //
//     yarp::sig::Vector fc_l_c_to_world = FC_FILTERED.subVector(0,size_fc/2-1)     ;
//     yarp::sig::Vector fc_r_c_to_world = FC_FILTERED.subVector(size_fc/2, size_fc-1) ;
//     //--------------------------------------------------------
//     //
//     robot.idynutils.updateiDyn3Model( q_sensed, true ); 
//     //
//     fc_l_c1_filt = FC_FILTERED.subVector(0,2) ;// - fc_offset_left.subVector(0,2)  ;  // Applied from the robot to the world
//     fc_l_c2_filt = FC_FILTERED.subVector(3,5) ;//- fc_offset_left.subVector(3,5)  ;
//     fc_l_c3_filt = FC_FILTERED.subVector(6,8) ;//- fc_offset_left.subVector(6,8)  ;
//     fc_l_c4_filt = FC_FILTERED.subVector(9,11);// - fc_offset_left.subVector(9,11) ;
// 
//     fc_r_c1_filt = FC_FILTERED.subVector(12,14) ;//- fc_offset_right.subVector(0,2) ; 
//     fc_r_c2_filt = FC_FILTERED.subVector(15,17) ;//- fc_offset_right.subVector(3,5)  ; 
//     fc_r_c3_filt = FC_FILTERED.subVector(18,20) ;//- fc_offset_right.subVector(6,8)  ; 
//     fc_r_c4_filt = FC_FILTERED.subVector(21,23) ;  
//     
//     //---------------------------------------------------------------------
//     
//      //----------------------------------------------------------------------------------------- 
//   // Defining the "Auxiliary World" Frame => {AW} // 
//   
//   T_w_aw_0 = locoman::utils::AW_world_posture(model, robot) ;
//   //   std::cout << "T_w_aw_0 = "  << T_w_aw_0.toString() << std::endl ; 
//  
//   
//   //---------------------------------------
//   T_aw_w_0 = locoman::utils::iHomogeneous(T_w_aw_0) ;    
// 
//   //-------------------------------------------------------------------------------------------------------------    
//   // Defining Useful Transformations
//   T_w_waist_0   = model.iDyn3_model.getPosition(waist_index) ;  
//   T_w_l_ankle_0 = model.iDyn3_model.getPosition(l_ankle_index) ;
//   T_w_l_c1_0    = model.iDyn3_model.getPosition(l_c1_index)    ;    
//   T_w_l_c2_0    = model.iDyn3_model.getPosition(l_c2_index)    ;  
//   T_w_l_c3_0    = model.iDyn3_model.getPosition(l_c3_index)    ;
//   T_w_l_c4_0    = model.iDyn3_model.getPosition(l_c4_index)    ;    
//     
//   T_w_r_ankle_0 = model.iDyn3_model.getPosition(r_ankle_index) ;
//   T_w_r_c1_0    = model.iDyn3_model.getPosition(r_c1_index)    ;    
//   T_w_r_c2_0    = model.iDyn3_model.getPosition(r_c2_index)    ;  
//   T_w_r_c3_0    = model.iDyn3_model.getPosition(r_c3_index)    ;
//   T_w_r_c4_0    = model.iDyn3_model.getPosition(r_c4_index)    ;   
//     
// /*  T_w_l_hand_0  = model.iDyn3_model.getPosition( l_hand_index ) ;
//   T_w_r_hand_0  = model.iDyn3_model.getPosition( r_hand_index ) ;   
// 
//   T_w_l_wrist_0 = model.iDyn3_model.getPosition(l_wrist_index) ;
//   T_w_l1_hand_0 = model.iDyn3_model.getPosition(l_hand_c1_index)    ;    
//   T_w_l2_hand_0 = model.iDyn3_model.getPosition(l_hand_c2_index)    ;  
//   T_w_l3_hand_0 = model.iDyn3_model.getPosition(l_hand_c3_index)    ;
//   T_w_l4_hand_0 = model.iDyn3_model.getPosition(l_hand_c4_index)    ;    
//     
//   T_w_r_wrist_0 = model.iDyn3_model.getPosition(r_wrist_index) ;
//   T_w_r1_hand_0 = model.iDyn3_model.getPosition(r_hand_c1_index)    ;    
//   T_w_r2_hand_0 = model.iDyn3_model.getPosition(r_hand_c2_index)    ;  
//   T_w_r3_hand_0 = model.iDyn3_model.getPosition(r_hand_c3_index)    ;
//   T_w_r4_hand_0 = model.iDyn3_model.getPosition(r_hand_c4_index)    ;   */  
//   
//   // -----------------------------------------------------------------------
//   T_waist_w_0   = locoman::utils::iHomogeneous(T_w_waist_0)  ;
//   T_l_ankle_w_0 = locoman::utils::iHomogeneous(T_w_l_ankle_0) ;
//   T_l_c1_w_0    = locoman::utils::iHomogeneous(T_w_l_c1_0) ;    
//   T_l_c2_w_0    = locoman::utils::iHomogeneous(T_w_l_c2_0) ;  
//   T_l_c3_w_0    = locoman::utils::iHomogeneous(T_w_l_c3_0) ;
//   T_l_c4_w_0    = locoman::utils::iHomogeneous(T_w_l_c4_0) ;   
//     
//   T_r_ankle_w_0 = locoman::utils::iHomogeneous(T_w_r_ankle_0) ;
//   T_r_c1_w_0    = locoman::utils::iHomogeneous(T_w_r_c1_0) ;    
//   T_r_c2_w_0    = locoman::utils::iHomogeneous(T_w_r_c2_0) ;  
//   T_r_c3_w_0    = locoman::utils::iHomogeneous(T_w_r_c3_0) ;
//   T_r_c4_w_0    = locoman::utils::iHomogeneous(T_w_r_c4_0) ;    
// 
// //   T_l_wrist_w_0 = locoman::utils::iHomogeneous(T_w_l_wrist_0)  ;
// //   T_l1_hand_w_0 = locoman::utils::iHomogeneous(T_w_l1_hand_0) ;    
// //   T_l2_hand_w_0 = locoman::utils::iHomogeneous(T_w_l2_hand_0) ;  
// //   T_l3_hand_w_0 = locoman::utils::iHomogeneous(T_w_l3_hand_0) ;
// //   T_l4_hand_w_0 = locoman::utils::iHomogeneous(T_w_l4_hand_0) ;    
// //     
// //   T_r_wrist_w_0 = locoman::utils::iHomogeneous(T_w_r_wrist_0) ;
// //   T_r1_hand_w_0 = locoman::utils::iHomogeneous(T_w_r1_hand_0) ;    
// //   T_r2_hand_w_0 = locoman::utils::iHomogeneous(T_w_r2_hand_0) ;  
// //   T_r3_hand_w_0 = locoman::utils::iHomogeneous(T_w_r3_hand_0) ;
// //   T_r4_hand_w_0 = locoman::utils::iHomogeneous(T_w_r4_hand_0) ;    
// // 
// //   //---------------------------------------------------------------------
// //   
// //   T_l_hand_w_0 = locoman::utils::iHomogeneous(T_w_l_hand_0) ;
// //   T_r_hand_w_0 = locoman::utils::iHomogeneous(T_w_r_hand_0) ;   
// //   
//   T_aw_l_c1_0 = T_aw_w_0 * T_w_l_c1_0 ;  // {AW} is fixed in a loop
//   T_aw_l_c2_0 = T_aw_w_0 * T_w_l_c2_0 ;  // in every loop the floating base is re-initialized 
//   T_aw_l_c3_0 = T_aw_w_0 * T_w_l_c3_0 ;  // coincident with {AW}
//   T_aw_l_c4_0 = T_aw_w_0 * T_w_l_c4_0 ;
// 
//   T_aw_r_c1_0 = T_aw_w_0 * T_w_r_c1_0 ;
//   T_aw_r_c2_0 = T_aw_w_0 * T_w_r_c2_0 ;
//   T_aw_r_c3_0 = T_aw_w_0 * T_w_r_c3_0 ;
//   T_aw_r_c4_0 = T_aw_w_0 * T_w_r_c4_0 ; 
// 
// //   T_aw_l1_hand_0 = T_aw_w_0 * T_w_l1_hand_0 ;  // {AW} is fixed in a loop
// //   T_aw_l2_hand_0 = T_aw_w_0 * T_w_l2_hand_0 ;  // in every loop the floating base is re-initialized 
// //   T_aw_l3_hand_0 = T_aw_w_0 * T_w_l3_hand_0 ;  // coincident with {AW}
// //   T_aw_l4_hand_0 = T_aw_w_0 * T_w_l4_hand_0 ;
// // 
// //   T_aw_r1_hand_0 = T_aw_w_0 * T_w_r1_hand_0 ;
// //   T_aw_r2_hand_0 = T_aw_w_0 * T_w_r2_hand_0 ;
// //   T_aw_r3_hand_0 = T_aw_w_0 * T_w_r3_hand_0 ;
// //   T_aw_r4_hand_0 = T_aw_w_0 * T_w_r4_hand_0 ; 
// //   
// 
//   //-----------------------------------------------------
//   model.iDyn3_model.getJacobian( l_c1_index, J_l_c1_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
//   model.iDyn3_model.getJacobian( l_c2_index, J_l_c2_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
//   model.iDyn3_model.getJacobian( l_c3_index, J_l_c3_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
//   model.iDyn3_model.getJacobian( l_c4_index, J_l_c4_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
// 
//   model.iDyn3_model.getJacobian( r_c1_index, J_r_c1_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
//   model.iDyn3_model.getJacobian( r_c2_index, J_r_c2_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
//   model.iDyn3_model.getJacobian( r_c3_index, J_r_c3_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
//   model.iDyn3_model.getJacobian( r_c4_index, J_r_c4_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
//  
//   model.iDyn3_model.getJacobian( l_hand_c1_index, J_l1_hand_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
//   model.iDyn3_model.getJacobian( l_hand_c2_index, J_l2_hand_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
//   model.iDyn3_model.getJacobian( l_hand_c3_index, J_l3_hand_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
//   model.iDyn3_model.getJacobian( l_hand_c4_index, J_l4_hand_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
// 
//   model.iDyn3_model.getJacobian( r_hand_c1_index, J_r1_hand_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
//   model.iDyn3_model.getJacobian( r_hand_c2_index, J_r2_hand_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
//   model.iDyn3_model.getJacobian( r_hand_c3_index, J_r3_hand_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
//   model.iDyn3_model.getJacobian( r_hand_c4_index, J_r4_hand_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
//     
//   model.iDyn3_model.getJacobian( l_hand_index, J_l_hand_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
//   model.iDyn3_model.getJacobian( r_hand_index, J_r_hand_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
// 
//   J_l_c1_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_c1_w_0), zero_3 ))* J_l_c1_mix_0 ;
//   J_l_c2_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_c2_w_0), zero_3 ))* J_l_c2_mix_0 ;
//   J_l_c3_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_c3_w_0), zero_3 ))* J_l_c3_mix_0 ;
//   J_l_c4_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_c4_w_0), zero_3 ))* J_l_c4_mix_0 ;
// 
//   J_r_c1_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_c1_w_0), zero_3 ))* J_r_c1_mix_0 ;
//   J_r_c2_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_c2_w_0), zero_3 ))* J_r_c2_mix_0 ;
//   J_r_c3_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_c3_w_0), zero_3 ))* J_r_c3_mix_0 ;
//   J_r_c4_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_c4_w_0), zero_3 ))* J_r_c4_mix_0 ;
// 
//   J_l_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_hand_w_0), zero_3 ))* J_l_hand_mix_0 ;
//   J_r_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_hand_w_0), zero_3 ))* J_r_hand_mix_0 ;
// 
//   J_l1_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l1_hand_w_0), zero_3 ))* J_l1_hand_mix_0 ;
//   J_l2_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l2_hand_w_0), zero_3 ))* J_l2_hand_mix_0 ;
//   J_l3_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l3_hand_w_0), zero_3 ))* J_l3_hand_mix_0 ;
//   J_l4_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l4_hand_w_0), zero_3 ))* J_l4_hand_mix_0 ;
// 
//   J_r1_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r1_hand_w_0), zero_3 ))* J_r1_hand_mix_0 ;
//   J_r2_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r2_hand_w_0), zero_3 ))* J_r2_hand_mix_0 ;
//   J_r3_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r3_hand_w_0), zero_3 ))* J_r3_hand_mix_0 ;
//   J_r4_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r4_hand_w_0), zero_3 ))* J_r4_hand_mix_0 ;
//   
//   //---------------------------------------------------------------------------------------------------------------------------------------------------------------
//   // Introducing Spatial Jacobian terms: Fixed base in {AW}
//   
//   J_aw_l_c1_spa_0 = locoman::utils::Adjoint(T_aw_l_c1_0)* J_l_c1_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c1_spa_0 ;// locoman::utils::Adjoint(T_aw_l_c1_0)* J_l_c1_body_0
//   J_aw_l_c2_spa_0 = locoman::utils::Adjoint(T_aw_l_c2_0)* J_l_c2_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c2_spa_0 ;
//   J_aw_l_c3_spa_0 = locoman::utils::Adjoint(T_aw_l_c3_0)* J_l_c3_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c3_spa_0 ;
//   J_aw_l_c4_spa_0 = locoman::utils::Adjoint(T_aw_l_c4_0)* J_l_c4_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c4_spa_0 ;
// 
//   J_aw_r_c1_spa_0 = locoman::utils::Adjoint(T_aw_r_c1_0)* J_r_c1_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c1_spa_0 ;
//   J_aw_r_c2_spa_0 = locoman::utils::Adjoint(T_aw_r_c2_0)* J_r_c2_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c2_spa_0 ;
//   J_aw_r_c3_spa_0 = locoman::utils::Adjoint(T_aw_r_c3_0)* J_r_c3_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c3_spa_0 ;
//   J_aw_r_c4_spa_0 = locoman::utils::Adjoint(T_aw_r_c4_0)* J_r_c4_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c4_spa_0 ;
// 
//   J_aw_l1_hand_spa_0 = locoman::utils::Adjoint(T_aw_l1_hand_0)* J_l1_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c1_spa_0 ;// locoman::utils::Adjoint(T_aw_l_c1_0)* J_l_c1_body_0
//   J_aw_l2_hand_spa_0 = locoman::utils::Adjoint(T_aw_l2_hand_0)* J_l2_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c2_spa_0 ;
//   J_aw_l3_hand_spa_0 = locoman::utils::Adjoint(T_aw_l3_hand_0)* J_l3_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c3_spa_0 ;
//   J_aw_l4_hand_spa_0 = locoman::utils::Adjoint(T_aw_l4_hand_0)* J_l4_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c4_spa_0 ;
// 
//   J_aw_r1_hand_spa_0 = locoman::utils::Adjoint(T_aw_r1_hand_0)* J_r1_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c1_spa_0 ;
//   J_aw_r2_hand_spa_0 = locoman::utils::Adjoint(T_aw_r2_hand_0)* J_r2_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c2_spa_0 ;
//   J_aw_r3_hand_spa_0 = locoman::utils::Adjoint(T_aw_r3_hand_0)* J_r3_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c3_spa_0 ;
//   J_aw_r4_hand_spa_0 = locoman::utils::Adjoint(T_aw_r4_hand_0)* J_r4_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c4_spa_0 ;
//   
//   J_aw_l_c1_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
//   J_aw_l_c2_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
//   J_aw_l_c3_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
//   J_aw_l_c4_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
//  
//   J_aw_r_c1_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
//   J_aw_r_c2_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
//   J_aw_r_c3_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
//   J_aw_r_c4_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
//  
//   J_aw_l1_hand_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
//   J_aw_l2_hand_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
//   J_aw_l3_hand_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
//   
//   J_aw_l4_hand_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
//  
//   J_aw_r1_hand_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
//   J_aw_r2_hand_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
//   J_aw_r3_hand_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
//   J_aw_r4_hand_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
//   
//   // Recomputing body Jacobian
// 
//   J_l_c1_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_l_c1_0) ) * J_aw_l_c1_spa_0 ;
//   J_l_c2_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_l_c2_0) ) * J_aw_l_c2_spa_0 ;
//   J_l_c3_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_l_c3_0) ) * J_aw_l_c3_spa_0 ;
//   J_l_c4_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_l_c4_0) ) * J_aw_l_c4_spa_0 ;
// 
//   J_r_c1_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_r_c1_0) ) * J_aw_r_c1_spa_0 ;
//   J_r_c2_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_r_c2_0) ) * J_aw_r_c2_spa_0 ;
//   J_r_c3_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_r_c3_0) ) * J_aw_r_c3_spa_0 ;
//   J_r_c4_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_r_c4_0) ) * J_aw_r_c4_spa_0 ;
// 
//   J_l1_hand_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_l1_hand_0) ) * J_aw_l1_hand_spa_0 ;
//   J_l2_hand_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_l2_hand_0) ) * J_aw_l2_hand_spa_0 ;
//   J_l3_hand_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_l3_hand_0) ) * J_aw_l3_hand_spa_0 ;
//   J_l4_hand_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_l4_hand_0) ) * J_aw_l4_hand_spa_0 ;
// 
//   J_r1_hand_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_r1_hand_0) ) * J_aw_r1_hand_spa_0 ;
//   J_r2_hand_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_r2_hand_0) ) * J_aw_r2_hand_spa_0 ;
//   J_r3_hand_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_r3_hand_0) ) * J_aw_r3_hand_spa_0 ;
//   J_r4_hand_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_r4_hand_0) ) * J_aw_r4_hand_spa_0 ;
//   //------------------------------------------------------------------------------------------------------------
//   //  
//   // Stance and Jacobian Matrices
//     
//   yarp::sig::Matrix Complete_Jac( 8*B.cols() , size_q + 6) ;
//   Complete_Jac.setSubmatrix( B.transposed()*J_l_c1_body_0 , 0 ,0 )  ;
//   Complete_Jac.setSubmatrix( B.transposed()*J_l_c2_body_0 , B.cols() ,0 )  ;
//   Complete_Jac.setSubmatrix( B.transposed()*J_l_c3_body_0 , 2*B.cols() ,0 )  ;
//   Complete_Jac.setSubmatrix( B.transposed()*J_l_c4_body_0 , 3*B.cols() ,0 )  ;
// 
//   Complete_Jac.setSubmatrix( B.transposed()*J_r_c1_body_0 , 4*B.cols() ,0 )  ;
//   Complete_Jac.setSubmatrix( B.transposed()*J_r_c2_body_0 , 5*B.cols() ,0 )  ;
//   Complete_Jac.setSubmatrix( B.transposed()*J_r_c3_body_0 , 6*B.cols() ,0 )  ;
//   Complete_Jac.setSubmatrix( B.transposed()*J_r_c4_body_0 , 7*B.cols() ,0 )  ;
// 
//   yarp::sig::Matrix J_c   = Complete_Jac.submatrix( 0,  Complete_Jac.rows()-1 , 6, Complete_Jac.cols()-1 ) ;
//   yarp::sig::Matrix S_c_T = Complete_Jac.submatrix( 0,  Complete_Jac.rows()-1 , 0, 5 ) ;
//   yarp::sig::Matrix S_c   = S_c_T.transposed() ;
//     
// //   yarp::sig::Matrix Complete_Jac_f_rh( 12*B.cols() , size_q + 6) ;
// //   Complete_Jac_f_rh.setSubmatrix( B.transposed()*J_l_c1_body_0 , 0 ,0 )  ;
// //   Complete_Jac_f_rh.setSubmatrix( B.transposed()*J_l_c2_body_0 , B.cols() ,0 )  ;
// //   Complete_Jac_f_rh.setSubmatrix( B.transposed()*J_l_c3_body_0 , 2*B.cols() ,0 )  ;
// //   Complete_Jac_f_rh.setSubmatrix( B.transposed()*J_l_c4_body_0 , 3*B.cols() ,0 )  ;
// // 
// //   Complete_Jac_f_rh.setSubmatrix( B.transposed()*J_r_c1_body_0 , 4*B.cols() ,0 )  ;
// //   Complete_Jac_f_rh.setSubmatrix( B.transposed()*J_r_c2_body_0 , 5*B.cols() ,0 )  ;
// //   Complete_Jac_f_rh.setSubmatrix( B.transposed()*J_r_c3_body_0 , 6*B.cols() ,0 )  ;
// //   Complete_Jac_f_rh.setSubmatrix( B.transposed()*J_r_c4_body_0 , 7*B.cols() ,0 )  ;
// // 
// //   Complete_Jac_f_rh.setSubmatrix( B.transposed()*J_r1_hand_body_0 , 8*B.cols() ,0 )  ;
// //   Complete_Jac_f_rh.setSubmatrix( B.transposed()*J_r2_hand_body_0 , 9*B.cols() ,0 )  ;
// //   Complete_Jac_f_rh.setSubmatrix( B.transposed()*J_r3_hand_body_0 , 10*B.cols() ,0 )  ;
// //   Complete_Jac_f_rh.setSubmatrix( B.transposed()*J_r4_hand_body_0 , 11*B.cols() ,0 )  ; 
// //   
// //   yarp::sig::Matrix J_c_f_rh   = Complete_Jac_f_rh.submatrix( 0,  Complete_Jac_f_rh.rows()-1 , 6, Complete_Jac_f_rh.cols()-1 ) ;
// //   yarp::sig::Matrix S_c_f_rh_T = Complete_Jac_f_rh.submatrix( 0,  Complete_Jac_f_rh.rows()-1 , 0, 5 ) ;
// //   yarp::sig::Matrix S_c_f_rh   = S_c_f_rh_T.transposed() ;
// 
//   // -------------------------------------------------------------------------------------------------------------
//   // Defining derivative Terms
// 
//   // Computing Derivative Terms
//   Q_aw_l_c1 = locoman::utils::Q_ci(J_aw_l_c1_spa_0, T_aw_l_c1_0, fc_l_c1_filt ) ;
//   Q_aw_l_c2 = locoman::utils::Q_ci(J_aw_l_c2_spa_0, T_aw_l_c2_0, fc_l_c2_filt ) ; // (size_q+ 6, size_q + 6) ;
//   Q_aw_l_c3 = locoman::utils::Q_ci(J_aw_l_c3_spa_0, T_aw_l_c3_0, fc_l_c3_filt ) ; //(size_q+ 6, size_q + 6) ; 
//   Q_aw_l_c4 = locoman::utils::Q_ci(J_aw_l_c4_spa_0, T_aw_l_c4_0, fc_l_c4_filt ) ; //(size_q+ 6, size_q + 6) ;
//   //
// 
//   Q_aw_r_c1 = locoman::utils::Q_ci(J_aw_r_c1_spa_0, T_aw_r_c1_0, fc_r_c1_filt ) ; //(size_q+ 6, size_q + 6) ;
//   Q_aw_r_c2 = locoman::utils::Q_ci(J_aw_r_c2_spa_0, T_aw_r_c2_0, fc_r_c2_filt ) ; //(size_q+ 6, size_q + 6) ;
//   Q_aw_r_c3 = locoman::utils::Q_ci(J_aw_r_c3_spa_0, T_aw_r_c3_0, fc_r_c3_filt ) ; //(size_q+ 6, size_q + 6) ; 
//   Q_aw_r_c4 = locoman::utils::Q_ci(J_aw_r_c4_spa_0, T_aw_r_c4_0, fc_r_c4_filt ) ; //(size_q+ 6, size_q + 6) ;
//   
// 
//   Q_aw_l_tot = Q_aw_l_c1 + Q_aw_l_c2 + Q_aw_l_c3 + Q_aw_l_c4;
//   Q_aw_r_tot = Q_aw_r_c1 + Q_aw_r_c2 + Q_aw_r_c3 + Q_aw_r_c4;
//   // Q_aw_r_hand_tot = Q_aw_r1_hand + Q_aw_r2_hand + Q_aw_r3_hand + Q_aw_r4_hand;
// 
//   Q_aw_c =  Q_aw_l_tot + Q_aw_r_tot ;  
// 
//   U_aw_s_cont = Q_aw_c.submatrix( 0 ,  5 , 0, 5) ;     
//   Q_aw_s_cont = Q_aw_c.submatrix( 0  , 5,  6,  (Q_aw_c.cols()-1)  ) ;
// 
// //   
//   //---------------------------------------------------------
//   
//   
//     
//   // Da qui ci metto i conti per il to_lf, giusto per fare una prova...  
//   
//   // mg = 1200;  
// 
//  
//   d_fc_des_to_world  = FC_DES - FC_FILTERED ; //  
//  
//    std::cout << " d_fc_des_to_world  =  "<< std::endl << d_fc_des_to_world.toString() << std::endl  ;  
// 
//   //-----------------------------------------------------------------------------------------------------
// //     std::cout << " J_c  =  "<< std::endl << J_c.toString() << std::endl  ;  
// //     std::cout << " S_c  =  "<< std::endl << S_c.toString() << std::endl  ;  
// //     std::cout << " Q_aw_s_cont  =  "<< std::endl << Q_aw_s_cont.toString() << std::endl  ;  
// //     std::cout << " U_aw_s_cont  =  "<< std::endl << U_aw_s_cont.toString() << std::endl  ;  
// //    
//    
//   yarp::sig::Matrix FLMM  = locoman::utils::FLMM_redu(J_c, S_c, Q_aw_s_cont, U_aw_s_cont, Kc ) ;
//   yarp::sig::Matrix cFLMM = locoman::utils::Pinv_trunc_SVD(FLMM.submatrix(0, FLMM.rows()-1 , 0, FLMM.rows()-1), 1E-10 ) * FLMM;
//    
//   yarp::sig::Matrix Rf_temp_2 = cFLMM.submatrix(0, size_fc-1, cFLMM.cols()-size_q, cFLMM.cols()-1) ;  
//   yarp::sig::Matrix Rf_temp_2_filt = locoman::utils::filter_SVD( Rf_temp_2,  1E-10); 
// 
//   
//   double err = norm( d_fc_des_to_world )  ;  // d_fc_des_to_world
// 
//   double regu_filter = 1E6 ; 
//   
//   
//   yarp::sig::Vector d_q_dsp_6 = -1.0* locoman::utils::Pinv_Regularized( Rf_temp_2_filt, regu_filter)* d_fc_des_to_world ;
// 
//   std::cout << " d_q_dsp_6  =  "<< std::endl << d_q_dsp_6.toString() << std::endl  ;  
// 
//     
//     
//     //------------------------------------------------------------------------
//     // ... sending back 
//     yarp::sig::Vector q_sensed_2 = d_q_dsp_6 ;
//     
//    // std::cout << "q_sensed_2 = "  << q_sensed_2.toString() << std::endl ; 
//     
//     yarp::sig::Vector &data = to_locoman_thread.prepare();
//     data.resize(q_sensed_2.size());
//     data = q_sensed_2;
//     to_locoman_thread.write();
//     // robot.idynutils.updateiDyn3Model( ) ;
    
}    
