#ifndef locoman_service_2_THREAD_H_
#define locoman_service_2_THREAD_H_

#include <GYM/control_thread.hpp>

#include <yarp/os/BufferedPort.h>

/**
 * @brief locoman_service_2 control thread
 * 
 **/
class locoman_service_2_thread : public control_thread
{
private:  
    // --- YARP PORT SECTION ---
   // yarp::os::BufferedPort<yarp::sig::Vector> from_locoman_thread;
   // yarp::sig::Vector* v_from_locoman_thread;
  
    yarp::os::BufferedPort<yarp::sig::Vector> receiving_q;
    yarp::sig::Vector* receiving_q_vect ;
    
    yarp::os::BufferedPort<yarp::sig::Vector> receiving_fc ;
    yarp::sig::Vector* receiving_fc_vect ;
    
    // Prorotype
    //yarp::os::BufferedPort<yarp::sig::Vector> to_locoman_thread;
    //yarp::os::BufferedPort<yarp::sig::Matrix> to_locoman_thread;

    yarp::os::BufferedPort<yarp::sig::Matrix> to_locoman_Matrix;
    yarp::os::BufferedPort<yarp::sig::Matrix> to_locoman_Big_J;
    yarp::os::BufferedPort<yarp::sig::Matrix> to_locoman_Big_Rf;
    
public:
    //iDynUtils new_Robot ; //   iDynUtils new_Robot("new_Robot",get_urdf_path(),get_srdf_path()) ;

    int mg =  1200 ;

    unsigned int size_q ;
    unsigned int waist_index ;
    unsigned int l_ankle_index ;
    unsigned int l_c1_index ;
    unsigned int l_c2_index ;
    unsigned int l_c3_index ;
    unsigned int l_c4_index ;
    unsigned int r_ankle_index ;
    unsigned int r_c1_index ;
    unsigned int r_c2_index ;
    unsigned int r_c3_index ;
    unsigned int r_c4_index ;
    unsigned int l_hand_index ;
    unsigned int r_hand_index ;
    unsigned int l_wrist_index ;
    unsigned int l_hand_c1_index ;
    unsigned int l_hand_c2_index ;
    unsigned int l_hand_c3_index ;
    unsigned int l_hand_c4_index ;
    unsigned int r_wrist_index ;
    unsigned int r_hand_c1_index ;
    unsigned int r_hand_c2_index ;
    unsigned int r_hand_c3_index ;
    unsigned int r_hand_c4_index ;
    
    bool cout_print = 1 ;
    
    double tic = 0.0 ;
    double toc = 0.0 ;
      //--------------
    yarp::sig::Vector CoM_waist_cmd  ;  
    yarp::sig::Matrix T_waist_l1_foot_cmd ; 
    yarp::sig::Matrix T_waist_r1_foot_cmd ; 
    yarp::sig::Matrix T_waist_l_hand_cmd ; 
    yarp::sig::Matrix T_waist_r_hand_cmd ; 
    yarp::sig::Matrix R_waist_aw_cmd ;     
    
    yarp::sig::Vector mg_vect ;
    
    yarp::sig::Vector q_sensed ;
    
    yarp::sig::Vector fc_received ;
    //----------------
    
    yarp::sig::Vector CoM_w_cmd ;  // variables registered at command time
    yarp::sig::Vector CoM_w_up ;
    yarp::sig::Vector CoM_w_dw ;
    
    yarp::sig::Matrix T_w_l1_cmd ; 
    yarp::sig::Matrix T_w_r1_cmd ; 
    
    yarp::sig::Matrix T_l1_r1_up ;
    yarp::sig::Matrix T_l1_r1_fw ;
    yarp::sig::Matrix T_l1_r1_dw ;
    
    yarp::sig::Matrix T_r1_l1_up ;
    yarp::sig::Matrix T_r1_l1_fw ;
    yarp::sig::Matrix T_r1_l1_dw ;
    
    int FC_size ;  
    int FC_HANDS_size ;
    int WINDOW_size;

    
    yarp::sig::Vector FC_DES ;  //     yarp::sig::Vector FC_DES( FC_size   ) ;
    yarp::sig::Vector FC_DES_LEFT_sensor ;
    yarp::sig::Vector FC_DES_RIGHT_sensor ;
    yarp::sig::Vector FC_SUM ;
    yarp::sig::Vector FC_FILTERED ;
    yarp::sig::Matrix FC_WINDOW ;  //    yarp::sig::Matrix FC_WINDOW(FC_size, WINDOW_filter ) ;
    
    yarp::sig::Vector FC_HANDS_DES ;  //     yarp::sig::Vector FC_DES( FC_size   ) ;
   // yarp::sig::Vector FC_DES_LEFT_HAND_sensor ;
    //yarp::sig::Vector FC_DES_RIGHT_HAND_sensor ;
  //  yarp::sig::Vector FC_HANDS_SUM ;
  //  yarp::sig::Vector FC_HANDS_FILTERED ;
  //  yarp::sig::Matrix FC_HANDS_WINDOW ;  //    yarp::sig::Matrix FC_WINDOW(FC_size, WINDOW_filter ) ;
    
    yarp::sig::Vector zero_3 ;
    yarp::sig::Matrix Zeros_6_6 ;
    yarp::sig::Matrix Eye_6 ;
    yarp::sig::Matrix Eye_3 ; 
    yarp::sig::Matrix Eye_4 ;
    yarp::sig::Matrix B ;
    
    unsigned int size_u = 6 ;
    unsigned int size_fc = 24;
    double kc ;
    yarp::sig::Matrix Kq ;
    yarp::sig::Matrix Kc ;
    yarp::sig::Matrix Kc_f_h ;
    yarp::sig::Matrix Kc_f_rh ;
    yarp::sig::Vector ft_l_ankle ;
    yarp::sig::Vector ft_r_ankle ;
    yarp::sig::Vector ft_l_wrist ;
    yarp::sig::Vector ft_r_wrist ;
    
    yarp::sig::Matrix map_l_fcToSens ;
    yarp::sig::Matrix map_r_fcToSens ;  
    yarp::sig::Matrix map_l_hand_fcToSens ;    
    yarp::sig::Matrix map_r_hand_fcToSens ;
    
    yarp::sig::Matrix map_l_fcToSens_PINV ;
    yarp::sig::Matrix map_r_fcToSens_PINV ;
    yarp::sig::Matrix map_l_hand_fcToSens_PINV ;
    yarp::sig::Matrix map_r_hand_fcToSens_PINV ;
    
    yarp::sig::Vector fc_offset_left ;
    yarp::sig::Vector fc_offset_right ;
    
 //   yarp::sig::Vector fc_offset_left_hand ;
 //   yarp::sig::Vector fc_offset_right_hand ;   
    
  yarp::sig::Vector fc_l_foot ; //= map_l_fcToSens_PINV*fc_received.subVector(  0,5  ) ;
  yarp::sig::Vector fc_r_foot ; //= map_r_fcToSens_PINV*fc_received.subVector(  6,11  ) ; 
  yarp::sig::Vector fc_feet ;
  yarp::sig::Vector fc_hands;
  
  yarp::sig::Vector fc_l_hand ; //= map_l_hand_fcToSens*fc_received.subVector( 12,17  ) ;
  yarp::sig::Vector fc_r_hand ; //= map_r_hand_fcToSens*fc_received.subVector( 18,23  ) ;  
    
  yarp::sig::Vector fc_l1_foot_filt ; //= FC_FILTERED.subVector(0,2)  ;  // Applied from the robot to the world
  yarp::sig::Vector fc_l2_foot_filt ; //= FC_FILTERED.subVector(3,5)  ;
  yarp::sig::Vector fc_l3_foot_filt ; //= FC_FILTERED.subVector(6,8)  ;
  yarp::sig::Vector fc_l4_foot_filt ; //= FC_FILTERED.subVector(9,11)  ;

  yarp::sig::Vector fc_r1_foot_filt ; //= FC_FILTERED.subVector(12,14)  ; 
  yarp::sig::Vector fc_r2_foot_filt ; //= FC_FILTERED.subVector(15,17)  ; 
  yarp::sig::Vector fc_r3_foot_filt ; //= FC_FILTERED.subVector(18,20)  ; 
  yarp::sig::Vector fc_r4_foot_filt ; //= FC_FILTERED.subVector(21,23)  ; 
  
  yarp::sig::Vector fc_l1_hand_filt ; //= FC_HANDS_FILTERED.subVector(0,2)  ;  // Applied from the robot to the world
  yarp::sig::Vector fc_l2_hand_filt ; // = FC_HANDS_FILTERED.subVector(3,5)  ;
  yarp::sig::Vector fc_l3_hand_filt ; //= FC_HANDS_FILTERED.subVector(6,8)  ;
  yarp::sig::Vector fc_l4_hand_filt ; //= FC_HANDS_FILTERED.subVector(9,11)  ;

  yarp::sig::Vector fc_r1_hand_filt ; //= FC_HANDS_FILTERED.subVector(12,14)  ; 
  yarp::sig::Vector fc_r2_hand_filt ; //= FC_HANDS_FILTERED.subVector(15,17)  ; 
  yarp::sig::Vector fc_r3_hand_filt ; //= FC_HANDS_FILTERED.subVector(18,20)  ; 
  yarp::sig::Vector fc_r4_hand_filt ; //= FC_HANDS_FILTERED.subVector(21,23)  ;   

  yarp::sig::Matrix T_w_aw_0 ; //= locoman::utils::AW_world_posture(model, robot) ;
  yarp::sig::Matrix T_aw_w_0 ; //= locoman::utils::iHomogeneous(T_w_aw_0) ;    

  //-------------------------------------------------------------------------------------------------------------    
  // Defining Useful Transformations
  yarp::sig::Matrix T_w_waist_0  ; // = model.iDyn3_model.getPosition(waist_index) ;  
  yarp::sig::Matrix T_w_l_ankle_0; // = model.iDyn3_model.getPosition(l_ankle_index) ;
  yarp::sig::Matrix T_w_l_c1_0    ; //= model.iDyn3_model.getPosition(l_c1_index)    ;    
  yarp::sig::Matrix T_w_l_c2_0   ; // = model.iDyn3_model.getPosition(l_c2_index)    ;  
  yarp::sig::Matrix T_w_l_c3_0   ; // = model.iDyn3_model.getPosition(l_c3_index)    ;
  yarp::sig::Matrix T_w_l_c4_0   ; // = model.iDyn3_model.getPosition(l_c4_index)    ;    
    
  yarp::sig::Matrix T_w_r_ankle_0 ; //= model.iDyn3_model.getPosition(r_ankle_index) ;
  yarp::sig::Matrix T_w_r_c1_0   ; // = model.iDyn3_model.getPosition(r_c1_index)    ;    
  yarp::sig::Matrix T_w_r_c2_0   ; // = model.iDyn3_model.getPosition(r_c2_index)    ;  
  yarp::sig::Matrix T_w_r_c3_0   ; // = model.iDyn3_model.getPosition(r_c3_index)    ;
  yarp::sig::Matrix T_w_r_c4_0   ; // = model.iDyn3_model.getPosition(r_c4_index)    ;   
    
  yarp::sig::Matrix T_w_l_hand_0  ; //= model.iDyn3_model.getPosition( l_hand_index ) ;
  yarp::sig::Matrix T_w_r_hand_0  ; //= model.iDyn3_model.getPosition( r_hand_index ) ;   

  yarp::sig::Matrix T_w_l_wrist_0 ; //= model.iDyn3_model.getPosition(l_wrist_index) ;
  yarp::sig::Matrix T_w_l1_hand_0 ; //= model.iDyn3_model.getPosition(l_hand_c1_index)    ;    
  yarp::sig::Matrix T_w_l2_hand_0 ; //= model.iDyn3_model.getPosition(l_hand_c2_index)    ;  
  yarp::sig::Matrix T_w_l3_hand_0 ; //= model.iDyn3_model.getPosition(l_hand_c3_index)    ;
  yarp::sig::Matrix T_w_l4_hand_0 ; //= model.iDyn3_model.getPosition(l_hand_c4_index)    ;    
    
  yarp::sig::Matrix T_w_r_wrist_0 ; //= model.iDyn3_model.getPosition(r_wrist_index) ;
  yarp::sig::Matrix T_w_r1_hand_0 ; //= model.iDyn3_model.getPosition(r_hand_c1_index)    ;    
  yarp::sig::Matrix T_w_r2_hand_0 ; //= model.iDyn3_model.getPosition(r_hand_c2_index)    ;  
  yarp::sig::Matrix T_w_r3_hand_0 ;  // = model.iDyn3_model.getPosition(r_hand_c3_index)    ;
  yarp::sig::Matrix T_w_r4_hand_0 ; //= model.iDyn3_model.getPosition(r_hand_c4_index)    ;     
//   
  // -----------------------------------------------------------------------
  yarp::sig::Matrix T_waist_w_0   ; //= locoman::utils::iHomogeneous(T_w_waist_0)  ;
  yarp::sig::Matrix T_l_ankle_w_0 ; //= locoman::utils::iHomogeneous(T_w_l_ankle_0) ;
  yarp::sig::Matrix T_l_c1_w_0    ; //= locoman::utils::iHomogeneous(T_w_l_c1_0) ;    
  yarp::sig::Matrix T_l_c2_w_0    ; //= locoman::utils::iHomogeneous(T_w_l_c2_0) ;  
  yarp::sig::Matrix T_l_c3_w_0    ; //= locoman::utils::iHomogeneous(T_w_l_c3_0) ;
  yarp::sig::Matrix T_l_c4_w_0    ; //= locoman::utils::iHomogeneous(T_w_l_c4_0) ;    
    
  yarp::sig::Matrix T_r_ankle_w_0 ; //= locoman::utils::iHomogeneous(T_w_r_ankle_0) ;
  yarp::sig::Matrix T_r_c1_w_0    ; //= locoman::utils::iHomogeneous(T_w_r_c1_0) ;    
  yarp::sig::Matrix T_r_c2_w_0    ; //= locoman::utils::iHomogeneous(T_w_r_c2_0) ;  
  yarp::sig::Matrix T_r_c3_w_0    ; //= locoman::utils::iHomogeneous(T_w_r_c3_0) ;
  yarp::sig::Matrix T_r_c4_w_0    ; //= locoman::utils::iHomogeneous(T_w_r_c4_0) ;    

  yarp::sig::Matrix T_l_wrist_w_0 ; //= locoman::utils::iHomogeneous(T_w_l_wrist_0)  ;
  yarp::sig::Matrix T_l1_hand_w_0 ; //= locoman::utils::iHomogeneous(T_w_l1_hand_0) ;    
  yarp::sig::Matrix T_l2_hand_w_0 ; //= locoman::utils::iHomogeneous(T_w_l2_hand_0) ;  
  yarp::sig::Matrix T_l3_hand_w_0 ; //= locoman::utils::iHomogeneous(T_w_l3_hand_0) ;
  yarp::sig::Matrix T_l4_hand_w_0 ; //= locoman::utils::iHomogeneous(T_w_l4_hand_0) ;    
    
  yarp::sig::Matrix T_r_wrist_w_0 ; //= locoman::utils::iHomogeneous(T_w_r_wrist_0) ;
  yarp::sig::Matrix T_r1_hand_w_0 ; //= locoman::utils::iHomogeneous(T_w_r1_hand_0) ;    
  yarp::sig::Matrix T_r2_hand_w_0 ; //= locoman::utils::iHomogeneous(T_w_r2_hand_0) ;  
  yarp::sig::Matrix T_r3_hand_w_0 ; //= locoman::utils::iHomogeneous(T_w_r3_hand_0) ;
  yarp::sig::Matrix T_r4_hand_w_0 ; //= locoman::utils::iHomogeneous(T_w_r4_hand_0) ;    

  //---------------------------------------------------------------------
  
  yarp::sig::Matrix T_l_hand_w_0 ; //= locoman::utils::iHomogeneous(T_w_l_hand_0) ;
  yarp::sig::Matrix T_r_hand_w_0 ; //= locoman::utils::iHomogeneous(T_w_r_hand_0) ;   
  
  yarp::sig::Matrix T_aw_l_c1_0 ; //= T_aw_w_0 * T_w_l_c1_0 ;  // {AW} is fixed in a loop
  yarp::sig::Matrix T_aw_l_c2_0 ; //= T_aw_w_0 * T_w_l_c2_0 ;  // in every loop the floating base is re-initialized 
  yarp::sig::Matrix T_aw_l_c3_0 ; //= T_aw_w_0 * T_w_l_c3_0 ;  // coincident with {AW}
  yarp::sig::Matrix T_aw_l_c4_0 ; //= T_aw_w_0 * T_w_l_c4_0 ;

  yarp::sig::Matrix T_aw_r_c1_0 ; //= T_aw_w_0 * T_w_r_c1_0 ;
  yarp::sig::Matrix T_aw_r_c2_0 ; //= T_aw_w_0 * T_w_r_c2_0 ;
  yarp::sig::Matrix T_aw_r_c3_0 ; //= T_aw_w_0 * T_w_r_c3_0 ;
  yarp::sig::Matrix T_aw_r_c4_0 ; //= T_aw_w_0 * T_w_r_c4_0 ; 

  yarp::sig::Matrix T_aw_l1_hand_0 ; //= T_aw_w_0 * T_w_l1_hand_0 ;  // {AW} is fixed in a loop
  yarp::sig::Matrix T_aw_l2_hand_0 ; //= T_aw_w_0 * T_w_l2_hand_0 ;  // in every loop the floating base is re-initialized 
  yarp::sig::Matrix T_aw_l3_hand_0 ; //= T_aw_w_0 * T_w_l3_hand_0 ;  // coincident with {AW}
  yarp::sig::Matrix T_aw_l4_hand_0 ; //= T_aw_w_0 * T_w_l4_hand_0 ;

  yarp::sig::Matrix T_aw_r1_hand_0 ; //= T_aw_w_0 * T_w_r1_hand_0 ;
  yarp::sig::Matrix T_aw_r2_hand_0 ; //= T_aw_w_0 * T_w_r2_hand_0 ;
  yarp::sig::Matrix T_aw_r3_hand_0 ; //= T_aw_w_0 * T_w_r3_hand_0 ;
  yarp::sig::Matrix T_aw_r4_hand_0 ; //= T_aw_w_0 * T_w_r4_hand_0 ;   
  
  //--------------------------------------------------
  
   // Jacobian Matrices 
  
  yarp::sig::Matrix Complete_Jac_feet ; //( 8*B.cols() , size_q + 6) ;    //  ( 24 , size_q + 6)  ;
  
  yarp::sig::Matrix Complete_Jac_f_h ; // ( 16*B.cols() , size_q + 6) ;
  
 // yarp::sig::Matrix J_feet ;  
  yarp::sig::Matrix J_c_feet ;      //  ( 24 , size_q ) ;
  yarp::sig::Matrix S_c_feet_T ;    //  ( 24 , 6 ) ; 
  yarp::sig::Matrix S_c_feet ;      //  ( 6 , 24 )   ; // = S_c_T.transposed() ;  

  yarp::sig::Matrix J_c_f_h   ; //  = Complete_Jac_f_h.submatrix( 0,  Complete_Jac_f_h.rows()-1 , 6, Complete_Jac_f_h.cols()-1 ) ;
  yarp::sig::Matrix S_c_f_h_T ; // = Complete_Jac_f_h.submatrix( 0,  Complete_Jac_f_h.rows()-1 , 0, 5 ) ;
  yarp::sig::Matrix S_c_f_h   ; // 
  
  yarp::sig::Matrix J_l_c1_mix_0 ; //( 6, ( size_q + 6 ) ) ; //
  yarp::sig::Matrix J_l_c2_mix_0 ; //( 6, ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  yarp::sig::Matrix J_l_c3_mix_0 ; //( 6, ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  yarp::sig::Matrix J_l_c4_mix_0 ; //( 6, ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ) ) ;

  yarp::sig::Matrix J_r_c1_mix_0 ; //( 6, ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  yarp::sig::Matrix J_r_c2_mix_0 ; //( 6, ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  yarp::sig::Matrix J_r_c3_mix_0 ; //( 6, ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  yarp::sig::Matrix J_r_c4_mix_0 ; //( 6, ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ) ) ;
  
  yarp::sig::Matrix J_l_hand_mix_0 ; //( 6, ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  yarp::sig::Matrix J_r_hand_mix_0 ; //( 6, ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ) ) ;
  
  yarp::sig::Matrix J_l1_hand_mix_0 ; //( 6, ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  yarp::sig::Matrix J_l2_hand_mix_0 ; //( 6, ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  yarp::sig::Matrix J_l3_hand_mix_0 ; //( 6, ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  yarp::sig::Matrix J_l4_hand_mix_0 ; //( 6, ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ) ) ;

  yarp::sig::Matrix J_r1_hand_mix_0 ; //( 6, ( size_q + 6 ) ) ; // robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  yarp::sig::Matrix J_r2_hand_mix_0 ; //( 6, ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  yarp::sig::Matrix J_r3_hand_mix_0 ; //( 6, ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  yarp::sig::Matrix J_r4_hand_mix_0 ; //( 6, ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ) ) ;

  //------------------------------------------------
  yarp::sig::Matrix J_l_c1_body_0 ; //= locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_c1_w_0), zero_3 ))* J_l_c1_mix_0 ;
  yarp::sig::Matrix J_l_c2_body_0 ; //= locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_c2_w_0), zero_3 ))* J_l_c2_mix_0 ;
  yarp::sig::Matrix J_l_c3_body_0 ; //= locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_c3_w_0), zero_3 ))* J_l_c3_mix_0 ;
  yarp::sig::Matrix J_l_c4_body_0 ; //= locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_c4_w_0), zero_3 ))* J_l_c4_mix_0 ;

  yarp::sig::Matrix J_r_c1_body_0 ; //= locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_c1_w_0), zero_3 ))* J_r_c1_mix_0 ;
  yarp::sig::Matrix J_r_c2_body_0 ; //= locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_c2_w_0), zero_3 ))* J_r_c2_mix_0 ;
  yarp::sig::Matrix J_r_c3_body_0 ; //= locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_c3_w_0), zero_3 ))* J_r_c3_mix_0 ;
  yarp::sig::Matrix J_r_c4_body_0 ; //= locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_c4_w_0), zero_3 ))* J_r_c4_mix_0 ;

  yarp::sig::Matrix J_l_hand_body_0 ; //= locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_hand_w_0), zero_3 ))* J_l_hand_mix_0 ;
  yarp::sig::Matrix J_r_hand_body_0 ; //= locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_hand_w_0), zero_3 ))* J_r_hand_mix_0 ;

  yarp::sig::Matrix J_l1_hand_body_0 ; //= locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l1_hand_w_0), zero_3 ))* J_l1_hand_mix_0 ;
  yarp::sig::Matrix J_l2_hand_body_0 ; //= locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l2_hand_w_0), zero_3 ))* J_l2_hand_mix_0 ;
  yarp::sig::Matrix J_l3_hand_body_0 ; //= locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l3_hand_w_0), zero_3 ))* J_l3_hand_mix_0 ;
  yarp::sig::Matrix J_l4_hand_body_0 ; //= locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l4_hand_w_0), zero_3 ))* J_l4_hand_mix_0 ;

  yarp::sig::Matrix J_r1_hand_body_0 ; //= locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r1_hand_w_0), zero_3 ))* J_r1_hand_mix_0 ;
  yarp::sig::Matrix J_r2_hand_body_0 ; //= locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r2_hand_w_0), zero_3 ))* J_r2_hand_mix_0 ;
  yarp::sig::Matrix J_r3_hand_body_0 ; //= locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r3_hand_w_0), zero_3 ))* J_r3_hand_mix_0 ;
  yarp::sig::Matrix J_r4_hand_body_0 ; //= locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r4_hand_w_0), zero_3 ))* J_r4_hand_mix_0 ;
  
  //---------------------------------------------------------------------------------------------------------------------------------------------------------------
  // Introducing Spatial Jacobian terms: Fixed base in {AW}
  
  yarp::sig::Matrix J_aw_l_c1_spa_0 ; //= locoman::utils::Adjoint(T_aw_l_c1_0)* J_l_c1_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c1_spa_0 ;// locoman::utils::Adjoint(T_aw_l_c1_0)* J_l_c1_body_0
  yarp::sig::Matrix J_aw_l_c2_spa_0 ; //= locoman::utils::Adjoint(T_aw_l_c2_0)* J_l_c2_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c2_spa_0 ;
  yarp::sig::Matrix J_aw_l_c3_spa_0 ; //= locoman::utils::Adjoint(T_aw_l_c3_0)* J_l_c3_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c3_spa_0 ;
  yarp::sig::Matrix J_aw_l_c4_spa_0 ; //= locoman::utils::Adjoint(T_aw_l_c4_0)* J_l_c4_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c4_spa_0 ;

  yarp::sig::Matrix J_aw_r_c1_spa_0 ; //= locoman::utils::Adjoint(T_aw_r_c1_0)* J_r_c1_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c1_spa_0 ;
  yarp::sig::Matrix J_aw_r_c2_spa_0 ; //= locoman::utils::Adjoint(T_aw_r_c2_0)* J_r_c2_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c2_spa_0 ;
  yarp::sig::Matrix J_aw_r_c3_spa_0 ; //= locoman::utils::Adjoint(T_aw_r_c3_0)* J_r_c3_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c3_spa_0 ;
  yarp::sig::Matrix J_aw_r_c4_spa_0 ; //= locoman::utils::Adjoint(T_aw_r_c4_0)* J_r_c4_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c4_spa_0 ;

  yarp::sig::Matrix J_aw_l1_hand_spa_0 ; //= locoman::utils::Adjoint(T_aw_l1_hand_0)* J_l1_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c1_spa_0 ;// locoman::utils::Adjoint(T_aw_l_c1_0)* J_l_c1_body_0
  yarp::sig::Matrix J_aw_l2_hand_spa_0 ; //= locoman::utils::Adjoint(T_aw_l2_hand_0)* J_l2_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c2_spa_0 ;
  yarp::sig::Matrix J_aw_l3_hand_spa_0 ; //= locoman::utils::Adjoint(T_aw_l3_hand_0)* J_l3_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c3_spa_0 ;
  yarp::sig::Matrix J_aw_l4_hand_spa_0 ; //= locoman::utils::Adjoint(T_aw_l4_hand_0)* J_l4_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c4_spa_0 ;

  yarp::sig::Matrix J_aw_r1_hand_spa_0 ; //= locoman::utils::Adjoint(T_aw_r1_hand_0)* J_r1_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c1_spa_0 ;
  yarp::sig::Matrix J_aw_r2_hand_spa_0 ; //= locoman::utils::Adjoint(T_aw_r2_hand_0)* J_r2_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c2_spa_0 ;
  yarp::sig::Matrix J_aw_r3_hand_spa_0 ; //= locoman::utils::Adjoint(T_aw_r3_hand_0)* J_r3_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c3_spa_0 ;
  yarp::sig::Matrix J_aw_r4_hand_spa_0 ; //= locoman::utils::Adjoint(T_aw_r4_hand_0)* J_r4_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c4_spa_0 ;
  
  //-----------------------------------------------------------
  yarp::sig::Matrix Q_mg ;
  
  yarp::sig::Matrix Q_aw_l_c1 ; //(size_q+ 6, size_q + 6)   ; //= Q_ci(J_aw_l_c1_spa_0, T_aw_l_c1_0, fc_l_c1_filt ) ;
  yarp::sig::Matrix Q_aw_l_c2 ; //(size_q+ 6, size_q + 6)   ; // = Q_ci(J_aw_l_c2_spa_0, T_aw_l_c2_0, fc_l_c2_filt ) ; // (size_q+ 6, size_q + 6) ;
  yarp::sig::Matrix Q_aw_l_c3 ; //(size_q+ 6, size_q + 6)   ; // = Q_ci(J_aw_l_c3_spa_0, T_aw_l_c3_0, fc_l_c3_filt ) ; //(size_q+ 6, size_q + 6) ; 
  yarp::sig::Matrix Q_aw_l_c4 ; //(size_q+ 6, size_q + 6)   ; // = Q_ci(J_aw_l_c4_spa_0, T_aw_l_c4_0, fc_l_c4_filt ) ; //(size_q+ 6, size_q + 6) ;

  yarp::sig::Matrix Q_aw_r_c1 ; //(size_q+ 6, size_q + 6)   ; // = Q_ci(J_aw_r_c1_spa_0, T_aw_r_c1_0, fc_r_c1_filt ) ; //(size_q+ 6, size_q + 6) ;
  yarp::sig::Matrix Q_aw_r_c2 ; //(size_q+ 6, size_q + 6)   ; // = Q_ci(J_aw_r_c2_spa_0, T_aw_r_c2_0, fc_r_c2_filt ) ; //(size_q+ 6, size_q + 6) ;
  yarp::sig::Matrix Q_aw_r_c3 ; //(size_q+ 6, size_q + 6)   ; // = Q_ci(J_aw_r_c3_spa_0, T_aw_r_c3_0, fc_r_c3_filt ) ; //(size_q+ 6, size_q + 6) ; 
  yarp::sig::Matrix Q_aw_r_c4 ; //(size_q+ 6, size_q + 6)   ; // = Q_ci(J_aw_r_c4_spa_0, T_aw_r_c4_0, fc_r_c4_filt ) ; //(size_q+ 6, size_q + 6) ;

  yarp::sig::Matrix Q_aw_l1_hand ; //(size_q+ 6, size_q + 6)   ; // = Q_ci(J_aw_r_c1_spa_0, T_aw_r_c1_0, fc_r_c1_filt ) ; //(size_q+ 6, size_q + 6) ;
  yarp::sig::Matrix Q_aw_l2_hand ; //(size_q+ 6, size_q + 6)   ; // = Q_ci(J_aw_r_c2_spa_0, T_aw_r_c2_0, fc_r_c2_filt ) ; //(size_q+ 6, size_q + 6) ;
  yarp::sig::Matrix Q_aw_l3_hand ; //(size_q+ 6, size_q + 6)   ; // = Q_ci(J_aw_r_c3_spa_0, T_aw_r_c3_0, fc_r_c3_filt ) ; //(size_q+ 6, size_q + 6) ; 
  yarp::sig::Matrix Q_aw_l4_hand ; //(size_q+ 6, size_q + 6)   ; // = Q_ci(J_aw_r_c4_spa_0, T_aw_r_c4_0, fc_r_c4_filt ) ; //(size_q+ 6, size_q + 6) ;
    
  yarp::sig::Matrix Q_aw_r1_hand ; //(size_q+ 6, size_q + 6)   ; // = Q_ci(J_aw_r_c1_spa_0, T_aw_r_c1_0, fc_r_c1_filt ) ; //(size_q+ 6, size_q + 6) ;
  yarp::sig::Matrix Q_aw_r2_hand ; //(size_q+ 6, size_q + 6)   ; // = Q_ci(J_aw_r_c2_spa_0, T_aw_r_c2_0, fc_r_c2_filt ) ; //(size_q+ 6, size_q + 6) ;
  yarp::sig::Matrix Q_aw_r3_hand ; //(size_q+ 6, size_q + 6)   ; // = Q_ci(J_aw_r_c3_spa_0, T_aw_r_c3_0, fc_r_c3_filt ) ; //(size_q+ 6, size_q + 6) ; 
  yarp::sig::Matrix Q_aw_r4_hand ; //(size_q+ 6, size_q + 6)   ; // = Q_ci(J_aw_r_c4_spa_0, T_aw_r_c4_0, fc_r_c4_filt ) ; //(size_q+ 6, size_q + 6) ;
  
  yarp::sig::Matrix Q_aw_l_tot ; //(size_q+ 6, size_q + 6)   ; // = Q_aw_l_c1 + Q_aw_l_c2 + Q_aw_l_c3 + Q_aw_l_c4;
  yarp::sig::Matrix Q_aw_r_tot ; //(size_q+ 6, size_q + 6)   ; // = Q_aw_r_c1 + Q_aw_r_c2 + Q_aw_r_c3 + Q_aw_r_c4;
  yarp::sig::Matrix Q_aw_l_hand_tot ; 
  yarp::sig::Matrix Q_aw_r_hand_tot ; //(size_q+ 6, size_q + 6)   ; // = Q_aw_r_c1 + Q_aw_r_c2 + Q_aw_r_c3 + Q_aw_r_c4;

  yarp::sig::Matrix Q_aw_c ; //(size_q+ 6, size_q + 6)   ; // =  Q_aw_l_tot + Q_aw_r_tot ;  
  yarp::sig::Matrix U_aw_s_cont ; //( 6 , 6) ; // = Q_aw_c.submatrix( 0 ,  5 , 0, 5) ;     
  yarp::sig::Matrix Q_aw_s_cont ; //( 6 , size_q ) ; //  = Q_aw_c.submatrix( 0  , 5,  6,  (Q_aw_c.cols()-1)  ) ;
  
  yarp::sig::Matrix Q_aw_c_f_rh ; //(size_q+ 6, size_q + 6)   ; // =  Q_aw_l_tot + Q_aw_r_tot ;  
  yarp::sig::Matrix U_aw_s_c_f_rh ; //( 6 , 6) ; // = Q_aw_c.submatrix( 0 ,  5 , 0, 5) ;     
  yarp::sig::Matrix Q_aw_s_c_f_rh ; //( 6 , size_q ) ; //  = Q_aw_c.submatrix( 0  , 5,  6,  (Q_aw_c.cols()-1)  ) ;
 
  yarp::sig::Matrix Q_aw_c_f_h ; // = Q_aw_c + Q_aw_l_hand_tot + Q_aw_r_hand_tot ;
  yarp::sig::Matrix U_aw_s_f_h ; // = Q_aw_c_f_h.submatrix( 0 ,  5 , 0, 5) ;     
  yarp::sig::Matrix Q_aw_s_f_h ; //
  //----------------------------------------------------------------------------------------
  
  yarp::sig::Matrix FLMM   ;   //    (30, size_q + 30)  ;  //= locoman::utils::FLMM_redu(J_c, S_c, Q_aw_s_cont, U_aw_s_cont, Kc ) ;
  yarp::sig::Matrix cFLMM  ;   //   ( 30, size_q + 30 ) ;
  yarp::sig::Matrix Rf     ;   //   (24, size_q)   ; 
  yarp::sig::Matrix Rf_filt  ; //   (24, size_q) ;
  yarp::sig::Matrix Rf_filt_pinv  ; //   ( size_q, 24) ;
  yarp::sig::Matrix Rf_filt_f_h ;
  
  yarp::sig::Matrix Big_J_new ;
  yarp::sig::Matrix Big_Rf_new ;
  
  yarp::sig::Vector d_fc_des_to_world ; //(size_fc)  ;
  yarp::sig::Vector d_EE_r_des ; //(6,0.0) ;
  yarp::sig::Vector d_EE_l_des ; //(6,0.0) ;
  yarp::sig::Matrix T_l_c1_r_c1_loop ; //(4,4) ;
  yarp::sig::Matrix T_r_c1_l_c1_loop ; //(4,4) ;
  yarp::sig::Matrix J_com_w ; //( 6, ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ) ) ;
  yarp::sig::Matrix J_com_w_redu ; //( 3,  ( size_q + 6 )  ) ; //( robot.getNumberOfKinematicJoints() + 6 ))   ;
  yarp::sig::Matrix J_com_aw ; //( 3,  ( size_q + 6 ) ) ; //( robot.getNumberOfKinematicJoints() + 6 ))   ;
  yarp::sig::Matrix J_com_waist ; //( 3,  ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ))   ;

  yarp::sig::Matrix J_com_w_q_i ; //( 6, ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ) ) ;
  yarp::sig::Matrix J_com_w_redu_q_i ; //( 3,  ( size_q + 6 )  ) ; //( robot.getNumberOfKinematicJoints() + 6 ))   ;
  yarp::sig::Matrix J_com_aw_q_i ;
  
  
  yarp::sig::Matrix J_r_c1_aw ; //( 6, ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ) ) ;
  yarp::sig::Matrix J_l_c1_aw ; //( 6, ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ) ) ;

    
    
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
