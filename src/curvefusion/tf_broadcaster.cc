/* Program:
 * This program is written for broadcasting coordinate frames to tf.
 * History:
 * 04/05/2018	
 * Copyright (C) Shitong Du
 */


#include "curvefusion/tf_broadcaster.h"


double time_s[5000];
std::vector<MatrixXd> matrix_new;
unsigned int num=0;
//void LaserCallback(const sensor_msgs::LaserScan& scan) {
void LaserCallback(const geometry_msgs::PoseStamped& slamout) {
static tf::TransformBroadcaster br;

//ros::Rate rate(10.0);
//  while (node.ok()){
for(unsigned int i=0;i<num;i++) {

  if(time_s[i]==(slamout.header.stamp.toSec())) {
  ros::Time current_time=slamout.header.stamp;
  geometry_msgs::TransformStamped msg1;
       tf::StampedTransform trans;
       trans.stamp_          = current_time;
       trans.frame_id_       = "map";
       trans.child_frame_id_ = "base_link";
       tf::Vector3 translation;
       
       cout<<i<<" "<<"timestamps :"<<" "<<slamout.header.stamp<<endl;
#ifdef POINT3D
       translation.setX((matrix_new[i])(0,3));
       translation.setY((matrix_new[i])(1,3)); 
       translation.setZ((matrix_new[i])(2,3));
#else
       translation.setX((matrix_new[i])(0,2));
       translation.setY((matrix_new[i])(1,2));
       translation.setZ(0.0);
#endif
       trans.setOrigin(translation);

#ifdef POINT3D           
       tf::Matrix3x3 Matrix((matrix_new[i])(0,0),(matrix_new[i])(0,1),(matrix_new[i])(0,2),
                        (matrix_new[i])(1,0),(matrix_new[i])(1,1),(matrix_new[i])(1,2),
                        (matrix_new[i])(2,0),(matrix_new[i])(2,1),(matrix_new[i])(2,2));
#else
       tf::Matrix3x3 Matrix((matrix_new[i])(0,0),(matrix_new[i])(0,1),0.0,
                        (matrix_new[i])(1,0),(matrix_new[i])(1,1),0.0,
                        0,0,1.0);
#endif
       trans.setBasis(Matrix);
       
       //transformStampedTFToMsg(trans,msg1);
   
       br.sendTransform(trans);

//
      // ros::Time current1_time=ros::Time::now();
       geometry_msgs::TransformStamped lmstrans;
       lmstrans.header.stamp          = current_time;
       lmstrans.header.frame_id       = "base_link";
       lmstrans.child_frame_id = "front_laser";
     
       lmstrans.transform.translation.x=0.0;
       lmstrans.transform.translation.y=0.0;
       lmstrans.transform.translation.z=0.0;

       lmstrans.transform.rotation.x=0.0;    
       lmstrans.transform.rotation.y=0.0;
       lmstrans.transform.rotation.z=0.0;
       lmstrans.transform.rotation.w=1.0;
       br.sendTransform(lmstrans);


       //ros::Time current_time=ros::Time::now();
       geometry_msgs::TransformStamped riegltrans;
       riegltrans.header.stamp          = current_time;
       riegltrans.header.frame_id       = "base_link";
       riegltrans.child_frame_id = "riegl";
     
       riegltrans.transform.translation.x=-0.2;
       riegltrans.transform.translation.y=0.0;
       riegltrans.transform.translation.z=0.2;

       riegltrans.transform.rotation.x=0.0;    
       riegltrans.transform.rotation.y=0.0;
       riegltrans.transform.rotation.z=0.0;
       riegltrans.transform.rotation.w=1.0;
       br.sendTransform(riegltrans);
      // ros::spinOnce();
       //rate.sleep();
     }
   }
  
}


tf_broadcaster::tf_broadcaster(std::vector<MatrixXd> &poses_result) {

 // ros::init(argc, argv, "my_tf_broadcaster");

  ros::NodeHandle node;
  
  ros::Subscriber sub = node.subscribe("slam_out_pose",5,LaserCallback);
   
  ros::spin();
  //return 0;
}





