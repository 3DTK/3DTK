/*
Program:
 * This program is written for broadcasting coordinate frames to tf.
 * History:
 * 04/05/2018	
 * Copyright (C) Shitong Du
 */


#include "curvefusion/tf_broadcaster.h"


double time_s[60000];
std::vector<MatrixXd> matrix_new;

//unsigned int i=0;
unsigned int i=0;
void LaserCallback(const geometry_msgs::PoseStamped& slamout) {

       
     //  if(i<=(matrix_new.size()-1)) {

       static tf::TransformBroadcaster br;
       
       //for(unsigned int i=0;i<=matrix_new.size()-1;i++) {

     //  if(time_s[i]==(slamout.header.stamp.toSec())) {

       
       ros::Time current_time=slamout.header.stamp;
       geometry_msgs::TransformStamped msg1;
       tf::StampedTransform trans;
       trans.stamp_          = ros::Time::now();
       trans.frame_id_       = "map";
       trans.child_frame_id_ = "base_link";
       tf::Vector3 translation;
       
       translation.setX((matrix_new[i])(0,3));
       translation.setY((matrix_new[i])(1,3)); 
       translation.setZ(0.0);

       trans.setOrigin(translation);

          
       tf::Matrix3x3 Matrix((matrix_new[i])(0,0),(matrix_new[i])(0,1),(matrix_new[i])(0,2),
                        (matrix_new[i])(1,0),(matrix_new[i])(1,1),(matrix_new[i])(1,2),
                        (matrix_new[i])(2,0),(matrix_new[i])(2,1),(matrix_new[i])(2,2));

       trans.setBasis(Matrix);
       
       
   
       br.sendTransform(trans);

       geometry_msgs::TransformStamped lmstrans;
       lmstrans.header.stamp          = ros::Time::now();
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



       geometry_msgs::TransformStamped riegltrans;
       riegltrans.header.stamp          = ros::Time::now();
       riegltrans.header.frame_id       = "base_link";
       riegltrans.child_frame_id = "riegl";
     
       riegltrans.transform.translation.x=-0.2;
       riegltrans.transform.translation.y=0.0;
       riegltrans.transform.translation.z=0.2;

       riegltrans.transform.rotation.x=0.0;    
       riegltrans.transform.rotation.y=0.0;
       riegltrans.transform.rotation.z=0.923879515254;   //1.0;
       riegltrans.transform.rotation.w=0.382683474029;     //2.6794896585e-08; 180degree
       br.sendTransform(riegltrans);
      // ros::spinOnce();
       //rate.sleep();
       cout<<i++<<" "<<"timestamps :"<<" "<<slamout.header.stamp<<endl;

   //    if(s==matrix_new.size())
  //    ros::shutdown();
  //    }
      
//     }
}


//tf_broadcaster::tf_broadcaster(std::vector<MatrixXd> &poses_result) {
int tf_broadcaster::broadcaster_tf() {
 // ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle node;
 // cout<<matrix_new.size()-1<<endl;
  system("gnome-terminal -e /home/du/Documents/xinde/play.sh");
  //if(s<20)
  ros::Subscriber sub = node.subscribe("slam_out_pose",3,LaserCallback);
   
  ros::spin();
  return 0;
}





