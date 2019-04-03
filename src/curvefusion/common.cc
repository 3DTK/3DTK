#include "curvefusion/common.h"
#define MAX(x,y) ((x)<(y)?(y):(x))
#define MIN(x,y) ((x)<(y)?(x):(y))
string pose_dir;
string point_dir;

string pose_dir_final;
string point_dir_final;

std::vector<Vector3d> V_point1;
std::vector<Vector3d> V_point2;
std::vector<Matrix2d> AbelianR;

unsigned int seq_interval[60000]; 
static std::vector<Curves*> Interpoints;
double inf = std::numeric_limits<double>::infinity();


void readkitti(std::vector<double> *hector_matrix,std::vector<double> *point_corr,std::string dir) {

  string FileName1 = dir +"od"+ ".txt";
  string FileName2 = dir +"gr"+ ".txt";

  std::vector<double> point1_curve[12];
  std::vector<double> point2_curve[12];
  cout<< FileName1<<endl;
  ifstream poseg_in;
  poseg_in.open(FileName1);
  double point_curve[12];
  double inMatrix[12], tMatrix[12];
  unsigned int a=0;
  geometry_msgs::PoseStamped pose1;
  geometry_msgs::PoseStamped pose2;

  while (poseg_in.good()){
   
   for (int i = 0; i < 12; i++) //{    converting KITTI coordinate system into ROS coordinate system
    poseg_in >> inMatrix[i];           //KITTI  x right, y down, z forward
                                       //ROS    x forward y left z upward
   // point1_curve[i].push_back(point_curve[i]);
  // }
    
     tMatrix[0] =  inMatrix[10];
     tMatrix[1] = -inMatrix[8];
     tMatrix[2] = -inMatrix[9];
     tMatrix[3] =  inMatrix[11];
     tMatrix[4] = -inMatrix[2];
     tMatrix[5] =  inMatrix[0];
     tMatrix[6] =  inMatrix[1];
     tMatrix[7] = -inMatrix[3];
     tMatrix[8] = -inMatrix[6];
     tMatrix[9] =  inMatrix[4];
     tMatrix[10] = inMatrix[5];
     tMatrix[11] = -inMatrix[7];
     //tMatrix[12] = 0.0;
     //tMatrix[13] = 0.0;
     //tMatrix[14] = 0.0;
     //tMatrix[15] = 1.0;
     
    for(int j=0;j<=11;j++)
     point1_curve[j].push_back( tMatrix[j]);
 }
   
   poseg_in.close();
   
  ifstream poseo_in;
  poseo_in.open(FileName2);
  
   while (poseo_in.good()){
   
   for (int i = 0; i < 12; i++) 
     poseo_in >> inMatrix[i];
  
     tMatrix[0] =  inMatrix[10];
     tMatrix[1] = -inMatrix[8];
     tMatrix[2] = -inMatrix[9];
     tMatrix[3] =  inMatrix[11];
     tMatrix[4] = -inMatrix[2];
     tMatrix[5] =  inMatrix[0];
     tMatrix[6] =  inMatrix[1];
     tMatrix[7] = -inMatrix[3];
     tMatrix[8] = -inMatrix[6];
     tMatrix[9] =  inMatrix[4];
     tMatrix[10] = inMatrix[5];
     tMatrix[11] = -inMatrix[7];
     //tMatrix[12] = 0.0;
     //tMatrix[13] = 0.0;
     //tMatrix[14] = 0.0;
     //tMatrix[15] = 1.0;
     
    for(int j=0;j<=11;j++)
     point2_curve[j].push_back(tMatrix[j]);
 }
   poseo_in.close();

  unsigned int sample_points = point1_curve[0].size();
  MatrixXd R_g(3,3); 
  cout<<sample_points<<endl;
  double qm[4];
  for(unsigned int i=0;i<= sample_points-2;i++) { 
     point_corr[0].push_back(i) ;
     point_corr[1].push_back(point1_curve[3][i]) ;  //x forward
     point_corr[2].push_back(point1_curve[7][i]) ;  //y left  //ground truth
     point_corr[3].push_back(point1_curve[11][i]) ; //z upward

     point_corr[4].push_back(point2_curve[3][i]) ;  //x forward
     point_corr[5].push_back(point2_curve[7][i]) ;  //y left  //odometry
     point_corr[6].push_back(point2_curve[11][i]) ; //z upward

     R_g(0,0)=point1_curve[0][i];
     R_g(0,1)=point1_curve[1][i];
     R_g(0,2)=point1_curve[2][i];
   
     R_g(1,0)=point1_curve[4][i];
     R_g(1,1)=point1_curve[5][i];
     R_g(1,2)=point1_curve[6][i];

     R_g(2,0)=point1_curve[8][i];
     R_g(2,1)=point1_curve[9][i];
     R_g(2,2)=point1_curve[10][i];
   
     Matrix3ToQuat(R_g,qm);
     hector_matrix[4].push_back(qm[0]);
     hector_matrix[0].push_back(i);
     hector_matrix[1].push_back(qm[1]);
     hector_matrix[2].push_back(qm[2]);
     hector_matrix[3].push_back(qm[3]);
     hector_matrix[5].push_back(point1_curve[3][i]);
     hector_matrix[6].push_back(point1_curve[7][i]);
     hector_matrix[7].push_back(point1_curve[11][i]);

   // cout<<point1_curve[11][i]<<endl;
  }
}


void curvspace(unsigned int s_num,curvesVector &points)
{
   Curves *curves_ini;
   curves_ini=points[0];
   Vector3d currentpt1=curves_ini->points1;
   Vector3d currentpt2=curves_ini->points2;
   //double   current_time=curves_ini->time_stamps;
   Vector3d newpt1(0,0,0);
   unsigned int indfirst1=1;
   unsigned int indfirst2=1;
   Vector3d q1=currentpt1;
   Vector3d q2=currentpt2;
   V_point1.clear();
   V_point2.clear();
   //V_time.clear();
   V_point1.push_back(q1);
   V_point2.push_back(q2);
   //V_time.push_back(current_time);
   double distsum1;
   Vector3d ptnow1;
   int kk1;
   Vector3d pttarget1;
   double remainder1;
   Vector3d diss1;   
   double disttmp1;
  
   unsigned int len=points.size();
   double dist1[len-1];
   double dist2[len-1];
   double totaldist1=0;
   double totaldist2=0;
   double intv1=0;
   double intv2=0;
 
   for(unsigned int i=0;i<len-1;i++){
   
     Curves* c_point = points[i];
     Curves* n_point = points[i+1];
     Vector3d c_p1=c_point->points1;
     Vector3d n_p1=n_point->points1;

     Vector3d c_p2=c_point->points2;
     Vector3d n_p2=n_point->points2;

     Vector3d dis1=c_p1-n_p1;
     Vector3d dis2=c_p2-n_p2;
#ifdef POINT3D
     dist1[i]=sqrt(dis1(0)*dis1(0)+dis1(1)*dis1(1)+dis1(2)*dis1(2));
     dist2[i]=sqrt(dis2(0)*dis2(0)+dis2(1)*dis2(1)+dis2(2)*dis2(2));
#else
     dist1[i]=sqrt(dis1(0)*dis1(0)+dis1(1)*dis1(1));
     dist2[i]=sqrt(dis2(0)*dis2(0)+dis2(1)*dis2(1));
#endif
    }
   
    for(unsigned int j=0;j<len-1;j++){
     totaldist1+=dist1[j];
     totaldist2+=dist2[j]; 
    }
     intv1=totaldist1/(s_num-1);
     intv2=totaldist2/(s_num-1);
    
    for(unsigned int k = 1; k <= s_num-1; k++){
    
      newpt1<<0,0,0;
      distsum1=0;
      ptnow1=currentpt1;
      //pt_time=current_time;
      kk1=0;
      pttarget1=points[indfirst1]->points1;
      remainder1 = intv1;
      while ((newpt1(0)==0)&&(newpt1(1)==0)&&(newpt1(2)==0)){
      
       diss1=ptnow1-pttarget1;

#ifdef POINT3D        
       disttmp1=sqrt(diss1(0)*diss1(0)+diss1(1)*diss1(1)+diss1(2)*diss1(2));
#else
       disttmp1=sqrt(diss1(0)*diss1(0)+diss1(1)*diss1(1));
#endif       
       distsum1=distsum1+disttmp1;
       
       if (distsum1>=intv1){
         interpintv(ptnow1,pttarget1,remainder1,newpt1);
        // new_time=pt_time;
       }
       else {
         remainder1=remainder1 - disttmp1;
         ptnow1=pttarget1;
         kk1=kk1+1;
         if((indfirst1+kk1)>(len-1))
            newpt1=((points[len-1])->points1);
         
         else  pttarget1 = ((points[indfirst1+kk1])->points1);
       
       }

       }

       Vector3d newpt2(0,0,0);
       double distsum2=0;
       Vector3d ptnow2=currentpt2;
       int kk2=0;
       Vector3d pttarget2=points[indfirst2]->points2;
       double remainder2 = intv2;
       while ((newpt2(0)==0)&&(newpt2(1)==0)&&(newpt2(2)==0)){

       Vector3d diss2=ptnow2-pttarget2;

       double disttmp2=sqrt(diss2(0)*diss2(0)+diss2(1)*diss2(1)+diss2(2)*diss2(2));
       distsum2=distsum2+disttmp2;
        if (distsum2>=intv2)
         interpintv(ptnow2,pttarget2,remainder2,newpt2);
        else {
         remainder2=remainder2 - disttmp2;
         ptnow2=pttarget2;
         kk2=kk2+1;

         if((indfirst2+kk2)>(len-1))
            newpt2=((points[len-1])->points2);
         else  pttarget2 = ((points[indfirst2+kk2])->points2);
       
         }

        }
         
         V_point1.push_back(newpt1);
         V_point2.push_back(newpt2);
         currentpt1=newpt1;
         currentpt2=newpt2;
        
         indfirst1 = indfirst1 + kk1;
         indfirst2 = indfirst2 + kk2;
        
      }
       
}


void curve_rep(curvesVector &points,int type)
{
  
  if(type==1){
  
  for(curvesVector::iterator it = points.begin();
      it != points.end();
      ++it) {
    
       Curves *CurrentPoint = *it;

       if(it==(points.end()-1)){
       Curves *NextPoint = points.front();
       CurrentPoint->Trans_Mat(CurrentPoint,NextPoint);
       }     
       else{
       Curves *NextPoint = *(it+1);
       CurrentPoint->Trans_Mat(CurrentPoint,NextPoint);
       }     
             
   }
  }

  else{
    for(curvesVector::iterator it = points.begin();
      it != (points.end()-1);
      ++it) {
        
       Curves *CurrentPoint = *it;
       Curves *NextPoint = *(it+1);
       CurrentPoint->Trans_Mat(CurrentPoint,NextPoint);
    }
       
  }
      
}      

/*void correspondence(curvesVector &points,int type)
{
  inv_rep(points,0);
  if(type==1) 
  {
    




  }


}*/

void general_align(curvesVector &points,int type)
{
  
   Matrix2d m;

   m(0,0) = 1;
   m(0,1) = 0;
   m(1,0) = 0;
   m(1,1) = 1;
   AbelianR.push_back(m);

   m(0,0) = -1;
   m(0,1) = 0;
   m(1,0) = 0;
   m(1,1) = 1;
   AbelianR.push_back(m);

   m(0,0) = 1;
   m(0,1) = 0;
   m(1,0) = 0;
   m(1,1) = -1;
   AbelianR.push_back(m);

   m(0,0) = -1;
   m(0,1) = 0;
   m(1,0) = 0;
   m(1,1) = -1;
   AbelianR.push_back(m);

   for(int i=0;i<4;i++){
     
      for(unsigned int j = 0; j < points.size(); j++){
          
          Vector2d point2d;
          Curves *curves;
          curves=points[j];
          for(int k=0; k<2;k++)
          point2d(k)=(curves->points2(k));
          point2d=(AbelianR[i]*point2d);
          for(int h=0;h<2;h++ )
          (curves->points2(h))=point2d(h);    
      }
      
      curve_rep(points,type);
     // correspondence(points,type);
 

   }



}


double Aligndata(const vector<PtPair>& pairs,
                        Matrix3d &R_icp,
                        Vector3d &translation_icp,
                        const double centroid_m[3],
                        const double centroid_d[3])
{
  double error = 0;
  double sum = 0.0;
  bool quiet=false;

     
  // Get centered PtPairs
  double** m = new double*[pairs.size()];
  double** d = new double*[pairs.size()];

  for(unsigned int i = 0; i <  pairs.size(); i++){
    m[i] = new double[3];
    d[i] = new double[3];
    m[i][0] = pairs[i].p1.x - centroid_m[0];
    m[i][1] = pairs[i].p1.y - centroid_m[1];
    m[i][2] = pairs[i].p1.z - centroid_m[2];
    d[i][0] = pairs[i].p2.x - centroid_d[0];
    d[i][1] = pairs[i].p2.y - centroid_d[1];
    d[i][2] = pairs[i].p2.z - centroid_d[2];

    sum += sqr(pairs[i].p1.x - pairs[i].p2.x)
      + sqr(pairs[i].p1.y - pairs[i].p2.y)
      + sqr(pairs[i].p1.z - pairs[i].p2.z) ;

  }

  error = sqrt(sum / (double)pairs.size());

  if (!quiet) {
    cout.setf(ios::basefield);
    cout << "SVD RMS point-to-point error = "
      << resetiosflags(ios::adjustfield) << setiosflags(ios::internal)
      << resetiosflags(ios::floatfield) << setiosflags(ios::fixed)
      << std::setw(10) << std::setprecision(7)
      << error
      << "  using " << std::setw(6) << (int)pairs.size() << " points" << endl;
  }

  // Fill H matrix
  Matrix3d  H, R;
  for(int j = 0; j < 3; j++){
    for(int k = 0; k < 3; k++){
      H(j, k) = 0.0;     
    }
  }
  
  for(unsigned int i = 0; i < pairs.size(); i++){
    for(int j = 0; j < 3; j++){
      for(int k = 0; k < 3; k++){
        H(j, k) += d[i][j]*m[i][k];
      }
    }
  }
    
  // Make SVD
  JacobiSVD<MatrixXd>svd(H, ComputeFullU | ComputeFullV); 
  MatrixXd U = svd.matrixU();

  MatrixXd V = svd.matrixV();
 

  // Get rotation
  R = V*(U.transpose());

  // Check if R turned out to be a reflection (likely because of large errors or coplanar input)
  // (cf. Arun, Huang, and Blostein: "Least-Squares Fitting of Two 3-D Point Sets")
  if(R.determinant() < 0) {
    V(0, 2) = -V(0, 2);
    V(1, 2) = -V(1, 2);
    V(2, 2) = -V(2, 2);
    // unless input is extremely noisy or colinear, R should now be a rotation
    R = V*(U.transpose());

    if(R.determinant() < 0) {
      // if we still failed, at least give a warning
      cerr << "Warning - icp6D_SVD::Align failed to compute a matching transformation - the returned solution is a reflection!" << endl;
    }
  }

  // Calculate translation
//  double translation[3];
  Vector3d col_vec(3);
  for(int j = 0; j < 3; j++)
    col_vec(j) = centroid_d[j];
  Vector3d r_time_colVec = R*col_vec;
  translation_icp(0)= centroid_m[0] - r_time_colVec(0);
  translation_icp(1)= centroid_m[1] - r_time_colVec(1);
  translation_icp(2)= centroid_m[2] - r_time_colVec(2);


  // Fill result
/*  alignfx[0] = R(1,1);
  alignfx[1] = R(2,1);
  alignfx[2] = 0;
  alignfx[2] = R(3,1);
  alignfx[3] = 0;
  alignfx[4] = R(1,2);
  alignfx[5] = R(2,2);
  alignfx[6] = R(3,2);
  alignfx[7] = 0;
  alignfx[8] = R(1,3);
  alignfx[9] = R(2,3);
  alignfx[10] = R(3,3);
  alignfx[11] = 0;
  alignfx[12] = translation[0];
  alignfx[13] = translation[1];
  alignfx[14] = translation[2];

  alignfx[15] = 1;
*/
  R_icp=R;


  for(unsigned int i = 0; i <  pairs.size(); i++) {
    delete [] m[i];
    delete [] d[i];
  }
  delete [] m;
  delete [] d;

  return error;
}

void interpintv(Vector3d pt1,Vector3d pt2,double intv,Vector3d &newpt)
{
  Vector3d dirvec=pt2-pt1;

#ifndef POINT3D  
  dirvec(2)=0;
#endif

  double value=dirvec.norm();
  dirvec=dirvec/value;
  double l=dirvec(0);
  double m=dirvec(1);
  newpt(0)=intv*l+pt1(0);
  newpt(1)=intv*m+pt1(1);
  newpt(2)=intv*dirvec(2)+pt1(2);  
}



void inv_rep(curvesVector &Samplepoints,int fuse_flag)
{
  
 // unsigned int l=Samplepoints.size();
  //Curves* s_point;
  VectorXd im(DIMENSIONS+1);
  
  for(int k = 0; k < DIMENSIONS; k++)
  im(k)=0;
  im(DIMENSIONS)=1.0; 
  

  Curves* s_point = Samplepoints[0];

  if(fuse_flag==1){
  s_point->Fuspoints=im;
  for(int j=0;j<3;j++)
  s_point->Fus_points(j)=s_point->Fuspoints(j);

  }
  else {
  s_point->cr_points1=im;
  s_point->cr_points2=im;
  }
  for(size_t i = 1; i < Samplepoints.size(); ++i){
   
  //这里matlab用的l是rep的个数，所以闭曲线和开曲线是有差别的，但是我用的是点的个数，所以可以跳过判断曲线开闭的步骤。
  Curves* c_point = Samplepoints[i];
  Curves* p_point = Samplepoints[i-1];
  
    if(fuse_flag==1){

    
     c_point->Fuspoints=(p_point->Fus_transfor)*(p_point->Fuspoints);
     for(int j=0;j<3;j++)
     c_point->Fus_points(j)=c_point->Fuspoints(j);
    }
   
    else {

     c_point->cr_points1=(p_point->transformation1)*(p_point->cr_points1);
     c_point->cr_points2=(p_point->transformation2)*(p_point->cr_points2);
     cout<<55555<<endl;
     for(int j=0;j<3;j++){

     c_point->points1(j)=c_point->cr_points1(j);
     c_point->points2(j)=c_point->cr_points2(j);
     }

    }

  }

}

void QuatToMatrix3(const double *quat,
                               MatrixXd &mat)
 {
    //  double q00 = quat[0]*quat[0];
   double q11 = quat[1]*quat[1];
   double q22 = quat[2]*quat[2];
   double q33 = quat[3]*quat[3];
   double q03 = quat[0]*quat[3];
   double q13 = quat[1]*quat[3];
   double q23 = quat[2]*quat[3];
   double q02 = quat[0]*quat[2];
   double q12 = quat[1]*quat[2];
   double q01 = quat[0]*quat[1];
   mat(0,0) = 1 - 2 * (q22 + q33);
   mat(1,1) = 1 - 2 * (q11 + q33);
   mat(2,2) = 1 - 2 * (q11 + q22);

   mat(0,1) = 2.0*(q12-q03);
   mat(0,2) = 2.0*(q13+q02);

   mat(1,0) = 2.0*(q12+q03);
   mat(1,2) = 2.0*(q23-q01);
 
   mat(2,0) = 2.0*(q13-q02);
   mat(2,1) = 2.0*(q23+q01);
  
 }

void Matrix3ToQuat( MatrixXd &mat,
                                 double *quat)
{
  
  double T, S, X, Y, Z, W;
  T = 1 + mat(0,0) + mat(1,1) + mat(2,2);
  if ( T > 0.00000001 ) { // to avoid large distortions!                                 //There are some differences in T<0; But the data I am processing did not appear to be less than 0 
    S = sqrt(T) * 2;
    X = ( mat(2,1) - mat(1,2) ) / S;
    Y = ( mat(0,2) - mat(2,0) ) / S;
    Z = ( mat(1,0) - mat(0,1) ) / S;
    W = 0.25 * S;
  } else if ( mat(0,0) > mat(1,1) && mat(0,0) > mat(2,2) )  { // Column 0: 
    S  = sqrt( 1.0 + mat(0,0) - mat(1,1) - mat(2,2) ) * 2;
    X = 0.25 * S;
    Y = (mat(1,0) + mat(0,1) ) / S;                   //in 3DTk,Y and Z values are swapped
    Z = (mat(0,2) + mat(2,0) ) / S;
    W = (mat(2,1) - mat(1,2) ) / S;
    cout<<"出现2"<<endl;
  } else if ( mat(1,1) > mat(2,2) ) {                    // Column 1: 
    S  = sqrt( 1.0 + mat(1,1) - mat(0,0) - mat(2,2) ) * 2;
    X = (mat(0,1) + mat(1,0) ) / S;
    Y = 0.25 * S;
    Z = (mat(2,1) + mat(1,2) ) / S;
    W = (mat(0,2) - mat(2,0) ) / S;
    cout<<"出现3"<<endl;
  } else {                                            // Column 2:
    S  = sqrt( 1.0 + mat(2,2) - mat(0,0) - mat(1,1) ) * 2;
    X = (mat(0,2) + mat(2,0) ) / S;
    Y = (mat(1,2) + mat(2,1) ) / S;
    Z = 0.25 * S;
    W = (mat(1,0) - mat(0,1) ) / S;
    cout<<"出现4"<<endl;
  }
  quat[0] = W;
  quat[1] = X;
  quat[2] = Y;
  quat[3] = Z;
  
  Normalize4(quat);
  
}

double deg(const double rad)
{
  return ( (rad * 360.0) / (2.0 * M_PI) ); 
}
 void Matrix4ToEuler( double *alignxf,
                                  double *rPosTheta,
                                  double *rPos)
{
  
  double _trX, _trY;

  // Calculate Y-axis angle 
  if(alignxf[0] > 0.0) {
    rPosTheta[1] = asin(alignxf[8]);
  } else {
    rPosTheta[1] = M_PI  - asin(alignxf[8]);
  }

  double  C    =  cos( rPosTheta[1] );
  if ( fabs( C ) > 0.005 )  {                 // Gimbal lock? 
    _trX      =  alignxf[10] / C;             // No, so get X-axis angle 
    _trY      =  -alignxf[9] / C;
    rPosTheta[0]  = atan2( _trY, _trX );
    _trX      =  alignxf[0] / C;              // Get Z-axis angle 
    _trY      = -alignxf[4] / C;
    rPosTheta[2]  = atan2( _trY, _trX );
  } else {                                    // Gimbal lock has occurred 
    rPosTheta[0] = 0.0;                       // Set X-axis angle to zero 
    _trX      =  alignxf[5];  //1                // And calculate Z-axis angle 
    _trY      =  alignxf[1];  //2
    rPosTheta[2]  = atan2( _trY, _trX );
  }
  
  rPosTheta[0] = deg(rPosTheta[0]);
  rPosTheta[1] = deg(rPosTheta[1]);
  rPosTheta[2] = deg(rPosTheta[2]);
 if (rPos != 0) {
    rPos[0] = alignxf[12];
    rPos[1] = alignxf[13];
    rPos[2] = alignxf[14];
      
  }
}

void save_final_result(vector<MatrixXd> poses_result_final,vector<VectorXd> point_result_final,string &pose_dir_final,string &point_dir_final,int count,int steps) {

     string PoseFileName =pose_dir_final +to_string(count,2)+ ".txt";
     string PointFileName = point_dir_final +to_string(count,2)+ ".txt";
     
     // open file  
     FILE *fp1 = fopen(PoseFileName.c_str(),"w");
     FILE *fp2 = fopen(PointFileName.c_str(),"w");
 
          
     

        //cout<<fuse_curve.size()<<endl;
     for(unsigned int i = 0; i <= poses_result_final.size()-1; i++)
      {
     
  
       cout<<i<<endl;
       MatrixXd posematrix(3,3);
       posematrix=poses_result_final[i];
       VectorXd point=point_result_final[i];
        
   

       fprintf(fp1,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
                                posematrix(0,0),posematrix(0,1),posematrix(0,2),point(0),
                                posematrix(1,0),posematrix(1,1),posematrix(1,2),point(1),
                                posematrix(2,0),posematrix(2,1),posematrix(2,2),point(2),
                                0.0,0.0,0.0,1.0);
       fprintf(fp2,"%lf %lf %lf\n",point(0),point(1),point(2));



      }

     

}

void save_fusion_pose(vector<MatrixXd> poses_result_fusion,string dir,int count) {

     
    string  pose_dir_fusion        = dir + "fuse_result_final/";
   
   // create output directories
     system(("mkdir " +  pose_dir_fusion).c_str());

     string PoseFileName =pose_dir_fusion +to_string(count,2)+ ".txt";
     //string PointFileName = point_dir_final +to_string(count,2)+ ".txt";
     
     // open file  
     FILE *fp1 = fopen(PoseFileName.c_str(),"w");
    // FILE *fp2 = fopen(PointFileName.c_str(),"w");
 
          
     

        //cout<<fuse_curve.size()<<endl;
     for(unsigned int i = 0; i <= poses_result_fusion.size()-1; i++)
      {
     
  
       MatrixXd posematrix(3,3);
       posematrix=poses_result_fusion[i];
        
   

       fprintf(fp1,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
                                posematrix(0,0),posematrix(0,1),posematrix(0,2),posematrix(0,3),
                                posematrix(1,0),posematrix(1,1),posematrix(1,2),posematrix(1,3),
                                posematrix(2,0),posematrix(2,1),posematrix(2,2),posematrix(2,3));
      }

     

}


void ros2frames(vector<MatrixXd> poses_result_fusion,string dir,int count) {

     
    string  frames_dir_fusion        = dir + "fuse_frames/"+to_string(count,2)+"/";
   
   // create output directories
     system(("mkdir " +  frames_dir_fusion).c_str());
    double tMatrix[16];
    double rPos[3], rPosTheta[16];
    char poseFileName[255];
    char frameFileName[255];

    ofstream pose_out;
    ofstream frames_out;

    for (unsigned int i = 0; i <= poses_result_fusion.size()-1; i++) {
   
       MatrixXd fusionmatrix(3,3);
       fusionmatrix=poses_result_fusion[i];
       snprintf(poseFileName,255,"%sscan%.3d.pose",frames_dir_fusion.c_str(),i);
       snprintf(frameFileName,255,"%sscan%.3d.frames",frames_dir_fusion.c_str(),i);
      //// cout << "Reading fusion pose " << frames_dir_fusion << "..." << endl;
   
  
       tMatrix[ 0] = fusionmatrix(1,1);
       tMatrix[ 1] =-fusionmatrix(2,1);
       tMatrix[ 2] = -fusionmatrix(0,1);
       tMatrix[ 3] = -0;
       tMatrix[ 4] = -fusionmatrix(1,2);
       tMatrix[ 5] = fusionmatrix(2,2);
       tMatrix[ 6] = fusionmatrix(0,2);
       tMatrix[ 7] = 0;
       tMatrix[8] = -fusionmatrix(1,0);
       tMatrix[ 9] = fusionmatrix(2,0);
       tMatrix[10] = fusionmatrix(0,0);
       tMatrix[11] = 0;
       tMatrix[12] = -fusionmatrix(1,3)*100;
       tMatrix[13] = fusionmatrix(2,3)*100;
       tMatrix[14] = fusionmatrix(0,3)*100;
       tMatrix[15] = 1.0;

       Matrix4ToEuler(tMatrix, rPosTheta, rPos);
    
       pose_out.open(poseFileName);
       frames_out.open(frameFileName);
     ////  cout << "Writing pose file... " << poseFileName << endl;
    
       for(int i = 0; i < 3; i++) {
        pose_out << rPos[i] << " ";
       }
        pose_out << endl; 

       for(int i = 0; i < 3; i++) {
        pose_out <<rPosTheta[i]<< " ";
        }

     ////  cout << "Writing frames... " << poseFileName << endl;
  
       for(int j = 0; j < 2; j++) {
        for (int i=0; i < 16; i++) {
         frames_out << tMatrix[i] << " ";
       }
       frames_out << "2" << endl;
       }
    
      pose_out.close();
      pose_out.clear();

      frames_out.close();
      frames_out.clear();

    
     //// cout << " done." << endl;
  }

}

void matrix_tf_final(vector<MatrixXd> poses_result_final,vector<VectorXd> point_result_final,int count,int steps,std::vector<double> *h_matrix,vector<MatrixXd> poses_result_final_h, string dir) {

         // tf_broadcaster broadcaster;

         Matrix3d R_current,R_increment,R_h;
         Matrix4d T;
         Matrix3d  base_point,tangent_rotation;
         vector<MatrixXd> final_trans_ma;

         vector<MatrixXd> poses_result_hector;
         vector<VectorXd> point_result_hector;

        for(unsigned int l=0; l<= h_matrix[0].size()-1; l++) {
      
          double qm[4];
          MatrixXd mat_h(3,3);
          Vector3d t_h;
         
          qm[0] = h_matrix[4][l];
          qm[1] = h_matrix[1][l];
          qm[2] = h_matrix[2][l];
          qm[3] = h_matrix[3][l];
          
          t_h[0]= h_matrix[5][l];
          t_h[1]= h_matrix[6][l];
          t_h[2]= h_matrix[7][l];

          QuatToMatrix3(qm,mat_h);
          
          
          poses_result_hector.push_back(mat_h);
          point_result_hector.push_back(t_h);
          //cout<<l<<endl;

        }

     if(count==steps) {

     
         //Matrix4d T; 
         // unsigned int s_num=Samplepoints.size();
        for(unsigned int i = 0; i <= poses_result_hector.size()-1; i++)
          {
     
            
        
           for(int h=0;h<3;h++) {
             for(int l=0;l<3;l++) {
            
              T(h,l)=poses_result_hector[i](h,l);
             }
            }

            
            for(int j = 0; j < 3; j++){
             T(j,3)= point_result_hector[i](j);
             T(3,j)=0;
            }
            
             T(3,3)=1.0;
             matrix_new.push_back(T);
             final_trans_ma.push_back(T);
            time_s[i]=h_matrix[0][i];
           // cout<<i<<endl;
          }
          
          save_fusion_pose(final_trans_ma, dir,count);
          ros2frames(final_trans_ma,dir,count);
      }

   else {
        //poses_result_final.size()
   
        for(unsigned int i = 0; i <= poses_result_final.size()-1; i++) {

           if(i==0) {
            
            R_increment(0,0)=1;
            R_increment(0,1)=0;
            R_increment(0,2)=0;
            R_increment(1,0)=0;
            R_increment(1,1)=1;
            R_increment(1,2)=0;
            R_increment(2,0)=0;
            R_increment(2,1)=0;
            R_increment(2,2)=1; 
            R_h=R_increment;
           }

           else {
           
           for(int h=0;h<3;h++) {
             for(int l=0;l<3;l++) {
              R_increment(h,l)=poses_result_final[i-1](h,l);
              R_h(h,l)=poses_result_final_h[i-1](h,l);
             }
            }
           }
           // for(unsigned int c=0; c<= h_matrix[0].size()-1; c++) {

          //  if(time_s[i]==(h_matrix[0][c])) {
            
            //// base_point = (R_increment*(R_h.transpose())); 
            //// tangent_rotation=base_point*(((R_h.transpose())*poses_result_hector[i]).log());
            //// R_current = R_increment*(tangent_rotation.exp());

             //R_current=R_increment*poses_result_hector[i];
           // if(i==0)
              /// R_current=(R_increment*R_h.inverse())*poses_result_hector[i];
            R_current=poses_result_hector[i];
            for(int h=0;h<3;h++) {
            for(int l=0;l<3;l++) { 
              T(h,l)=R_current(h,l);
            }
           }
 
           for(int j = 0; j < 3; j++){
            T(j,3)=point_result_final[i](j);
            T(3,j)=0;
           }
            
            T(3,3)=1.0;
            matrix_new.push_back(T);
            final_trans_ma.push_back(T);
            time_s[i]=h_matrix[0][i];
            //cout<<i<<endl;
          
       } 

       save_fusion_pose(final_trans_ma, dir,count);
       ros2frames(final_trans_ma,dir,count);
    }
}


void interpolation(curvesVector &Samplepoints,string dir,string &pose_dir_final,string &point_dir_final,int count,int steps,int pose_index,std::vector<double> *h_matrix,unsigned int *h_seq) {

     VectorXd t_i;
     unsigned int t_num=0;
     double quat_c[4],quat_n[4],quat_i[4],quat_h_c[4],quat_h_n[4],quat_h_i[4];
     vector<MatrixXd> poses_result_final;
     vector<VectorXd> point_result_final;
     vector<MatrixXd> poses_result_final_h;
   if(count==0) {
      
       MatrixXd posematrix_c(3,3);
       MatrixXd posematrix_n(3,3);
       MatrixXd posematrix_i(3,3);
       MatrixXd posematrix_h_c(3,3);
       MatrixXd posematrix_h_n(3,3);
       MatrixXd posematrix_h_i(3,3);
     
      for(unsigned int i = 0; i < Samplepoints.size()-1; i++) {


       Curves* c_point = Samplepoints[i];
       Curves* n_point = Samplepoints[i+1];

     
       for(int h=0;h<DIMENSIONS;h++) {
        for(int g=0;g<DIMENSIONS;g++) {
          posematrix_c(h,g)=(c_point->rot1(h,g));
          posematrix_n(h,g)=(n_point->rot1(h,g));
          posematrix_h_c(h,g)=(c_point->rot2(h,g));
          posematrix_h_n(h,g)=(n_point->rot2(h,g));
        }
       }
       
     VectorXd t_c=c_point->points1;  
     VectorXd t_n=n_point->points1;    
#ifndef POINT3D 
       for(int m=0;m<2;m++) {
         posematrix_c(2,m)=0.0;
         posematrix_n(2,m)=0.0;
         posematrix_h_c(2,m)=0.0;
         posematrix_h_n(2,m)=0.0;

         posematrix_c(m,2)=0.0;
         posematrix_n(m,2)=0.0;
         posematrix_h_c(m,2)=0.0;
         posematrix_h_n(m,2)=0.0;
       }
       
       posematrix_c(2,2)=1.0;
       posematrix_n(2,2)=1.0;
       posematrix_h_c(2,2)=1.0;
       posematrix_h_n(2,2)=1.0;
       
       t_c(2)=0.0;
       t_n(2)=0.0;
#endif
        
      //VectorXd t_c=c_point->points1;
       Matrix3ToQuat(posematrix_c,quat_c);
       Matrix3ToQuat(posematrix_n,quat_n);
       Matrix3ToQuat(posematrix_h_c,quat_h_c);
       Matrix3ToQuat(posematrix_h_n,quat_h_n);

       t_num=h_seq[i]; 
      
       point_result_final.push_back(t_c);
       poses_result_final.push_back(posematrix_c);
       poses_result_final_h.push_back(posematrix_h_c);
      
     for(unsigned int h=1;h<t_num;h++) {

       double s=double(h);
       double n=double(t_num);
       t_i=t_c+(s/n)*(t_n-t_c);
       
       slerp(quat_c, quat_n, s/n, quat_i);
       QuatToMatrix3(quat_i,posematrix_i);
     
       slerp(quat_h_c, quat_h_n, s/n, quat_h_i);
       QuatToMatrix3(quat_h_i,posematrix_h_i);

       point_result_final.push_back(t_i); 
       poses_result_final.push_back(posematrix_i);
       poses_result_final_h.push_back(posematrix_h_i);
     }
       
      if(i==Samplepoints.size()-2) {

       point_result_final.push_back(t_n); 
       poses_result_final.push_back(posematrix_n); 
       poses_result_final_h.push_back(posematrix_h_n);
      } 
       
        
       
      // poses_result_final_h.push_back(posematrix_h);
      // timestamp.push_back(c_point->time_stamps);
   }
     
     //// save_final_result(poses_result_final,point_result_final,pose_dir_final,point_dir_final,count,steps);

      /////if(pose_index==count)
      matrix_tf_final(poses_result_final,point_result_final,count,steps,h_matrix,poses_result_final_h,dir);
   }

      

      else {

          MatrixXd posematrix_c(3,3);
          MatrixXd posematrix_n(3,3);
          MatrixXd posematrix_i(3,3);
          
          MatrixXd posematrix_h_c(3,3);
          MatrixXd posematrix_h_n(3,3);
          MatrixXd posematrix_h_i(3,3);

        for(unsigned int i = 0; i < Samplepoints.size()-1; i++) {

          Curves* c_point = Samplepoints[i];
          Curves* n_point = Samplepoints[i+1];

        
        for(int h=0;h<DIMENSIONS;h++) {
         for(int g=0;g<DIMENSIONS;g++) {
          posematrix_c(h,g)=(c_point->Fus_transfor(h,g));
          posematrix_n(h,g)=(n_point->Fus_transfor(h,g));
          posematrix_h_c(h,g)=(c_point->rot2(h,g));
          posematrix_h_n(h,g)=(n_point->rot2(h,g));
         }
        }
       
        VectorXd t_c=c_point->Fus_points;
        VectorXd t_n=n_point->Fus_points; 
#ifndef POINT3D 
        for(int m=0;m<2;m++) {

         posematrix_c(2,m)=0.0;
         posematrix_n(2,m)=0.0;

         posematrix_h_c(2,m)=0.0;
         posematrix_h_n(2,m)=0.0;

         posematrix_c(m,2)=0.0;
         posematrix_n(m,2)=0.0;

         posematrix_h_c(m,2)=0.0;
         posematrix_h_n(m,2)=0.0;

         
        }
       
         posematrix_c(2,2)=1.0;
         posematrix_n(2,2)=1.0;
         posematrix_h_c(2,2)=1.0;
         posematrix_h_n(2,2)=1.0;

         t_c(2)=0.0;
         t_n(2)=0.0;
#endif
        
        
       Matrix3ToQuat(posematrix_c,quat_c);
       Matrix3ToQuat(posematrix_n,quat_n);

       Matrix3ToQuat(posematrix_h_c,quat_h_c);
       Matrix3ToQuat(posematrix_h_n,quat_h_n);

       t_num=h_seq[i]; 

       point_result_final.push_back(t_c);
       poses_result_final.push_back(posematrix_c);
       poses_result_final_h.push_back(posematrix_h_c);

     for(unsigned int h=1;h<t_num;h++) {

       double s=double(h);
       double n=double(t_num);
       t_i=t_c+(s/n)*(t_n-t_c);

       slerp(quat_c, quat_n, s/n, quat_i);
       QuatToMatrix3(quat_i,posematrix_i);

       slerp(quat_h_c, quat_h_n, s/n, quat_h_i);
       QuatToMatrix3(quat_h_i,posematrix_h_i);

       point_result_final.push_back(t_i); 
       poses_result_final.push_back(posematrix_i);
       poses_result_final_h.push_back(posematrix_h_i);

     }
       
      if(i==Samplepoints.size()-2) {

       point_result_final.push_back(t_n); 
       poses_result_final.push_back(posematrix_n); 
       poses_result_final_h.push_back(posematrix_h_n);

      } 
       
       
       
       //poses_result_final_h.push_back(posematrix_h);
      // timestamp.push_back(c_point->time_stamps);
    }
     ////  save_final_result(poses_result_final,point_result_final,pose_dir_final,point_dir_final,count,steps);
       /////if(pose_index==count)
       matrix_tf_final(poses_result_final,point_result_final,count,steps,h_matrix,poses_result_final_h,dir);
     
   }
}


void savedata(curvesVector &Samplepoints,string dir,string &pose_dir,string &point_dir,int count,int steps)
{
   
  
   
     string PoseFileName = pose_dir +to_string(count,2)+ ".txt";
     string PointFileName = point_dir +to_string(count,2)+ ".txt";
     
     // open file  
     FILE *fp1 = fopen(PoseFileName.c_str(),"w");
     FILE *fp2 = fopen(PointFileName.c_str(),"w");

     if(count==0) {

         
          
         //slerp(idQ, deltaQ, weights[3][0], rPosQuat);  

        //cout<<fuse_curve.size()<<endl;
       for(unsigned int i = 0; i < Samplepoints.size(); i++)
       {
     
         Curves* c_point = Samplepoints[i];

         MatrixXd posematrix(DIMENSIONS+1,DIMENSIONS+1);
      
         posematrix=(c_point->transformation1);
       //  if(i==Samplepoints.size()-1)
       //  posematrix=(c_point-1)->transformation1;
         VectorXd point=c_point->points1;
        
        // parameters
#ifdef POINT3D  
  

         fprintf(fp1,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
                                posematrix(0,0),posematrix(0,1),posematrix(0,2),posematrix(0,3),
                                posematrix(1,0),posematrix(1,1),posematrix(1,2),posematrix(1,3),
                                posematrix(2,0),posematrix(2,1),posematrix(2,2),posematrix(2,3),
                                posematrix(3,0),posematrix(3,1),posematrix(3,2),posematrix(3,3));
         fprintf(fp2,"%lf %lf %lf\n",point(0),point(1),point(2));

#else
         fprintf(fp1,"%lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
                                posematrix(0,0),posematrix(0,1),posematrix(0,2),
                                posematrix(1,0),posematrix(1,1),posematrix(1,2),
                                posematrix(2,0),posematrix(2,1),posematrix(2,2));
         fprintf(fp2,"%lf %lf\n",point(0),point(1));
#endif


       }

      

      }

    else if(count==steps){
     
       
         for(unsigned int i = 0; i < Samplepoints.size(); i++)
       {
     
         Curves* c_point = Samplepoints[i];
         MatrixXd posematrix=c_point->transformation2;
        // if(i==Samplepoints.size()-1)
      //   posematrix=(c_point-1)->transformation2;
         VectorXd point=c_point-> points2;
        //MatrixXd P=MatrixXd::Identity(3,3);
        // parameters
#ifdef POINT3D
         fprintf(fp1,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
                                 posematrix(0,0),posematrix(0,1),posematrix(0,2),posematrix(0,3),
                                 posematrix(1,0),posematrix(1,1),posematrix(1,2),posematrix(1,3),
                                 posematrix(2,0),posematrix(2,1),posematrix(2,2),posematrix(2,3),
                                 posematrix(3,0),posematrix(3,1),posematrix(3,2),posematrix(3,3));
         fprintf(fp2,"%lf %lf %lf\n",point(0),point(1),point(2));
#else
        fprintf(fp1,"%lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
                                 posematrix(0,0),posematrix(0,1),posematrix(0,2),
                                 posematrix(1,0),posematrix(1,1),posematrix(1,2),
                                 posematrix(2,0),posematrix(2,1),posematrix(2,2));
         fprintf(fp2,"%lf %lf\n",point(0),point(1));
#endif
       }
          

     } 

     else {
        for(unsigned int i = 0; i < Samplepoints.size(); i++)
        {
     
         Curves* c_point = Samplepoints[i];
         
         MatrixXd posematrix=c_point->Fus_transfor;
        // if(i==Samplepoints.size()-1)
        // posematrix=(c_point-1)->Fus_transfor;
         VectorXd point=c_point->Fus_points;
         //MatrixXd P=MatrixXd::Identity(3,3);
         // parameters
#ifdef POINT3D
         fprintf(fp1,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
                                posematrix(0,0),posematrix(0,1),posematrix(0,2),posematrix(0,3),
                                posematrix(1,0),posematrix(1,1),posematrix(1,2),posematrix(1,3),
                                posematrix(2,0),posematrix(2,1),posematrix(2,2),posematrix(2,3),
                                posematrix(3,0),posematrix(3,1),posematrix(3,2),posematrix(3,3));
                                fprintf(fp2,"%lf %lf %lf\n",point(0),point(1),point(2));
#else
        fprintf(fp1,"%lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
                                posematrix(0,0),posematrix(0,1),posematrix(0,2),
                                posematrix(1,0),posematrix(1,1),posematrix(1,2),
                                posematrix(2,0),posematrix(2,1),posematrix(2,2));
         fprintf(fp2,"%lf %lf\n",point(0),point(1));
#endif
        }
     }
    // close file
     fclose(fp1);
     fclose(fp2);
 // }
   

}


void geodesic_path(curvesVector &Samplepoints,int steps,string dir,int pose_index,std::vector<double> *h_matrix,unsigned int *h_seq)
{
   double timestep=1/((double)steps);
   double step=0;
   unsigned int num_sample=Curves::Samplepoints.size();
   //Matrix2d R_c;
   
   MatrixXcd Rc(DIMENSIONS, DIMENSIONS);
   VectorXd trans(DIMENSIONS);
   MatrixXd R1(DIMENSIONS, DIMENSIONS);
   MatrixXd R2(DIMENSIONS, DIMENSIONS);
   VectorXd tra1(DIMENSIONS);
   VectorXd tra2(DIMENSIONS); 
   MatrixXd Ri(DIMENSIONS, DIMENSIONS);
   
   MatrixXcd R5(DIMENSIONS, DIMENSIONS);
   MatrixXd R(DIMENSIONS, DIMENSIONS);

   pose_dir        = dir + "fuse_pose/";
   pose_dir_final  = dir+ "fuse_pose_final/";

   point_dir       = dir + "new_point/";
   point_dir_final = dir + "new_point_final/";

   // create output directories
   system(("mkdir " + pose_dir).c_str());
   system(("mkdir " + pose_dir_final).c_str()); 

   system(("mkdir " + point_dir).c_str());
   system(("mkdir " + point_dir_final).c_str());
   //int i;

  
  for(unsigned int j = 0; j < num_sample; j++){

   Curves *curves;
   curves=Curves::Samplepoints[j];
   Curves::Oripoints.push_back(curves);
  }
   
  for(int i=0;i<=steps;i++){

      if(i==0) {
        savedata(Curves::Oripoints,dir,pose_dir,point_dir,i,steps);

        interpolation(Curves::Oripoints,dir,pose_dir_final,point_dir_final,i,steps,pose_index,h_matrix,h_seq);
        
      }

      else if(i==steps) {
      
        savedata(Curves::Oripoints,dir,pose_dir,point_dir,i,steps);
      //interpolation(Curves::Oripoints,dir,pose_dir_final,point_dir_final,i,steps);
     // save_final_result(poses_result_final,point_result_final,pose_dir_final,point_dir_final,count,steps);
        vector<MatrixXd> poses_result_final;
        vector<VectorXd> point_result_final;
        vector<MatrixXd> poses_result_final_h;
 
     //  for(int b=0;b<Curves::Samplepoints.size();b++) {      //这个循环和 if(time_s[b]==(h_matrix[0][l])) 循环可以决定保存的curve2数据是原始的还是被优化采样以后的

        for(unsigned int l=0; l<= h_matrix[0].size()-1; l++) {


         // if(time_s[b]==(h_matrix[0][l])) {
            double qm[4];
            MatrixXd mat_h(3,3);
            Vector3d t_h;
            qm[0] = h_matrix[4][l];
            qm[1] = h_matrix[1][l];
            qm[2] = h_matrix[2][l];
            qm[3] = h_matrix[3][l];
          
            t_h[0]= h_matrix[5][l];
            t_h[1]= h_matrix[6][l];
            t_h[2]= h_matrix[7][l];

            QuatToMatrix3(qm,mat_h);
          
            poses_result_final.push_back(mat_h);
            point_result_final.push_back(t_h);
            poses_result_final_h.push_back(mat_h);
           // timestamp.push_back(h_matrix[0][l]);

         // }
        //}
      }
         // save_final_result(poses_result_final,point_result_final,pose_dir_final,point_dir_final,i,steps);

          ////if(pose_index==steps)
          matrix_tf_final(poses_result_final,point_result_final,i,steps,h_matrix,poses_result_final_h,dir);
      }

      else{
   
      step = step + timestep;
      for(curvesVector::iterator it = Curves::Samplepoints.begin();
      it != Curves::Samplepoints.end();
      ++it) {
       
       Curves *C_Point = *it;
       R1=C_Point->rot1;
       R2=C_Point->rot2;
       tra1=C_Point->trans1;
       tra2=C_Point->trans2;
       
       //这里matlab中用的R1的转置，但是论文中写的是逆。
      Ri=(R1.transpose())*R2;
   
      Rc.real()= Ri;
      
#ifdef POINT3D
      Rc.imag()<<0,0,0,0,0,0,0,0,0; 
        
      MatrixPower<Matrix3cd> Apow(Rc); 
#else
      Rc.imag()<<0,0,0,0; 
      MatrixPower<Matrix2cd> Apow(Rc);
#endif  
       
      R= (R1*Apow(step)).real();
     
      trans=tra1+(tra2-tra1)*step;  
      
      for(int j = 0; j < DIMENSIONS; j++){
       for(int k = 0; k < DIMENSIONS; k++){
        C_Point->Fus_transfor(j, k) =R(j, k);     
        }
       }

       for(int j = 0; j < DIMENSIONS; j++){
        C_Point->Fus_transfor(j, DIMENSIONS)=trans(j);
       }

       for(int k = 0; k < DIMENSIONS; k++)
        C_Point->Fus_transfor(DIMENSIONS, k)=0; 

        C_Point->Fus_transfor(DIMENSIONS, DIMENSIONS)=1.0;
              
     }
         
        inv_rep(Curves::Samplepoints,1);
        
        savedata(Samplepoints,dir,pose_dir,point_dir,i,steps);
        
        interpolation(Samplepoints,dir,pose_dir_final,point_dir_final,i,steps,pose_index,h_matrix,h_seq);
   }   
    
  }

   
}

vector<int32_t> computeRoi1 (vector<MatrixXd> &poses_result){
  
  float x_min = numeric_limits<int32_t>::max();
  float x_max = numeric_limits<int32_t>::min();
  float y_min = numeric_limits<int32_t>::max();
  float y_max = numeric_limits<int32_t>::min();
 
  for(unsigned int i = 0; i < poses_result.size(); i++){

#ifdef POINT3D  
   Matrix4d P=poses_result[i];
   double x = P(0,3);
   double y = P(1,3);// 
#else
   Matrix3d P=poses_result[i];
   double x = P(0,2);
   double y = P(1,2);
#endif
   if (x<x_min) x_min = x; if (x>x_max) x_max = x;
   if (y<y_min) y_min = y; if (y>y_max) y_max = y;
  }

  float dx = 1.1*(x_max-x_min);
  float dy = 1.1*(y_max-y_min);
  float mx = 0.5*(x_max+x_min);
  float my = 0.5*(y_max+y_min);
  float r  = 0.5*max(dx,dy);
  
  vector<int32_t> roi;
  roi.push_back((int32_t)(mx-r));
  roi.push_back((int32_t)(mx+r));
  roi.push_back((int32_t)(my-r));
  roi.push_back((int32_t)(my+r));
  return roi;
}

vector<int32_t> computeRoi2 (vector<VectorXd> &point_result){
  
  float x_min = numeric_limits<int32_t>::max();
  float x_max = numeric_limits<int32_t>::min();
  float y_min = numeric_limits<int32_t>::max();
  float y_max = numeric_limits<int32_t>::min();
  
  for(unsigned int i = 0; i < point_result.size(); i++){

#ifdef POINT3D  
   Vector3d P=point_result[i];
   double x = P(0);
   double y = P(1);
#else
   Vector2d P=point_result[i];
   double x = P(0);
   double y = P(1);
#endif
   if (x<x_min) x_min = x; if (x>x_max) x_max = x;
   if (y<y_min) y_min = y; if (y>y_max) y_max = y;
  }

  float dx = 1.1*(x_max-x_min);
  float dy = 1.1*(y_max-y_min);
  float mx = 0.5*(x_max+x_min);
  float my = 0.5*(y_max+y_min);
  float r  = 0.5*max(dx,dy);
  
  vector<int32_t> roi;
  roi.push_back((int32_t)(mx-r));
  roi.push_back((int32_t)(mx+r));
  roi.push_back((int32_t)(my-r));
  roi.push_back((int32_t)(my+r));
  return roi;
}



void savePathPlot (vector<MatrixXd> &poses_result,vector<VectorXd> &point_result,string file1_name,string file2_name){

  // parameters
  int32_t step_size = 3;

  // open file  
  FILE *fp1 = fopen(file1_name.c_str(),"w");
  // save x/y coordinates of all frames to file
  for (unsigned int i=0; i<poses_result.size(); i+=step_size)

#ifdef POINT3D
  fprintf(fp1,"%f %f\n",(poses_result[i])(0,3),(poses_result[i])(1,3));
#else
  fprintf(fp1,"%f %f\n",(poses_result[i])(0,2),(poses_result[i])(1,2));
#endif
  fclose(fp1);

  FILE *fp2 = fopen(file2_name.c_str(),"w");
  for (unsigned int j=0; j<point_result.size(); j++)

#ifndef POINT3D
  fprintf(fp2,"%f %f\n",(point_result[j])(0),(point_result[j])(1));
  // close file
#else
  fprintf(fp2,"%f %f\n",(point_result[j])(0),(point_result[j])(1));
#endif  
  fclose(fp2);
}

void plotPathPlot(string dir,vector<int32_t> &roi,int idx,int steps)
{
        // gnuplot file name
  char command[1024];
  char file_name[256];
  sprintf(file_name,"%02d.gp",idx);
  string full_name = dir + "/" + file_name;
  
  // create png + eps
  for (int i=0; i<=steps; i++){

    // open file  
    FILE *fp = fopen(full_name.c_str(),"w");

    // save gnuplot instructions
    if (i==0) {
      fprintf(fp,"set term png size 900,900\n");
      fprintf(fp,"set output \"%02d.png\"\n",idx);
    } else {
      fprintf(fp,"set term postscript eps enhanced color\n");
      fprintf(fp,"set output \"%02d.eps\"\n",idx);
    }

    fprintf(fp,"set size square\n");
    ////fprintf(fp,"set xrange [%d:%d]\n",-30,60);
    ////fprintf(fp,"set yrange [%d:%d]\n",-20,90);
    
    ////fprintf(fp,"set xlabel \"x [m]\" font ',22'\n");
    ////fprintf(fp,"set ylabel \"y [m]\" font ',22'\n");
    ////fprintf(fp,"set xtics font ',22'\n");
    ////fprintf(fp,"set ytics font ',22'\n");
    ////fprintf(fp,"set key  font ',22'\n");
    ////fprintf(fp,"plot \"%02d.txt\" using 1:2 lc rgb \"#FF0000\" title '' w lines,",idx);
    ////fprintf(fp,"\"< head -1 %02d.txt\" using 1:2 lc rgb \"#000000\" pt 4 ps 2 lw 3 title  'Sequence Start' w points\n",idx);
    

    fprintf(fp,"set size ratio -1\n");
    fprintf(fp,"set xrange [%d:%d]\n",roi[0],roi[1]);
    fprintf(fp,"set yrange [%d:%d]\n",roi[2],roi[3]);
    fprintf(fp,"set xlabel \"x [m]\"\n");
    fprintf(fp,"set ylabel \"y [m]\"\n");
    fprintf(fp,"plot \"%02d.txt\" using 1:2 lc rgb \"#FF0000\" title 'trajectory ' w lines,",idx);
    fprintf(fp,"\"%02d.txt\" using 3:4 lc rgb \"#0000FF\" title 'Visual Odometry' w lines,",idx);
    fprintf(fp,"\"< head -1 %02d.txt\" using 1:2 lc rgb \"#000000\" pt 4 ps 1 lw 2 title 'Sequence Start' w points\n",idx);
    
    // close file
    fclose(fp);
    
    // run gnuplot => create png + eps
    sprintf(command,"cd %s; gnuplot %s",dir.c_str(),file_name);
    system(command);
  }
  
  // create pdf and crop
  sprintf(command,"cd %s; ps2pdf %02d.eps %02d_large.pdf",dir.c_str(),idx,idx);
  system(command);
  sprintf(command,"cd %s; pdfcrop %02d_large.pdf %02d.pdf",dir.c_str(),idx,idx);
  system(command);
  sprintf(command,"cd %s; rm %02d_large.pdf",dir.c_str(),idx);
  system(command);

}

vector<MatrixXd> loadPoses(string file_name){
  vector<MatrixXd> poses;
  FILE *fp = fopen(file_name.c_str(),"r");
  if (!fp)
    return poses;
  while (!feof(fp)) {

    

#ifdef POINT3D  
    MatrixXd P = MatrixXd::Identity(4, 4);
    if (fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                   &P(0,0), &P(0,1), &P(0,2), &P(0,3),
                   &P(1,0), &P(1,1), &P(1,2), &P(1,3),
                   &P(2,0), &P(2,1), &P(2,2), &P(2,3),
                   &P(3,0), &P(3,1), &P(3,2), &P(3,3))==16) {
      poses.push_back(P);
    }

#else
    MatrixXd P = MatrixXd::Identity(3, 3);
    if (fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf",
                   &P(0,0), &P(0,1), &P(0,2),
                   &P(1,0), &P(1,1), &P(1,2),
                   &P(2,0), &P(2,1), &P(2,2))==9) {
      poses.push_back(P);
    }

#endif
  }
  fclose(fp);
  return poses;
}

vector<VectorXd> loadPoses1(string file_name) {
  vector<VectorXd> point;
  FILE *fp = fopen(file_name.c_str(),"r");
  if (!fp)
    return point;
#ifdef POINT3D
   Vector3d P;
#else
   Vector2d P;
#endif
  while (!feof(fp)) {
   // MatrixXd P = MatrixXd::Identity(3, 3);
#ifdef POINT3D
    if (fscanf(fp, "%lf %lf %lf",
                  &P(0),&P(1),&P(2))==3) {
      point.push_back(P);
    }

#else
     if (fscanf(fp, "%lf %lf",
                  &P(0),&P(1))==2) {
      point.push_back(P);

  }
#endif
}
  fclose(fp);
  return point;
}

void save_correspodence(std::vector<double> *point_s,string dir) {

     string FileName =dir +"point_correspodence"+ ".txt";
    
     
     // open file  
     FILE *fp1 = fopen(FileName.c_str(),"w");
  
        
     for(unsigned int i = 0; i <= point_s[0].size()-1; i++)
    
       fprintf(fp1,"%lf %lf %lf %lf %lf %lf %lf\n",point_s[0][i],point_s[1][i],point_s[2][i],point_s[3][i],point_s[4][i],point_s[5][i],point_s[6][i]);
     fclose(fp1);
}


void timestamp_correspondence(std::vector<double> *point_curve1,std::vector<double> *point_curve2,std::vector<double> *point_corr,unsigned int *h_seq,string dir) {

// now the timetables are set up and you can look for corresponding stamps.
// since the gps topic seems to have a lower frequency than the hector topic,
// look for them in the hector timetable 
  unsigned int sample_points = point_curve1[0].size(); 
  unsigned int index[sample_points];
  for(unsigned int i=0;i<= sample_points-1;i++) {            
       
    double dt_min = 1000.0;
    double stamp=point_curve1[0][i];            
// for each gps pose look for the closest hecotr pose wrt time

     std::map<double, geometry_msgs::PoseStamped>::iterator hector;
    
   for(unsigned int j=0;j<= point_curve2[0].size()-1;j++){
     
     double dt = fabs(point_curve2[0][j] - stamp);
      if(dt < dt_min) {
       index[i]=j;
       dt_min = dt;           
       }
     }
       
                 
// now pose and pose_min are a corresponding pair
// and you can write the poses to a file
// stamp gps.x gps.y gps.z hector.x ...
   /*****   if(i==0) {
      time_s[i]=(point_curve2[0][0]);
      point_corr[0].push_back(point_curve2[0][0]) ;
     // point_corr[0].push_back(point_curve1[0][0]) ; 
      point_corr[1].push_back(0) ;
      point_corr[2].push_back(0) ;
      point_corr[3].push_back(0) ;
      
      index[i]=0;
      }

      else {********/      
      time_s[i]=(point_curve2[0][index[i]]);
       //time_s[i]=(point_curve1[0][i]);
      point_corr[0].push_back(point_curve2[0][index[i]]) ; 
     // point_corr[0].push_back(point_curve1[0][i]) ;
      point_corr[1].push_back(point_curve2[1][index[i]]) ;
      point_corr[2].push_back(point_curve2[2][index[i]]) ;
      point_corr[3].push_back(point_curve2[3][index[i]]) ;
    //////////////}
  
      point_corr[4].push_back(point_curve1[1][i]) ;
      point_corr[5].push_back(point_curve1[2][i]) ;
      point_corr[6].push_back(point_curve1[3][i]) ;
       //point[0] << endl;          
   }

      save_correspodence(point_corr,dir);

   for(unsigned int j=0;j<sample_points-1;j++) 
     h_seq[j]=index[j+1]-index[j];
     
}


void closest_correspondence(std::vector<double> *point_curve1,std::vector<double> *point_curve2,std::vector<double> *point_corr,unsigned int *h_seq,string dir) {

// now the timetables are set up and you can look for corresponding stamps.
// since the gps topic seems to have a lower frequency than the hector topic,
// look for them in the hector timetable 
  unsigned int sample_points = point_curve1[0].size(); 
  unsigned int num = point_curve2[0].size();
  unsigned int index[sample_points];
  index[0]=0;
  for(unsigned int i=0;i<= sample_points-1;i++) {            
       
    double dt_min = 1000.0;
    double x=point_curve1[1][i]; 
    double y=point_curve1[2][i];
    double z=point_curve1[3][i];
   if(i>0) {
// for each gps pose look for the closest hecotr pose wrt time
    //int start=index[i-1]+1;
    //int end=index[i-1]+2;
    //if(start>num)
   // start=num;
   // if(end>num)
   // end=num;
 int p=2*i;
       int start=MAX(2,p);
       int las = MIN(num,p);
   for(unsigned int j=start;j<= las;j++){
     //int b=0;
     //for(int a=1;a<i;a++){
    // if(j==index[a])
     //b=2;}
     
    // if(b=2) continue;
     

     double xj=point_curve2[1][j];
     double yj=point_curve2[2][j];
     double zj=point_curve2[3][j];
     double dis2=pow((xj-x),2)+pow((yj-y),2);
     double dis=sqrt(dis2);
     
      if(dis < dt_min) {
       index[i]=j;
       dt_min = dis;  
       }
     }
    }
      cout<<index[i]<<endl;           
// now pose and pose_min are a corresponding pair
// and you can write the poses to a file
// stamp gps.x gps.y gps.z hector.x ...
      
      time_s[i]=(point_curve2[0][index[i]]);
       //time_s[i]=(point_curve1[0][i]);
      point_corr[0].push_back(point_curve2[0][index[i]]) ; 
     // point_corr[0].push_back(point_curve1[0][i]) ;
      point_corr[1].push_back(point_curve2[1][index[i]]) ;
      point_corr[2].push_back(point_curve2[2][index[i]]) ;
      point_corr[3].push_back(point_curve2[3][index[i]]) ;
     
  
      point_corr[4].push_back(point_curve1[1][i]) ;
      point_corr[5].push_back(point_curve1[2][i]) ;
      point_corr[6].push_back(point_curve1[3][i]) ;
       //point[0] << endl;          
   }

      save_correspodence(point_corr,dir);

   for(unsigned int j=0;j<sample_points-1;j++) 
     h_seq[j]=index[j+1]-index[j];
     
}


void Curve_length(std::vector<double> *point_curve2,int sliding_rate,double *uni_rate,int type) {

  
   // curve length for open curves
   vector<VectorXd> points_sampled;
   Vector3d point;
   MatrixXd tangent(1,3);
   double sum;
   for(unsigned int i=0;i<=point_curve2[0].size()-1;i=i+sliding_rate) {
     
       point(0)=point_curve2[1][i];
       point(1)=point_curve2[2][i];
       point(2)=point_curve2[3][i];
       points_sampled.push_back(point);        
   } 


   if (type == 0) 
     
     for(unsigned int j=0;j<points_sampled.size()-1;j++) {

       tangent(0,0)=points_sampled[j+1](0)-points_sampled[j](0);
       tangent(0,0)*=tangent(0,0);
       tangent(0,1)=points_sampled[j+1](1)-points_sampled[j](1);
       tangent(0,1)*=tangent(0,1);
       tangent(0,2)=points_sampled[j+1](2)-points_sampled[j](2);
       tangent(0,2)*=tangent(0,2);
       sum=tangent(0,0)+tangent(0,1)+tangent(0,2);
       uni_rate[j]=sqrt(sum); 
     }
     

 
 

   else {

        tangent(0,0)=points_sampled[0](0)-points_sampled[points_sampled.size()-1](0);
        tangent(0,0)*=tangent(0,0);
        tangent(0,1)=points_sampled[0](1)-points_sampled[points_sampled.size()-1](1);
        tangent(0,1)*=tangent(0,1);
        tangent(0,2)=points_sampled[0](2)-points_sampled[points_sampled.size()-1](2);
        tangent(0,2)*=tangent(0,2);  
        sum=tangent(0,0)+tangent(0,1)+tangent(0,2);
        uni_rate[0]=sqrt(sum); 
  
       
     for(unsigned int j=1;j<=points_sampled.size()-1;j++) {
     
       
       tangent(0,0)=points_sampled[j](0)-points_sampled[j-1](0);
       tangent(0,0)*=tangent(0,0);
       tangent(0,1)=points_sampled[j](1)-points_sampled[j-1](1);
       tangent(0,1)*=tangent(0,1);
       tangent(0,2)=points_sampled[j](2)-points_sampled[j-1](2);
       tangent(0,2)*=tangent(0,2);
       sum=tangent(0,0)+tangent(0,1)+tangent(0,2);
       uni_rate[j]=sqrt(sum);
       
     } 

     
   }
      
      

}

void Curve_area(std::vector<double> *point_curve2,int sliding_rate,double *uni_rate,int type ) {
  
  Vector2d point;
  vector<VectorXd> points_sampled;
  vector<double> points_sampled2;
  
  for(unsigned int i=0;i<point_curve2[0].size();i=i+sliding_rate) {

    point(0)=point_curve2[1][i];
    point(1)=point_curve2[2][i];
    //laser_points(2)=hector_matrix[7][i];
    
    points_sampled.push_back(point);
    points_sampled2.push_back(point(1));
  }
    
   if(type==1) {
    point(0)=point_curve2[1][0];
    point(1)=point_curve2[2][0];
    
    points_sampled.push_back(point);
    points_sampled2.push_back(point(1));
   }
    double avg_y[points_sampled.size()];
    //double integral[points_sampled.size()-1];
    double sum=0;
  for(unsigned int j=0;j<points_sampled.size()-1;j++) {
 
    avg_y[j]=0.5*(points_sampled2[j]+points_sampled2[j+1]);
    double diff=points_sampled[j+1](0)-points_sampled[j](0);
    uni_rate[j]=diff*(avg_y[j]);
    sum+=uni_rate[j];    
  }
   // cout<<fabs(sum)<<endl;
    //return fabs(sum);
    
}

double matirx_norm2(MatrixXd rot) {

   int n=rot.rows();
   MatrixXd A(n,n);
   A=(rot.transpose())*rot;
   EigenSolver<MatrixXd> es(A);
   MatrixXd D = es.pseudoEigenvalueMatrix();
   VectorXd Eigen_value(n);

   for(int i=0;i<n;i++)
   Eigen_value(i)=D(i,i);
   
   double max_r=Eigen_value.minCoeff();
   return sqrt(max_r);

}

double geo_dist_SE(MatrixXd mat1,MatrixXd mat2) {

   int n=mat1.rows();
   // weigh each component if necessary
   double dM = 1.0;
   double rM = 1.0;
   int tr = n-1;
   MatrixXd R1(tr,tr);
   MatrixXd R2(tr,tr);
   VectorXd trans1(tr);
   VectorXd trans2(tr);
   MatrixXd rot;
   VectorXd trans(tr);
   double val;
   for(int h=0;h<n-1;h++) {
     for(int l=0;l<n-1;l++) {
       R1(h,l)=mat1(h,l);
       R2(h,l)=mat2(h,l);
     }
   }
   
   for(int h=0;h<tr;h++) {
     
      trans1(h)=mat1(h,tr);
      trans2(h)=mat2(h,tr);
   }
   
   
   rot = rM*((R1.transpose()*R2).log());
   trans  = dM*(trans1-trans2);
   val = sqrt(pow(matirx_norm2(rot),2) + pow(trans.norm(),2));
   return val;
}


void opt_subStructure(int j,std::vector<double> *point_g,std::vector<double> *hector_matrix,int s,double *uni_rate,double alpha,double beta,int F,VectorXd &temp1,VectorXd &temp2,VectorXd &temp3,int uni) {

   int len=hector_matrix[0].size();

   //VectorXd cost1(len);
   //VectorXd cost2(len);
  // VectorXd cost3(len);

   for(int i=0;i<len;i++) {
  
    temp1(i)=inf;
    temp2(i)=inf;
    temp3(i)=inf;
   }
   
   string t="gps";
   for(unsigned int h = 0; h < point_g[0].size(); h++) {

     Vector3d gps_point;
     gps_point(0)=point_g[1][h];
     gps_point(1)=point_g[2][h];
     gps_point(2)=point_g[3][h];
     Curves::gpspoints.push_back(new Curves(gps_point,1,h,t));
   } 
   
   t="laser";
   for(unsigned int g = 0; g < hector_matrix[0].size(); g++) {

     Vector3d laser_point;
     laser_point(0)=hector_matrix[5][g];
     laser_point(1)=hector_matrix[6][g];
     laser_point(2)=hector_matrix[7][g];
     
     Curves::laserpoints.push_back(new Curves(laser_point,1,g,t));
   }


   curve_rep(Curves::gpspoints,1); 
  
   MatrixXd fix =Curves::gpspoints[F]->transformation1;
 
   for(int k=s-1;k<=(j-2);k++) {

    Curves* c_point = Curves::laserpoints[k];
    Curves* n_point = Curves::laserpoints[j-1];

    c_point->Trans_Mat(c_point,n_point);

    MatrixXd pose_laser=c_point->transformation2;

#ifndef POINT3D    
    
    double dx =(Curves::laserpoints[k]->points2(0))-(Curves::laserpoints[j-1]->points2(0));
    double dy = 0.5*((Curves::laserpoints[k]->points2(1))+(Curves::laserpoints[j-1]->points2(1)));
    
    // NOTE the absolute value is used to ensure that this is a cost and
    // and the lowest possible value is zero.
   
    double constraint = fabs(fabs(uni_rate[F]) - fabs(dx*dy));
  //  cout<<"dx"<<" "<<dx<<" "<<"dy"<<" "<<dy<<" "<<"constraint"<<" "<<constraint<<endl;        
    // closing constraint
    if (F== (uni-2)) {

      dx =(Curves::laserpoints[j-1]->points2(0))-(Curves::laserpoints[0]->points2(0));
      dy =0.5*((Curves::laserpoints[j-1]->points2(1))+(Curves::laserpoints[0]->points2(1)));

      constraint = constraint + fabs(fabs(uni_rate[F+1]) - fabs(dx*dy));
    }
#else

   // length based constraint for curved shapes in > 2 dimensional space 
      double l_segment =fabs((Curves::laserpoints[k]->points2(0))-(Curves::laserpoints[j-1]->points2(0)));
      double constraint = fabs(fabs(uni_rate[F]) - l_segment);

#endif

    temp1(k) =  alpha*pow(geo_dist_SE(fix,pose_laser),2);
    temp2(k) =  beta*constraint;

     // closing deformation cost
     if (F== (uni-2)) {

        Curves* cc_point = Curves::laserpoints[j-1];
        Curves* nn_point = Curves::laserpoints[0];

        cc_point->Trans_Mat(cc_point,nn_point);

        MatrixXd pose_laserr=cc_point->transformation2;
        MatrixXd pose_gpss=Curves::gpspoints[F+1]->transformation1;           
        temp1(k) = temp1(k) + alpha*pow(geo_dist_SE(pose_gpss,pose_laserr),2);
     }


    temp3(k) =  temp1(k) + temp2(k);
   }

}

void dp_optimal_point_sampling_Both(std::vector<double> *point_g,std::vector<double> *hector_matrix,int *opt_index,double alpha,double beta,int window_size) {








}
void dp_optimal_point_sampling_Single(std::vector<double> *point_curve1,std::vector<double> *point_curve2,unsigned int *opt_index,double alpha,double beta,int window_size,int type) {

   // defines the radius of the window size
    int r=point_curve1[0].size();            //curve 1,2的点的个数
    int c=point_curve2[0].size();

    double c_i=double(c);
    double r_i=double(r);
 
   // Variables for intermidate computations
   
    MatrixXd cost1=MatrixXd::Constant(c,r,inf);
    MatrixXd cost2=MatrixXd::Constant(c,r,inf);
    MatrixXd cost3=MatrixXd::Constant(c,r,inf);
    

    VectorXi index1 = VectorXi::Zero(r);
    VectorXi index2 = VectorXi::Zero(r);
    VectorXi index3 = VectorXi::Zero(r);

    MatrixXd temp_ind1 =MatrixXd::Zero(c,r);
    MatrixXd temp_ind2 =MatrixXd::Zero(c,r);
    MatrixXd temp_ind3 =MatrixXd::Zero(c,r);


    cost1(0,0) = 0;
    cost2(0,0) = 0;
    cost3(0,0) = 0;

    index1(0) = 1;
    index2(0) = 1;
    index3(0) = 1;


    temp_ind1(0,0) = 1;
    temp_ind2(0,0) = 1;
    temp_ind3(0,0) = 1;

    VectorXd temp1(c); 
    VectorXd temp2(c);
    VectorXd temp3(c);
   
    int sliding_rate = floor(c_i/r_i);
    int uni=floor(c_i/sliding_rate);
    
    // int sliding_rate = ceil(c_i/r_i);
    //int uni=ceil(c_i/sliding_rate);
    
    if(type==0)
    uni=uni-1;
    double uni_rate[uni];
    //MatrixXd cost3=MatrixXd::Constant(c,r,inf);
   // cout<<uni<<endl;
    Vector3d gps_point;
    Vector3d laser_point;
    string t="gps";
    
    
   
    for( int h = 0; h < r; h++) {

     
     gps_point(0)=point_curve1[1][h];
     gps_point(1)=point_curve1[2][h];
     gps_point(2)=point_curve1[3][h];
     Curves::gpspoints.push_back(new Curves(gps_point,1,h,t));
   } 
   
   t="laser";
    
   

   for( int g = 0; g < c; g++) {

     
     laser_point(0)=point_curve2[1][g];
     laser_point(1)=point_curve2[2][g];
     laser_point(2)=point_curve2[3][g];
     
     Curves::laserpoints.push_back(new Curves(laser_point,1,g,t));
   }
    
   

    curve_rep(Curves::gpspoints,1); 
    
#ifndef POINT3D                                //平面，二维
         // area based constraint for planar curved shapes 
    Curve_area(point_curve2,sliding_rate,uni_rate,type);
#else
         // length based constraint for curved shapes in > 2 dimensional space
    Curve_length(point_curve2,sliding_rate,uni_rate,type);
#endif

    cout<<"start to process second curve using optimal sampling solution"<<endl;
    
  
    for( int i=0;i<r-1;i++) {
       int p=sliding_rate*i;
       int start=MAX(2,p-window_size);
       int las = MIN(c,window_size+p);
       int pre = MAX(2,start-sliding_rate);
       
    if(i==0)
       pre=1;
    
    for( int j=start;j<=las;j++) {
        
      for( int m=0;m<c;m++) {
  
        temp1(m)=inf;
        temp2(m)=inf;
        temp3(m)=inf;
      }
  
     MatrixXd fix =Curves::gpspoints[i]->transformation1;
 
   for( int k=pre-1;k<=(j-2);k++) {

     Curves* c_point = Curves::laserpoints[k];
     Curves* n_point = Curves::laserpoints[j-1];

     c_point->Trans_Mat(c_point,n_point);

     MatrixXd pose_laser=c_point->transformation2;

#ifndef POINT3D    
    
    double dx =(Curves::laserpoints[k]->points2(0))-(Curves::laserpoints[j-1]->points2(0));
    double dy = 0.5*((Curves::laserpoints[k]->points2(1))+(Curves::laserpoints[j-1]->points2(1)));
    
    // NOTE the absolute value is used to ensure that this is a cost and
    // and the lowest possible value is zero.
   
    double constraint = fabs(fabs(uni_rate[i]) - fabs(dx*dy));
  //  cout<<"dx"<<" "<<dx<<" "<<"dy"<<" "<<dy<<" "<<"constraint"<<" "<<constraint<<endl;        
    // closing constraint
    if ((i== (uni-2))&&(type==1)) {

      dx =(Curves::laserpoints[j-1]->points2(0))-(Curves::laserpoints[0]->points2(0));
      dy =0.5*((Curves::laserpoints[j-1]->points2(1))+(Curves::laserpoints[0]->points2(1)));

      constraint = constraint + fabs(fabs(uni_rate[i+1]) - fabs(dx*dy));
    }
#else

// length based constraint for curved shapes in > 2 dimensional space 
      double l_segment =fabs((Curves::laserpoints[k]->points2(0))-(Curves::laserpoints[j-1]->points2(0)));
      double constraint = fabs(fabs(uni_rate[i]) - l_segment);

#endif

    temp1(k) =  alpha*pow(geo_dist_SE(fix,pose_laser),2);
    temp2(k) =  beta*constraint;

     // closing deformation cost
     if ((i== (uni-2))&&(type==1)) {

        Curves* cc_point = Curves::laserpoints[j-1];
        Curves* nn_point = Curves::laserpoints[0];

        cc_point->Trans_Mat(cc_point,nn_point);

        MatrixXd pose_laserr=cc_point->transformation2;
        MatrixXd pose_gpss=Curves::gpspoints[i+1]->transformation1;           
        temp1(k) = temp1(k) + alpha*pow(geo_dist_SE(pose_gpss,pose_laserr),2);
     }


    temp3(k) =  temp1(k) + temp2(k);
   }
   
        
        MatrixXd::Index minRow, minCol;

        double  min1 = (cost1.col(i) + temp1).minCoeff(&minRow, &minCol);
        cost1(j-1,i+1)=min1;
        temp_ind1(j-1,i+1)=minRow;
        
        double  min2 = (cost2.col(i) + temp2).minCoeff(&minRow, &minCol);
        cost2(j-1,i+1)=min2;
        temp_ind2(j-1,i+1)=minRow;

        double  min3 = (cost3.col(i) + temp3).minCoeff(&minRow, &minCol);
        cost3(j-1,i+1)=min3;
        temp_ind3(j-1,i+1)=minRow; 
        //cout<<min3<<" "<< minRow<<endl;

        cout<<"the"<<" "<<i <<"th loop;"<<" "<<"the window size loop:"<<" "<<j<<endl;
        
     }

       
    
   }

     
     // Work backwards to get the optimal sampling solution
      MatrixXd::Index minRow,minCol;
      double a=(cost1.col(r-1)).minCoeff(&minRow, &minCol);
      index1(r-1) =minRow; 
      a=(cost2.col(r-1)).minCoeff(&minRow, &minCol);
      index2(r-1) =minRow; 
      a=(cost3.col(r-1)).minCoeff(&minRow, &minCol);
      index3(r-1) =minRow;

      
      
     for( int h=r-1;h>=1;h--) {
        // cost of deformation
        index1(h-1) = temp_ind1(index1(h),h);
        // cost of the area
        index2(h-1) = temp_ind2(index2(h),h);
        // cost of constrained objective functional
        index3(h-1) = temp_ind3(index3(h),h);
      }  
     

     for( int j=0;j<r;j++) 
     opt_index[j]=index3(j);
     cout<<"Point correspondence calculation completed"<<endl;

}


void Optimal_correspondence(std::vector<double> *point_curve1,std::vector<double> *point_curve2,std::vector<double> *point_corr,unsigned int *h_seq,string dir,int type,double beta,int window_size) {

  unsigned int sample_points = point_curve1[0].size();    //the number of GPS positions.
  
 // unsigned int num_points    = hector_matrix[0].size();

  double alpha = 1.0;
 

  unsigned int opt_index[sample_points];

  //if(type==1) {

   dp_optimal_point_sampling_Single(point_curve1,point_curve2,opt_index,alpha,beta,window_size,type);

   for(unsigned int i=0;i<=point_curve1[0].size()-1;i++) {

     time_s[i]=(point_curve2[0][opt_index[i]]);
     //time_s[i]=(point_curve1[0][i]);
     point_corr[0].push_back(point_curve2[0][opt_index[i]]);
     //point_corr[0].push_back(point_curve1[0][i]);
     point_corr[1].push_back(point_curve2[1][opt_index[i]]);
     point_corr[2].push_back(point_curve2[2][opt_index[i]]);
     point_corr[3].push_back(point_curve2[3][opt_index[i]]);
 
     point_corr[4].push_back(point_curve1[1][i]);
     point_corr[5].push_back(point_curve1[2][i]);
     point_corr[6].push_back(point_curve1[3][i]);  
    // cout<<opt_index[i]<<endl;
   }

     cout<<"save point correspondence result"<<endl;
     save_correspodence(point_corr,dir);
 // }

 // else {

  //   dp_optimal_point_sampling_Both(point_g,hector_matrix,opt_index,alpha,beta,window_size);

   for(unsigned int j=0;j<sample_points-1;j++) 
     h_seq[j]=opt_index[j+1]-opt_index[j];

}

