#include "utils.h"

//OpenCV
#include <opencv2/core/eigen.hpp>

using namespace cv;

Mat makeQMatrix(Point2d image_center,double focal_length, double baseline)
{
   Mat Q=Mat::eye(4,4,CV_64F);
   Q.at<double>(0,3)=-image_center.x;
   Q.at<double>(1,3)=-image_center.y;
   Q.at<double>(2,3)=focal_length;
   Q.at<double>(3,3)=0.0;
   Q.at<double>(2,2)=0.0;
   Q.at<double>(3,2)=1.0/baseline;
 
/*
Q = [ 1 0   0      -Cx      ]
    [ 0 1   0      -Cy      ]
    [ 0 0   0       Fx      ]
    [ 0 0 -1/Tx (Cx-Cx')/Tx ]*/

   return Q;
}

bool loadIntrinsicFromYAML(const char *filename, sensor_msgs::CameraInfo &info_cam, cv::Mat& M, cv::Mat& D, cv::Mat& R, cv::Mat& P)
{
   cv::FileStorage fs(filename, cv::FileStorage::READ);
   double width,height;
   fs["K"] >> M;
   fs["D"] >> D;
   fs["R"] >> R;
   fs["P"] >> P;
   fs["width"] >> width;
   fs["height"] >> height;

   //Distortion parameters
   info_cam.D.push_back(D.at<float>(0));//k1
   info_cam.D.push_back(D.at<float>(1));//k2
   info_cam.D.push_back(D.at<float>(2));//t1
   info_cam.D.push_back(D.at<float>(3));//t2
   info_cam.D.push_back(D.at<float>(4));//k3

   //Intrinsic camera matrix
   info_cam.K[0]=M.at<float>(0,0);//fx
   info_cam.K[1]=M.at<float>(0,1);
   info_cam.K[2]=M.at<float>(0,2);//cx
   info_cam.K[3]=M.at<float>(1,0);
   info_cam.K[4]=M.at<float>(1,1);//fy
   info_cam.K[5]=M.at<float>(1,2);//cy
   info_cam.K[6]=M.at<float>(2,0);
   info_cam.K[7]=M.at<float>(2,1);
   info_cam.K[8]=M.at<float>(2,2);

   info_cam.distortion_model="plumb_bob";

   info_cam.binning_x=0;
   info_cam.binning_y=0;
   info_cam.roi.x_offset=0;
   info_cam.roi.y_offset=0;
   info_cam.roi.do_rectify=false;

   info_cam.width = width;
   info_cam.height = height;

   info_cam.roi.width=info_cam.width;
   info_cam.roi.height=info_cam.height;

   //Rectification matrix
   info_cam.R[0]=R.at<double>(0,0);
   info_cam.R[1]=R.at<double>(0,1);
   info_cam.R[2]=R.at<double>(0,2);
   info_cam.R[3]=R.at<double>(1,0);
   info_cam.R[4]=R.at<double>(1,1);
   info_cam.R[5]=R.at<double>(1,2);
   info_cam.R[6]=R.at<double>(2,0);
   info_cam.R[7]=R.at<double>(2,1);
   info_cam.R[8]=R.at<double>(2,2);

   info_cam.P[0]=P.at<double>(0,0);//fx
   info_cam.P[1]=P.at<double>(0,1);
   info_cam.P[2]=P.at<double>(0,2);//cx
   info_cam.P[3]=P.at<double>(0,3);//Tx
   info_cam.P[4]=P.at<double>(1,0);
   info_cam.P[5]=P.at<double>(1,1);//fy
   info_cam.P[6]=P.at<double>(1,2);//cy
   info_cam.P[7]=P.at<double>(1,3);//Ty
   info_cam.P[8]=P.at<double>(2,0);
   info_cam.P[9]=P.at<double>(2,1);
   info_cam.P[10]=P.at<double>(2,2);
   info_cam.P[11]=P.at<double>(2,3);//1.0

   fs.release();
   return true;
}

bool loadExtrinsicFromXML(const char *filename, sensor_msgs::CameraInfo &info_caml, sensor_msgs::CameraInfo &info_camr, Mat& R1, Mat& P1, Mat& R2, Mat& P2)
{
   FileStorage fs;
   Mat R, T;
   fs.open(filename, CV_STORAGE_READ);
   if(!fs.isOpened())
   {
      printf("Failed to open file %s\n", filename);
      return false;
   }
   printf("open file %s\n", filename);
   fs["R_left"] >> R1;
   fs["P_left"] >> P1;
   fs["R_right"] >> R2;
   fs["P_right"] >> P2;
   fs["R"] >> R;
   fs["T"] >> T;

//    if(P2.at<double>(0,3)==0)
   P2.at<double>(0,3) = P2.at<double>(0,0)*T.at<double>(0,0);

   //Rectification matrix
   info_caml.R[0]=R1.at<double>(0,0);
   info_caml.R[1]=R1.at<double>(0,1);
   info_caml.R[2]=R1.at<double>(0,2);
   info_caml.R[3]=R1.at<double>(1,0);
   info_caml.R[4]=R1.at<double>(1,1);
   info_caml.R[5]=R1.at<double>(1,2);
   info_caml.R[6]=R1.at<double>(2,0);
   info_caml.R[7]=R1.at<double>(2,1);
   info_caml.R[8]=R1.at<double>(2,2);

   info_camr.R[0]=R2.at<double>(0,0);
   info_camr.R[1]=R2.at<double>(0,1);
   info_camr.R[2]=R2.at<double>(0,2);
   info_camr.R[3]=R2.at<double>(1,0);
   info_camr.R[4]=R2.at<double>(1,1);
   info_camr.R[5]=R2.at<double>(1,2);
   info_camr.R[6]=R2.at<double>(2,0);
   info_camr.R[7]=R2.at<double>(2,1);
   info_camr.R[8]=R2.at<double>(2,2);

   //Projection camera matrix
   info_caml.P[0]=P1.at<double>(0,0);//fx
   info_caml.P[1]=P1.at<double>(0,1);
   info_caml.P[2]=P1.at<double>(0,2);//cx
   info_caml.P[3]=P1.at<double>(0,3);//Tx
   info_caml.P[4]=P1.at<double>(1,0);
   info_caml.P[5]=P1.at<double>(1,1);//fy
   info_caml.P[6]=P1.at<double>(1,2);//cy
   info_caml.P[7]=P1.at<double>(1,3);//Ty
   info_caml.P[8]=P1.at<double>(2,0);
   info_caml.P[9]=P1.at<double>(2,1);
   info_caml.P[10]=P1.at<double>(2,2);
   info_caml.P[11]=P1.at<double>(2,3);//1.0

   info_camr.P[0]=P2.at<double>(0,0);//fx
   info_camr.P[1]=P2.at<double>(0,1);
   info_camr.P[2]=P2.at<double>(0,2);//cx
   info_camr.P[3]=P2.at<double>(0,3);//Tx
   info_camr.P[4]=P2.at<double>(1,0);
   info_camr.P[5]=P2.at<double>(1,1);//fy
   info_camr.P[6]=P2.at<double>(1,2);//cy
   info_camr.P[7]=P2.at<double>(1,3);//Ty
   info_camr.P[8]=P2.at<double>(2,0);
   info_camr.P[9]=P2.at<double>(2,1);
   info_camr.P[10]=P2.at<double>(2,2);
   info_camr.P[11]=P2.at<double>(2,3);//1.0

   fs.release();
   return true;
}

bool loadIntrinsicFromXML(const char *filename, sensor_msgs::CameraInfo &info_cam, Mat& M1, Mat& D1)
{
   double width,height;
   FileStorage fs;
   fs.open(filename, CV_STORAGE_READ);
   if(!fs.isOpened())
   {
      printf("Failed to open file %s\n", filename);
      return false;
   }
   printf("open file %s\n", filename);

   fs["M"] >> M1;
   fs["D"] >> D1;
   fs["width"] >> width;
   fs["height"] >> height;

   //Distortion parameters
   info_cam.D.push_back(D1.at<float>(0));//k1
   info_cam.D.push_back(D1.at<float>(1));//k2
   info_cam.D.push_back(D1.at<float>(2));//t1
   info_cam.D.push_back(D1.at<float>(3));//t2
   info_cam.D.push_back(D1.at<float>(4));//k3

   //Intrinsic camera matrix
   info_cam.K[0]=M1.at<float>(0,0);//fx
   info_cam.K[1]=M1.at<float>(0,1);
   info_cam.K[2]=M1.at<float>(0,2);//cx
   info_cam.K[3]=M1.at<float>(1,0);
   info_cam.K[4]=M1.at<float>(1,1);//fy
   info_cam.K[5]=M1.at<float>(1,2);//cy
   info_cam.K[6]=M1.at<float>(2,0);
   info_cam.K[7]=M1.at<float>(2,1);
   info_cam.K[8]=M1.at<float>(2,2);

   info_cam.distortion_model="plumb_bob";

   info_cam.binning_x=0;
   info_cam.binning_y=0;
   info_cam.roi.x_offset=0;
   info_cam.roi.y_offset=0;
   info_cam.roi.do_rectify=false;

   info_cam.width = width;
   info_cam.height = height;

   info_cam.roi.width=info_cam.width;
   info_cam.roi.height=info_cam.height;

   fs.release();
   return true;
}

void fromQuaternionToRotationMatrix(double qx,double qy,double qz, double qw, Eigen::Matrix3d &rot)
{
   rot(0,0)=pow(qw,2.0) + pow(qx,2.0) - pow(qy,2.0) - pow(qz,2.0);
   rot(0,1)=2.0*qx*qy - 2.0*qw*qz;
   rot(0,2)=2.0*qx*qz + 2.0*qw*qy;
   rot(1,0)=2.0*qx*qy + 2.0*qw*qz;
   rot(1,1)=pow(qw,2.0) - pow(qx,2.0) + pow(qy,2.0) - pow(qz,2.0);
   rot(1,2)=2.0*qy*qz - 2.0*qw*qx;
   rot(2,0)=2.0*qx*qz - 2.0*qw*qy;
   rot(2,1)=2.0*qy*qz + 2.0*qw*qx;
   rot(2,2)=pow(qw,2.0) - pow(qx,2.0) - pow(qy,2.0) + pow(qz,2.0);
}

bool loadExtrinsicFromYAML(const char *filename, sensor_msgs::CameraInfo &info_caml, sensor_msgs::CameraInfo &info_camr, Mat& R1, Mat& P1, Mat& R2, Mat& P2, Mat& M1, Mat& M2)
{
   Mat R,T,Quick;   
   Eigen::Matrix3d rot;
   cv::FileStorage fs(filename, cv::FileStorage::READ);

   if (!fs.isOpened())
   {
      std::cout<<"error"<<std::endl;
      return false;
   }
   cv::FileNode n = fs["transform"];
   double qx = static_cast<double>(n["q_x"]);
   double qy = static_cast<double>(n["q_y"]);
   double qz = static_cast<double>(n["q_z"]);
   double qw = static_cast<double>(n["q_w"]);
   double tx = static_cast<double>(n["t_x"]);
   double ty = static_cast<double>(n["t_y"]);
   double tz = static_cast<double>(n["t_z"]);

   fromQuaternionToRotationMatrix(qx,qy,qz,qw,rot);
   cv::eigen2cv(rot,R);
   T.push_back(tx);
   T.push_back(ty);
   T.push_back(tz);

   stereoRectify(M1, cv::Mat::zeros(1, 5, CV_32F), M2, cv::Mat::zeros(1, 5, CV_32F), Size(1296,966), R, T, R1, R2, P1, P2, Quick, CALIB_ZERO_DISPARITY, -1);

//    if(P2.at<double>(0,3)==0)
   P2.at<double>(0,3) = P2.at<double>(0,0)*T.at<double>(0,0);

   //Rectification matrix
   info_caml.R[0]=R1.at<double>(0,0);
   info_caml.R[1]=R1.at<double>(0,1);
   info_caml.R[2]=R1.at<double>(0,2);
   info_caml.R[3]=R1.at<double>(1,0);
   info_caml.R[4]=R1.at<double>(1,1);
   info_caml.R[5]=R1.at<double>(1,2);
   info_caml.R[6]=R1.at<double>(2,0);
   info_caml.R[7]=R1.at<double>(2,1);
   info_caml.R[8]=R1.at<double>(2,2);

   info_camr.R[0]=R2.at<double>(0,0);
   info_camr.R[1]=R2.at<double>(0,1);
   info_camr.R[2]=R2.at<double>(0,2);
   info_camr.R[3]=R2.at<double>(1,0);
   info_camr.R[4]=R2.at<double>(1,1);
   info_camr.R[5]=R2.at<double>(1,2);
   info_camr.R[6]=R2.at<double>(2,0);
   info_camr.R[7]=R2.at<double>(2,1);
   info_camr.R[8]=R2.at<double>(2,2);

   //Projection camera matrix
   info_caml.P[0]=P1.at<double>(0,0);//fx
   info_caml.P[1]=P1.at<double>(0,1);
   info_caml.P[2]=P1.at<double>(0,2);//cx
   info_caml.P[3]=P1.at<double>(0,3);//Tx
   info_caml.P[4]=P1.at<double>(1,0);
   info_caml.P[5]=P1.at<double>(1,1);//fy
   info_caml.P[6]=P1.at<double>(1,2);//cy
   info_caml.P[7]=P1.at<double>(1,3);//Ty
   info_caml.P[8]=P1.at<double>(2,0);
   info_caml.P[9]=P1.at<double>(2,1);
   info_caml.P[10]=P1.at<double>(2,2);
   info_caml.P[11]=P1.at<double>(2,3);//1.0

   info_camr.P[0]=P2.at<double>(0,0);//fx
   info_camr.P[1]=P2.at<double>(0,1);
   info_camr.P[2]=P2.at<double>(0,2);//cx
   info_camr.P[3]=P2.at<double>(0,3);//Tx
   info_camr.P[4]=P2.at<double>(1,0);
   info_camr.P[5]=P2.at<double>(1,1);//fy
   info_camr.P[6]=P2.at<double>(1,2);//cy
   info_camr.P[7]=P2.at<double>(1,3);//Ty
   info_camr.P[8]=P2.at<double>(2,0);
   info_camr.P[9]=P2.at<double>(2,1);
   info_camr.P[10]=P2.at<double>(2,2);
   info_camr.P[11]=P2.at<double>(2,3);//1.0

   fs.release();
   return true;
}

Mat disparity(int alg, Mat framel, Mat framer)
{
   Mat tmpl,tmpr;
   Mat disp;
   Mat disp8;

   equalizeHist(framel,tmpl);
   equalizeHist(framer,tmpr);

   int numberOfDisparities = 96;

   switch(alg)
   {
      case 0:
      {
        StereoBM bm;
        bm.state->preFilterCap = 10;
        bm.state->SADWindowSize = 21;
        bm.state->minDisparity = -1;
        bm.state->numberOfDisparities = numberOfDisparities;
        bm.state->textureThreshold = 1000;
        bm.state->uniquenessRatio = 3.5;
        bm.state->speckleWindowSize = 1400;
        bm.state->speckleRange = 31;
        bm.state->disp12MaxDiff = 1;
        bm.state->preFilterSize = 109;

        bm(tmpl, tmpr, disp);
        break;
      }
      case 1:
      {
        StereoSGBM sgbm;
        sgbm.preFilterCap = 10;
        sgbm.SADWindowSize = 15;
        int cn = tmpl.channels();
        sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
        sgbm.P2 = 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
        sgbm.minDisparity = 0;
        sgbm.numberOfDisparities = numberOfDisparities;
        sgbm.uniquenessRatio = 30;
        sgbm.speckleWindowSize = 700;
        sgbm.speckleRange = 32;
        sgbm.disp12MaxDiff = 1;
        sgbm(tmpl, tmpr, disp);
        break;
      }
   }
   disp.convertTo(disp8, CV_8U, 1.0/16);//255/(numberOfDisparities*16.));

   return disp8;
}

Mat coloredDisparity(Mat disp8)
{
   // this is great. It converts your grayscale image into a tone-mapped one, 
   // much more pleasing for the eye
   // function is found in contrib module, so include contrib.hpp 
   // and link accordingly
   cv::Mat falseColorsMap;
   disp8.convertTo(falseColorsMap, CV_8U, (96*16.)/255);
   applyColorMap(falseColorsMap, disp8, cv::COLORMAP_JET);
   return disp8;
}

template <class T>
static void fillOcclusion_(Mat& src, const T invalidvalue, const T maxval)
{
   const int MAX_LENGTH=(int)(src.cols*0.5);

   for(int j=0;j<src.rows;j++)
   {
      T* s = src.ptr<T>(j);

      s[0]=maxval;
      s[src.cols-1]=maxval;

      for(int i=1;i<src.cols-1;i++)
      {
         if(s[i]==invalidvalue)
         {
            int t=i;
            do
            {
               t++;
               if(t>src.cols-1)break;
            }while(s[t]==invalidvalue);

            const T dd = min(s[i-1],s[t]);
            if(t-i>MAX_LENGTH)
            {
               for(int n=0;n<src.cols;n++)
               {
                  s[n]=invalidvalue;
               }
            }
            else
            {
               for(;i<t;i++)
               {
                  s[i]=dd;
               }
            }
         }
      }
      s[0]=s[1];
      s[src.cols-1]=s[src.cols-2];
   }
}

void fillOcclusion(Mat& src, int invalidvalue)
{
   if(src.type()==CV_8U)
   {
      fillOcclusion_<uchar>(src, (uchar)invalidvalue,255);
   }
   else if(src.type()==CV_16S)
   {
      fillOcclusion_<short>(src, (short)invalidvalue,SHRT_MAX);
   }
   else if(src.type()==CV_16U)
   {
      fillOcclusion_<unsigned short>(src, (unsigned short)invalidvalue,USHRT_MAX);
   }
   else if(src.type()==CV_32F)
   {
      fillOcclusion_<float>(src, (float)invalidvalue,FLT_MAX);
   }
}

cv::Mat rectifiedPair(cv::Mat framel, cv::Mat framer)
{
   Mat leftImg,rightImg;
   cv::Size img_size = framel.size();
   cvtColor(framel, leftImg, CV_GRAY2BGR);
   cvtColor(framer, rightImg, CV_GRAY2BGR);
   Mat pair = Mat( img_size.height, img_size.width*2,CV_8UC3 );
   Mat left (pair, Rect(0, 0, img_size.width, img_size.height));
   leftImg.copyTo(left);
   Mat right (pair, Rect(img_size.width, 0, img_size.width, img_size.height));
   rightImg.copyTo(right);
   for(int j = 0; j < img_size.height; j += 16 )
      line( pair, Point2f(0,j), Point2f(img_size.width*2,j), CV_RGB(0,255,0));

   return pair;
}

