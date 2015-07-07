#include "utils_mei.h"

#include <opencv2/opencv.hpp>

namespace MEI
{

bool readFromYamlFile(const std::string& filename, 
                      parameters& param)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        return false;
    }

    if (!fs["model_type"].isNone())
    {
        std::string sModelType;
        fs["model_type"] >> sModelType;

        if (sModelType.compare("MEI") != 0)
        {
            return false;
        }
    }

    param.imageWidth = static_cast<int>(fs["image_width"]);
    param.imageHeight = static_cast<int>(fs["image_height"]);

    cv::FileNode n = fs["mirror_parameters"];
    param.xi = static_cast<double>(n["xi"]);

    n = fs["distortion_parameters"];
    param.k1 = static_cast<double>(n["k1"]);
    param.k2 = static_cast<double>(n["k2"]);
    param.p1 = static_cast<double>(n["p1"]);
    param.p2 = static_cast<double>(n["p2"]);

    n = fs["projection_parameters"];
    param.gamma1 = static_cast<double>(n["gamma1"]);
    param.gamma2 = static_cast<double>(n["gamma2"]);
    param.u0 = static_cast<double>(n["u0"]);
    param.v0 = static_cast<double>(n["v0"]);

    param.fx = 300;
    param.fy = 300;
    param.cx = param.imageWidth/2;
    param.cy = param.imageHeight/2;
    return true;
}

bool loadIntrinsicFromYAML(const char *filename, sensor_msgs::CameraInfo &info_cam, MEI::parameters& param)
{
   if(!readFromYamlFile(filename, param))
   {
      printf("Failed to open file %s\n", filename);
      return false;
   }
    
   //Distortion parameters
   info_cam.D.push_back(0);   //xi
   info_cam.D.push_back(0);   //k1
   info_cam.D.push_back(0);   //t2
   info_cam.D.push_back(0);   //p1
   info_cam.D.push_back(0);   //p2

   //Intrinsic camera matrix
   info_cam.K[0]=param.gamma1;//fx
   info_cam.K[1]=0;
   info_cam.K[2]=param.u0;    //cx
   info_cam.K[3]=0;
   info_cam.K[4]=param.gamma2;//fy
   info_cam.K[5]=param.v0;    //cy
   info_cam.K[6]=0;
   info_cam.K[7]=0;
   info_cam.K[8]=1;

   info_cam.distortion_model="plumb_bob";

   info_cam.binning_x=0;
   info_cam.binning_y=0;
   info_cam.roi.x_offset=0;
   info_cam.roi.y_offset=0;
   info_cam.roi.do_rectify=false;

   info_cam.width = param.imageWidth;
   info_cam.height = param.imageHeight;

   info_cam.roi.width=info_cam.width;
   info_cam.roi.height=info_cam.height;

   //Projection camera matrix
   info_cam.P[0]=param.fx; //fx
   info_cam.P[1]=0;
   info_cam.P[2]=param.cx; //cx
   info_cam.P[3]=0;        //Tx
   info_cam.P[4]=0;
   info_cam.P[5]=param.fy; //fy
   info_cam.P[6]=param.cy; //cy
   info_cam.P[7]=0;        //Ty
   info_cam.P[8]=0;
   info_cam.P[9]=0;
   info_cam.P[10]=1;       //1.0
   info_cam.P[11]=0;

   return true;
}

bool loadExtrinsicFromYAML(const char *filename, sensor_msgs::CameraInfo &info_caml, sensor_msgs::CameraInfo &info_camr, cv::Mat& R1_cv, cv::Mat& R2_cv)
{
   cv::Mat R,T,Quick;   
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
   double baseline = sqrt(tx*tx+ty*ty+tz*tz);

   fromQuaternionToRotationMatrix(qx,qy,qz,qw,rot);
   cv::eigen2cv(rot,R);
   T.push_back(tx);
   T.push_back(ty);
   T.push_back(tz);


   Eigen::Matrix4d H_cam1 = Eigen::Matrix4d::Identity();
   Eigen::Matrix4d H_cam2 = Eigen::Matrix4d::Identity();
   Eigen::Vector3d Tr(tx,ty,tz);

   H_cam2.block<3,3>(0,0)= rot;
   H_cam2.block<3,1>(0,3)= Tr.transpose();

   // compute average rotation between camera pair
   Eigen::JacobiSVD<Eigen::Matrix3d> svd(H_cam1.block<3,3>(0,0) +
                                         H_cam2.block<3,3>(0,0),
                                         Eigen::ComputeFullU | Eigen::ComputeFullV);

   Eigen::Matrix3d avgR = svd.matrixU() * svd.matrixV().transpose();
   Eigen::Matrix3d R1 = avgR * H_cam1.block<3,3>(0,0).transpose();
   Eigen::Matrix3d R2 = avgR * H_cam2.block<3,3>(0,0).transpose();

   cv::eigen2cv(R1, R1_cv);
   cv::eigen2cv(R2, R2_cv);

   //Rectification matrix
   info_caml.R[0]=R1_cv.at<double>(0,0);
   info_caml.R[1]=R1_cv.at<double>(0,1);
   info_caml.R[2]=R1_cv.at<double>(0,2);
   info_caml.R[3]=R1_cv.at<double>(1,0);
   info_caml.R[4]=R1_cv.at<double>(1,1);
   info_caml.R[5]=R1_cv.at<double>(1,2);
   info_caml.R[6]=R1_cv.at<double>(2,0);
   info_caml.R[7]=R1_cv.at<double>(2,1);
   info_caml.R[8]=R1_cv.at<double>(2,2);

   info_camr.R[0]=R2_cv.at<double>(0,0);
   info_camr.R[1]=R2_cv.at<double>(0,1);
   info_camr.R[2]=R2_cv.at<double>(0,2);
   info_camr.R[3]=R2_cv.at<double>(1,0);
   info_camr.R[4]=R2_cv.at<double>(1,1);
   info_camr.R[5]=R2_cv.at<double>(1,2);
   info_camr.R[6]=R2_cv.at<double>(2,0);
   info_camr.R[7]=R2_cv.at<double>(2,1);
   info_camr.R[8]=R2_cv.at<double>(2,2);

   //Projection camera matrix
   info_camr.P[3]=-info_camr.P[0]*baseline;   //Tx

   fs.release();
   return true;
}

cv::Mat initUndistortRectifyMap(cv::Mat& map1, cv::Mat& map2, cv::Mat rmat, 
                                parameters param)
{
    cv::Size imageSize(param.imageWidth, param.imageHeight);

    cv::Mat mapX = cv::Mat::zeros(imageSize.height, imageSize.width, CV_32F);
    cv::Mat mapY = cv::Mat::zeros(imageSize.height, imageSize.width, CV_32F);

    Eigen::Matrix3f K_rect;

    if (param.cx == -1.0f && param.cy == -1.0f)
    {
        K_rect << param.fx, 0, imageSize.width / 2,
                  0, param.fy, imageSize.height / 2,
                  0, 0, 1;
    }
    else
    {
        K_rect << param.fx, 0, param.cx,
                  0, param.fy, param.cy,
                  0, 0, 1;
    }

    if (param.fx == -1.0f || param.fy == -1.0f)
    {
        K_rect(0,0) = param.gamma1;
        K_rect(1,1) = param.gamma2;
    }

    Eigen::Matrix3f K_rect_inv = K_rect.inverse();

    Eigen::Matrix3f R, R_inv;
    cv::cv2eigen(rmat, R);
    R_inv = R.inverse();

    for (int v = 0; v < imageSize.height; ++v)
    {
        for (int u = 0; u < imageSize.width; ++u)
        {
            Eigen::Vector3f xo;
            xo << u, v, 1;

            Eigen::Vector3f uo = R_inv * K_rect_inv * xo;

            Eigen::Vector2d p;
            spaceToPlane(uo.cast<double>(), p, param);

            mapX.at<float>(v,u) = p(0);
            mapY.at<float>(v,u) = p(1);
        }
    }

    cv::convertMaps(mapX, mapY, map1, map2, CV_32FC1, false);

    cv::Mat K_rect_cv;
    cv::eigen2cv(K_rect, K_rect_cv);
    return K_rect_cv;
}

/** 
 * \brief Apply distortion to input point (from the normalised plane)
 *  
 * \param p_u undistorted coordinates of point on the normalised plane
 * \return to obtain the distorted point: p_d = p_u + d_u
 */
void
distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u, parameters param ) 
{
    double k1 = param.k1;
    double k2 = param.k2;
    double p1 = param.p1;
    double p2 = param.p2;

    double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

    mx2_u = p_u(0) * p_u(0);
    my2_u = p_u(1) * p_u(1);
    mxy_u = p_u(0) * p_u(1);
    rho2_u = mx2_u + my2_u;
    rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;
    d_u << p_u(0) * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u),
           p_u(1) * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);
}

void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p, 
                  parameters param)
{
    Eigen::Vector2d p_u, p_d;

    // Project points to the normalised plane
    double z = P(2) + param.xi * P.norm();
    p_u << P(0) / z, P(1) / z;

    {
        // Apply distortion
        Eigen::Vector2d d_u;
        distortion(p_u, d_u, param);
        p_d = p_u + d_u;
    }

    // Apply generalised projection matrix
    p << param.gamma1 * p_d(0) + param.u0,
         param.gamma2 * p_d(1) + param.v0;
}

void liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P,
                    parameters param)
{

    double m_inv_K11 = 1.0 / param.gamma1;
    double m_inv_K13 = -param.u0 / param.gamma1;
    double m_inv_K22 = 1.0 / param.gamma2;
    double m_inv_K23 = -param.v0 / param.gamma2;

    double mx_d, my_d,mx2_d, mxy_d, my2_d, mx_u, my_u;
    double rho2_d, rho4_d, radDist_d, Dx_d, Dy_d, inv_denom_d;

    // Lift points to normalised plane
    mx_d = m_inv_K11 * p(0) + m_inv_K13;
    my_d = m_inv_K22 * p(1) + m_inv_K23;


        if (0)
        {
            double k1 = param.k1;
            double k2 = param.k2;
            double p1 = param.p1;
            double p2 = param.p2;

            // Apply inverse distortion model
            // proposed by Heikkila
            mx2_d = mx_d*mx_d;
            my2_d = my_d*my_d;
            mxy_d = mx_d*my_d;
            rho2_d = mx2_d+my2_d;
            rho4_d = rho2_d*rho2_d;
            radDist_d = k1*rho2_d+k2*rho4_d;
            Dx_d = mx_d*radDist_d + p2*(rho2_d+2*mx2_d) + 2*p1*mxy_d;
            Dy_d = my_d*radDist_d + p1*(rho2_d+2*my2_d) + 2*p2*mxy_d;
            inv_denom_d = 1/(1+4*k1*rho2_d+6*k2*rho4_d+8*p1*my_d+8*p2*mx_d);

            mx_u = mx_d - inv_denom_d*Dx_d;
            my_u = my_d - inv_denom_d*Dy_d;
        }
        else
        {
            // Recursive distortion model
            int n = 8;
            Eigen::Vector2d d_u;
            distortion(Eigen::Vector2d(mx_d, my_d), d_u,param);
            // Approximate value
            mx_u = mx_d - d_u(0);
            my_u = my_d - d_u(1);

            for (int i = 1; i < n; ++i)
            {
                distortion(Eigen::Vector2d(mx_u, my_u), d_u,param);
                mx_u = mx_d - d_u(0);
                my_u = my_d - d_u(1);
            }
        }
    

    // Obtain a projective ray
    double xi = param.xi;
    if (xi == 1.0)
    {
        P << mx_u, my_u, (1.0 - mx_u * mx_u - my_u * my_u) / 2.0;
    }
    else
    {
        // Reuse variable
        rho2_d = mx_u * mx_u + my_u * my_u;
        P << mx_u, my_u, 1.0 - xi * (rho2_d + 1.0) / (xi + sqrt(1.0 + (1.0 - xi * xi) * rho2_d));
    }
}

void rectifyImagePoints(const std::vector<cv::Point2f>& src, std::vector<cv::Point2f>& dst,
                        parameters param)
{
    dst.resize(src.size());

    for (size_t i = 0; i < src.size(); ++i)
    {
        const cv::Point2f& p = src.at(i);

        Eigen::Vector3d P;
        liftProjective(Eigen::Vector2d(p.x, p.y), P, param);

        P /= P(2);

        dst.at(i) = cv::Point2f(P(0)*param.fx+param.cx, P(1)*param.fy+param.cy);
    }
}

void rectifyImagePoint(const cv::Point2f& src, cv::Point2f& dst,
                       parameters param)
{
    Eigen::Vector3d P;

    liftProjective(Eigen::Vector2d(src.x, src.y), P, param);

    P /= P(2);

    dst.x = P(0)*param.fx+param.cx;
    dst.y = P(1)*param.fy+param.cy;
}
}
