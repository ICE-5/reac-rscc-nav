#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

const double PI = 3.1415926;

double scanPeriod = 0.100859904 - 20.736e-6;

bool useImuAcceleration = true;

double odomTimeCur = 0;
double odomTimeLast = 0;
bool odomInit = false;

float odomRollCur, odomPitchCur, odomYawCur;
float odomRollLast, odomPitchLast, odomYawLast;

float odomShiftXCur, odomShiftYCur, odomShiftZCur;
float odomShiftXLast, odomShiftYLast, odomShiftZLast;

int imuPointerFront = 0;
int imuPointerLast = -1;
const int imuQueLength = 200;

float imuRollRec = 0, imuPitchRec = 0, imuYawAccu = 0;
float imuRollCur = 0, imuPitchCur = 0, imuYawCur = 0;
float imuAccXCur = 0, imuAccYCur = 0, imuAccZCur = 0;

float imuRollLast = 0, imuPitchLast = 0, imuYawLast = 0;
float imuAccXLast = 0, imuAccYLast = 0, imuAccZLast = 0;

float imuRollCov = 0, imuPitchCov = 0, imuYawCov = 0;
float imuShiftCovX = 0, imuShiftCovY = 0, imuShiftCovZ = 0;

float imuVeloX = 0, imuVeloY = 0, imuVeloZ = 0;
float imuVeloSX1 = 0, imuVeloSY1 = 0, imuVeloSZ1 = 0;
float imuVeloSX2 = 0, imuVeloSY2 = 0, imuVeloSZ2 = 0;
float imuVeloSX3 = 0, imuVeloSY3 = 0, imuVeloSZ3 = 0;
float imuVeloSX4 = 0, imuVeloSY4 = 0, imuVeloSZ4 = 0;
float imuShiftX = 0, imuShiftY = 0, imuShiftZ = 0;
float imuShiftSX = 0, imuShiftSY = 0, imuShiftSZ = 0;

int imuAccSamNum = 0;
double imuTimeSam[imuQueLength] = {0};
float imuRollSam[imuQueLength] = {0};
float imuPitchSam[imuQueLength] = {0};
float imuYawSam[imuQueLength] = {0};
float imuAccXSam[imuQueLength] = {0};
float imuAccYSam[imuQueLength] = {0};
float imuAccZSam[imuQueLength] = {0};

double imuTime[imuQueLength] = {0};
float imuRoll[imuQueLength] = {0};
float imuPitch[imuQueLength] = {0};
float imuYaw[imuQueLength] = {0};
float imuAccX[imuQueLength] = {0};
float imuAccY[imuQueLength] = {0};
float imuAccZ[imuQueLength] = {0};

double aftMappedTime = 0;
float transformSum[6] = {0};
float transformMapped[6] = {0};
float transformBefMapped[6] = {0};
float transformAftMapped[6] = {0};

ros::Publisher *pubOdometryPointer = NULL;
tf::TransformBroadcaster *tfBroadcasterPointer = NULL;
nav_msgs::Odometry odometry;
tf::StampedTransform odometryTrans;

void accuRotation(float cx, float cy, float cz, float lx, float ly, float lz, 
                  float &ox, float &oy, float &oz)
{
  float srx = cos(lx)*cos(cx)*sin(ly)*sin(cz) - cos(cx)*cos(cz)*sin(lx) - cos(lx)*cos(ly)*sin(cx);
  ox = -asin(srx);

  float srycrx = sin(lx)*(cos(cy)*sin(cz) - cos(cz)*sin(cx)*sin(cy)) + cos(lx)*sin(ly)*(cos(cy)*cos(cz) 
               + sin(cx)*sin(cy)*sin(cz)) + cos(lx)*cos(ly)*cos(cx)*sin(cy);
  float crycrx = cos(lx)*cos(ly)*cos(cx)*cos(cy) - cos(lx)*sin(ly)*(cos(cz)*sin(cy) 
               - cos(cy)*sin(cx)*sin(cz)) - sin(lx)*(sin(cy)*sin(cz) + cos(cy)*cos(cz)*sin(cx));
  oy = atan2(srycrx / cos(ox), crycrx / cos(ox));

  float srzcrx = sin(cx)*(cos(lz)*sin(ly) - cos(ly)*sin(lx)*sin(lz)) + cos(cx)*sin(cz)*(cos(ly)*cos(lz) 
               + sin(lx)*sin(ly)*sin(lz)) + cos(lx)*cos(cx)*cos(cz)*sin(lz);
  float crzcrx = cos(lx)*cos(lz)*cos(cx)*cos(cz) - cos(cx)*sin(cz)*(cos(ly)*sin(lz) 
               - cos(lz)*sin(lx)*sin(ly)) - sin(cx)*(sin(ly)*sin(lz) + cos(ly)*cos(lz)*sin(lx));
  oz = atan2(srzcrx / cos(ox), crzcrx / cos(ox));
}

void diffRotation(float cx, float cy, float cz, float lx, float ly, float lz, 
                  float &ox, float &oy, float &oz)
{
  float srx = cos(cx)*cos(cy)*(sin(ly)*sin(lz) + cos(ly)*cos(lz)*sin(lx)) 
            - cos(cx)*sin(cy)*(cos(ly)*sin(lz) - cos(lz)*sin(lx)*sin(ly)) - cos(lx)*cos(lz)*sin(cx);
  ox = -asin(srx);

  float srycrx = cos(cx)*sin(cy)*(cos(ly)*cos(lz) + sin(lx)*sin(ly)*sin(lz)) 
               - cos(cx)*cos(cy)*(cos(lz)*sin(ly) - cos(ly)*sin(lx)*sin(lz)) - cos(lx)*sin(cx)*sin(lz);
  float crycrx = sin(cx)*sin(lx) + cos(cx)*cos(cy)*cos(lx)*cos(ly) + cos(cx)*cos(lx)*sin(cy)*sin(ly);
  oy = atan2(srycrx / cos(ox), crycrx / cos(ox));

  float srzcrx = cos(cx)*cos(lx)*cos(lz)*sin(cz) - (cos(cz)*sin(cy) 
               - cos(cy)*sin(cx)*sin(cz))*(sin(ly)*sin(lz) + cos(ly)*cos(lz)*sin(lx)) 
               - (cos(cy)*cos(cz) + sin(cx)*sin(cy)*sin(cz))*(cos(ly)*sin(lz) - cos(lz)*sin(lx)*sin(ly));
  float crzcrx = (sin(cy)*sin(cz) + cos(cy)*cos(cz)*sin(cx))*(sin(ly)*sin(lz) 
               + cos(ly)*cos(lz)*sin(lx)) + (cos(cy)*sin(cz) - cos(cz)*sin(cx)*sin(cy))*(cos(ly)*sin(lz) 
               - cos(lz)*sin(lx)*sin(ly)) + cos(cx)*cos(cz)*cos(lx)*cos(lz);
  oz = atan2(srzcrx / cos(ox), crzcrx / cos(ox));
}

void associateToInit(float ix, float iy, float iz, float pitch, float yaw, float roll, 
                     float &ox, float &oy, float &oz)
{
  float x1 = cos(roll) * ix - sin(roll) * iy;
  float y1 = sin(roll) * ix + cos(roll) * iy;
  float z1 = iz;

  float x2 = x1;
  float y2 = cos(pitch) * y1 - sin(pitch) * z1;
  float z2 = sin(pitch) * y1 + cos(pitch) * z1;

  ox = cos(yaw) * x2 + sin(yaw) * z2;
  oy = y2;
  oz = -sin(yaw) * x2 + cos(yaw) * z2;
}

void associateToCur(float ix, float iy, float iz, float pitch, float yaw, float roll, 
                    float &ox, float &oy, float &oz)
{
  float x1 = cos(yaw) * ix - sin(yaw) * iz;
  float y1 = iy;
  float z1 = sin(yaw) * ix + cos(yaw) * iz;

  float x2 = x1;
  float y2 = cos(pitch) * y1 + sin(pitch) * z1;
  float z2 = -sin(pitch) * y1 + cos(pitch) * z1;

  ox = cos(roll) * x2 + sin(roll) * y2;
  oy = -sin(roll) * x2 + cos(roll) * y2;
  oz = z2;
}

void transformAssociateToMap()
{
  float pitch = transformSum[0];
  float yaw = transformSum[1];
  float roll = transformSum[2];
  float tx = transformSum[3];
  float ty = transformSum[4];
  float tz = transformSum[5];

  cv::Mat mat_prior(4, 4, CV_64F, cv::Scalar::all(0));
  mat_prior.at<double>(0, 0) = cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw);
  mat_prior.at<double>(0, 1) = cos(roll)*sin(pitch)*sin(yaw) - cos(yaw)*sin(roll);
  mat_prior.at<double>(0, 2) = cos(pitch)*sin(yaw);
  mat_prior.at<double>(1, 0) = cos(pitch)*sin(roll);
  mat_prior.at<double>(1, 1) = cos(pitch)*cos(roll);
  mat_prior.at<double>(1, 2) = -sin(pitch);
  mat_prior.at<double>(2, 0) = cos(yaw)*sin(pitch)*sin(roll) - cos(roll)*sin(yaw);
  mat_prior.at<double>(2, 1) = sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch);
  mat_prior.at<double>(2, 2) = cos(pitch)*cos(yaw);
  mat_prior.at<double>(0, 3) = tx;
  mat_prior.at<double>(1, 3) = ty;
  mat_prior.at<double>(2, 3) = tz;
  mat_prior.at<double>(3, 3) = 1.0;

  pitch = transformBefMapped[0];
  yaw = transformBefMapped[1];
  roll = transformBefMapped[2];
  tx = transformBefMapped[3];
  ty = transformBefMapped[4];
  tz = transformBefMapped[5];

  cv::Mat mat_prior_last(4, 4, CV_64F, cv::Scalar::all(0));
  mat_prior_last.at<double>(0, 0) = cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw);
  mat_prior_last.at<double>(0, 1) = cos(roll)*sin(pitch)*sin(yaw) - cos(yaw)*sin(roll);
  mat_prior_last.at<double>(0, 2) = cos(pitch)*sin(yaw);
  mat_prior_last.at<double>(1, 0) = cos(pitch)*sin(roll);
  mat_prior_last.at<double>(1, 1) = cos(pitch)*cos(roll);
  mat_prior_last.at<double>(1, 2) = -sin(pitch);
  mat_prior_last.at<double>(2, 0) = cos(yaw)*sin(pitch)*sin(roll) - cos(roll)*sin(yaw);
  mat_prior_last.at<double>(2, 1) = sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch);
  mat_prior_last.at<double>(2, 2) = cos(pitch)*cos(yaw);
  mat_prior_last.at<double>(0, 3) = tx;
  mat_prior_last.at<double>(1, 3) = ty;
  mat_prior_last.at<double>(2, 3) = tz;
  mat_prior_last.at<double>(3, 3) = 1.0;

  pitch = transformAftMapped[0];
  yaw = transformAftMapped[1];
  roll = transformAftMapped[2];
  tx = transformAftMapped[3];
  ty = transformAftMapped[4];
  tz = transformAftMapped[5];

  cv::Mat mat_post_last(4, 4, CV_64F, cv::Scalar::all(0));
  mat_post_last.at<double>(0, 0) = cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw);
  mat_post_last.at<double>(0, 1) = cos(roll)*sin(pitch)*sin(yaw) - cos(yaw)*sin(roll);
  mat_post_last.at<double>(0, 2) = cos(pitch)*sin(yaw);
  mat_post_last.at<double>(1, 0) = cos(pitch)*sin(roll);
  mat_post_last.at<double>(1, 1) = cos(pitch)*cos(roll);
  mat_post_last.at<double>(1, 2) = -sin(pitch);
  mat_post_last.at<double>(2, 0) = cos(yaw)*sin(pitch)*sin(roll) - cos(roll)*sin(yaw);
  mat_post_last.at<double>(2, 1) = sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch);
  mat_post_last.at<double>(2, 2) = cos(pitch)*cos(yaw);
  mat_post_last.at<double>(0, 3) = tx;
  mat_post_last.at<double>(1, 3) = ty;
  mat_post_last.at<double>(2, 3) = tz;
  mat_post_last.at<double>(3, 3) = 1.0;

  cv::Mat mat_frame = mat_prior_last.inv() * mat_prior;
  cv::Mat mat_post = mat_post_last * mat_frame;

  pitch = -asin(mat_post.at<double>(1, 2));
  roll = atan2(mat_post.at<double>(1, 0) / cos(pitch), mat_post.at<double>(1, 1) / cos(pitch));
  yaw = atan2(mat_post.at<double>(0, 2) / cos(pitch), mat_post.at<double>(2, 2) / cos(pitch));
  tx = mat_post.at<double>(0, 3);
  ty = mat_post.at<double>(1, 3);
  tz = mat_post.at<double>(2, 3);

  transformMapped[0] = pitch;
  transformMapped[1] = yaw;
  transformMapped[2] = roll;
  transformMapped[3] = tx;
  transformMapped[4] = ty;
  transformMapped[5] = tz;
}

void imuHandler(const sensor_msgs::Imu::ConstPtr& imuData)
{
  double imuTimeCur = imuData->header.stamp.toSec();

  double roll, pitch, yaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(imuData->orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  imuPointerLast = (imuPointerLast + 1) % imuQueLength;
  int imuPointerBack = (imuPointerLast + imuQueLength - 1) % imuQueLength;
  float timeDiff = imuTimeCur - imuTime[imuPointerBack];
  float drx, dry, drz;
  if (timeDiff < 1.0) {
    drx = imuData->angular_velocity.y * timeDiff;
    dry = imuData->angular_velocity.z * timeDiff;
    drz = imuData->angular_velocity.x * timeDiff;
    accuRotation(imuPitchRec, imuYawAccu, imuRollRec, drx, dry, drz, imuPitchRec, imuYawAccu, imuRollRec);
  }

  float accX = imuData->linear_acceleration.y - sin(roll) * cos(pitch) * 9.81;
  float accY = imuData->linear_acceleration.z - cos(roll) * cos(pitch) * 9.81;
  float accZ = imuData->linear_acceleration.x + sin(pitch) * 9.81;

  if (!useImuAcceleration) {
    accX = 0;
    accY = 0;
    accZ = 0;
  }

  imuTime[imuPointerLast] = imuTimeCur;
  imuRoll[imuPointerLast] = roll;
  imuPitch[imuPointerLast] = pitch;
  imuYaw[imuPointerLast] = yaw; //imuYawAccu;
  imuAccX[imuPointerLast] = accX;
  imuAccY[imuPointerLast] = accY;
  imuAccZ[imuPointerLast] = accZ;

  imuRollRec = roll;
  imuPitchRec = pitch;

  if (!odomInit) {
    return;
  }

  float roll2, pitch2, yaw2;
  diffRotation(pitch, yaw /*imuYawAccu*/, roll, imuPitchCur, imuYawCur, imuRollCur, drx, dry, drz);
  accuRotation(odomPitchCur, odomYawCur, odomRollCur, drx, dry, drz, pitch2, yaw2, roll2);

  float accX2, accY2, accZ2;
  associateToInit(accX, accY, accZ, pitch2, yaw2, roll2, accX2, accY2, accZ2);

  //imuShiftX += imuVeloX * timeDiff + accX2 * timeDiff * timeDiff / 2;
  //imuShiftY += imuVeloY * timeDiff + accY2 * timeDiff * timeDiff / 2;
  //imuShiftZ += imuVeloZ * timeDiff + accZ2 * timeDiff * timeDiff / 2;

  imuShiftX += imuVeloSX2 * timeDiff;
  imuShiftY += imuVeloSY2 * timeDiff;
  imuShiftZ += imuVeloSZ2 * timeDiff;

  imuShiftSX = 0.9 * imuShiftSX + 0.1 * imuShiftX;
  imuShiftSY = 0.9 * imuShiftSY + 0.1 * imuShiftY;
  imuShiftSZ = 0.9 * imuShiftSZ + 0.1 * imuShiftZ;

  imuVeloX += accX2 * timeDiff;
  imuVeloY += accY2 * timeDiff;
  imuVeloZ += accZ2 * timeDiff;

  imuVeloSX2 = 0.95 * imuVeloSX2 + 0.05 * imuVeloX;
  imuVeloSY2 = 0.95 * imuVeloSY2 + 0.05 * imuVeloY;
  imuVeloSZ2 = 0.95 * imuVeloSZ2 + 0.05 * imuVeloZ;

  associateToCur(imuVeloSX2, imuVeloSY2, imuVeloSZ2, pitch2, yaw2, roll2, imuVeloSX3, imuVeloSY3, imuVeloSZ3);

  imuVeloSX4 = 0.9 * imuVeloSX4 + 0.1 * imuVeloSX3;
  imuVeloSY4 = 0.9 * imuVeloSY4 + 0.1 * imuVeloSY3;
  imuVeloSZ4 = 0.9 * imuVeloSZ4 + 0.1 * imuVeloSZ3;

  geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(roll2, pitch2, yaw2);

  odometry.header.stamp = imuData->header.stamp;
  odometryTrans.stamp_ = imuData->header.stamp;

  odometry.pose.pose.orientation.x = geoQuat.x;
  odometry.pose.pose.orientation.y = geoQuat.y;
  odometry.pose.pose.orientation.z = geoQuat.z;
  odometry.pose.pose.orientation.w = geoQuat.w;
  odometry.pose.pose.position.x = imuShiftSZ;
  odometry.pose.pose.position.y = imuShiftSX;
  odometry.pose.pose.position.z = imuShiftSY;
  odometry.twist.twist.angular.x = imuData->angular_velocity.x;
  odometry.twist.twist.angular.y = imuData->angular_velocity.y;
  odometry.twist.twist.angular.z = imuData->angular_velocity.z;
  odometry.twist.twist.linear.x = imuVeloSZ4;
  odometry.twist.twist.linear.y = imuVeloSX4;
  odometry.twist.twist.linear.z = imuVeloSY4;

  odometryTrans.setRotation(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));
  odometryTrans.setOrigin(tf::Vector3(imuShiftSZ, imuShiftSX, imuShiftSY));

  imuRollCov = roll2;
  imuPitchCov = pitch2;
  imuYawCov = yaw2;
  imuShiftCovX = imuShiftSX;
  imuShiftCovY = imuShiftSY;
  imuShiftCovZ = imuShiftSZ;

  pubOdometryPointer->publish(odometry);
  tfBroadcasterPointer->sendTransform(odometryTrans);
}

void odometryHandler(const nav_msgs::Odometry::ConstPtr& odometryIn)
{
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odometryIn->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

  transformAssociateToMap();

  odomPitchLast = transformMapped[0];
  odomYawLast = transformMapped[1];
  odomRollLast = transformMapped[2];
  odomShiftXLast = transformMapped[3];
  odomShiftYLast = transformMapped[4];
  odomShiftZLast = transformMapped[5];

  transformSum[0] = -pitch;
  transformSum[1] = -yaw;
  transformSum[2] = roll;

  transformSum[3] = odometryIn->pose.pose.position.x;
  transformSum[4] = odometryIn->pose.pose.position.y;
  transformSum[5] = odometryIn->pose.pose.position.z;

  transformAssociateToMap();

  odomPitchCur = transformMapped[0];
  odomYawCur = transformMapped[1];
  odomRollCur = transformMapped[2];
  odomShiftXCur = transformMapped[3];
  odomShiftYCur = transformMapped[4];
  odomShiftZCur = transformMapped[5];

  odomTimeLast = odomTimeCur;
  odomTimeCur = odometryIn->header.stamp.toSec() + scanPeriod;

  float timeDiff = odomTimeCur - odomTimeLast;

  if (imuPointerLast >= 0) {
    imuRollLast = imuRollCur;
    imuPitchLast = imuPitchCur;
    imuYawLast = imuYawCur;
    imuAccXLast = imuAccXCur;
    imuAccYLast = imuAccYCur;
    imuAccZLast = imuAccZCur;

    imuAccSamNum = 1;
    imuTimeSam[0] = odomTimeLast;
    imuRollSam[0] = imuRollLast;
    imuPitchSam[0] = imuPitchLast;
    imuYawSam[0] = imuYawLast;
    imuAccXSam[0] = imuAccXLast;
    imuAccYSam[0] = imuAccYLast;
    imuAccZSam[0] = imuAccZLast;

    while (imuPointerFront != imuPointerLast) {
      if (odomTimeCur < imuTime[imuPointerFront]) {
        break;
      }

      imuTimeSam[imuAccSamNum] = imuTime[imuPointerFront];
      imuRollSam[imuAccSamNum] = imuRoll[imuPointerFront];
      imuPitchSam[imuAccSamNum] = imuPitch[imuPointerFront];
      imuYawSam[imuAccSamNum] = imuYaw[imuPointerFront];
      imuAccXSam[imuAccSamNum] = imuAccX[imuPointerFront];
      imuAccYSam[imuAccSamNum] = imuAccY[imuPointerFront];
      imuAccZSam[imuAccSamNum] = imuAccZ[imuPointerFront];
      imuAccSamNum++;

      imuPointerFront = (imuPointerFront + 1) % imuQueLength;
    }

    if (odomTimeCur > imuTime[imuPointerFront]) {
      imuRollCur = imuRoll[imuPointerFront];
      imuPitchCur = imuPitch[imuPointerFront];
      imuYawCur = imuYaw[imuPointerFront];
      imuAccXCur = imuAccX[imuPointerFront];
      imuAccYCur = imuAccY[imuPointerFront];
      imuAccZCur = imuAccZ[imuPointerFront];
    } else {
      int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
      float ratioFront = (odomTimeCur - imuTime[imuPointerBack]) 
                       / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
      float ratioBack = (imuTime[imuPointerFront] - odomTimeCur) 
                      / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);

      if (fabs(imuYaw[imuPointerFront] - imuYaw[imuPointerBack]) > PI / 2 && 
          fabs(imuYaw[imuPointerFront] - imuYaw[imuPointerBack] + 2 * PI) > PI / 2 && 
          fabs(imuYaw[imuPointerFront] - imuYaw[imuPointerBack] - 2 * PI) > PI / 2) {
        if (imuRoll[imuPointerBack] > 0) {
          imuRoll[imuPointerBack] -= PI;
        } else {
          imuRoll[imuPointerBack] += PI;
        }
        if (imuPitch[imuPointerBack] > 0) {
          imuPitch[imuPointerBack] = -imuPitch[imuPointerBack] + PI;
        } else {
          imuPitch[imuPointerBack] = -imuPitch[imuPointerBack] - PI;
        }
        if (imuYaw[imuPointerBack] > 0) {
          imuYaw[imuPointerBack] -= PI;
        } else {
          imuYaw[imuPointerBack] += PI;
        }
      }

      if (imuRoll[imuPointerFront] - imuRoll[imuPointerBack] > PI) {
        imuRoll[imuPointerBack] += 2 * PI;
      } else if (imuRoll[imuPointerFront] - imuRoll[imuPointerBack] < -PI) {
        imuRoll[imuPointerBack] -= 2 * PI;
      }
      if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] > PI) {
        imuYaw[imuPointerBack] += 2 * PI;
      } else if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] < -PI) {
        imuYaw[imuPointerBack] -= 2 * PI;
      }

      imuRollCur = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;
      imuPitchCur = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;
      imuYawCur = imuYaw[imuPointerFront] * ratioFront + imuYaw[imuPointerBack] * ratioBack;

      imuAccXCur = imuAccX[imuPointerFront] * ratioFront + imuAccX[imuPointerBack] * ratioBack;
      imuAccYCur = imuAccY[imuPointerFront] * ratioFront + imuAccY[imuPointerBack] * ratioBack;
      imuAccZCur = imuAccZ[imuPointerFront] * ratioFront + imuAccZ[imuPointerBack] * ratioBack;
    }

    imuTimeSam[imuAccSamNum] = odomTimeCur;
    imuRollSam[imuAccSamNum] = imuRollCur;
    imuPitchSam[imuAccSamNum] = imuPitchCur;
    imuYawSam[imuAccSamNum] = imuYawCur;
    imuAccXSam[imuAccSamNum] = imuAccXCur;
    imuAccYSam[imuAccSamNum] = imuAccYCur;
    imuAccZSam[imuAccSamNum] = imuAccZCur;
    imuAccSamNum++;

    if (odomTimeLast > 0.005) {
      float imudrx, imudry, imudrz;
      diffRotation(imuPitchCur, imuYawCur, imuRollCur, imuPitchLast, imuYawLast, imuRollLast, 
                   imudrx, imudry, imudrz);
      float odomdrx, odomdry, odomdrz;
      diffRotation(odomPitchCur, odomYawCur, odomRollCur, odomPitchLast, odomYawLast, odomRollLast, 
                   odomdrx, odomdry, odomdrz);

      float biasrx = (odomdrx - imudrx) / timeDiff;
      float biasry = (odomdry - imudry) / timeDiff;
      float biasrz = (odomdrz - imudrz) / timeDiff;

      float roll2, pitch2, yaw2;
      float accX, accY, accZ;
      float veloX = 0, veloY = 0, veloZ = 0;
      float shiftX = 0, shiftY = 0, shiftZ = 0;
      for (int i = 1; i < imuAccSamNum; i++) {
        float timeDiff2 = imuTimeSam[i] - odomTimeLast;

        diffRotation(imuPitchSam[i], imuYawSam[i], imuRollSam[i], 
                     imuPitchLast, imuYawLast, imuRollLast, imudrx, imudry, imudrz);
        imudrx += biasrx * timeDiff2;
        imudry += biasry * timeDiff2;
        imudrz += biasrz * timeDiff2;
        accuRotation(odomPitchLast, odomYawLast, odomRollLast, imudrx, imudry, imudrz, pitch2, yaw2, roll2);

        float timeDiff3 = imuTimeSam[i] - imuTimeSam[i - 1];

        associateToInit(imuAccXSam[i], imuAccYSam[i], imuAccZSam[i], pitch2, yaw2, roll2, accX, accY, accZ);

        shiftX += veloX * timeDiff3 + accX * timeDiff3 * timeDiff3 / 2;
        shiftY += veloY * timeDiff3 + accY * timeDiff3 * timeDiff3 / 2;
        shiftZ += veloZ * timeDiff3 + accZ * timeDiff3 * timeDiff3 / 2;

        veloX += accX * timeDiff3;
        veloY += accY * timeDiff3;
        veloZ += accZ * timeDiff3;
      }

      imuVeloX = (odomShiftXCur - odomShiftXLast - shiftX) / timeDiff + veloX;
      imuVeloY = (odomShiftYCur - odomShiftYLast - shiftY) / timeDiff + veloY;
      imuVeloZ = (odomShiftZCur - odomShiftZLast - shiftZ) / timeDiff + veloZ;

      if (!odomInit) {
        imuVeloSX1 = imuVeloX;
        imuVeloSY1 = imuVeloY;
        imuVeloSZ1 = imuVeloZ;

        odomInit = true;
      } else {
        imuVeloSX1 = 0.8 * imuVeloSX1 + 0.2 * imuVeloX;
        imuVeloSY1 = 0.8 * imuVeloSY1 + 0.2 * imuVeloY;
        imuVeloSZ1 = 0.8 * imuVeloSZ1 + 0.2 * imuVeloZ;
      }

      imuVeloSX2 = imuVeloSX1;
      imuVeloSY2 = imuVeloSY1;
      imuVeloSZ2 = imuVeloSZ1;

      float timeDiff3 = imuTime[imuPointerFront] - odomTimeCur;

      diffRotation(imuPitch[imuPointerFront], imuYaw[imuPointerFront], imuRoll[imuPointerFront], 
                   imuPitchCur, imuYawCur, imuRollCur, imudrx, imudry, imudrz);
      accuRotation(odomPitchCur, odomYawCur, odomRollCur, imudrx, imudry, imudrz, pitch2, yaw2, roll2);

      associateToInit(imuAccX[imuPointerFront], imuAccY[imuPointerFront], imuAccZ[imuPointerFront], pitch2, yaw2, roll2,
                      accX, accY, accZ);

      //imuShiftX = odomShiftXCur + imuVeloX * timeDiff3 + accX * timeDiff3 * timeDiff3 / 2;
      //imuShiftY = odomShiftYCur + imuVeloY * timeDiff3 + accY * timeDiff3 * timeDiff3 / 2;
      //imuShiftZ = odomShiftZCur + imuVeloZ * timeDiff3 + accZ * timeDiff3 * timeDiff3 / 2;

      imuShiftX = odomShiftXCur + imuVeloSX2 * timeDiff3;
      imuShiftY = odomShiftYCur + imuVeloSY2 * timeDiff3;
      imuShiftZ = odomShiftZCur + imuVeloSZ2 * timeDiff3;

      imuVeloX += accX * timeDiff3;
      imuVeloY += accY * timeDiff3;
      imuVeloZ += accZ * timeDiff3;

      imuVeloSX2 = 0.95 * imuVeloSX2 + 0.05 * imuVeloX;
      imuVeloSY2 = 0.95 * imuVeloSY2 + 0.05 * imuVeloY;
      imuVeloSZ2 = 0.95 * imuVeloSZ2 + 0.05 * imuVeloZ;

      int startID = (imuPointerFront + 1) % imuQueLength;
      int endID = (imuPointerLast + 1) % imuQueLength;
      for (int i = startID; i != endID; i = (i + 1) % imuQueLength) {
        int j = (i + imuQueLength - 1) % imuQueLength;
        float timeDiff3 = imuTime[i] - imuTime[j];

        diffRotation(imuPitch[i], imuYaw[i], imuRoll[i], imuPitchCur, imuYawCur, imuRollCur, imudrx, imudry, imudrz);
        accuRotation(odomPitchCur, odomYawCur, odomRollCur, imudrx, imudry, imudrz, pitch2, yaw2, roll2);

        associateToInit(imuAccX[i], imuAccY[i], imuAccZ[i], pitch2, yaw2, roll2, accX, accY, accZ);

        //imuShiftX += imuVeloX * timeDiff3 + accX * timeDiff3 * timeDiff3 / 2;
        //imuShiftY += imuVeloY * timeDiff3 + accY * timeDiff3 * timeDiff3 / 2;
        //imuShiftZ += imuVeloZ * timeDiff3 + accZ * timeDiff3 * timeDiff3 / 2;

        imuShiftX += imuVeloSX2 * timeDiff3;
        imuShiftY += imuVeloSY2 * timeDiff3;
        imuShiftZ += imuVeloSZ2 * timeDiff3;

        imuVeloX += accX * timeDiff3;
        imuVeloY += accY * timeDiff3;
        imuVeloZ += accZ * timeDiff3;

        imuVeloSX2 = 0.95 * imuVeloSX2 + 0.05 * imuVeloX;
        imuVeloSY2 = 0.95 * imuVeloSY2 + 0.05 * imuVeloY;
        imuVeloSZ2 = 0.95 * imuVeloSZ2 + 0.05 * imuVeloZ;
      }
    }
  }
}

void mappingHandler(const nav_msgs::Odometry::ConstPtr& mappingIn)
{
  if (mappingIn->header.stamp.toSec() > aftMappedTime) {
    aftMappedTime = mappingIn->header.stamp.toSec();

    double roll, pitch, yaw;
    geometry_msgs::Quaternion geoQuat = mappingIn->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

    transformAftMapped[0] = -pitch;
    transformAftMapped[1] = -yaw;
    transformAftMapped[2] = roll;

    transformAftMapped[3] = mappingIn->pose.pose.position.x;
    transformAftMapped[4] = mappingIn->pose.pose.position.y;
    transformAftMapped[5] = mappingIn->pose.pose.position.z;

    transformBefMapped[0] = mappingIn->twist.twist.angular.x;
    transformBefMapped[1] = mappingIn->twist.twist.angular.y;
    transformBefMapped[2] = mappingIn->twist.twist.angular.z;

    transformBefMapped[3] = mappingIn->twist.twist.linear.x;
    transformBefMapped[4] = mappingIn->twist.twist.linear.y;
    transformBefMapped[5] = mappingIn->twist.twist.linear.z;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "transformMaintenance");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("useImuAcceleration", useImuAcceleration);

  ros::Subscriber subImu = nh.subscribe<sensor_msgs::Imu> ("/imu/data", 50, imuHandler);

  ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry> ("/laser_odom_to_init", 5, odometryHandler);

  ros::Subscriber subMapping = nh.subscribe<nav_msgs::Odometry> ("/aft_mapped_to_init", 5, mappingHandler);

  ros::Publisher pubOdometry = nh.advertise<nav_msgs::Odometry> ("/integrated_to_init", 50);
  pubOdometryPointer = &pubOdometry;
  odometry.header.frame_id = "sensor_init";
  odometry.child_frame_id = "sensor";

  tf::TransformBroadcaster tfBroadcaster;
  tfBroadcasterPointer = &tfBroadcaster;
  odometryTrans.frame_id_ = "sensor_init";
  odometryTrans.child_frame_id_ = "sensor";

  ros::spin();

  return 0;
}
