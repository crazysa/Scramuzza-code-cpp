/*------------------------------------------------------------------------------
   Example code that shows the use of the 'cam2world" and 'world2cam" functions
   Shows also how to undistort images into perspective or panoramic images

   NOTE, IF YOU WANT TO SPEED UP THE REMAP FUNCTION I STRONGLY RECOMMEND TO INSTALL
   INTELL IPP LIBRARIES ( http://software.intel.com/en-us/intel-ipp/ )
   YOU JUST NEED TO INSTALL IT AND INCLUDE ipp.h IN YOUR PROGRAM

   Copyright (C) 2009 DAVIDE SCARAMUZZA, ETH Zurich
   Author: Davide Scaramuzza - email: davide.scaramuzza@ieee.org
------------------------------------------------------------------------------*/

#include "ocam_functions.h"
#include <string>

using namespace std;

int main(int argc, char *argv[])
{
  /* --------------------------------------------------------------------*/
  /* Read the parameters of the omnidirectional camera from the TXT file */
  /* --------------------------------------------------------------------*/
  struct ocam_model o, o_cata; // our ocam_models for the fisheye and catadioptric cameras
  std::string fileName = "/home/shubham/Desktop/deepentools/stats/server/Scripts/loam_velodyne_pcd/src/Ftheta/calib_results_fisheye.txt";
  get_ocam_model(&o, fileName);
  /* --------------------------------------------------------------------*/
  /* Print ocam_model parameters                                         */
  /* --------------------------------------------------------------------*/
  int i;
  printf("pol =\n");
  for (i = 0; i < o.length_pol; i++)
  {
    printf("\t%e\n", o.pol[i]);
  };
  printf("\n");
  printf("invpol =\n");
  for (i = 0; i < o.length_invpol; i++)
  {
    printf("\t%e\n", o.invpol[i]);
  };
  printf("\n");
  printf("\nxc = %f\nyc = %f\n\nwidth = %d\nheight = %d\n", o.xc, o.yc, o.width, o.height);

  /* --------------------------------------------------------------------*/
  /* WORLD2CAM projects 3D point into the image                          */
  /* NOTE!!! The coordinates are expressed according the C convention,   */
  /* that is, from the origin (0,0) instead than from 1 (MATLAB).        */
  /* --------------------------------------------------------------------*/
  double point3D[3] = {100, 200, -300}; // a sample 3D point
  double point2D[2];                    // the image point in pixel coordinates
  world2cam(point2D, point3D, &o);      // The behaviour of this function is the same as in MATLAB

  /* --------------------------------------------------------------------*/
  /* Display re-projected coordinates                                    */
  /* --------------------------------------------------------------------*/
  printf("\nworld2cam: pixel coordinates reprojected onto the image\n");
  printf("m_row= %2.4f, m_col=%2.4f\n", point2D[0], point2D[1]);

  /* --------------------------------------------------------------------*/
  /* CAM2WORLD back-projects pixel points on to the unit sphere          */
  /* The behaviour of this function is the same as in MATLAB             */
  /* --------------------------------------------------------------------*/

  cam2world(point3D, point2D, &o);

  /* --------------------------------------------------------------------*/
  /* Display back-projected normalized coordinates (on the unit sphere)  */
  /* --------------------------------------------------------------------*/
  printf("\ncam2world: coordinates back-projected onto the unit sphere (x^2+y^2+z^2=1)\n");
  printf("x= %2.4f, y=%2.4f, z=%2.4f\n", point3D[0], point3D[1], point3D[2]);

  /* --------------------------------------------------------------------*/
  /* Allocate space for the unistorted images                            */
  /* --------------------------------------------------------------------*/
  cv::Mat src1 = cv::imread("/home/shubham/Desktop/deepentools/stats/server/Scripts/loam_velodyne_pcd/src/Ftheta/test_fisheye.jpg"); // source image 1
  // cv::Mat src2 = cv::imread("/home/shubham/Desktop/deepentools/stats/server/Scripts/loam_velodyne_pcd/src/Ftheta/test_catadioptric.jpg"); // source image 2
  cv::Mat dst_persp = cv::Mat(src1.size(),CV_8UC1); // undistorted perspective and panoramic image
  // CvSize size_pan_image = cvSize(1200, 400);                  // size of the undistorted panoramic image
  // cv::Mat dst_pan = cv::Mat(size_pan_image, CV_8UC3);         // undistorted panoramic image

  // CvMat *mapx_persp = cvCreateMat(src1.rows, src1.cols, CV_32FC1);
  // CvMat *mapy_persp = cvCreateMat(src1.rows, src1.cols, CV_32FC1);

  cv::Mat mapx_persp(src1.size(), CV_32FC1);
  cv::Mat mapy_persp(src1.size(), CV_32FC1);

  // cv::Mat mapx_pan(dst_pan.size(), CV_32FC1);
  // cv::Mat mapy_pan(dst_pan.size(), CV_32FC1);

  // CvMat *mapx_pan = cvCreateMat(dst_pan.rows, dst_pan.cols, CV_32FC1);
  // CvMat *mapy_pan = cvCreateMat(dst_pan.rows, dst_pan.cols, CV_32FC1);

  /* --------------------------------------------------------------------  */
  /* Create Look-Up-Table for perspective undistortion                     */
  /* SF is kind of distance from the undistorted image to the camera       */
  /* (it is not meters, it is justa zoom fator)                            */
  /* Try to change SF to see how it affects the result                     */
  /* The undistortion is done on a  plane perpendicular to the camera axis */
  /* --------------------------------------------------------------------  */
  float sf = 4;
  create_perspecive_undistortion_LUT(mapx_persp, mapy_persp, &o, sf);

  /* --------------------------------------------------------------------  */
  /* Create Look-Up-Table for panoramic undistortion                       */
  /* The undistortoin is just a simple cartesia-to-polar transformation    */
  /* Note, only the knowledge of image center (xc,yc) is used to undisort the image      */
  /* xc, yc are the row and column coordinates of the image center         */
  /* Note, if you would like to flip the image, just inverte the sign of theta in this function */
  /* --------------------------------------------------------------------  */
  // float Rmax = 470; // the maximum    nb  radius of the region you would like to undistort into a panorama
  // float Rmin = 20;  // the minimum radius of the region you would like to undistort into a panorama
  // create_panoramic_undistortion_LUT(mapx_pan, mapy_pan, Rmin, Rmax, o_cata.xc, o_cata.yc);

  /* --------------------------------------------------------------------*/
  /* Undistort using specified interpolation method                      */
  /* Other possible values are (see OpenCV doc):                         */
  /* CV_INTER_NN - nearest-neighbor interpolation,                       */
  /* CV_INTER_LINEAR - bilinear interpolation (used by default)          */
  /* CV_INTER_AREA - resampling using pixel area relation. It is the preferred method for image decimation that gives moire-free results. In case of zooming it is similar to CV_INTER_NN method. */
  /* CV_INTER_CUBIC - bicubic interpolation.                             */
  /* --------------------------------------------------------------------*/
  cv::remap(src1, dst_persp, mapx_persp, mapy_persp, cv::INTER_LINEAR,cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
  // cv::remap(src2, dst_pan, mapx_pan, mapy_pan, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

  /* --------------------------------------------------------------------*/
  /* Display image                                                       */
  /* --------------------------------------------------------------------*/
  cv::namedWindow("Original fisheye camera image", cv::WINDOW_AUTOSIZE);
  cv::imshow("Original fisheye camera image", src1);

  cv::namedWindow("Undistorted Perspective Image", cv::WINDOW_AUTOSIZE);
  cv::imshow("Undistorted Perspective Image", dst_persp);

  // cv::namedWindow("Original Catadioptric camera image", cv::WINDOW_AUTOSIZE);
  // cv::imshow("Original Catadioptric camera image", src2);

  // cv::namedWindow("Undistorted Panoramic Image", cv::WINDOW_AUTOSIZE);
  // cv::imshow("Undistorted Panoramic Image", dst_pan);

  /* --------------------------------------------------------------------*/
  /* Save image                                                          */
  /* --------------------------------------------------------------------*/
  cv::imwrite("undistorted_perspective.jpg", dst_persp);
  printf("\nImage %s saved\n", "undistorted_perspective.jpg");

  // cv::imwrite("undistorted_panoramic.jpg", dst_pan);
  // printf("\nImage %s saved\n", "undistorted_panoramic.jpg");

  /* --------------------------------------------------------------------*/
  /* Wait until key presses                                              */
  /* --------------------------------------------------------------------*/
  cvWaitKey();

  /* --------------------------------------------------------------------*/
  /* Free memory                                                         */
  /* --------------------------------------------------------------------*/
  // cvReleaseImage(&src1);
  // cvReleaseImage(&src2);
  // cvReleaseImage(&dst_persp);
  // cvReleaseImage(&dst_pan);
  // cvReleaseMat(&mapx_persp);
  // cvReleaseMat(&mapy_persp);
  // cvReleaseMat(&mapx_pan);
  // cvReleaseMat(&mapy_pan);
  // cv::Mat::release();

  return 0;
}
