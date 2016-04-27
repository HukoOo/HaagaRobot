#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#define TOPVIEW true


using namespace std;
using namespace cv;

void readData_R(const FileStorage& node);
void readData_L(const FileStorage& node);
void readData_H(const FileStorage& node);

cv::Mat right_H, left_H;
cv::Mat topview_L, topview_R;
cv::Mat cameraMatrix_R, cameraMatrix_L;
cv::Mat distCoeffs_R, distCoeffs_L;
cv::Mat extrinsicParam_R, extrinsicParam_L;
int imageWidth, imageHeight;


int main(int argc, char** argv)
{
	const string camera_light_file ="calibration/right_camera.xml";
	const string camera_left_file = "calibration/left_camera.xml";
	const string topview_file = "calibration/H.xml";

	FileStorage fs_R(camera_light_file, FileStorage::READ); // Read the settings
	FileStorage fs_L(camera_light_file, FileStorage::READ);
	FileStorage fs_H(topview_file, FileStorage::READ);

	readData_R(fs_R);
	readData_L(fs_L);
	readData_H(fs_H);


	cv::VideoCapture cap_R(0);
	cv::VideoCapture cap_L(2);
	if (!cap_R.isOpened() || !cap_L.isOpened())
		return -1;

	cap_R.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
	cap_R.set(CV_CAP_PROP_FRAME_WIDTH, imageWidth);
	cap_R.set(CV_CAP_PROP_FRAME_HEIGHT, imageHeight);
	cap_L.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
	cap_L.set(CV_CAP_PROP_FRAME_WIDTH, imageWidth);
	cap_L.set(CV_CAP_PROP_FRAME_HEIGHT, imageHeight);


	cv::Mat frame_R,frame_L;
	cv::Mat roi_R, roi_L;
	cv::Mat undistorted_frame_R, undistorted_frame_L;

	Mat topview = Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
	while (1)
	{
		cap_R >> frame_R;
		cap_L >> frame_L;
		int scale = 1;

		Mat frame_chess;

		Mat newCamMat_R, newCamMat_L;
		float dummy_query_data_R[10] = { 156.00713303635887, 0, 139.5047668805756, 0, 155.83043278608287, 305.505831367789, 0, 0, 1 };
		float dummy_query_data_L[10] = { 156.00713303635887, 0, 439.5047668805756, 0, 155.83043278608287, 305.505831367789, 0, 0, 1 };


		newCamMat_R = cv::Mat(3, 3, CV_32F, dummy_query_data_R);
		newCamMat_L = cv::Mat(3, 3, CV_32F, dummy_query_data_L);

		fisheye::undistortImage(frame_R, undistorted_frame_R, cameraMatrix_R, distCoeffs_R, newCamMat_R, cv::Size(frame_R.cols*scale, frame_R.rows*scale));
		fisheye::undistortImage(frame_L, undistorted_frame_L, cameraMatrix_L, distCoeffs_L, newCamMat_L, cv::Size(frame_L.cols * scale, frame_L.rows * scale));
		
	
		if (!TOPVIEW) {
			cv::imshow("camera_R", frame_R);
			cv::imshow("camera_L", frame_L);
			cv::imshow("undistorted_R", undistorted_frame_R);
			cv::imshow("undistorted_L", undistorted_frame_L);
		}
		else 
		{
			// Warping
			cv::warpPerspective(undistorted_frame_R, topview_R, right_H, cv::Size(frame_R.cols, frame_R.rows));
			cv::warpPerspective(undistorted_frame_L, topview_L, left_H, cv::Size(frame_L.cols, frame_L.rows));

			// ROI copy
			roi_R = topview_R(cv::Rect(topview_R.cols / 2, 0, topview_R.cols / 2, topview_R.rows));
			roi_L = topview_L(cv::Rect(0, 0, topview_R.cols / 2, topview_R.rows));
			roi_R.copyTo(topview(cv::Rect(topview.cols / 2, 0, topview.cols / 2, topview.rows)));
			roi_L.copyTo(topview(cv::Rect(0, 0, topview_R.cols / 2, topview_R.rows)));

			cv::imshow("topview", topview);
		}
		char keyInput = (char)cv::waitKey(10);
		if( keyInput == 'q')
			break;
		else if (keyInput == 's')
		{
			cout << "Image saved" << endl;
			cv::imwrite("distored_image_R.bmp", frame_R);
			cv::imwrite("undistored_image_R.bmp", undistorted_frame_R);

			cv::imwrite("distored_image_L.bmp", frame_L);
			cv::imwrite("undistored_image_L.bmp", undistorted_frame_L);
		}
	}
	cv::destroyAllWindows();

	return 0;
}

void readData_R(const FileStorage& node)                          //Read serialization for this class
{
	node["image_width"] >> imageWidth;
	node["image_height"] >> imageHeight;
	node["camera_matrix"] >> cameraMatrix_R;
	node["distortion_coefficients"] >> distCoeffs_R;
	node["extrinsic_parameters"] >> extrinsicParam_R;
}
void readData_L(const FileStorage& node)                         
{
	node["image_width"] >> imageWidth;
	node["image_height"] >> imageHeight;
	node["camera_matrix"] >> cameraMatrix_L;
	node["distortion_coefficients"] >> distCoeffs_L;
	node["extrinsic_parameters"] >> extrinsicParam_L;
}
void readData_H(const FileStorage& node)                          
{
	node["RightH"] >> right_H;
	node["LeftH"] >> left_H;
}