
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include "IPM.h"

using namespace std;
using namespace cv;



Mat cameraMatrix_R, cameraMatrix_L;
Mat distCoeffs_R, distCoeffs_L;
Mat extrinsicParam_R, extrinsicParam_L;
int imageWidth, imageHeight;
cv::Size boardSize(9, 6);
bool found;

vector<Point2f> p_left_obj, p_right_obj;
vector<Point2f> p_left_dst, p_right_dst;
vector<Point2f> p_right_tmp, p_left_tmp;
vector<cv::Scalar> circleColor;

void writeData(FileStorage& fs, cv::Mat right_H, cv::Mat left_H);
void readData_R(const FileStorage& node);
void readData_L(const FileStorage& node);
void setPts(const FileStorage& node);

int main(int argc, char** argv)
{
	Mat blackWindow = Mat(480, 640, CV_8UC3, cv::Scalar(0,0,0));
	const string camera_light_file = argv[1];
	const string camera_left_file = argv[2];
	const string pt_lists = argv[5];
	FileStorage fs_R(camera_light_file, FileStorage::READ); // Read the settings
	FileStorage fs_L(camera_light_file, FileStorage::READ);
	FileStorage fs_pts(pt_lists, FileStorage::READ);
	readData_R(fs_R);
	readData_L(fs_L);

	const string homography_data_file =  "H.xml";
	FileStorage fs(homography_data_file, FileStorage::WRITE); // Write H matrix 

	cv::Mat front,rear,right,left;
	right = cv::imread(argv[3]);
	left = cv::imread(argv[4]);

	cv::Mat undistorted_front, undistorted_rear, undistorted_right, undistorted_left;
	cv::Mat top_front,top_rear,top_right,top_left;

	circleColor.push_back(cv::Scalar(0, 0, 255));
	circleColor.push_back(cv::Scalar(0, 255, 0));
	circleColor.push_back(cv::Scalar(255, 0, 0));
	circleColor.push_back(cv::Scalar(255, 0, 255));
	

	Mat newCamMat_R,newCamMat_L;
	float dummy_query_data_R[10] = { 156.00713303635887, 0, 159.5047668805756, 0, 155.83043278608287, 85.505831367789, 0, 0, 0.5};
	float dummy_query_data_L[10] = { 156.00713303635887, 0, 119.5047668805756, 0, 155.83043278608287, 85.505831367789, 0, 0, 0.5};
	newCamMat_R = cv::Mat(3, 3, CV_32F, dummy_query_data_R);
	newCamMat_L = cv::Mat(3, 3, CV_32F, dummy_query_data_L);
	
	cv::imshow("right", right);
	cv::imshow("left", left);


	fisheye::undistortImage(right, undistorted_right, cameraMatrix_R, distCoeffs_R, newCamMat_R, cv::Size(right.cols * 1, right.rows * 1));
	fisheye::undistortImage(left, undistorted_left, cameraMatrix_L, distCoeffs_L, newCamMat_L, cv::Size(left.cols * 1, left.rows * 1));


	setPts(fs_pts);
	cv::fisheye::undistortPoints(p_right_obj, p_right_tmp, cameraMatrix_R, distCoeffs_R, Matx33d::eye(), newCamMat_R);
	cv::fisheye::undistortPoints(p_left_obj, p_left_tmp, cameraMatrix_L, distCoeffs_L, Matx33d::eye(), newCamMat_L);
	
	IPM H_right(Size(imageWidth, imageHeight), Size(640, 480), p_right_tmp, p_right_dst);
	IPM H_left(Size(imageWidth, imageHeight), Size(640, 480), p_left_tmp, p_left_dst);

	H_right.applyHomography(undistorted_right, top_right);
	H_left.applyHomography(undistorted_left, top_left);

	writeData(fs, H_right.m_H, H_left.m_H);
	

	for (int i = 0; i < 4; i++)
	{
		cv::circle(undistorted_right, p_right_tmp[i], 2, cv::Scalar(0, 0, 255), 1);
		cv::circle(undistorted_left, p_left_tmp[i], 2, cv::Scalar(0, 0, 255), 1);

		cv::circle(blackWindow, p_right_dst[i], 2, cv::Scalar(0, 0, 255), 1);
		cv::circle(blackWindow, p_left_dst[i], 2, cv::Scalar(0, 0, 255), 1);

		cv::circle(top_right, p_right_dst[i], 2, cv::Scalar(0, 0, 255), 1);
		cv::circle(top_right, p_left_dst[i], 2, cv::Scalar(0, 0, 255), 1);

		cv::circle(top_left, p_right_dst[i], 2, cv::Scalar(0, 0, 255), 1);
		cv::circle(top_left, p_left_dst[i], 2, cv::Scalar(0, 0, 255), 1);
	}

	cv::imshow("undistorted_right", undistorted_right);
	cv::imshow("undistorted_left", undistorted_left);

	cv::imshow("top_right", top_right);
	cv::imshow("top_left", top_left);


	// ROI copy
	cv::Mat roi_R, roi_L;
	roi_R = top_right(cv::Rect(top_right.cols / 2, 0, top_right.cols / 2, top_right.rows));
	roi_L = top_left(cv::Rect(0, 0, top_left.cols / 2, top_left.rows));
	roi_R.copyTo(blackWindow(cv::Rect(blackWindow.cols / 2, 0, blackWindow.cols / 2, blackWindow.rows)));
	roi_L.copyTo(blackWindow(cv::Rect(0, 0, blackWindow.cols / 2, blackWindow.rows)));

	cv::imshow("window", blackWindow);

	while (true)
	{
		char keyInput = (char)cv::waitKey(20);
		if( keyInput == 'q')
			break;
		else if (keyInput == 's')
		{
			cout << "Image saved" << endl;
		}
	}
	
	imwrite("topview_right.bmp", top_right);
	imwrite("topview_left.bmp", top_left);

	cv::destroyAllWindows;

	return 0;
}

void setPts(const FileStorage& node)
{
	node["R_pts_obj"] >> p_right_obj;
	node["R_pts_dst"] >> p_right_dst;
	node["L_pts_obj"] >> p_left_obj;
	node["L_pts_dst"] >> p_left_dst;


}
void writeData(FileStorage& fs, cv::Mat right_H, cv::Mat left_H)                     //Write serialization for this class
{
	fs << "RightH" << right_H
		<< "LeftH" << left_H
		;
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
