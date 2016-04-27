#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include "IPM.h"

using namespace std;
using namespace cv;

void writeData(FileStorage& fs, cv::Mat right_H, cv::Mat left_H);
void readData_R(const FileStorage& node);
void readData_L(const FileStorage& node);


Mat cameraMatrix_R, cameraMatrix_L;
Mat distCoeffs_R, distCoeffs_L;
Mat extrinsicParam_R, extrinsicParam_L;
int imageWidth, imageHeight;
cv::Size boardSize(9, 6);
bool found;
int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;
vector<Point2f> p_front, p_rear, p_left, p_right;
vector<Point2f> p_front2, p_rear2, p_left2, p_right2;
vector<Point2f> p_front_obj, p_rear_obj, p_left_obj, p_right_obj;
vector<Point2f> p_front_dst, p_rear_dst, p_left_dst, p_right_dst;
vector<Point2f> p_right_tmp, p_left_tmp;
vector<cv::Scalar> circleColor;

void setPts();

int main(int argc, char** argv)
{
	Mat blackWindow = Mat(480, 640, CV_8UC3, cv::Scalar(0,0,0));
	const string camera_light_file = "calibration/right_camera.xml";
	const string camera_left_file = "calibration/left_camera.xml";
	FileStorage fs_R(camera_light_file, FileStorage::READ); // Read the settings
	FileStorage fs_L(camera_light_file, FileStorage::READ);
	readData_R(fs_R);
	readData_L(fs_L);

	const string homography_data_file =  "H.xml";
	FileStorage fs2(homography_data_file, FileStorage::WRITE); // Write H matrix 

	cv::Mat front,rear,right,left;
	cv::Mat undistorted_front, undistorted_rear, undistorted_right, undistorted_left;
	cv::Mat top_front,top_rear,top_right,top_left;

	circleColor.push_back(cv::Scalar(0, 0, 255));
	circleColor.push_back(cv::Scalar(0, 255, 0));
	circleColor.push_back(cv::Scalar(255, 0, 0));
	circleColor.push_back(cv::Scalar(255, 0, 255));

	//front = cv::imread("distored_front.bmp");
	//rear = cv::imread("distored_rear.bmp");
	right = cv::imread("UNLV_cart/distored_image_R.bmp");
	left = cv::imread("UNLV_cart/distored_image_L.bmp");

	Mat newCamMat_R,newCamMat_L;
	float dummy_query_data_R[10] = { 156.00713303635887, 0, 159.5047668805756, 0, 155.83043278608287, 85.505831367789, 0, 0, 0.5};
	float dummy_query_data_L[10] = { 156.00713303635887, 0, 119.5047668805756, 0, 155.83043278608287, 85.505831367789, 0, 0, 0.5};
	newCamMat_R = cv::Mat(3, 3, CV_32F, dummy_query_data_R);
	newCamMat_L = cv::Mat(3, 3, CV_32F, dummy_query_data_L);
	//fisheye::estimateNewCameraMatrixForUndistortRectify(cameraMatrix_R, distCoeffs_R, front.size(), Matx33d::eye(), newCamMat, 1, cv::Size(right.cols, right.rows), 1);

	// calibrate
	/*Mat viewGray;
	found = findChessboardCorners(front, boardSize, p_front, chessBoardFlags);	
	cvtColor(front, viewGray, COLOR_BGR2GRAY);
	//cornerSubPix(viewGray, p_front, Size(11, 11),Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
	drawChessboardCorners(front, boardSize, Mat(p_front), found);

	found = findChessboardCorners(rear, boardSize, p_rear, chessBoardFlags);
	cvtColor(rear, viewGray, COLOR_BGR2GRAY);
	//cornerSubPix(viewGray, p_rear, Size(11, 11),Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
	drawChessboardCorners(rear, boardSize, Mat(p_rear), found);

	found = findChessboardCorners(right, boardSize, p_right, chessBoardFlags);
	cvtColor(right, viewGray, COLOR_BGR2GRAY);
	//cornerSubPix(viewGray, p_right, Size(11, 11),Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
	drawChessboardCorners(right, boardSize, Mat(p_right), found);

	found = findChessboardCorners(left, boardSize, p_left, chessBoardFlags);
	cvtColor(left, viewGray, COLOR_BGR2GRAY);
	//cornerSubPix(viewGray, p_left, Size(11, 11),Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
	drawChessboardCorners(left, boardSize, Mat(p_left), found);

	cv::imshow("front", front);
	cv::imshow("rear", rear);*/
	cv::imshow("right", right);
	cv::imshow("left", left);

	/*//undistortion
	fisheye::undistortImage(front, undistorted_front, cameraMatrix, distCoeffs, newCamMat, cv::Size(front.cols * 2, front.rows * 2));
	fisheye::undistortPoints(p_front, p_front2, cameraMatrix, distCoeffs, Matx33d::eye(), newCamMat);

	fisheye::undistortImage(rear, undistorted_rear, cameraMatrix, distCoeffs, newCamMat, cv::Size(front.cols * 2, front.rows * 2));
	fisheye::undistortPoints(p_rear, p_rear2, cameraMatrix, distCoeffs, Matx33d::eye(), newCamMat);
	*/

	fisheye::undistortImage(right, undistorted_right, cameraMatrix_R, distCoeffs_R, newCamMat_R, cv::Size(right.cols * 1, right.rows * 1));
	fisheye::undistortImage(left, undistorted_left, cameraMatrix_L, distCoeffs_L, newCamMat_L, cv::Size(left.cols * 1, left.rows * 1));

	
		//

	setPts();
	cv::fisheye::undistortPoints(p_right_obj, p_right_tmp, cameraMatrix_R, distCoeffs_R, Matx33d::eye(), newCamMat_R);
	cv::fisheye::undistortPoints(p_left_obj, p_left_tmp, cameraMatrix_L, distCoeffs_L, Matx33d::eye(), newCamMat_L);
	//IPM H_front(Size(imageWidth, imageHeight), Size(600, 700), p_front_obj, p_front_dst);
	//IPM H_rear(Size(imageWidth, imageHeight), Size(600, 700), p_rear_obj, p_rear_dst);

	IPM H_right(Size(imageWidth, imageHeight), Size(640, 480), p_right_tmp, p_right_dst);
	IPM H_left(Size(imageWidth, imageHeight), Size(640, 480), p_left_tmp, p_left_dst);

	//H_front.applyHomography(undistorted_front, top_front);
	//H_rear.applyHomography(undistorted_rear, top_rear);
	H_right.applyHomography(undistorted_right, top_right);
	H_left.applyHomography(undistorted_left, top_left);

	writeData(fs2, H_right.m_H, H_left.m_H);
	

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

	//cv::imshow("undistorted_front", undistorted_front);
	//cv::imshow("undistorted_rear", undistorted_rear);
	cv::imshow("undistorted_right", undistorted_right);
	cv::imshow("undistorted_left", undistorted_left);

	//cv::imshow("top_front", top_front);
	//cv::imshow("top_rear", top_rear);
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
	
	//imwrite("topview_front.bmp", top_front);
	//imwrite("topview_rear.bmp", top_rear);
	imwrite("topview_right.bmp", top_right);
	imwrite("topview_left.bmp", top_left);

	cv::destroyAllWindows;

	return 0;
}

void setPts()
{
	/*//front
	p_front_obj.push_back(p_front2[0]);
	p_front_obj.push_back(p_front2[8]);
	p_front_obj.push_back(p_front2[45]);
	p_front_obj.push_back(p_front2[53]);
	
	p_front_dst.push_back(cv::Point2f(300 - (2.4 * 4) * 2, 350 + (-22.5 - 45 - 2.4 - 2.4 * 5) * 2));
	p_front_dst.push_back(cv::Point2f(300 + (2.4 * 4) * 2, 350 + (-22.5 - 45 - 2.4 - 2.4 * 5) * 2));
	p_front_dst.push_back(cv::Point2f(300 - (2.4 * 4) * 2, 350 + (-22.5 - 45 - 2.4) * 2));
	p_front_dst.push_back(cv::Point2f(300 + (2.4 * 4) * 2, 350 + (-22.5 - 45 - 2.4) * 2));

	//rear
	p_rear_obj.push_back(p_rear2[0]);
	p_rear_obj.push_back(p_rear2[8]);
	p_rear_obj.push_back(p_rear2[45]);
	p_rear_obj.push_back(p_rear2[53]);

	p_rear_dst.push_back(cv::Point2f(300 + (2.4 * 4) * 2, 350 + (22.5 + 45 + 2.4 + 2.4 * 5) * 2));
	p_rear_dst.push_back(cv::Point2f(300 - (2.4 * 4) * 2, 350 + (22.5 + 45 + 2.4 + 2.4 * 5) * 2));
	p_rear_dst.push_back(cv::Point2f(300 + (2.4 * 4)*2, 350 + (22.5 + 45 + 2.4)*2));
	p_rear_dst.push_back(cv::Point2f(300 - (2.4 * 4)*2, 350 + (22.5 + 45 + 2.4)*2));
	*/
	// UNLV set
	/*//right
	p_right_obj.push_back(cv::Point2f(170.5, 207.5));
	p_right_obj.push_back(cv::Point2f(356.5, 176.5));
	p_right_obj.push_back(cv::Point2f(472, 255));
	p_right_obj.push_back(cv::Point2f(141, 358));

	p_right_dst.push_back(cv::Point2f(640 / 2 , 480 - 63 - 40 * 2.54 * 2));
	p_right_dst.push_back(cv::Point2f(640 / 2 + 125 * 2, 480 - 63 - 40 * 2.54 * 2));
	p_right_dst.push_back(cv::Point2f(640 / 2 + 125 * 2, 480 - 63 ));
	p_right_dst.push_back(cv::Point2f(640 / 2 , 480 - 63 ));

	//left
	p_left_obj.push_back(cv::Point2f(291, 188));
	p_left_obj.push_back(cv::Point2f(476, 217));
	p_left_obj.push_back(cv::Point2f(509, 367));
	p_left_obj.push_back(cv::Point2f(173.5, 267));

	p_left_dst.push_back(cv::Point2f(640 / 2 - 125 * 2, 480 - 63 - 40*2.54*2));
	p_left_dst.push_back(cv::Point2f(640 / 2 , 480 - 63 - 40 * 2.54*2));
	p_left_dst.push_back(cv::Point2f(640 / 2  , 480 - 63));
	p_left_dst.push_back(cv::Point2f(640 / 2 - 125 * 2, 480 - 63));
	*/
	//UNLV_cart
	p_right_obj.push_back(cv::Point2f(191, 241));
	p_right_obj.push_back(cv::Point2f(333, 212));
	p_right_obj.push_back(cv::Point2f(408, 271));
	p_right_obj.push_back(cv::Point2f(186, 335));

	p_right_dst.push_back(cv::Point2f(640 / 2, 480 - 90 - 40 * 2.54 * 2));
	p_right_dst.push_back(cv::Point2f(640 / 2 + 125 * 2, 480 - 90 - 40 * 2.54 * 2));
	p_right_dst.push_back(cv::Point2f(640 / 2 + 125 * 2, 480 - 90));
	p_right_dst.push_back(cv::Point2f(640 / 2, 480 - 90));

	//left
	p_left_obj.push_back(cv::Point2f(315, 227));
	p_left_obj.push_back(cv::Point2f(458, 255));
	p_left_obj.push_back(cv::Point2f(467, 350));
	p_left_obj.push_back(cv::Point2f(240, 288));

	p_left_dst.push_back(cv::Point2f(640 / 2 - 125 * 2, 480 - 90 - 40 * 2.54 * 2));
	p_left_dst.push_back(cv::Point2f(640 / 2, 480 - 90 - 40 * 2.54 * 2));
	p_left_dst.push_back(cv::Point2f(640 / 2, 480 - 90));
	p_left_dst.push_back(cv::Point2f(640 / 2 - 125 * 2, 480 - 90));


}
void writeData(FileStorage& fs, cv::Mat right_H, cv::Mat left_H)                     //Write serialization for this class
{
	fs << "RightH" << right_H
		<< "LeftH" << left_H;
}
void readData_R(const FileStorage& node)                          //Read serialization for this class
{
	node["image_width"] >> imageWidth;
	node["image_height"] >> imageHeight;
	node["camera_matrix"] >> cameraMatrix_R;
	node["distortion_coefficients"] >> distCoeffs_R;
	node["extrinsic_parameters"] >> extrinsicParam_R;
}
void readData_L(const FileStorage& node)                          //Read serialization for this class
{
	node["image_width"] >> imageWidth;
	node["image_height"] >> imageHeight;
	node["camera_matrix"] >> cameraMatrix_L;
	node["distortion_coefficients"] >> distCoeffs_L;
	node["extrinsic_parameters"] >> extrinsicParam_L;
}
