#pragma once
#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

int threshold1 = 25;
int threshold2 = 100;
int gaussblur = 5;
float deg;

void setwindowSettings() {

	cv::namedWindow("canny_var", 100);

	cv::createTrackbar("threshold1", "canny_var", &threshold1, 500, NULL);
	cv::createTrackbar("threshold2", "canny_var", &threshold2, 500, NULL);
	cv::createTrackbar("blur", "canny_var", &gaussblur, 20, NULL);

}
int main(int argc, char** argv)
{
	cv::Mat img = imread(argv[1]);
	cv::Mat img_gray, img_blur, img_canny;

	while (1)
	{
		setwindowSettings();

		Mat result;

		cv::cvtColor(img, img_gray, CV_BGR2GRAY);
		cv::GaussianBlur(img_gray, img_blur, cv::Size(gaussblur*2-1, gaussblur*2-1), 3);
		cv::Canny(img_blur, img_canny, threshold1, threshold2, 3);

		cvtColor(img_canny, result, CV_GRAY2BGR);

		vector<Vec4i> lines;
		cv::HoughLinesP(img_canny, lines, 1, CV_PI / 180, 50, 20, 10);
		std::vector<int> candidates;
		for (size_t i = 0; i < lines.size(); i++)
		{
			if ((lines[i][2] > 430 && lines[i][0] > 430) && (lines[i][1] > img.rows / 2 && lines[i][3] > img.rows / 2))
				candidates.push_back(i);
		}
		int minidx = 0;
		if (candidates.size() > 1)
		{
			for (size_t i = 1; i < candidates.size(); i++)
			{
				if (lines[candidates[minidx]][0]>lines[candidates[i]][0])
					minidx = i;
			}
		}
		else if (candidates.size() == 1)
			minidx = 0;

		if (candidates.size()>0)
		{
			Vec4i l = lines[candidates[minidx]];
			float dx, dy;
			float length, theta;
			dy = l[2] - l[0];
			dx = l[1] - l[3];
			theta = atan2f(dy, dx);
			if (theta * 180 / CV_PI > 90)
				deg = -1 * (180 - theta * 180 / CV_PI);
			else
				deg = theta * 180 / CV_PI;
			line(result, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 2, CV_AA);
		}

		String headingText="Heading=";
		stringstream ss;
		ss << deg;
		headingText += ss.str();
		cv::putText(result, headingText, cv::Point(lines[candidates[minidx]][0], lines[candidates[minidx]][1]), FONT_HERSHEY_PLAIN, 1.1, cv::Scalar(0, 0, 255));
		imshow("source", img);
		imshow("detected lines", result);

		char c = (char)cv::waitKey(10);
		if (c == 'q')
			break;
	}

	return 1;
}
