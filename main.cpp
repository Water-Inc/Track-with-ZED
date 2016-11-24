#include <iostream>

#include <opencv2/opencv.hpp>
#include <zed/Camera.hpp>
#include <zed/utils/GlobalDefine.hpp>

inline void maxDiff(cv::Mat &img1, cv::Mat &img2, cv::Mat &res)
{
	cv::Mat img1_split[4];
	cv::Mat img2_split[4];
	
	cv::split(img1, img1_split);
	cv::split(img2, img2_split);
	res = cv::abs(img1_split[0] - img2_split[0]);
	
	for (int i = 1; i < 3; i++)
		res = cv::max(res, abs(img1_split[i] - img2_split[i]));
}

inline void procImage(cv::Mat &img, int threshold, int erode_size, int dilate_size)
{
	cv::Mat erode_elem = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(erode_size + 1, erode_size + 1));
	cv::Mat dilate_elem_1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(dilate_size + 1, dilate_size + 1));
	cv::Mat dilate_elem_2 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(dilate_size + 1, dilate_size + 1));
	
	cv::threshold(img, img, threshold, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
	
	cv::erode(img, img, erode_elem);
	cv::dilate(img, img, dilate_elem_1);
	cv::dilate(img, img, dilate_elem_2);
}

inline cv::Rect findMaxContour(cv::Mat &img)
{
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(img, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
	
	double max_area = 0;
	std::vector<cv::Point> max_contour;
	
	for (int i = 0; i < contours.size(); i++)
	{
		double area = cv::contourArea(contours[i]);
		
		if (area > max_area)
		{
			max_area = area;
			max_contour = contours[i];
		}
	}
	
	return cv::boundingRect(max_contour);
}

int main(int argc, char *argv[])
{
	sl::zed::Camera *zed = new sl::zed::Camera(sl::zed::VGA);
	sl::zed::InitParams params;
	sl::zed::ERRCODE err = zed->init(params);
	sl::zed::SENSING_MODE dm_type = sl::zed::STANDARD;
	zed->setDepthClampValue(5000);
	
	cv::Size size(zed->getImageSize().width, zed->getImageSize().height);
	
	cv::Mat depth(size, CV_8UC4);
	cv::Mat origin(size, CV_8UC4);
	cv::Mat real_time(size, CV_8UC4);
	cv::Mat diff(size, CV_8UC1);
	
	//cv::namedWindow("Cap");
	//cv::namedWindow("Diff");
	//cv::namedWindow("Proc");
	//cv::namedWindow("ZED Test");
	
	slMat2cvMat(zed->retrieveImage(static_cast<sl::zed::SIDE>(sl::zed::STEREO_LEFT))).copyTo(origin);
	
	char key;
	while (key != 'q')
	{
		zed->grab(dm_type);
		slMat2cvMat(zed->normalizeMeasure(sl::zed::MEASURE::DEPTH)).copyTo(depth);  // get the depth map
		slMat2cvMat(zed->retrieveImage(static_cast<sl::zed::SIDE>(sl::zed::STEREO_LEFT))).copyTo(real_time);    // get left image for tracking
		
		maxDiff(origin, real_time, diff);
		procImage(diff, 50, 12, 18);
		
		// find the object and draw a rectangle on it
		cv::Rect max_contour = findMaxContour(diff);
		cv::rectangle(real_time, max_contour, cv::Scalar(0, 0, 255));
		cv::rectangle(diff, max_contour, 127);
		
		cv::imshow("Diff", diff);
		cv::imshow("ZED Track", depth);
		
		key = (char) cv::waitKey(5);
		if (key == 'c')     // press C to recapture the origin image
			slMat2cvMat(zed->retrieveImage(static_cast<sl::zed::SIDE>(sl::zed::STEREO_LEFT))).copyTo(origin);
	}
	
	delete zed;
	
	return 0;
}
