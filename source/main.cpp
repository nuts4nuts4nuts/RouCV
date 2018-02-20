#include <iostream>
#include <fstream>
#include <cmath>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Windows.h>
#include <chrono>

#ifndef M_PI
	#define M_PI 3.14159265358979323846
#endif

using namespace std;
using namespace cv;

struct RouPoint
{
	cv::Point point;
	std::chrono::steady_clock::time_point time;

	RouPoint(cv::Point p)
	{
		point = p;
	}
};

struct FinishedPoint
{
	float radius;
	float angle;
	int timeAround;
	std::chrono::steady_clock::time_point time;

	FinishedPoint(float r, float a, int tA, std::chrono::steady_clock::time_point t)
	{
		radius = r;
		angle = a;
		timeAround = tA;
		time = t;
	}
};

//*******************************************************************************//
//Motion tracking code modified from https://www.youtube.com/watch?v=X6rPdRZzgjg //
//*******************************************************************************//

//our sensitivity value to be used in the threshold function
const static int SENSITIVITY_VALUE = 40;
//our sensitivity value to be used in the threshold function for green tracking
const static int SENSITIVITY_VALUE_GREEN = 80;
//size of blur used to smooth the intensity image output from absdiff() function
const static int BLUR_SIZE = 10;

int referenceWindowLeft = 0;
int referenceWindowTop = 0;
int referenceWindowWidth = 0;
int referenceWindowHeight = 0;

int greenMaskRadius = 100;

int rouletteOrder[37] = { 0, 23, 6, 35, 4, 19, 10, 31, 16, 27, 18, 14, 33, 12, 25, 2, 21, 8, 29, 3, 24, 5, 28, 17, 20, 7, 36, 11, 32, 30, 15, 26, 1, 22, 9, 34, 13 };

cv::Point2f ToPolar(cv::Point center, cv::Point point)
{
	cv::Point translatedPoint = point - center;

	float radius = sqrtf(powf((float)translatedPoint.x, 2.f) + powf((float)translatedPoint.y, 2.f));

	float angleRadians = atan2f((float)translatedPoint.y, (float)translatedPoint.x);
	float angleDegrees = (angleRadians + (float)M_PI) * 180 / (float)M_PI;

	return cv::Point2f(radius, angleDegrees);
}

float GetAngleDifference(float zeroAngle, float angle)
{
	return fmodf((angle - zeroAngle + 180.f + 360.f), 360.f) - 180.f;
}

bool IsPointBetweenTwoPoints(cv::Point center, cv::Point point, cv::Point point1, cv::Point point2)
{
	cv::Point2f pointPolar = ToPolar(center, point);
	cv::Point2f point1Polar = ToPolar(center, point1);
	cv::Point2f point2Polar = ToPolar(center, point2);

	float anglePoint1 = GetAngleDifference(pointPolar.y, point1Polar.y);
	float anglePoint2 = GetAngleDifference(pointPolar.y, point2Polar.y);
	
	return (anglePoint1 >= 0.f && anglePoint1 < 90.f && anglePoint2 < 0.f && anglePoint2 > -90.f) ||
		   (anglePoint2 >= 0.f && anglePoint2 < 90.f && anglePoint1 < 0.f && anglePoint1 > -90.f);
}

Mat hwnd2mat(HWND hwnd)
{
	HDC hwindowDC, hwindowCompatibleDC;

	HBITMAP hbwindow;
	Mat src;
	BITMAPINFOHEADER  bi;

	hwindowDC = GetDC(hwnd);
	hwindowCompatibleDC = CreateCompatibleDC(hwindowDC);
	SetStretchBltMode(hwindowCompatibleDC, COLORONCOLOR);

	RECT windowsize;    // get the height and width of the screen
	GetClientRect(hwnd, &windowsize);

	int srcwidth = referenceWindowWidth;
	int srcheight = referenceWindowHeight;
	int srcWidthOffset = referenceWindowLeft;
	int srcHeightOffset = referenceWindowTop;

	int width = referenceWindowWidth;
	int height = referenceWindowHeight;  //change this to whatever size you want to resize to

	src.create(height, width, CV_8UC4);

	// create a bitmap
	hbwindow = CreateCompatibleBitmap(hwindowDC, width, height);
	bi.biSize = sizeof(BITMAPINFOHEADER);    //http://msdn.microsoft.com/en-us/library/windows/window/dd183402%28v=vs.85%29.aspx
	bi.biWidth = width;
	bi.biHeight = -height;  //this is the line that makes it draw upside down or not
	bi.biPlanes = 1;
	bi.biBitCount = 32;
	bi.biCompression = BI_RGB;
	bi.biSizeImage = 0;
	bi.biXPelsPerMeter = 0;
	bi.biYPelsPerMeter = 0;
	bi.biClrUsed = 0;
	bi.biClrImportant = 0;

	// use the previously created device context with the bitmap
	SelectObject(hwindowCompatibleDC, hbwindow);
	// copy from the window device context to the bitmap device context
	StretchBlt(hwindowCompatibleDC, 0, 0, width, height, hwindowDC, srcWidthOffset, srcHeightOffset, srcwidth, srcheight, SRCCOPY); //change SRCCOPY to NOTSRCCOPY for wacky colors !
	GetDIBits(hwindowCompatibleDC, hbwindow, 0, height, src.data, (BITMAPINFO *)&bi, DIB_RGB_COLORS);  //copy from hwindowCompatibleDC to hbwindow

	// avoid memory leak
	DeleteObject(hbwindow);
	DeleteDC(hwindowCompatibleDC);
	ReleaseDC(hwnd, hwindowDC);

	return src;
}

//int to string helper function
string intToString(int number)
{
	//this function has a number input and string output
	std::stringstream ss;
	ss << number;
	return ss.str();
}

void placeCrosshair(Mat &cameraFeed, Point position)
{
	//make some temp x and y variables so we dont have to type out so much
	int x = position.x;
	int y = position.y;

	//draw some crosshairs around the object
	cv::circle(cameraFeed, Point(x, y), 20, Scalar(0, 255, 0), 2);
	cv::line(cameraFeed, Point(x, y), Point(x, y - 25), Scalar(0, 255, 0), 2);
	cv::line(cameraFeed, Point(x, y), Point(x, y + 25), Scalar(0, 255, 0), 2);
	cv::line(cameraFeed, Point(x, y), Point(x - 25, y), Scalar(0, 255, 0), 2);
	cv::line(cameraFeed, Point(x, y), Point(x + 25, y), Scalar(0, 255, 0), 2);

	//write the position of the object to the screen
	cv::putText(cameraFeed, "Tracking object at (" + intToString(x) + "," + intToString(y) + ")", Point(x, y), 1, 1, Scalar(255, 0, 0), 2);
}

float GetEstimatedRadiusDifference(cv::Point wheelCenter, cv::Point resetPoint, cv::Point point, const std::vector<RouPoint>& pointsVector)
{
	float radiusDifference = 0;

	if (pointsVector.empty())
	{
		return radiusDifference;
	}

	cv::Point2f pointPolar = ToPolar(wheelCenter, point);

	//Find the nearest point in both directions in the previous list
	const RouPoint* nearestPointPos = nullptr;
	float nearestDistPos = 360.f;

	const RouPoint* nearestPointNeg = nullptr;
	float nearestDistNeg = -360.f;

	cv::Point2f resetPointPolar = ToPolar(wheelCenter, resetPoint);
	float distanceToReset = GetAngleDifference(pointPolar.y, resetPointPolar.y);
	if (distanceToReset > 0)
	{
		nearestDistPos = distanceToReset;
	}
	else
	{
		nearestDistNeg = distanceToReset;
	}

	for (const RouPoint& p : pointsVector)
	{
		cv::Point2f pPolar = ToPolar(wheelCenter, p.point);

		float distance = GetAngleDifference(pointPolar.y, pPolar.y);

		if (distance >= 0 && distance < nearestDistPos)
		{
			nearestDistPos = distance;
			nearestPointPos = &p;
		}

		if (distance <= 0 && distance > nearestDistNeg)
		{
			nearestDistNeg = distance;
			nearestPointNeg = &p;
		}
	}

	//Interpolate between the two points' radii based on their distance to the current point to find the estimated radius of the current point
	if (nearestPointNeg != nullptr && nearestPointPos != nullptr)
	{
		//Set it up such that distNeg = 0, distPos = 1.0, and 0 < distCurrent < 1.0
		float distCurrent = -nearestDistNeg;
		float distPos = nearestDistPos - nearestDistNeg;

		distCurrent = distPos == 0 ? 0 : distCurrent / distPos;

		//Linearly interpolate between the time at the negative point and the time at the positive point
		float negativePointRadius = ToPolar(wheelCenter, nearestPointNeg->point).x;
		float positivePointRadius = ToPolar(wheelCenter, nearestPointPos->point).x;

		float distCurrentInverse = 1.f - distCurrent;
		float projectedNearestPointRadius = distCurrentInverse * negativePointRadius + distCurrent * positivePointRadius;

		radiusDifference = std::fabsf(projectedNearestPointRadius - pointPolar.x);
	}
	else if (nearestPointNeg != nullptr)
	{
		radiusDifference = std::fabsf(ToPolar(wheelCenter, nearestPointNeg->point).x - pointPolar.x);
	}
	else if (nearestPointPos != nullptr)
	{
		radiusDifference = std::fabsf(ToPolar(wheelCenter, nearestPointPos->point).x - pointPolar.x);
	}
	
	return radiusDifference;
}

//Returns time around in milliseconds
int GetTimeAround(cv::Point wheelCenter, cv::Point resetPoint, const RouPoint& currentPoint, const std::vector<RouPoint>& oldPointVector)
{
	int timeAround = -1;

	if (oldPointVector.empty())
	{
		return timeAround;
	}

	cv::Point2f currentPointPolar = ToPolar(wheelCenter, currentPoint.point);

	//Find the nearest point in both directions in the previous list
	const RouPoint* nearestPointPos = nullptr;
	float nearestDistPos = 360.f;

	const RouPoint* nearestPointNeg = nullptr;
	float nearestDistNeg = -360.f;

	cv::Point2f resetPointPolar = ToPolar(wheelCenter, resetPoint);
	float distanceToReset = GetAngleDifference(currentPointPolar.y, resetPointPolar.y);
	if (distanceToReset > 0)
	{
		nearestDistPos = distanceToReset;
	}
	else
	{
		nearestDistNeg = distanceToReset;
	}

	for (const RouPoint& p : oldPointVector)
	{
		cv::Point2f pPolar = ToPolar(wheelCenter, p.point);

		float distance = GetAngleDifference(currentPointPolar.y, pPolar.y);

		if (distance >= 0 && distance < nearestDistPos)
		{
			nearestDistPos = distance;
			nearestPointPos = &p;
		}

		if (distance <= 0 && distance > nearestDistNeg)
		{
			nearestDistNeg = distance;
			nearestPointNeg = &p;
		}
	}

	//Interpolate between the two points' times based on their distance to the current point
	if (nearestPointNeg != nullptr && nearestPointPos != nullptr)
	{
		//Set it up such that distNeg = 0, distPos = 1.0, and 0 < distCurrent < 1.0
		float distCurrent = -nearestDistNeg;
		float distPos = nearestDistPos - nearestDistNeg;

		distCurrent = distPos == 0 ? 0 : distCurrent / distPos;

		//Linearly interpolate between the time at the negative point and the time at the positive point
		auto negativePointTime = std::chrono::duration_cast<std::chrono::milliseconds>(nearestPointNeg->time.time_since_epoch()).count();
		auto positivePointTime = std::chrono::duration_cast<std::chrono::milliseconds>(nearestPointPos->time.time_since_epoch()).count();

		float distCurrentInverse = 1.f - distCurrent;
		int projectedNearestPointTime = (int)(distCurrentInverse * negativePointTime + distCurrent * positivePointTime);

		timeAround = (int)std::chrono::duration_cast<std::chrono::milliseconds>(currentPoint.time.time_since_epoch()).count() - projectedNearestPointTime;
	}
	//else if (nearestPointNeg != nullptr)
	//{
	//	timeAround = (int)std::chrono::duration_cast<std::chrono::milliseconds>(currentPoint.time - nearestPointNeg->time).count();
	//}
	//else if (nearestPointPos != nullptr)
	//{
	//	timeAround = (int)std::chrono::duration_cast<std::chrono::milliseconds>(currentPoint.time - nearestPointPos->time).count();
	//}
	
	return timeAround;
}

void searchForMovement(Mat thresholdImage, Mat &cameraFeed, Point& previousPoint)
{
	//notice how we use the '&' operator for objectDetected and cameraFeed. This is because we wish
	//to take the values passed into the function and manipulate them, rather than just working with a copy.
	//eg. we draw to the cameraFeed to be displayed in the main() function.
	bool objectDetected = false;
	Mat temp;
	thresholdImage.copyTo(temp);
	//these two vectors needed for output of findContours
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	//findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );// retrieves all contours
	findContours(temp, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));// retrieves external contours

	if (contours.size() > 0)
	{
		objectDetected = true;
	}
	else
	{
		objectDetected = false;
	}

	if (objectDetected)
	{
		//the largest contour is found at the end of the contours vector
		vector<Point> contour;
		contour = contours.at(contours.size() - 1);

		//make a bounding rectangle around the largest contour then find its center
		//this will be the object's final estimated position.
		Rect objectBoundingRectangle = boundingRect(contour);
		int xpos = objectBoundingRectangle.x + objectBoundingRectangle.width / 2;
		int ypos = objectBoundingRectangle.y + objectBoundingRectangle.height / 2;

		//update the objects positions by changing the 'theObject' array values
		previousPoint.x = xpos, previousPoint.y = ypos;
	}
	else
	{
		previousPoint.x = -1, previousPoint.y = -1;
	}

	if (previousPoint.x != -1 && previousPoint.y != -1)
	{
		placeCrosshair(cameraFeed, previousPoint);
	}
}

int main()
{
	namedWindow("ReferenceFrame", WINDOW_NORMAL);

	HWND referenceWindowHandle = FindWindow(0, "ReferenceFrame");
	if (referenceWindowHandle == nullptr)
	{
		printf("Couldn't find reference window handle!");
		return -1;
	}
	
	//-Set window to be click-through.
	LONG lExStyle = GetWindowLong(referenceWindowHandle, GWL_EXSTYLE);
	lExStyle |=  WS_EX_LAYERED;
	SetWindowLong(referenceWindowHandle, GWL_EXSTYLE, lExStyle);
	SetLayeredWindowAttributes(referenceWindowHandle, RGB(255, 0, 0), 0, LWA_COLORKEY);

	Mat transparentImage(1, 1, CV_8UC4);
	transparentImage = cv::Scalar(0, 0, 255, 255);
	cv::imshow("ReferenceFrame", transparentImage);

	//some boolean variables for added functionality
	bool objectDetected = false;
	//this can be toggled with 'd'
	bool debugMode = false;
	//this can be toggled with 't'
	bool trackingEnabled = false;
	//this can be toggled with 'g'
	bool greenDebug = false;
	//this can be toggled with 's'
	bool spinTrack = false;
	//pause and resume code
	bool pause = false;

	//set up the matrices that we will need
	//the current frame
	Mat currentFrame;
	//their grayscale images (needed for absdiff() function)
	Mat currentGrayImage, previousGrayImage;
	//resulting difference image
	Mat differenceImage;
	//thresholded difference image (for use in findContours() function)
	Mat thresholdImage;

	//images filtered for green to look for the 0
	Mat currentGreenImage, previousGreenImage;
	//resulting difference image
	Mat differenceImageGreen;
	//thresholded difference image (for use in findContours() function)
	Mat thresholdImageGreen;

	HWND hwndDesktop = GetDesktopWindow();

	Point ballCenter(-1, -1);
	Point greenCenter(-1, -1);
	Point wheelCenter(-1, -1);

	Point greenPointPrevious(-1, -1);
	Point ballPointPrevious(-1, -1);

	cv::Point resetPointGreen(-1, -1);
	cv::Point resetPointBall(-1, -1);

	std::vector<RouPoint> innerWheelPoints;
	std::vector<RouPoint> innerWheelPointsPrevious;

	std::vector<RouPoint> ballPoints;
	std::vector<RouPoint> ballPointsPrevious;
	std::vector<RouPoint> ballPointsRadiusDecay;

	std::vector<FinishedPoint> ballSpeeds;
	std::vector<FinishedPoint> wheelSpeeds;

	auto startTime = std::chrono::high_resolution_clock::now();
	auto currentTime = std::chrono::high_resolution_clock::now();
	int numFrames = 0;

	while (1)
	{
		RECT windowRectangle;
		GetWindowRect(referenceWindowHandle, &windowRectangle);

		referenceWindowLeft = windowRectangle.left + 9;
		referenceWindowTop = windowRectangle.top + 32;
		referenceWindowWidth = windowRectangle.right - windowRectangle.left - 9 - 8;
		referenceWindowHeight = windowRectangle.bottom - windowRectangle.top - 32 - 8;

		if (wheelCenter == Point(-1, -1))
		{
			wheelCenter = Point(referenceWindowWidth / 2, referenceWindowHeight / 2);
		}

		//capture frame
		currentFrame = hwnd2mat(hwndDesktop);

		//convert frame1 to gray scale for frame differencing
		cv::cvtColor(currentFrame, currentGrayImage, COLOR_BGR2GRAY);
		cv::circle(currentGrayImage, cv::Point(referenceWindowWidth / 2, referenceWindowHeight / 2), greenMaskRadius, cv::Scalar(0, 255, 0), -1);

		//filter frame for green
		cv::cvtColor(currentFrame, currentGreenImage, COLOR_BGR2HSV);
		cv::inRange(currentGreenImage, cv::Scalar(45, 51, 51), cv::Scalar(90, 255, 204), currentGreenImage);

		if (greenDebug == true)
		{
			cv::imshow("Green Image", currentGreenImage);
		}
		else
		{
			cv::destroyWindow("Green Image");
		}

		//If there is a previous image to compare to, do the rest
		bool grayImageValid = !previousGrayImage.empty() && previousGrayImage.cols == currentGrayImage.cols && previousGrayImage.rows == currentGrayImage.rows;
		bool greenImageValid = !previousGreenImage.empty() && previousGreenImage.cols == currentGreenImage.cols && previousGreenImage.rows == currentGreenImage.rows;
		if (grayImageValid && greenImageValid)
		{
			//Get threshold image of the whole frame
			{
				//perform frame differencing with the sequential images. This will output an "intensity image"
				//do not confuse this with a threshold image, we will need to perform thresholding afterwards.
				cv::absdiff(currentGrayImage, previousGrayImage, differenceImage);
				//threshold intensity image at a given sensitivity value
				cv::threshold(differenceImage, thresholdImage, SENSITIVITY_VALUE, 255, THRESH_BINARY);
				if (debugMode == true)
				{
					//show the difference image and threshold image
					cv::imshow("Difference Image", differenceImage);
					cv::imshow("Threshold Image", thresholdImage);
				}
				else
				{
					//if not in debug mode, destroy the windows so we don't see them anymore
					cv::destroyWindow("Difference Image");
					cv::destroyWindow("Threshold Image");
				}

				//blur the image to get rid of the noise. This will output an intensity image
				cv::blur(thresholdImage, thresholdImage, cv::Size(BLUR_SIZE, BLUR_SIZE));
				//threshold again to obtain binary image from blur output
				cv::threshold(thresholdImage, thresholdImage, SENSITIVITY_VALUE, 255, THRESH_BINARY);
				if (debugMode == true)
				{
					//show the threshold image after it's been "blurred"
					cv::imshow("Final Threshold Image", thresholdImage);
				}
				else
				{
					//if not in debug mode, destroy the windows so we don't see them anymore
					cv::destroyWindow("Final Threshold Image");
				}
			}

			//Get threshold image of just the green stuff
			{
				cv::absdiff(currentGreenImage, previousGreenImage, differenceImageGreen);

				cv::threshold(differenceImageGreen, thresholdImageGreen, SENSITIVITY_VALUE_GREEN, 255, THRESH_BINARY);
				if (greenDebug == true)
				{
					cv::imshow("Difference Image Green", differenceImageGreen);
				}
				else
				{
					cv::destroyWindow("Difference Image Green");
				}

				cv::blur(thresholdImageGreen, thresholdImageGreen, cv::Size(BLUR_SIZE, BLUR_SIZE));

				cv::threshold(thresholdImageGreen, thresholdImageGreen, SENSITIVITY_VALUE_GREEN, 255, THRESH_BINARY);

				if (greenDebug == true)
				{
					cv::imshow("Final Threshold Image Green", thresholdImageGreen);
				}
				else
				{
					cv::destroyWindow("Final Threshold Image Green");
				}
			}

			//if tracking enabled, search for contours in our thresholded image
			if (trackingEnabled)
			{
				searchForMovement(thresholdImage, currentFrame, ballCenter);
				searchForMovement(thresholdImageGreen, currentFrame, greenCenter);
			}

			//If tracking the spin, write the positions
			if(spinTrack)
			{
				//Track the green 0
				if(greenCenter.x != -1 && greenCenter.y != -1)
				{
					//If we don't already have a reset point
					if (resetPointGreen == cv::Point(-1, -1))
					{
						resetPointGreen = greenCenter;
					}
					else
					{
						if (innerWheelPoints.size() > 0)
						{
							greenPointPrevious = innerWheelPoints.back().point;
						}

						if (IsPointBetweenTwoPoints(wheelCenter, resetPointGreen, greenCenter, greenPointPrevious))
						{
							innerWheelPointsPrevious = std::vector<RouPoint>(innerWheelPoints);
							innerWheelPoints.clear();
							//printf("RESET\n");
						}

						RouPoint point(greenCenter);
						point.time = std::chrono::high_resolution_clock::now();
						innerWheelPoints.push_back(point);

						int timeAround = GetTimeAround(wheelCenter, resetPointGreen, point, innerWheelPointsPrevious);
						if (timeAround > 0)
						{
							cv::Point2f currentPointPolar = ToPolar(wheelCenter, greenCenter);
							wheelSpeeds.push_back(FinishedPoint(currentPointPolar.x, currentPointPolar.y, timeAround, point.time));
							//printf("Green time around: %d\n", timeAround);
							printf("%d,", timeAround);
						}
					}

					int avgX = 0;
					int avgY = 0;
					for (RouPoint p : innerWheelPointsPrevious)
					{
						avgX += p.point.x;
						avgY += p.point.y;
					}

					if (innerWheelPointsPrevious.size() != 0)
					{
						cv::Point newCenter = cv::Point(avgX / (int)innerWheelPointsPrevious.size(), avgY / (int)innerWheelPointsPrevious.size());
						wheelCenter = (wheelCenter + newCenter) / 2;
					}
				}

				//Track the ball
				if(ballCenter.x != -1 && ballCenter.y != -1)
				{
					//If we don't already have a reset point
					if (resetPointBall == cv::Point(-1, -1))
					{
						resetPointBall = ballCenter;
					}
					else
					{
						if (ballPoints.size() > 0)
						{
							ballPointPrevious = ballPoints.back().point;
						}

						if (IsPointBetweenTwoPoints(wheelCenter, resetPointBall, ballCenter, ballPointPrevious))
						{
							ballPointsPrevious = std::vector<RouPoint>(ballPoints);
							ballPoints.clear();
							//printf("Ball RESET\n");
						}

						RouPoint point(ballCenter);
						point.time = std::chrono::high_resolution_clock::now();
						ballPoints.push_back(point);

						int timeAround = GetTimeAround(wheelCenter, resetPointBall, point, ballPointsPrevious);

						if (timeAround > 0)
						{
							cv::Point2f currentPointPolar = ToPolar(wheelCenter, ballCenter);
							ballSpeeds.push_back(FinishedPoint(currentPointPolar.x, currentPointPolar.y, timeAround, point.time));
							//printf("Ball time around: %d\n", timeAround);
							//printf("%d,", timeAround);
						}

						if (GetEstimatedRadiusDifference(wheelCenter, resetPointBall, ballCenter, ballPointsPrevious) > 5.f)
						{
							ballPointsRadiusDecay.push_back(point);
						}
					}
				}

				cv::circle(currentFrame, wheelCenter, 5, Scalar(0, 0, 255), -1);
				cv::line(currentFrame, wheelCenter, resetPointGreen, Scalar(255, 0, 0), 2);
				cv::line(currentFrame, wheelCenter, resetPointBall, Scalar(0, 255, 255), 2);

				for (RouPoint p : innerWheelPointsPrevious)
				{
					cv::circle(currentFrame, p.point, 2, Scalar(255, 255, 0), -1);
				}
				for (RouPoint p : innerWheelPoints)
				{
					cv::circle(currentFrame, p.point, 2, Scalar(255, 0, 0), -1);
				}
				for (RouPoint p : ballPointsPrevious)
				{
					cv::circle(currentFrame, p.point, 2, Scalar(255, 255, 0), -1);
				}
				for (RouPoint p : ballPoints)
				{
					cv::circle(currentFrame, p.point, 2, Scalar(255, 0, 0), -1);
				}
				for (RouPoint p : ballPointsRadiusDecay)
				{
					cv::circle(currentFrame, p.point, 2, Scalar(0, 0, 255), -1);
				}
			}

			//Overlay the mask we use for the grayscale images for reference
			Mat overlayFrame;
			overlayFrame = currentFrame.clone();
			cv::circle(overlayFrame, cv::Point(referenceWindowWidth / 2, referenceWindowHeight / 2), greenMaskRadius, cv::Scalar(0, 255, 0), -1);
			double alpha = 0.5;

			cv::addWeighted(overlayFrame, alpha, currentFrame, 1.0 - alpha, 0.0, currentFrame);

			//show our captured frame
			cv::imshow("FinalFrame", currentFrame);
			//check to see if a button has been pressed.
			//this 10ms delay is necessary for proper operation of this program
			//if removed, frames will not have enough time to referesh and a blank 
			//image will appear.
			switch (waitKey(10))
			{
			case 27: //'esc' key has been pressed, exit program.
				return 0;
			case 116: //'t' has been pressed. this will toggle tracking
				trackingEnabled = !trackingEnabled;
				if (trackingEnabled == false)
				{
					cout << "Tracking disabled." << endl;
				}
				else
				{
					cout << "Tracking enabled.\n" << endl;
				}
				break;
			case 100: //'d' has been pressed. this will toggle debug mode
				debugMode = !debugMode;
				if (debugMode == false)
				{
					cout << "Debug mode disabled." << endl;
				}
				else
				{
					cout << "Debug mode enabled." << endl;
				}
				break;
			case 103: //'g' has been pressed. this will toggle green debug mode
				greenDebug = !greenDebug;
				if (greenDebug == false)
				{
					cout << "Green debug mode disabled." << endl;
				}
				else
				{
					cout << "Green debug mode enabled." << endl;
				}
				break;
			case 114: //'r' has been pressed. this will reset the tracking arrays
				resetPointGreen = cv::Point(-1, -1);
				resetPointBall = cv::Point(-1, -1);

				innerWheelPointsPrevious.clear();
				innerWheelPoints.clear();

				ballPointsPrevious.clear();
				ballPoints.clear();
				ballPointsRadiusDecay.clear();

				ballSpeeds.clear();
				wheelSpeeds.clear();
				break;
			case 115: //'s' has been pressed. this will toggle writing to the spin tracker
				spinTrack = !spinTrack;
				if (spinTrack == false)
				{
					cout << "Spin tracking disabled." << endl;
					resetPointGreen = cv::Point(-1, -1);
					resetPointBall = cv::Point(-1, -1);

					innerWheelPointsPrevious.clear();
					innerWheelPoints.clear();

					ballPointsPrevious.clear();
					ballPoints.clear();
					ballPointsRadiusDecay.clear();

					ballSpeeds.clear();
					wheelSpeeds.clear();
				}
				else
				{
					cout << "Spin tracking enabled." << endl;
				}
				break;
			case 109: //'m' has been pressed. This will increase the radius of the green mask circle
				greenMaskRadius = greenMaskRadius < referenceWindowWidth / 2 ? greenMaskRadius + 1 : greenMaskRadius;
				cout << "Green mask radius increased, it is now: " << greenMaskRadius << endl;
				break;
			case 110: //'n' has been pressed. This will decrease the radius of the green mask circle
				greenMaskRadius = greenMaskRadius > 1 ? greenMaskRadius - 1 : greenMaskRadius;
				cout << "Green mask radius decreased, it is now: " << greenMaskRadius << endl;
				break;
			case 112: //'p' has been pressed. this will pause/resume the code.
				pause = !pause;
				if (pause == true)
				{
					cout << "Code paused, press 'p' again to resume" << endl;
					while (pause == true)
					{
						//stay in this loop until 
						switch (waitKey())
						{
							case 112:
								//change pause back to false
								pause = false;
								cout << "Code Resumed" << endl;
							break;
						}
					}
				}
			}
		}

		previousGrayImage = currentGrayImage.clone();
		previousGreenImage = currentGreenImage.clone();
		
		numFrames++;
		currentTime = std::chrono::high_resolution_clock::now();

		if (currentTime - startTime >= std::chrono::seconds(1))
		{
			//std::printf("Frames in the last second: %d\n", numFrames);
			startTime = currentTime;
			numFrames = 0;
		}
	}

	return 0;
}