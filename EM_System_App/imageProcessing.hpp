#ifndef IMAGEPROCESSING_HPP
#define IMAGEPROCESSING_HPP

#endif // IMAGEPROCESSING_HPP

#include <QCoreApplication>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>

using namespace cv;
using namespace std;

namespace magbotImageProcessing {

    // Functions
    vector<vector<Point>> getContours(Mat imgDil, Mat img);
    double calcAngle(Point p1, Point pmid, Point p2);
    Mat filterVideo(Mat frame, int kernelSize, int blurSize);
    void filterContours(vector<vector<Point>> contours);

    enum View {
        TOP_VIEW,
        SIDE_VIEW,
    };

    struct magbotShape {
        Mat rawImage;
        Mat filtered;
        Mat pointPlot;
        bool isFlipped;
        vector<vector<Point>> contour;
        View view;
        double jointAngle;
    };

    void threshSegments(magbotShape &magbot, double threshSize);
    void blackOut(Mat& img, int xmin, int xmax, int ymin, int ymax);

    vector<Point> findInterestPoints(vector<Point> hull, View view);

    vector<Point> findInterestPoints_Top(vector<Point> hull, Rect boundingRect, int threshold);

    double getJointAngle(vector<Point> interestPoints, View view);

    double getJointAngle2(Rect boundRect, double h, double y0);

    double getJointAngle3_side(Rect boundRect, vector<Point> hull, int tolerance, double bias);

    double getJointAngle3_top(Rect boundRect, vector<Point> hull, int tolerance);

    double getJointAngle_Top(vector<Point> interestPoints);

    double getJointAngle_Top2(vector<Point> contour, int cutoff);


    void processMagbot(magbotShape& magbot);

}

