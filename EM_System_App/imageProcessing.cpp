#include <QCoreApplication>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "imageProcessing.hpp"

#include <iostream>

using namespace cv;
using namespace std;

namespace magbotImageProcessing {


vector<vector<Point>> getContours(Mat imgDil, Mat img){

   vector<vector<Point>> contours;
   vector<Vec4i> hierarchy;
   findContours(imgDil, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
   string objectType;
   vector<Rect> boundRect(contours.size());

   vector<vector<Point>> conPoly(contours.size());
   for (int i = 0; i < contours.size(); i++){
       int area = contourArea(contours[i]);

       if (area > 1000) {
           float peri = arcLength(contours[i], true);
           approxPolyDP(contours[i], conPoly[i], 0.02*peri, true);
           drawContours(img, conPoly, i, Scalar(255, 0, 255), 2);

           /*
           boundRect[i] = boundingRect(conPoly[i]);

           int objCor = (int) conPoly[i].size();

           if (objCor == 3) {objectType = "Tri";};
           if (objCor == 4) {objectType = "Rect";};
           if (objCor > 4) {objectType = "Cir";};

           putText(img, objectType, {boundRect[i].x, boundRect[i].y - 5}, FONT_HERSHEY_PLAIN, 0.75, Scalar(0, 69, 255), 1);
           rectangle(img, boundRect[i].tl(), boundRect[i].br(), Scalar(0, 255, 0), 5 ); */
       }
   }
   return conPoly;
}


double calcAngle(Point p1, Point pmid, Point p2){

    Point l1 = pmid - p1;
    Point l2 = pmid - p2;

    double dot = l1.ddot(l2);
    double angle = (std::acos(dot/(cv::norm(l1)*cv::norm(l2))))*(180.0/3.141592653589793238463);

    return angle;
}


Mat filterVideo(Mat frame, int kernelSize, int blurSize){
    Mat filter, canny;

    cvtColor(frame, filter, COLOR_BGR2GRAY);
    medianBlur(filter, filter, blurSize);
    Mat kernel = getStructuringElement(MORPH_RECT, Size(kernelSize, kernelSize));
    //dilate(filter, filter, kernel);
    erode(filter, filter, kernel);

    //threshold(imgSide, cannySide, 10, 255, 0);
    //dilate(cannySide, cannySide, kernel);

    threshold(filter, canny, 25, 255, 0);
    //erode(canny, canny, kernel);

    return canny;
}



void filterContours(vector<vector<Point>> &contours){
    int i = 0;
        vector<vector<Point>>::iterator it = contours.begin();
        bool runLoop = true;
        while(runLoop){

            if (contours[i].empty()){
                contours.erase(it);
                it = contours.begin();
            } else {

                if (i+1 == (int) contours.size()){
                    runLoop = false;
                } else {
                    it++;
                    i++;
                }
            }
        }
}


void threshSegments(magbotShape &magbot, double threshSize) {
    /* Params:
     * Input: src = source image, threshSize = threshold size, decimal value of total image size
     * Output: output image
     * */
    // Stolen from stack overflow: https://stackoverflow.com/questions/19732431/how-to-filter-small-segments-from-image-in-opencv?rq=1
    // FindContours:
    Mat& src = magbot.filtered;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    Mat srcBuffer, output;
    src.copyTo(srcBuffer);
    findContours(srcBuffer, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_TC89_KCOS);

    vector<vector<Point> > allSegments;

    // For each segment:
    for (size_t i = 0; i < contours.size(); ++i) {
        cv::drawContours(srcBuffer, contours, i, Scalar(200, 0, 0), 1, 8, hierarchy, 0, Point());
        cv::Rect brect = cv::boundingRect(contours[i]);
        cv::rectangle(srcBuffer, brect, Scalar(255, 0, 0));

        int result;
        vector<Point> segment;
        for (unsigned int row = brect.y; row < brect.y + brect.height; ++row) {
            for (unsigned int col = brect.x; col < brect.x + brect.width; ++col) {
                result = pointPolygonTest(contours[i], Point(col, row), false);
                if (result == 1 || result == 0) {
                    segment.push_back(Point(col, row));
                }
            }
        }
        allSegments.push_back(segment);
    }

    output = Mat::zeros(src.size(), CV_8UC3);
    int totalSize = output.rows*output.cols;
    vector<vector<Point>> outContours;
    Vec3b whiteColor = Vec3b(255, 255, 255);
    for (int segmentCount = 0; segmentCount < allSegments.size(); ++segmentCount) {
        vector<Point> segment = allSegments[segmentCount];
        if(segment.size() > totalSize*threshSize){
            for (int idx = 0; idx < segment.size(); ++idx) {
                output.at<Vec3b>(segment[idx].y, segment[idx].x) = whiteColor;
            }
            outContours.push_back(contours[segmentCount]);
        }
    }

    magbot.filtered = output;
    magbot.contour = outContours;


}

void blackOut(Mat& img, int cmin, int cmax, int rmin, int rmax){

    for (int i = rmin; i < rmax; i++){
        for (int j=cmin; j < cmax; j++){
            img.at<uchar>(i,j) = 0;
        }
    }
}

vector<Point> findInterestPoints(vector<Point> hull, View view){

    vector<Point> interestPoints;
    // Get interest points

    if (view == View::TOP_VIEW){
        Point maxPoint;

        int ymin = 10000;

        for (int pointCount = 0; pointCount < hull.size(); ++pointCount) {
            Point hullPoint = hull[pointCount];

            if (hullPoint.y < ymin){
                ymin = hullPoint.y;
                maxPoint = hullPoint;
            }
        }

        Point secondPoint;
        int secondymin = 10000;

        for (int pointCount = 0; pointCount < hull.size(); ++pointCount) {
            Point hullPoint = hull[pointCount];

            if (hullPoint.y < secondymin && hullPoint.x < (maxPoint.x -10)){
                secondymin = hullPoint.y;
                secondPoint = hullPoint;
            }
        }

        /*
        interestPoints.push_back(thirdMaxPoint); // Point 2
        // Point 1
        if (maxPoint.x < secondMaxPoint.x) {
            interestPoints.push_back(maxPoint);
        } else {
            interestPoints.push_back(secondMaxPoint);
        } */

        interestPoints.push_back(secondPoint); // Point 2
        interestPoints.push_back(maxPoint); // Point 2

    } else if (view == View::SIDE_VIEW){
         Point firstZero;
         Point secondZero;
         Point xmax;
         Point ymin;
         Point ymax;

         int xPointMax = 0;
         int miny = 10000;
         int maxy = 0;
         int zerocount = 0;

         int miny1 = 10000;
         int maxy1 = 0;


         for (int pointCount = 0; pointCount < hull.size(); ++pointCount) {
             Point hullPoint = hull[pointCount];

             if (hullPoint.x < 130 && hullPoint.y < miny1) {
                 firstZero = hullPoint;
                 zerocount++;
             } else if (hullPoint.x < 130 && hullPoint.y > maxy1){
                 secondZero = hullPoint;
                 zerocount++;
             }

             if (hullPoint.x > xPointMax){
                 xPointMax = hullPoint.x;
                 xmax = hullPoint;
             }

             if (hullPoint.y > maxy){
                 maxy = hullPoint.y;
                 ymax = hullPoint;
             }

             if (hullPoint.y < miny){
                 miny = hullPoint.y;
                 ymin = hullPoint;
             }

           }

         Point p2;
         Point p1;
         if (firstZero.y < secondZero.y) {
             p1 = firstZero;
             p2 = secondZero;
         } else{
             p2 = firstZero;
             p1 = secondZero;
         }

         if (ymax.y > p2.y) {
             interestPoints.push_back(ymax);
             interestPoints.push_back(p2);
         } else {
             interestPoints.push_back(ymin);
             interestPoints.push_back(p1);
         }

    }
    return interestPoints;
}



double getJointAngle(vector<Point> interestPoints, View view){

    double a;
    double b;

    double c = cv::norm(interestPoints[0] - interestPoints[1]);

    double theta;
    // a and b are different depending on view
    // Use CAD to get a and b for each respective view. Use base-to camera transform to convert to pixels
    if (view == View::TOP_VIEW){
        a = 145.11;
        b = 147;
        cout << "Line = " << c << endl;
        theta = std::acos((pow(a, 2) + pow(b, 2) - pow(c, 2))/(2*a*b));
    } else {
        a = 141.17;
        b = 332;
        theta = 3.14159265359 - std::acos((pow(a, 2) + pow(b, 2) - pow(c, 2))/(2*a*b)); // pi - theta
    }

    theta = (180/3.14159265359)*theta;

    return theta;
}

double getJointAngle2(Rect boundRect, double h, double y0){

    double y1 = boundRect.tl().y;

    return (180/3.14159265359)*std::asin((y0-y1)/h);
}


double getJointAngle3_side(Rect boundRect, vector<Point> hull, int tolerance, double bias){

    Vec4f line;
    fitLine(hull, line, DIST_L2, 0, 0.01, 0.01);
    bool upwards;
    if (line[1] < 0){
        upwards = true;
    } else {
        upwards = false;
    }
    vector<Point> topPoints;
    vector<Point> bottomPoints;
    for (int pointCount = 0; pointCount < hull.size(); ++pointCount) {
        Point hullPoint = hull[pointCount];

        if (upwards) {
            if (hullPoint.y < (boundRect.tl().y + tolerance) && hullPoint.y > (boundRect.tl().y - tolerance)){
                topPoints.push_back(hullPoint);
            } else if (hullPoint.x < (boundRect.br().x + tolerance) && hullPoint.x > (boundRect.br().x - tolerance)) {
                bottomPoints.push_back(hullPoint);
            }
        } else {
            if (hullPoint.y < (boundRect.br().y + tolerance) && hullPoint.y > (boundRect.br().y - tolerance)){
                bottomPoints.push_back(hullPoint);
            } else if (hullPoint.x < (boundRect.br().x + tolerance) && hullPoint.x > (boundRect.br().x - tolerance)) {
                topPoints.push_back(hullPoint);
            }
        }
    }

    Point top;
    Point bottom;
    if (topPoints.size() > 1){

        if (upwards){
        int xmax = 0;
        //int xsum = 0;
        //int ysum = 0;
        for (int pointCount = 0; pointCount < topPoints.size(); ++pointCount) {

            Point topPoint = topPoints[pointCount];
            if (topPoint.x > xmax){
                top = topPoint;
                xmax = topPoint.x;
            }
            //xsum += topPoint.x;
            //ysum += topPoint.y;
        }
        //top.x = xsum/topPoints.size();
        //top.y = ysum/topPoints.size();
        } else {
            int ymax = 0;
            //int xsum = 0;
            //int ysum = 0;
            for (int pointCount = 0; pointCount < topPoints.size(); ++pointCount) {

                Point topPoint = topPoints[pointCount];
                if (topPoint.y > ymax){
                    top = topPoint;
                    ymax = topPoint.y;
                }
            }
        }

    } else {
        top = topPoints[0];
    }

    if (bottomPoints.size() > 1){
        if (upwards){
            int ymin = 1000;
            //int xsum = 0;
            //int ysum = 0;
            for (int pointCount = 0; pointCount < bottomPoints.size(); ++pointCount) {
                Point bottomPoint = bottomPoints[pointCount];
                if (bottomPoint.y < ymin){
                    bottom = bottomPoint;
                    ymin = bottomPoint.y;

                }
                //xsum += bottomPoint.x;
                //ysum += bottomPoint.y;
            }
            //bottom.x = xsum/bottomPoints.size();
            //bottom.y = ysum/bottomPoints.size();
        } else {
            int xmax = 0;
            for (int pointCount = 0; pointCount < bottomPoints.size(); ++pointCount) {
                Point bottomPoint = bottomPoints[pointCount];
                if (bottomPoint.x > xmax){
                    bottom = bottomPoint;
                    xmax = bottomPoint.x;
                }
            }

        }
    } else {
        bottom = bottomPoints[0];
    }

    Point l = top - bottom;
    Point yaxis = Point(0, -1);

    double theta;
    if (upwards){
        theta = (180/3.14159265359)*std::acos((l.x*yaxis.x + l.y*yaxis.y)/(norm(l)*norm(yaxis))) - bias;
    } else {
        theta = -((180/3.14159265359)*std::acos((l.x*yaxis.x + l.y*yaxis.y)/(norm(l)*norm(yaxis))) + bias);
    }


    return theta;
}

double getJointAngle_Top(vector<Point> interestPoints){
    Point yaxis = Point(0, -1);
    Point l = interestPoints[1] - interestPoints[0];

    double theta = (180/3.14159265359)*std::acos((l.x*yaxis.x + l.y*yaxis.y)/(norm(l)*norm(yaxis)));

    return 180 - 2*theta;
}

double getJointAngle_Top2(vector<Point> contour, int cutoff){
    vector<Point> interestPoints;
    for (int pointCount = 0; pointCount < contour.size(); ++pointCount) {
        Point point = contour[pointCount];
        if (point.x < cutoff){
            interestPoints.push_back(point);
        }
    }

    Vec4f line;
    fitLine(contour, line, DIST_L12, 0, 0.01, 0.01);
    Point yaxis = Point(0, 1);
    int dx = (int) (line[0]*1000);
    int dy = (int) (line[1]*1000);
    Point l = Point(dx, dy);
    return (180/3.14159265359)*std::acos((l.x*yaxis.x + l.y*yaxis.y)/(norm(l)*norm(yaxis)));
}


vector<Point> findInterestPoints_Top(vector<Point> hull, Rect boundingRect, int threshold){

    vector<Point> interestPoints;

    Point maxPoint;

    int ymin = 10000;

    for (int pointCount = 0; pointCount < hull.size(); ++pointCount) {
        Point hullPoint = hull[pointCount];

        if (hullPoint.y < ymin){
            ymin = hullPoint.y;
            maxPoint = hullPoint;
        }
    }

    Point secondPoint;
    int secondymin = 10000;

    for (int pointCount = 0; pointCount < hull.size(); ++pointCount) {
        Point hullPoint = hull[pointCount];

        if (hullPoint.y < secondymin && hullPoint.x < (maxPoint.x -10)){
            secondymin = hullPoint.y;
            secondPoint = hullPoint;
        }
    }

    Point leftPoint;
    int x = 0;
    int y = 0;
    int counter = 0;
    for (int pointCount = 0; pointCount < hull.size(); ++pointCount) {
        Point hullPoint = hull[pointCount];

        if(hullPoint.x < (boundingRect.tl().x + threshold) && hullPoint.x > (boundingRect.tl().x - threshold)) {
            x += hullPoint.x;
            y += hullPoint.y;
            counter++;
        }
    }

    leftPoint.x = x/counter;
    leftPoint.y = y/counter;

    Point p1;
    p1.x = (leftPoint.x + secondPoint.x)/2;
    p1.y = (leftPoint.y + secondPoint.y)/2;
    interestPoints.push_back(p1);
    interestPoints.push_back(maxPoint);

    return interestPoints;

}

void processMagbot(magbotShape& magbot)
{

    if (magbot.view == SIDE_VIEW && magbot.isFlipped) {
        flip(magbot.rawImage, magbot.rawImage, 1);
    }

    magbot.filtered = filterVideo(magbot.rawImage, 10, 15);
    blackOut(magbot.filtered, 0, 130, 0, magbot.filtered.rows); // Adjust this depending on image
    threshSegments(magbot, 0.04);

    Mat& canny = magbot.filtered;
    View& view = magbot.view;
    vector<vector<Point>>& contour = magbot.contour;

    if (contour.size() > 1 ){
        cout << "More than one area identified, consider increasing threshold" << endl;
    }


    if (!contour.empty()){

        vector<Point> hull;
        convexHull(contour[0], hull, true);
        Rect boundRect = boundingRect(contour[0]);

        rectangle(canny, boundRect, Scalar(255, 0, 0));

        polylines(canny, hull, true, Scalar(0, 255, 0), 1, LINE_AA);

        double ja;
        if (view == SIDE_VIEW){
            // Custom function for calculating joint angle from side
            ja = getJointAngle3_side(boundRect, contour[0], 2, 3.88);
        } else {
            Mat draw = Mat::zeros(canny.size(), CV_8UC3);
            vector<Point> interestPoints = findInterestPoints_Top(hull, boundRect, 1);  //findInterestPoints(hull, s.view);

            line(draw, interestPoints[0], interestPoints[1], Scalar(255, 0, 0), 3);
            for (int pointCount = 0; pointCount < hull.size(); ++pointCount) {
                Point hullPoint = hull[pointCount];
                //circle(draw, hullPoint, 3, Scalar(0, 255, 255));
                putText(draw, to_string(pointCount), hullPoint, FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 255));
            }
            magbot.pointPlot = draw;

            // Function for calculating from top, this one works best
            ja = getJointAngle_Top(interestPoints);
        }

        magbot.jointAngle = ja;
     }

}



}
