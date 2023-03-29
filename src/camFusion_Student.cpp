
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;

// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0);
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        {
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}

/*
 * The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size.
 * However, you can make this function work for other sizes too.
 * For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
 */
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for (auto it1 = boundingBoxes.begin(); it1 != boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0, 150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top = 1e8, left = 1e8, bottom = 0.0, right = 0.0;
        float xwmin = 1e8, ywmin = 1e8, ywmax = -1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin < xw ? xwmin : xw;
            ywmin = ywmin < yw ? ywmin : yw;
            ywmax = ywmax > yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top < y ? top : y;
            left = left < x ? left : x;
            bottom = bottom > y ? bottom : y;
            right = right > x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 0, 0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left - 250, bottom + 50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax - ywmin);
        putText(topviewImg, str2, cv::Point2f(left - 250, bottom + 125), cv::FONT_ITALIC, 2, currColor);
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 0);
    cv::imshow(windowName, topviewImg);

    if (bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}

// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    /**
     * @brief: 
     * 1. Loop through the matched points and check if they are inside the bounding box
     * 2. Log the points that are inside the bounding box and compute their distance
     * 3. Set a distance threshold based on the mean distance value, for example 1.2 times the mean distance value
     * 4. Loop through the points logged in step 1 and add them to the bounding box keypoints if their distance is lower than the threshold
    */
    vector<pair<double,std::vector<cv::DMatch>::iterator>> kptInBoundingBox;

    double distThr = 0.0;
    for(std::vector<cv::DMatch>::iterator itr = kptMatches.begin(); itr != kptMatches.end(); ++itr){

        const cv::KeyPoint &prevKpt = kptsPrev[itr -> queryIdx];
        const cv::KeyPoint &currKpt = kptsCurr[itr -> trainIdx];

        if(boundingBox.roi.contains(currKpt.pt) == true){
            double&& tempDist = cv::norm(currKpt.pt-prevKpt.pt);
            kptInBoundingBox.emplace_back(tempDist, itr);
            distThr += tempDist;
        }
    }

    distThr /= kptInBoundingBox.size();
    distThr *= 1.2;

    for(pair<double,std::vector<cv::DMatch>::iterator> kpt : kptInBoundingBox){
        if(kpt.first < distThr){
            const cv::KeyPoint &currKpt = kptsCurr[kpt.second->trainIdx];
            
            boundingBox.keypoints.emplace_back(currKpt);
            boundingBox.kptMatches.emplace_back(*kpt.second);
        }
    }
}

// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr,
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // compute distance ratios between all matched keypoints
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer kpt. loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner kpt.-loop

            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    std::sort(distRatios.begin(), distRatios.end());
    long medIndex = floor(distRatios.size() / 2.0);
    double medDistRatio = distRatios.size() % 2 == 0 ? (distRatios[medIndex - 1] + distRatios[medIndex]) / 2.0 : distRatios[medIndex]; // compute median dist. ratio to remove outlier influence

    double dT = 1 / frameRate;
    TTC = -dT / (1 - medDistRatio);
}

pair<double, double> normalDistribution(const std::vector<LidarPoint> &points){
    double mean = 0.0;
    for(const LidarPoint& point : points)
        mean += point.x;
    mean /= points.size();

    double stdDev = 0.0;
    for (const LidarPoint &point : points)
        stdDev += pow((mean - point.x), 2);

    stdDev = sqrt(stdDev / points.size());

    return pair<double,double>(mean, stdDev);
}

    void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev, std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    /**
     * @brief:
     * 1. normal distribution - filter out nonliners
    */

    sort(lidarPointsPrev.begin(), lidarPointsPrev.end(), [](const LidarPoint &lhs, const LidarPoint &rhs){ return lhs.x < rhs.x; });
    sort(lidarPointsCurr.begin(), lidarPointsCurr.end(), [](const LidarPoint &lhs, const LidarPoint &rhs){ return lhs.x < rhs.x; });

    pair<double, double> prevNormal = normalDistribution(lidarPointsPrev);
    pair<double, double> currNormal = normalDistribution(lidarPointsCurr);
    double minXPrevRaw = lidarPointsPrev.front().x;
    double minXCurrRaw = lidarPointsCurr.front().x;

    double scale = 1.0;

    std::vector<LidarPoint> prevPointsFilter;
    double&& lowThr = prevNormal.first - scale * prevNormal.second;
    double&& highThr = prevNormal.first + scale * prevNormal.second;

    for (const LidarPoint &point : lidarPointsPrev)
    {
        if(point.x >= lowThr && point.x <= highThr)
            prevPointsFilter.emplace_back(point);
    }

    std::vector<LidarPoint> currPointsFilter;
    lowThr = currNormal.first - scale * currNormal.second;
    highThr = currNormal.first + scale * currNormal.second;

    for (const LidarPoint &point : lidarPointsCurr)
    {
        if (point.x >= lowThr && point.x <= highThr)
            currPointsFilter.emplace_back(point);
    }

    double minXPrev = prevPointsFilter.front().x;
    double minXCurr = currPointsFilter.front().x;

    double &&dt = 1.0 / frameRate;
    double &&speed = (minXPrev - minXCurr) / dt;  //or minXPrevRas  minXCurrRaw 

    TTC = minXCurr / speed;

    lidarPointsPrev = move(prevPointsFilter);
    lidarPointsCurr = move(currPointsFilter);
    cout << "[+] Min. distance raw data: " << minXCurrRaw << " min. distance filter: " << minXCurr << endl;
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    /**
     * @brief: 
     * 1. Use matches to find the links between bounding boxes in previous and current frame
     * 2. Consider a point could fall in multiple bounding boxes, use a 2D array to count how
     *    many points fall in previous and current frames bounding boxes
     * 3. Pick pairs in each row with highest scores
     */
    int &&prevBoxlen = prevFrame.boundingBoxes.size();
    int &&currBoxlen = currFrame.boundingBoxes.size();
    vector<vector<size_t>> count(prevBoxlen, vector<size_t>(currBoxlen, 0U));

    vector<bool> inPrevBoxLog;
    vector<bool> inCurrBoxLog;

    // time complexity o(n): n * (boxes prev + boxes curr)
    for (const cv::DMatch &match : matches){

        const cv::KeyPoint &prevKpt = prevFrame.keypoints[match.queryIdx];
        const cv::KeyPoint &currKpt = currFrame.keypoints[match.trainIdx];

        //check if a match keypoint pair in previous and current boundingboxes 
        for (int prevBoxId = 0; prevBoxId < prevBoxlen; ++prevBoxId){
            if (prevFrame.boundingBoxes[prevBoxId].roi.contains(prevKpt.pt) != true)
                continue;
            
            for (int currBoxId = 0; currBoxId < currBoxlen; ++currBoxId){
                if (currFrame.boundingBoxes[currBoxId].roi.contains(currKpt.pt) == true)
                    ++count[prevBoxId][currBoxId];
            }             
        }

    }

    // find the best pairs
    for (int prevBoxId = 0; prevBoxId < prevBoxlen; ++prevBoxId){
        int maxPts = 0;
        int bestCurrBoxId = -1;
        for (int currBoxId = 0; currBoxId < currBoxlen; ++currBoxId){
            if(count[prevBoxId][currBoxId] > maxPts){
                maxPts = count[prevBoxId][currBoxId];
                bestCurrBoxId = currBoxId;
            }     
        }
        
        if(bestCurrBoxId != -1)
            bbBestMatches[prevFrame.boundingBoxes[prevBoxId].boxID] = currFrame.boundingBoxes[bestCurrBoxId].boxID;
    }
}
