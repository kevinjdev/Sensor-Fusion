
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
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

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


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
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
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    // Populate the keyPointMatches
    float totalKptMatchDistance = 0.0;
    float meanThreshold = 1.02;
    for (cv::DMatch match: kptMatches)
    {
        if (boundingBox.roi.contains(kptsCurr[match.trainIdx].pt))
        {
            boundingBox.kptMatches.push_back(match);
            totalKptMatchDistance += match.distance;
        }
    }
    // std::cout << "Added " << boundingBox.kptMatches.size() << " matches to the bounding box with Id " << boundingBox.boxID << std::endl;

    // Erase outliers if distance is greater than mean threshold
    if (boundingBox.kptMatches.size() > 0)
    {
        float meanKptMatchDistance = totalKptMatchDistance / boundingBox.kptMatches.size();

        for (int i = 0; i < boundingBox.kptMatches.size(); ++i)
        {
            if (boundingBox.kptMatches[i].distance > (meanKptMatchDistance * meanThreshold))
            {
                boundingBox.kptMatches.erase(boundingBox.kptMatches.begin() + i);
            }
        }

        // std::cout << "Bounding box matches after removing outliers: " << boundingBox.kptMatches.size() << std::endl;
        // std::cout << "Bounding box keypoints size " << boundingBox.keypoints.size() << std::endl;
    }    
}

// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // std::cout << "Keypoints Prev: " << kptsPrev.size() << "Keypoints Curr " << kptsCurr.size() << std::endl;
    // std::cout << "Camera kptMatchers size " << kptMatches.size() << std::endl;
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

    // compute camera-based TTC from distance ratios
    // double meanDistRatio = std::accumulate(distRatios.begin(), distRatios.end(), 0.0) / distRatios.size();
    // std::cout << "meanDistRatio " << meanDistRatio << " distRatios size " << distRatios.size() << std::endl;
    std::sort(distRatios.begin(), distRatios.end());
    double medianDistRatio = (distRatios.size() % 2 == 0) ? ((distRatios[distRatios.size() / 2] + distRatios[(distRatios.size() / 2) - 1]) / 2) : distRatios[distRatios.size() / 2];
    // std::cout << "medianDistRatio " << medianDistRatio << std::endl;

    double dT = 1 / frameRate;
    // TTC = -dT / (1 - meanDistRatio);
    TTC = -dT / (1 - medianDistRatio);
    std::cout << "Median TTC camera is " << TTC << std::endl;
    // std::cout << "Mean TTC Camera is " << TTC << std::endl;
    // STUDENT TASK (replacement for meanDistRatio)
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    double dT = 1 / frameRate;        // time between two measurements in seconds
    
    std::sort(lidarPointsPrev.begin(), lidarPointsPrev.end(), [] (const LidarPoint& l1, const LidarPoint& l2)-> bool
    {
        return l1.x > l2.x;
    });
    double medianXPrev = (lidarPointsPrev.size() % 2 == 0) ? ((lidarPointsPrev[lidarPointsPrev.size() / 2].x + lidarPointsPrev[(lidarPointsPrev.size() / 2) - 1].x) / 2) : lidarPointsPrev[lidarPointsPrev.size() / 2].x;

    std::sort(lidarPointsCurr.begin(), lidarPointsCurr.end(), [] (const LidarPoint& l1, const LidarPoint& l2)-> bool
    {
        return l1.x > l2.x;
    });
    double medianXCurr = (lidarPointsCurr.size() % 2 == 0) ? ((lidarPointsCurr[lidarPointsCurr.size() / 2].x + lidarPointsCurr[(lidarPointsCurr.size() / 2) - 1].x) / 2) : lidarPointsCurr[lidarPointsCurr.size() / 2].x;

    TTC = medianXCurr * dT / (medianXPrev - medianXCurr);
    std::cout << "MedianXCurr: " << medianXCurr << " dt " << dT << " medianXPrev " << medianXPrev << std::endl;
    std::cout << "TTC Lidar is: " << TTC << std::endl;
}

void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    std::multimap<int,int> box_candidates;
    
    // Iterate through all matches and store the Box index of each keypoint match
    // between the previous and the current Image if the match is in a BoundingBox
    for (int i = 0; i < matches.size(); ++i)
    {   
        bool keypointIsInPrevFrameBox = false;
        bool keypointIsInCurrFrameBox = false;
        int boundingBoxIdPrevFrame = -1;
        int boundingBoxIdCurrFrame = -1;
        
        // Iterate through the Previous and Current images' bounding boxes checking if the keypoint is in a bounding box
        // If it is, then save the boxId associated with the keypoint
        for(int j = 0; j < prevFrame.boundingBoxes.size(); ++j)
        {
            if (prevFrame.boundingBoxes[j].roi.contains(prevFrame.keypoints[matches[i].queryIdx].pt))
            {
                keypointIsInPrevFrameBox = true;
                boundingBoxIdPrevFrame = prevFrame.boundingBoxes[j].boxID;
                break;
            }
        }
        for(int j = 0; j < currFrame.boundingBoxes.size(); ++j)
        {
            if (currFrame.boundingBoxes[j].roi.contains(currFrame.keypoints[matches[i].trainIdx].pt))
            {
                keypointIsInCurrFrameBox = true;
                boundingBoxIdCurrFrame = currFrame.boundingBoxes[j].boxID;
                break;
            }
        }

        // If the keypoint match from the previous and current images are both in a bounding box,
        // then store the boxIds in a multimap
        if (keypointIsInPrevFrameBox && keypointIsInCurrFrameBox)
        {
            // std::cout << "found a match: " << boundingBoxIdPrevFrame << " , " << boundingBoxIdCurrFrame << std::endl;

            box_candidates.insert(std::pair<int,int>(boundingBoxIdPrevFrame, boundingBoxIdCurrFrame));

        }
    }
    // Now that we have a list of all matches between each previous Image's bounding boxes and the current image,
    // Find the most common Bounding box in the current frame that has been associated with the prevFrame boundingbox
    for (int i = 0; i < prevFrame.boundingBoxes.size(); ++i)
    {
        std::vector<int> currFrame_box_candidates;
        
        // Get a list of the matches from the current image that are associated with the previous image's bounding box
        auto it = box_candidates.equal_range(prevFrame.boundingBoxes[i].boxID);
        for (auto itr = it.first; itr != it.second; ++itr)
        {
            // std::cout << "potential match for previous bounding box id: " << prevFrame.boundingBoxes[i].boxID << " is " << itr->second << std::endl;
            currFrame_box_candidates.push_back(itr->second);
        }
        // Find the most common occuring element in the second bounding box
        if (currFrame_box_candidates.size() > 0)
        {
            std::map<int,int> box_candidates_freq;
            for(auto i : currFrame_box_candidates)
            {
                ++box_candidates_freq[i];
            }
            // find the max value which will be the match for
            auto maxElement = std::max_element(box_candidates_freq.begin(), box_candidates_freq.end(), 
                [] (const pair<int,int>& p1, const pair<int,int>& p2)
                {
                    return p1.second < p2.second;
                });
            int currBoxMatch = maxElement->first;
            // std::cout << "most common match for previous bounding box id: " << prevFrame.boundingBoxes[i].boxID << " is " << currBoxMatch << std::endl; 
            bbBestMatches.insert(std::pair<int,int>(prevFrame.boundingBoxes[i].boxID,currBoxMatch));
        }
    }
}
