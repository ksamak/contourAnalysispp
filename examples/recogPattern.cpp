#include "templateMatch.h"
#include "contourTemplate.h"
#include "templateFinder.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include "opencv2/contrib/contrib.hpp"

#include <tuple>
#include <iostream>
#include <vector>

#define DOWNSCALE_RATIO 2

capp::TemplateFinder _finder;

cv::Mat _source, _processed;
cv::VideoCapture capture;
cv::Mat image;
bool image_mode = false;

std::vector<capp::ContourTemplate> recog_patterns;
capp::ContourTemplate pattern;

int minACF = 96;
int minICF = 85;
int resizeVal = 0;
float aspectRatio = 0;
int blurVal = 1;
int cannyVal = 0;
int adaptiveThresholdVal = 4;
int threshold_kernel_size = 26;
int thresholdVal = 0;
//Unsharp mask values
int unsharp_radius = 15;
int unsharp_sigma = 1000;  // TODO adapt size of gaussian to ratio of characters.
int unsharp_amount = 1;
int unsharp_threshold = 0;

cv::Mat unsharpMask(const cv::Mat &source, int radius, double sigma, int amount, int threshold1, int threshold2) {
    cv::Mat blurred;
    GaussianBlur(source, blurred, cv::Size(radius, radius), sigma, sigma);
    cv::Mat lowContrastMask = abs(source - blurred) < threshold1;
    cv::Mat sharpened = source * (1 + amount) + blurred * (-amount);
    source.copyTo(sharpened, lowContrastMask);
    if ( threshold2 > 0 ) {
        cv::threshold(sharpened, sharpened, threshold2, 255, CV_THRESH_BINARY);
    }
    return sharpened;
}


std::vector<std::vector<cv::Point>> filterContours(std::vector<std::vector<cv::Point>> raw, cv::Mat canny_) {
    static const unsigned int minContourLength = 15;
    static const unsigned int minContourArea = 10;
    static const unsigned int minFormFactor = 5;

    int maxArea = canny_.rows * canny_.cols / 1.1; // arbitrary
    //int maxArea = canny_.rows * canny_.cols / 5; // arbitrary

    std::vector<std::vector<cv::Point>> res;

    for (auto const &cont : raw) {
        double contArea = cv::contourArea(cont);
//        std::cout << "max area " << maxArea << std::endl;
//        std::cout << "contour size " << cont.size() << std::endl;
//        std::cout << "contArea " << contArea << std::endl;
//        std::cout << "contArea / size" << contArea / cont.size() << std::endl << std::endl;
        if(cont.size() < minContourLength ||
          contArea < minContourArea ||
          contArea > maxArea ||
          contArea / cont.size() <= minFormFactor) {
            continue;
        }
       // cv::Point p1 = cont.front();
       // cv::Point p2 = cont.at(cont.size() / 2);
       // std::cout << "canny_.at(): " << static_cast<int>(canny_.at<char>(p1)) << std::endl;
       // if (canny_.at<char>(p1) <= 1 // FIXME check that, he uses doubleEpsilon in cCrap#
       //         && canny_.at<char>(p2) <= 1) {
       //     continue;
       // }
        res.push_back(cont);
    }
    return res;
}

capp::Contour cvContour2ComplexVector(const std::vector<cv::Point>& rawContour) {
    capp::Contour contour;
    auto previous = rawContour.begin();
    for (auto point = rawContour.begin() + 1; point != rawContour.end(); ++point) {
        contour.push_back(capp::CVector(point->y - previous->y, point->x - previous->x));
        previous = point;
    }
    return contour;
}

std::vector<std::tuple<capp::ContourTemplate, std::vector<cv::Point>>> generateContours(const cv::Mat& source) {
    cv::Mat proc_ = source.clone();
    if (proc_.type() == CV_8UC3) {
        cv::cvtColor(proc_, proc_, CV_BGR2GRAY);
    }
    cv::Mat canny_, smoothed;
    std::vector<std::tuple<capp::ContourTemplate, std::vector<cv::Point>>> res;

    cv::equalizeHist(proc_, proc_);

    cv::pyrDown(proc_, smoothed, cv::Size(source.cols / DOWNSCALE_RATIO, source.rows / DOWNSCALE_RATIO));
    cv::pyrUp(smoothed, smoothed, cv::Size(source.cols, source.rows));
    cv::Canny(smoothed, canny_, 50, 50);

    // possible noise filter, unsharp mask, blur
    cv::adaptiveThreshold(proc_, proc_, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, 5, 1.2);
    cv::bitwise_not(proc_, proc_);
    cv::bitwise_or(proc_, canny_, proc_);

    std::vector<std::vector<cv::Point>> contours_raw, contours_filtered;
    cv::findContours(proc_, contours_raw, CV_CHAIN_APPROX_NONE, CV_RETR_LIST);
    contours_filtered = filterContours(contours_raw, canny_);

    /////////////////////////////// TODO REMOVE
    cv::Mat proc2_ = cv::Mat::zeros(source.rows, source.cols, CV_8UC1);
    cv::Mat proc3_ = cv::Mat::zeros(source.rows, source.cols, CV_8UC1);
    cv::drawContours(proc2_, contours_filtered, -1, cv::Scalar(255));
    cv::drawContours(proc3_, contours_raw, -1, cv::Scalar(255));
    cv::imshow("contoursRaw", proc3_);
    cv::imshow("contoursFiltered", proc2_);
    //////////////////////////////////////////

    // pragma //
    for (auto const& rawCont : contours_filtered) {
        auto area = cv::contourArea(rawCont);
        res.push_back(std::make_tuple(capp::ContourTemplate(cvContour2ComplexVector(rawCont), area), rawCont));
    }
    return res;
}

std::tuple<capp::ContourTemplate, std::vector<cv::Point>> generateTemplate(const cv::Mat& sourcee) {
    auto candidates = generateContours(sourcee);
    double sourceArea = 0;
    capp::ContourTemplate res;
    std::vector<cv::Point> resPoints; // TODO setOriginalContour?
    for (auto const& candidate_tuple : candidates) {
        auto const& candidate = std::get<0>(candidate_tuple);
        if (candidate.getSourceArea() > sourceArea) {
            sourceArea = candidate.getSourceArea();
            res = candidate;
            resPoints = std::get<1>(candidate_tuple);
        }
    }
    return std::make_tuple(res, resPoints);
}

void processImg(cv::Mat &processed) {
    if (blurVal != 0) {
        cv::GaussianBlur(processed, processed, cv::Size(7,7), blurVal/10., blurVal/10.);
    }
    if (unsharp_amount != 0) {
        if (unsharp_radius % 2 == 0) {
            unsharp_radius ++;
        }
        processed = unsharpMask(processed, unsharp_radius, unsharp_sigma / 100. , unsharp_amount, unsharp_threshold, 0);
    }
    if (thresholdVal !=0) {
        cv::threshold(processed, processed, thresholdVal, 255, CV_THRESH_BINARY);
    }
    if (adaptiveThresholdVal !=0) {
        if (processed.type() == CV_8UC3) {
            cv::cvtColor(processed, processed, CV_BGR2GRAY);
        }
        if (threshold_kernel_size % 2 == 0) {
            threshold_kernel_size ++;
        }
        cv::adaptiveThreshold(processed, processed, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, threshold_kernel_size, adaptiveThresholdVal);
    }
//    processed = cv::Mat::zeros(200, 200, CV_8UC1);
//    cv::rectangle(processed, cv::Point(10, 10), cv::Point(100, 100), cv::Scalar(255), 1);
}

void initAntipatterns() {
    // patterns to prevent portnawak
//    cv::Mat img = cv::Mat::zeros(200, 200, CV_8UC1);
//    cv::rectangle(img, cv::Point(10, 10), cv::Point(190, 190), cv::Scalar(255), 1);
//    auto antip = std::get<0>(generateTemplate(img));
//    antip.setName("antipattern");
//    recog_patterns.push_back(antip);
//
//    img = cv::Mat::zeros(200, 200, CV_8UC1);
//    cv::circle(img, cv::Point(100, 100), 50, cv::Scalar(255), 1);
//    antip = std::get<0>(generateTemplate(img));
//    antip.setName("antipattern");
//    recog_patterns.push_back(antip);
    //cv::imshow("pattern_looked_for", img);
}

void initCustomPatterns(std::vector<cv::Mat> input) {  // chooses three patterns, from the image, then adds it to be recognized
    std::cout << "Initializing custom patterns" << std::endl;
    if (input.empty()) {
        for (int i = 0; i < 3 ; i++) {
            std::cout << "pattern" << i;
            for (int i = 0; i < 15; i++) { // skip some frames
                capture >> _source;
            }
            capp::ContourTemplate patt;
            std::vector<cv::Point> points;
            while (patt.empty()) {
                capture >> _source;
                _processed = _source.clone();
                processImg(_processed);
                std::tie(patt, points) = generateTemplate(_processed);
            }
            patt.setName("pattern");
            recog_patterns.push_back(patt);
            std::cout << " done" << std::endl;
            std::vector<std::vector<cv::Point>> temp;
            temp.push_back(points);
            cv::drawContours(_source, temp, -1, cv::Scalar(255));
            cv::imshow("Playground", _source);
            cv::waitKey(4000);
        }
    } else {
        for (auto const& im : input) {
            capp::ContourTemplate patt;
            std::vector<cv::Point> points;
            std::tie(patt, points) = generateTemplate(im);
            patt.setName("pattern");
            std::vector<std::vector<cv::Point>> temp;
            temp.push_back(points);
            cv::drawContours(_source, temp, -1, cv::Scalar(255));
            cv::imshow("Playground", _source);
            cv::waitKey(800);
            assert(patt.getContour().size() >= 0);
            recog_patterns.push_back(patt);

        }
    }
}


void locatePattern(const cv::Mat& source, const std::vector<capp::ContourTemplate>& templates) {
    auto f = generateContours(source);
    std::vector<std::vector<cv::Point>> toDraw;
    for (auto const& sample : f) {
        auto result = _finder.findTemplate(templates, std::get<0>(sample));
        if (!result.empty()) {
            toDraw.push_back(std::get<1>(sample));
            std::cout << "angle " << result.getAngle() * 180 / 6.28 << " rate: " << result.getRate() << " template: " << result.getTemplate().getName() << std::endl;
        }
    }
    cv::drawContours(_source, toDraw, -1, cv::Scalar(0, 0, 255), 3);
}

void draw(int, void*) {
    _processed = _source.clone();

    if (resizeVal != 0) {
        cv::resize(_processed, _processed, cv::Size((resizeVal)*(aspectRatio), resizeVal));
    }

    processImg(_processed);

    locatePattern(_processed, recog_patterns);
    cv::imshow("Playground", _processed);
    cv::imshow("results", _source);
}

void setICF(int min, void*) {
    _finder.setMinICF(min / 100.);
}

void setACF(int min, void*) {
    _finder.setMinACF(min / 100.);
}

void initGUI() {
    aspectRatio = 1.0 * static_cast<float>(_source.cols) / static_cast<float>(_source.rows);

    cv::namedWindow("sliders", CV_WINDOW_NORMAL);
    cv::namedWindow("Playground", CV_WINDOW_NORMAL);

    // Put in desired order if necessary. //

    //playground trackbars.
    cv::createTrackbar("resize", "sliders", &resizeVal, _source.rows, *draw);
    cv::createTrackbar("gaussian blur", "sliders", &blurVal, 50, *draw);

    // Unsharp mask sliders. //
    cv::createTrackbar("unsharp gaussian radius", "sliders", &unsharp_radius, 50, *draw);
    cv::createTrackbar("unsharp gaussian Sigma (x100)", "sliders", &unsharp_sigma, 3000, *draw);
    //cv::createTrackbar("unsharp gaussian Sigma (x1000)", "sliders", &unsharp_sigma, 1000, *draw); // TODO sigma 2
    cv::createTrackbar("unsharp amount", "sliders", &unsharp_amount, 100, *draw);
    cv::createTrackbar("unsharp threshold", "sliders", &unsharp_threshold, 255, *draw);

//    cv::createTrackbar("edge detection", "sliders", &cannyVal, 150);
    cv::createTrackbar("min ACF", "sliders", &minACF, 100, *setACF);
    cv::createTrackbar("min ICF", "sliders", &minICF, 100, *setICF);
    cv::createTrackbar("adaptive threshold", "sliders", &adaptiveThresholdVal, 50, *draw);
    cv::createTrackbar("adaptive threshold kernel size", "sliders", &threshold_kernel_size, 50, *draw);
    cv::createTrackbar("simple threshold value", "sliders", &thresholdVal, 255, *draw);
}

int main(int argc, char** argv) {
    argc = argc; // anti warnings
    argv = argv;
    _finder = capp::TemplateFinder();
    capture = cv::VideoCapture(-1);
    if(!capture.isOpened()) {
        std::cout << "problem opening camera" << std::endl;
        image = cv:: imread(argv[1], CV_LOAD_IMAGE_COLOR);
        _source = image;
        image_mode = true;
    } else {
        capture >> _source;
    }

    std::vector<cv::Mat> patternImages;
    for (int i = 1; i < argc; i++) {
        patternImages.push_back(cv::imread(argv[i]));
    }
    initAntipatterns();
    initCustomPatterns(patternImages);

    //::exit(0);
    initGUI();
    cv::waitKey(10);
    while(true) {
        if (!image_mode) {
            capture >> _source;
        }
        draw(0, NULL);
        if (cv::waitKey(1) == 113) {
            ::exit(0);
        }
    }
}
