//
// Created by student on 5.5.25.
//

#ifndef ARUCO_DETECTOR_HPP
#define ARUCO_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

namespace algorithms {

    class ArucoDetector {
    public:

        // Represents one detected marker
        struct Aruco {
            int id;
            std::vector<cv::Point2f> corners;

            Aruco(int marker_id, const std::vector<cv::Point2f>& marker_corners) : id(marker_id), corners(marker_corners) {}
        };

        ArucoDetector() {
            // Initialize dictionary with 4x4 markers (50 possible IDs)
            dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        }

        ~ArucoDetector() = default;

        // Detect markers in the input image
        std::vector<Aruco> detect(cv::Mat frame) {
            std::vector<Aruco> arucos;

            std::vector<int> marker_ids;
            std::vector<std::vector<cv::Point2f>> marker_corners;

            // Detect markers using OpenCV
            cv::aruco::detectMarkers(frame, dictionary_, marker_corners, marker_ids);

            if (!marker_ids.empty()) {
                std::cout << "Arucos found: ";
                for (size_t i = 0; i < marker_ids.size(); i++) {
                    std::cout << marker_ids[i] << " ";

                    // Create Aruco struct and add to result vector
                    arucos.emplace_back(marker_ids[i], marker_corners[i]);
                }
                std::cout << std::endl;
            }

            return arucos;
        }

    private:
        cv::Ptr<cv::aruco::Dictionary> dictionary_;
    };
}


#endif //ARUCO_DETECTOR_HPP
