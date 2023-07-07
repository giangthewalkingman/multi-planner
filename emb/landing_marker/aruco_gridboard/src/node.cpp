
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "node.h"
#include "names.h"

static const std::string OPENCV_WINDOW = "Image window";

namespace aruco_gridboard
{
    Node::Node() :
        n_("~"),
        queue_size_(1),
        board_path_(),
        model_description_(),
        detector_param_path_(),
        camera_frame_name_(),
        debug_display_(false),
        status_tracker_(false),
        cv_ptr(),
        image_header_(),
        got_image_(false),
        lastHeaderSeq_(0)
    {
        //get the tracker configuration file
        //this file contains all of the tracker's parameters, they are not passed to ros directly.
        n_.param<std::string>("board_path", board_path_, "");
        n_.param<bool>("debug_display", debug_display_, false);
        n_.param<std::string>("detector_param_path", detector_param_path_, "");
        n_.param<std::string>("camera_frame_name", camera_frame_name_, "camera_color_optical_frame");
        n_.param("frequency", freq_, 30);
        n_.param("camera_offset_x", camera_offset_x_, 0.0);
        n_.param("camera_offset_y", camera_offset_y_, 0.0);
        n_.param("camera_offset_z", camera_offset_z_, 0.0);
        posePub_ = n_.advertise<geometry_msgs::Point>("/landing/marker_pose/aruco",1);
        ROS_INFO("Detector parameter file =%s",detector_param_path_.c_str());
        ROS_INFO("Board config file: =%s",board_path_.c_str());
    }

    Node::~Node()
    {
    }

    void Node::waitForImage()
    {
        while ( ros::ok ()){
            if(got_image_) return;
            ros::spinOnce();
        }
    }

    //Read parameters from a file
    static bool readDetectorParameters(std::string filename, cv::Ptr<cv::aruco::DetectorParameters> &params)
    {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if(!fs.isOpened())
            return false;
        fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
        fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
        fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
        fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
        fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
        fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
        fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
        fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
        fs["minDistanceToBorder"] >> params->minDistanceToBorder;
        fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
        fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
        fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
        fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
        fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
        fs["markerBorderBits"] >> params->markerBorderBits;
        fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
        fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
        fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
        fs["minOtsuStdDev"] >> params->minOtsuStdDev;
        fs["errorCorrectionRate"] >> params->errorCorrectionRate;
        return true;
    }

    void Node::imageCallback(const sensor_msgs::ImageConstPtr & image)
    {
        image_header_ = image->header;
        try
        {
            boost::mutex::scoped_lock(lock_);
            cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        got_image_ = true;
    }

    void Node::camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& cam_info)
    {
        try
        {
            camera_model_.fromCameraInfo(cam_info);

            camMatrix_ = cv::Mat(camera_model_.fullIntrinsicMatrix());
            distCoeffs_= cv::Mat(camera_model_.distortionCoeffs());
            if (distCoeffs_.size[1] < 4)
                distCoeffs_ = cv::Mat(5, 1, CV_64FC1, cv::Scalar::all(0));
            //ROS_INFO("Camera info %f", distCoeffs_.at<double>(0, 0));
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

  
    void Node::spin()
    {
        ros::Subscriber caminfo_sub = n_.subscribe(camera_info_topic, 1, &Node::camInfoCallback, this);
        image_transport::ImageTransport imt(n_);
        image_transport::Subscriber img_sub = imt.subscribe(image_topic, 1, &Node::imageCallback, this);

        // Define Publisher
        ros::Publisher object_pose_publisher = n_.advertise<geometry_msgs::PoseStamped>(object_position_topic, queue_size_);
        ros::Publisher camera_pose_publisher = n_.advertise<geometry_msgs::PoseStamped>(camera_position_topic, queue_size_);
        ros::Publisher status_publisher = n_.advertise<std_msgs::Int8>(status_topic, queue_size_);

        //wait for an image to be ready
        waitForImage();

        geometry_msgs::PoseStamped msg_pose;
        std_msgs:: Int8 status;
        ros::Rate rate(freq_);
        unsigned int cont = 0;

        // Read config file describing the board
        cv::FileStorage fs(board_path_, cv::FileStorage::READ);

        float mm_px =  fs["mm_per_unit"] ;
        mm_px *= 0.001;

        // Parse corners
        cv::FileNode corners_node = fs["corners"];
        cv::FileNodeIterator it = corners_node.begin(), it_end = corners_node.end();
        int idx = 0;
        std::vector<std::vector<float> > lbpval;
        std::vector< std::vector<cv::Point3f> > objPoints_;
        for( ; it != it_end; ++it, idx++ )
        {
            (*it) >> lbpval;
            std::vector<cv::Point3f> points;
            points.push_back(cv::Point3f(mm_px*lbpval[0][0], mm_px*lbpval[0][1], mm_px*lbpval[0][2]));
            points.push_back(cv::Point3f(mm_px*lbpval[1][0], mm_px*lbpval[1][1], mm_px*lbpval[1][2]));
            points.push_back(cv::Point3f(mm_px*lbpval[2][0], mm_px*lbpval[2][1], mm_px*lbpval[2][2]));
            points.push_back(cv::Point3f(mm_px*lbpval[3][0], mm_px*lbpval[3][1], mm_px*lbpval[3][2]));
            objPoints_.push_back(points);
        }

        // Parse ids
        std::vector< int > ids_;
        fs["ids"]  >> ids_;

        fs.release();

        // Create a board
        //cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(1));
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(2));
        cv::Ptr<cv::aruco::Board> board = cv::aruco::Board::create(objPoints_,dictionary,ids_);

        cv::Vec3d rvec, tvec;

        // for (unsigned int i = 0; i < objPoints_.size(); i++)
        // {
        //     for (unsigned int j = 0; j< objPoints_[i].size(); j++)
            //     std::cout<< objPoints_[i][j] << " ";
            // std::cout << std::endl;
        // }

        // for (unsigned int i = 0; i < ids_.size(); i++)
        // {
            // std::cout<< ids_[i] << std::endl;
        // }

        while(ros::ok())
        {

            if (lastHeaderSeq_ != image_header_.seq)
            {
                // Get the picture
                cv::Mat imageCopy;

                {
                    boost::mutex::scoped_lock(lock_);
                    cv_ptr->image.copyTo(imageCopy);
                }

                // Load parameters for the detector
                cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
                bool readOk = readDetectorParameters(detector_param_path_, detectorParams);
                if(!readOk) {
                    std::cerr << "Invalid detector parameters file" << std::endl;
                    return ;
                }
                detectorParams->cornerRefinementMethod = 1; // do corner refinement in markers

                // Now detect the markers
                std::vector< int > ids;
                std::vector< std::vector< cv::Point2f > > corners, rejected;

                cv::aruco::detectMarkers(imageCopy, board->dictionary, corners, ids, detectorParams, rejected);
              
                // Now estimate the pose of the board
                int markersOfBoardDetected = 0;

                if(ids.size() > 0)
                    markersOfBoardDetected = cv::aruco::estimatePoseBoard(corners, ids, board, camMatrix_, distCoeffs_, rvec, tvec);

                if (markersOfBoardDetected && cv::norm(tvec) > 0.00001)
                {
                    // Publish pose
                    geometry_msgs::Point aruco;
                    aruco.x = tvec[0];
                    aruco.y = tvec[1];
                    aruco.z = tvec[2];
                    posePub_.publish(aruco);
                }
                cont++;

                lastHeaderSeq_ = image_header_.seq;
            }

            ros::spinOnce();
            rate.sleep();

        } //end while
    } // end spin
}
