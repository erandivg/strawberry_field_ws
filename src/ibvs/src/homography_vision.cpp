#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include "vision_utils.h"

#include <opencv2/calib3d.hpp>
//#include <opencv2/core/eigen.hpp>
#include <Eigen/Dense>
//#include <std_msgs/Float32MultiArray.h>

//------ Variables Globales --------//
cv::Mat currImg, refImage;
float error_sub; 
int currentImageIndex = 0;
sensor_msgs::ImagePtr outputMsg = nullptr, matchesMsg = nullptr;
// Detectar puntos clave y calcular descriptores
std::vector<cv::KeyPoint> keypointsCurr, keypointsRef;
ros::Publisher trackedPointsPub, matchesPub;
Matrix_t K(3,3);  // Matriz de calibración de la cámara (3x3)


Matrix_t H(3,3);
Matrix_t rotations(3,3);
Vector3D_t translations;
Vector3D_t normals;
FPTYPE d;
homographySolution homSolution;
int counter = 0; // Contador
//std::vector<int> solutions;


void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        // Convertir la imagen de ROS a formato OpenCV
        currImg = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;

        // Convertir a escala de grises para la detección de puntos clave
        cv::Mat currImgray, refImgray;
        cv::cvtColor(currImg, currImgray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(refImage, refImgray, cv::COLOR_BGR2GRAY);

        EstimateHomography(refImage, currImg, K, H, counter);
        //std::cout << "Homo: \n " << H << std::endl;

        cv::Mat descriptorsCurr, descriptorsRef;
        cv::Ptr<cv::ORB> orb = cv::ORB::create(600);
        orb->detectAndCompute(currImgray, cv::Mat(), keypointsCurr, descriptorsCurr);
        orb->detectAndCompute(refImgray, cv::Mat(), keypointsRef, descriptorsRef);

        // Convertir descriptores a tipo flotante para FLANN
        if(descriptorsRef.type() != CV_32F) {
            descriptorsRef.convertTo(descriptorsRef, CV_32F);
        }
        if(descriptorsCurr.type() != CV_32F) {
            descriptorsCurr.convertTo(descriptorsCurr, CV_32F);
        }

        // Emparejar descriptores usando un matcher FLANN basado en LSH
        cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
        std::vector<std::vector<cv::DMatch>> knnMatches;
        matcher->knnMatch(descriptorsRef, descriptorsCurr, knnMatches, 2);

        // Filtrar emparejamientos usando la prueba de razón de Lowe
        const float ratio_thresh = 0.75f;
        std::vector<cv::DMatch> goodMatches;
        for (size_t i = 0; i < knnMatches.size(); i++) {
            if (knnMatches[i].size() == 2 && knnMatches[i][0].distance < ratio_thresh * knnMatches[i][1].distance) {
                goodMatches.push_back(knnMatches[i][0]);
            }
        }

        std::vector<cv::Point2f> refPoints2f;
        std::vector<cv::Point2f> currentPoints2f;


        for( size_t i = 0; i < goodMatches.size(); i++ )
        {
            //-- Get the keypoints from the good matches
            refPoints2f.push_back( keypointsRef[ goodMatches[i].queryIdx ].pt );
            currentPoints2f.push_back( keypointsCurr[ goodMatches[i].trainIdx ].pt );
        }


        // Dibujar puntos clave en la imagen
        cv::Mat imgKeypointsCurr;
        cv::drawKeypoints(currImg, keypointsCurr, imgKeypointsCurr, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DEFAULT);

        // Publicar la imagen con los puntos detectados
        outputMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgKeypointsCurr).toImageMsg();  

        // Visualizar y publicar los emparejamientos
        cv::Mat imgMatches;
        cv::drawMatches(refImage, keypointsRef, currImg, keypointsCurr, goodMatches, imgMatches, 
                        cv::Scalar(0, 0, 255), // Color de las líneas (rojo en este caso)
                        cv::Scalar::all(-1), 
                        std::vector<char>(), 
                        cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

        // Dibujar líneas más gruesas
        for (size_t i = 0; i < goodMatches.size(); i++) {
            cv::Point2f pt1 = keypointsRef[goodMatches[i].queryIdx].pt;
            cv::Point2f pt2 = keypointsCurr[goodMatches[i].trainIdx].pt + cv::Point2f((float)refImage.cols, 0);
            cv::line(imgMatches, pt1, pt2, cv::Scalar(0, 0, 255), 2); // 2 es el grosor de la línea y azul es el color
        }

        // Publicar la imagen de emparejamientos
        matchesMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgMatches).toImageMsg();

    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

std::vector<cv::Mat> readSequentialImages(const std::string& directory) {
    std::vector<cv::Mat> images;
    int index = 0;
    
    while (true) {
        std::string filename = directory + "/image" + std::to_string(index) + ".png";
        cv::Mat img = cv::imread(filename, cv::IMREAD_COLOR);
        if (img.empty()) {
            break;  // No more images to read
        }
        images.push_back(img);
        index++;
    }

    if (images.empty()) {
        std::cerr << "No se encontraron imágenes en el directorio " << directory << std::endl;
    }

    return images;
}


void errorCallback(const std_msgs::Float32::ConstPtr& msg)
{
    error_sub = msg->data;  // Guardamos el valor del mensaje en la variable
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "vision_node");
    ros::NodeHandle nh;

    // Leer todas las imágenes secuenciales del directorio de referencia
    std::vector<cv::Mat> desiredImages;
    std::string REF_IMAGE_DIR = "/ros/strawberry_ws/src/ibvs/image_reference/";
    desiredImages = readSequentialImages(REF_IMAGE_DIR);

    if (desiredImages.empty()) {
        std::cerr << "No se pudieron leer las imágenes de referencia" << std::endl;
        return 1;
    }

    // Asignar la primera imagen del vector a refImage
    refImage = desiredImages[currentImageIndex];

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_raw", 1, imageCallback);
    //ros::Subscriber err_sub = nh.subscribe("/error_topic", 1, errorCallback);

    trackedPointsPub = nh.advertise<sensor_msgs::Image>("/camera/rgb/points_image", 1);
    matchesPub = nh.advertise<sensor_msgs::Image>("/camera/rgb/vision_image_matches", 1);

    // Creación de los publicadores para R, t y normal
    ros::Publisher pub_H = nh.advertise<std_msgs::Float32MultiArray>("/homography_matrix", 1);
    ros::Publisher pub_R = nh.advertise<std_msgs::Float32MultiArray>("/rotation_matrix", 1);
    ros::Publisher pub_t = nh.advertise<std_msgs::Float32MultiArray>("/translation_vector", 1);
    ros::Publisher pub_normal = nh.advertise<std_msgs::Float32MultiArray>("/normal_vector", 1);
    ros::Publisher pub_index = nh.advertise<std_msgs::Float32>("/index", 1);

    std_msgs::Float32MultiArray H_msg;
    std_msgs::Float32MultiArray R_msg;
    std_msgs::Float32MultiArray t_msg;
    std_msgs::Float32MultiArray n_msg;
    std_msgs::Float32 index_msg;
    
    bool isFirstRun = true;

    K << 1206.889772,        0.0    , 960,
            0.0     ,  1206.889772  , 540,
            0.0     ,         0.0   , 1.0;

    ros::Rate rate(10);

    while(ros::ok()) {

        if(currentImageIndex==0 && isFirstRun){
            refImage = desiredImages[currentImageIndex];
            isFirstRun = false;
            std::cout << "\033[1;34mLa imagen actual deseada es: reference_image" << currentImageIndex << ".png\033[0m" << std::endl;
        }

        
        if (outputMsg != nullptr && matchesMsg != nullptr) {
            trackedPointsPub.publish(outputMsg);
            matchesPub.publish(matchesMsg);  
        }

        // Verificar si se detectaron suficientes puntos clave en ambas imágenes
        if (currImg.empty()) {
            std::cout << "Could not read the current image: "  << std::endl;
        } else {
            //(const Matrix_t &homography,Matrix_t &R, Vector3D_t &t, Vector3D_t &n, FPTYPE &distancePlane, const int current_iteration, homographySolution &homograpy_solution)
            RecoverFromHomography(H,rotations,translations,normals,d, counter, homSolution);

            // Limpia los mensajes antes de llenarlos
            H_msg.data.clear();  // Limpia datos previos
            R_msg.data.clear();
            t_msg.data.clear();
            n_msg.data.clear();

            // Llenar el mensaje con los datos de la matriz H=
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    H_msg.data.push_back(static_cast<float>(H(i, j)));
                }
            }
            //std::cout << "H: \n" << H << std::endl;
            
            // Llenar R_msg con la matriz de rotación
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    R_msg.data.push_back(rotations(i, j));
                }
            }

            // Llenar t_msg con el vector de traslación
            for (int i = 0; i < 3; ++i) {
                t_msg.data.push_back(translations(i));
            }

            // Llenar n_msg con el vector normal
            for (int i = 0; i < 3; ++i) {
                n_msg.data.push_back(normals(i));
            }

            //if (H(0,0)>0.96 && std::abs(H(0,1)) < 0.08 && std::abs(H(0,2)) < 0.08 && std::abs(H(2,0)) < 0.1 && std::abs(H(2,1)) < 0.1 && H(2,2)<=1.005){
            if (H(0,0)>0.90 && std::abs(H(0,1)) < 0.08 && std::abs(H(0,2)) < 0.2 && std::abs(H(2,0)) < 0.1 && std::abs(H(2,1)) < 0.2 && H(2,2)<=0.995){
                if (currentImageIndex + 1 < desiredImages.size()) {
                    currentImageIndex = currentImageIndex + 1;
                    refImage = desiredImages[currentImageIndex];
                } else {
                    // Handle case where currentImageIndex is at the end of the desiredImages array
                    currentImageIndex = currentImageIndex + 1;
                    std::cout << "Reached the end of the images array." << std::endl;
                }

            }

            index_msg.data = static_cast<float>(currentImageIndex);
            pub_index.publish(index_msg);
            pub_H.publish(H_msg);
            pub_R.publish(R_msg);
            pub_t.publish(t_msg);
            pub_normal.publish(n_msg);
            
            //std::cout << "R: \n " << rotations << std::endl;
            //std::cout << "t: \n " << translations << std::endl;
            counter++;
        }


        ros::spinOnce();  // Procesa los callbacks una vez
        rate.sleep();  // Duerme para mantener la tasa
    }

    return 0;
}
