#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

#include <visp/vpHomogeneousMatrix.h>
#include <visp3/vs/vpServo.h>
#include <visp/vpFeatureThetaU.h>
#include <visp/vpFeatureTranslation.h> 
#include <std_srvs/SetBool.h> // servicio boleano

vpTranslationVector t; // Vector de traslación
vpRotationMatrix R;    // Matriz de rotación
vpRotationMatrix H;    // Matriz de rotación
vpTranslationVector n; // Vector normal (si lo necesitas)

float index_value;
bool received_trn = false;

void indexCallback(const std_msgs::Float32::ConstPtr& msg) {
    index_value= msg->data;
    //ROS_INFO("Received index: %f", index;
}

void translationVectorCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {

    for (int i = 0; i < 3; ++i) {
        t[i] = msg->data[i];
    }

    received_trn = true;
    //std::cout << "\n t0 = " << t <<  std::endl;
}

void rotationMatrixCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R[i][j] = msg->data[i * 3 + j];
        }
    }
    //std::cout << "\n R0 = " << R <<  std::endl;

}

void homographyCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            H[i][j] = msg->data[i * 3 + j];
        }
    }
    //std::cout << "\n R0 = " << R <<  std::endl;

}

void normalVectorCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {

    for (int i = 0; i < 3; ++i) {
        n[i] = msg->data[i];
    }

}



int main(int argc,char* argv[])
{
    ros::init(argc, argv, "ibvs_node_control");
    ros::NodeHandle nh;
    cv::startWindowThread();

    ros::Subscriber sub_R = nh.subscribe("/rotation_matrix", 1, rotationMatrixCallback);
    ros::Subscriber sub_t = nh.subscribe("/translation_vector", 1, translationVectorCallback);
    ros::Subscriber sub_normal = nh.subscribe("/normal_vector", 1, normalVectorCallback);
    ros::Subscriber sub_H = nh.subscribe("/homography_matrix", 1, homographyCallback);
    ros::Subscriber sub_index = nh.subscribe("/index", 1, indexCallback);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Publisher error_pub = nh.advertise<std_msgs::Float32>("/error_topic", 1);



    //========================= VISP task ================================

    // Se construye una matriz de transformación homogénea utilizando una traslación (t) y una rotación (R).
    vpHomogeneousMatrix cMo;
    cMo.buildFrom(t, R);

    // Se construyen características visuales actuales utilizando la matriz de transformación homogénea.
    vpFeatureTranslation s_t(vpFeatureTranslation::cMo);
    vpFeatureThetaU s_R(vpFeatureThetaU::cdRc);
    s_t.buildFrom(cMo); // Se establecen los valores iniciales de la característica de translación.
    s_R.buildFrom(cMo); // Se establecen los valores iniciales de la característica de rotación.

    // Se construye la característica visual deseada s* = (0,0)
    vpFeatureTranslation s_star_t(vpFeatureTranslation::cMo); // Inicialización predeterminada a cero 
    vpFeatureThetaU s_star_R(vpFeatureThetaU::cdRc); // Inicialización predeterminada a cero 

    vpServo task;
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::CURRENT);
    //task.setLambda(0.1);  // Ganancia de servoing
    task.setLambda(0.165);
    task.addFeature(s_t, s_star_t);// características actuales y deseadas de translación.
    task.addFeature(s_R, s_star_R);// características actuales y deseadas de rotación.

    //========================= VISP task ================================
    
    vpColVector v;
    double error = 5; // Error de la tarea
    double new_error;
    std_msgs::Float32 error_msg;
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x  = 0;
    vel_msg.angular.z = 0;

    ros::Rate loop_rate(20); 
    sleep(20);  // en segundos

    do{

        if (received_trn){

            cMo.buildFrom(t, R);

            s_t.buildFrom(cMo); // Update translation visual feature
            s_R.buildFrom(cMo); // Update ThetaU visual feature
            // Calcular la ley de control
            v = task.computeControlLaw();
            error = (task.getError()).sumSquare();


            // Publicar el error
            error_msg.data = error;
            error_pub.publish(error_msg);

            // Asignar velocidades lineales y angulares
            vel_msg.linear.x = -v[2];    // Velocidad lineal en el eje x
            vel_msg.angular.z = v[4] * 2.1;   // Velocidad angular en el eje z
            vel_pub.publish(vel_msg);

        }

        else {
            printf("esperando informacion... \n");
        }

        ros::spinOnce();   //Activa callbacks
        loop_rate.sleep(); //Espera hasta completar el loop rate

    } while (index_value <= 8.0); //recomendable recibir valores de la homografia
    
    
    vel_msg.linear.x = 0;    // Velocidad lineal en el eje x
    vel_msg.angular.z = 0;   // Velocidad angular en el eje z
    vel_pub.publish(vel_msg);

    

    return 0;
}