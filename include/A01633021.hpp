#ifndef A01633021_HPP  
#define A01633021_HPP 

#include "OP3_Arm.hpp"
#include <math.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>
#include <deque>

#define PI 3.14159265


//Estructura para almecenar coordenadas angulares de las 3 uniones
typedef struct angular_coordinates_s{
    float coordinate_join1;
    float coordinate_join2;
    float coordinate_join3;
} angular_coordinates_t;

//Estructura para almecenar coordenadas cartesianas junto con el tiempo
typedef struct coordinates_time_s{
    float time;
    coordinates_t coordinates;
} coordinates_time_t;


//Estructure para almacenar coordenadas angulares junto con el tiempo
typedef struct angular_coordinates_time_s{
    float time;
    angular_coordinates_t coordinates;
} angular_coordinates_time_t;

class A01633021 : public OP3_Arm{
    private:
        std::deque<coordinates_time_t> queue_trajectory;       //queue para guardar coordenadas de trayectoria
        std::deque<angular_coordinates_time_t> queue_joints;   //queue para almacenar coordenadas angulares de uniones
        std::deque<coordinates_time_t> queue_efector;          //queue para guardar coordenadas angulares de efector
        std::deque<coordinates_time_t> queue_error;            //queue para guardar error de coordenadas 
    public:
        //Constructor
        A01633021(distance_t w_1, distance_t w_2, distance_t w_3, distance_t w_4, float _altura, float _l1, float _l2);
        //Sobre escribir metodos
        void calculate_forward_kinematics();
        void calculate_inverse_kinematics();
        //Leer trayectoria de archivo de texto para guardarlo en queue
        bool read_trajectory(std::string file_name);
        //Escribir coordenadas andgulare del queue en archivo de texto
        bool write_joints(std::string file_name);
        //Escribir coordenadas de efector final en archivo de texto
        bool write_efector(std::string file_name);
        //Escribit error en coordenadas en archivo de texto
        bool write_error(std::string file_name);
        //Funcion para procesar trayectoria,
        //Leera coordenadas de queue de trayectoria
        //Calculara cinematica inversa para cada punto y escribira las coordenadas angulares en queue de joints
        //Calculara la cinematica directa de las coordenadas angulares calculadas y escribira la coordenada en queue de efector
        //Calculara error entre las coordenadas obtenidas y lo escribira en queue de error
        void process_trajectory();
        //Vaciar queue de trayectoria
        void clear_trajectory();
        //Funciones auxiliares para calcular seno, coseno, tan y sus inversos en grados
        double sind(float degree);      
        double cosd(float degree);
        double tand(float degree);
        double asind(float value);
        double acosd(float value);
        double atand(float value);
};

#endif //A01633021_HPP