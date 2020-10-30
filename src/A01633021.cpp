#ifndef A01633021_CPP  
#define A01633021_CPP  

#include "../include/A01633021.hpp"
#include <string>


A01633021::A01633021(distance_t w_1, distance_t w_2, distance_t w_3, distance_t w_4, float _altura, float _l1, float _l2) : OP3_Arm(w_1, w_2, w_3, w_4, _altura, _l1, _l2){
    //Constructor
}


void A01633021::calculate_forward_kinematics(){
    //Declarar variables
    float ang1;
    float ang2;
    float ang3;
    float ang4;
    float ang5;
    float d1;
    float d2;
    float d3;
    float d4;
    float d5;
    float d6;
    float l1;
    float l2;
    float x;
    float y;
    float z;

    //Inicializar variables conocidas
    ang1 = coor_union1;
    ang2 = coor_union2;
    ang3 = 180-coor_union3;
    l1 = brazo;
    l2 = antebrazo;
    d2 = 0;


    //Encontrar d3 con trignonometria
    d3 = sind(ang2)*l1;
    // Encontrar ang 4 utilizando los otros 2 angulos del triangulo
    ang4 = 180 - (ang2 + 90);
    //Encontrar ANG utilizando el complemento de ANG4
    ang5 = ang3 - ang4;
    //Encontrar D6 cin trigonometria
    d6 = cosd(ang5) * l2;

    //Encontrar Z sumando las distancias encontradas previamente
    z = d2 +d3 -d6;

    d4 = cosd(ang2)*l1;
    d5 = sind(ang5)*l2;
    d1 = d4+d5;

    if (ang1==90){      //Plano horizontal
        x = z;
        y = d1;
        z = 0;
    }else{
        x = sind(ang1)*d1;  //Plano XY
        y = cosd(ang1)*d1;
    }

    
    // Agregar distancia de W con union 2
    coor_end_efector.x = x + dis_w_p2.dx;
    coor_end_efector.y = y + dis_w_p2.dy;
    coor_end_efector.z = z + altura;

}



void A01633021::calculate_inverse_kinematics(){
    //Definir variables
    float r;
    float x;
    float y;
    float z;
    float base;
    float elbow;
    float shoulder;
    float l1;
    float l2;
    float beta;
    float alfa;
    float gama;

    //Valores conocidos
    l1 = brazo;
    l2 = antebrazo;
 
    y = coor_end_efector.y - dis_w_p2.dy;
    x = coor_end_efector.x - dis_w_p2.dx;
    z = coor_end_efector.z - altura;

    //Radio del sistema
    r = sqrt((y*y)+(x*x)+(z*z));

    //Evitar indeterminaciones de atan
    if (y!=0){
        base = atand(x/y);
    }else if (x!=0){
        base = 90;
    }else{
        base = 0;
    }

    if (z==0){          //Plano horizontal
        z = x;
        base = 90;
    }

    alfa = asind(z/r);
    beta = acosd(((r*r)+(l1*l1)-(l2*l2))/(2*r*l1));
    gama = acosd(((l2*l2)+(l1*l1)-(r*r))/(2*l2*l1));

    elbow = 180-gama;
    shoulder = alfa + beta;


    coor_union1 = base;
    coor_union2 = shoulder;
    coor_union3 = elbow;

}


void A01633021::process_trajectory(){
    float time;
    coordinates_time_t tmp_trajectory;
    angular_coordinates_time_t tmp_joints;
    coordinates_time_t tmp_efector;
    coordinates_time_t tmp_error;
        //Iterate queue
    for (int idx=0; idx<queue_trajectory.size(); idx++){
        //Obtener tiempo
        time =  queue_trajectory[idx].time;
        //Obtener coordenadas deseadas
        tmp_trajectory     = queue_trajectory[idx];
        coor_end_efector.x = tmp_trajectory.coordinates.x;  
        coor_end_efector.y = tmp_trajectory.coordinates.y;
        coor_end_efector.z = tmp_trajectory.coordinates.z;
        //Aplicar cinematica inversa
        this->calculate_inverse_kinematics();
        //Guardar coordenadas angulares calculadas
        tmp_joints.time = time;
        tmp_joints.coordinates.coordinate_join1 = coor_union1;
        tmp_joints.coordinates.coordinate_join2 = coor_union2;
        tmp_joints.coordinates.coordinate_join3 = coor_union3;
        //Calcular cinematica directa con coordenadas angulares calculadas;
        this->calculate_forward_kinematics();
        //Guardar coordenada cartesiana de efector final calculado
        tmp_efector.time = time;
        tmp_efector.coordinates.x = coor_end_efector.x;
        tmp_efector.coordinates.y = coor_end_efector.y;
        tmp_efector.coordinates.z = coor_end_efector.z;
        //Calcular error y guardar errores
        tmp_error.time = time;
        tmp_error.coordinates.x = tmp_trajectory.coordinates.x - tmp_efector.coordinates.x;
        tmp_error.coordinates.y = tmp_trajectory.coordinates.y - tmp_efector.coordinates.y;
        tmp_error.coordinates.z = tmp_trajectory.coordinates.z - tmp_efector.coordinates.z;
        //Guardar estructuras en queue
        queue_joints.push_back(tmp_joints);
        queue_efector.push_back(tmp_efector);
        queue_error.push_back(tmp_error);
    }
}

bool A01633021::read_trajectory(std::string file_name){
    std::string tmp_data;
    std::string tmp_value;
    std::ifstream rd_file(file_name);
    coordinates_time_t tmp_struct;
    //check if file is open
    if (!rd_file.is_open()){
        std::cout << "Error, can´t open file to read array: " << file_name << "\n";
        return false;
    }

     //read file
    //remove header
    getline(rd_file, tmp_data);

    while ( getline (rd_file, tmp_data)){
        std::stringstream ss(tmp_data);
        ss >> tmp_value;                      //get time from file
        tmp_struct.time = std::stof(tmp_value);
        ss >> tmp_value;                      //Get x from file
        tmp_struct.coordinates.x = std::stof(tmp_value);
        ss >> tmp_value;                       //Get y from file
        tmp_struct.coordinates.y = std::stof(tmp_value);
        ss >> tmp_value;                        //Get z from file
        tmp_struct.coordinates.z = std::stof(tmp_value);
        queue_trajectory.push_back(tmp_struct);
    }

    rd_file.close();

    return true;
}

void A01633021::clear_trajectory(){
    queue_trajectory.clear();
}

bool A01633021::write_joints(std::string file_name){
    std::ofstream wr_file(file_name);
    angular_coordinates_time_t tmp_struct;
    //check if file is open
    if (!wr_file.is_open()){
        std::cout << "Error, can´t open file to read array: " << file_name << "\n";
        return false;
    }

     //read file
    //write header
    wr_file << "time union1 union2 union3\n";

    //Escribir vector
    while (!queue_joints.empty()){
        tmp_struct = queue_joints.front();
        queue_joints.pop_front();
        wr_file << tmp_struct.time << ", " << tmp_struct.coordinates.coordinate_join1 << ", " << tmp_struct.coordinates.coordinate_join2 << ", " << tmp_struct.coordinates.coordinate_join3 << "\n";
    }

    wr_file.close();
    return true;
}


bool A01633021::write_efector(std::string file_name){
    std::ofstream wr_file(file_name);
    coordinates_time_t tmp_struct;
    //check if file is open
    if (!wr_file.is_open()){
        std::cout << "Error, can´t open file to read array: " << file_name << "\n";
        return false;
    }

     //read file
    //write header
    wr_file << "time x y z\n";

    //Escribir vector
    while (!queue_efector.empty()){
        tmp_struct = queue_efector.front();
        queue_efector.pop_front();
        wr_file << tmp_struct.time << ", " << tmp_struct.coordinates.x << ", " << tmp_struct.coordinates.y << ", " << tmp_struct.coordinates.z << "\n";
    }

    wr_file.close();
    return true;
}

bool A01633021::write_error(std::string file_name){
    std::ofstream wr_file(file_name);
    coordinates_time_t tmp_struct;
    //check if file is open
    if (!wr_file.is_open()){
        std::cout << "Error, can´t open file to read array: " << file_name << "\n";
        return false;
    }

     //read file
    //write header
    wr_file << "time x y z\n";

    //Escribir vector
    while (!queue_error.empty()){
        tmp_struct = queue_error.front();
        queue_error.pop_front();
        wr_file << tmp_struct.time << ", " << tmp_struct.coordinates.x << ", " << tmp_struct.coordinates.y << ", " << tmp_struct.coordinates.z << "\n";
    }

    wr_file.close();
    return true;
}

double A01633021::sind(float degree){
    return sin(degree*PI/180);
}

double A01633021::cosd(float degree){
    return cos(degree*PI/180);
}

double A01633021::tand(float degree){
    return tan(degree*PI/180);
}

double A01633021::asind(float value){
    return asin(value) * 180.0/PI;
}

double A01633021::acosd(float value){
    return acos(value) * 180.0/PI;
}

double A01633021::atand(float value){
    return atan(value) * 180.0/PI;
}

#endif // A01633021_CPP