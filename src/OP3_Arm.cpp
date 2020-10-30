#ifndef OP3_ARM_CPP  
#define OP3_ARM_CPP  

#include "../include/OP3_Arm.hpp"

OP3_Arm::OP3_Arm(){
    //Constructor default no hacer nada
}

OP3_Arm::OP3_Arm(distance_t _dis_w_p1, distance_t _dis_w_p2, distance_t _dis_w_p3, distance_t _dis_w_p4, float _altura, float _l1, float _l2){
    //Inicializar parametros constantes
    dis_w_p1 = _dis_w_p1;
    dis_w_p2 = _dis_w_p2;
    dis_w_p3 = _dis_w_p3;
    dis_w_p4 = _dis_w_p4;
    altura = _altura;
    brazo = _l1;
    antebrazo = _l2;
}

OP3_Arm::~OP3_Arm(){
    //Nada que destruir
}
        
void OP3_Arm::set_dis_w_p1(distance_t _dis_w_p1){
    dis_w_p1 = _dis_w_p1;
}    

void OP3_Arm::set_dis_w_p2(distance_t _dis_w_p2){
    dis_w_p2 = _dis_w_p2;
}

void OP3_Arm::set_dis_w_p3(distance_t _dis_w_p3){
    dis_w_p3 = _dis_w_p3;
}

void OP3_Arm::set_dis_w_p4(distance_t _dis_w_p4){
    dis_w_p4 = _dis_w_p4;
}

void OP3_Arm::set_altura(float _altura){
    altura = _altura;
}

void OP3_Arm::set_brazo(float _brazo){
    brazo = _brazo;
}

void OP3_Arm::set_antebrazo(float _antebrazo){
    antebrazo = _antebrazo;
}

distance_t OP3_Arm::get_dis_w_p1(){
    return dis_w_p1;
}

distance_t OP3_Arm::get_dis_w_p2(){
    return dis_w_p2;
}

distance_t OP3_Arm::get_dis_w_p3(){
    return dis_w_p3;
}

distance_t OP3_Arm::get_dis_w_p4(){
    return dis_w_p4;
}

float OP3_Arm::get_altura(){
    return altura;
}

float OP3_Arm::get_brazo(){
    return brazo;
}

float OP3_Arm::get_antebrazo(){
    return antebrazo;
}

void OP3_Arm::set_coor_end_efector(coordinates_t _coor_end_efector){
    coor_end_efector = _coor_end_efector;
}

coordinates_t OP3_Arm::get_coor_end_efector(){
    return coor_end_efector;
}

void OP3_Arm::set_coor_union1(float _coor_union1){
    coor_union1 = _coor_union1;
}

void OP3_Arm::set_coor_union2(float _coor_union2){
    coor_union2 = _coor_union2;
}

void OP3_Arm::set_coor_union3(float _coor_union3){
    coor_union3 = _coor_union3;
}

float OP3_Arm::get_coor_union1(){
    return coor_union1;
}

float OP3_Arm::get_coor_union2(){
    return coor_union2;
}

float OP3_Arm::get_coor_union3(){
    return coor_union3;
}

void OP3_Arm::calculate_forward_kinematics(){
    //Sobre escribir en clase hija
}

void OP3_Arm::calculate_inverse_kinematics(){
    //Sobre escribir en clase hija
}


#endif // OP3_ARM_CPP