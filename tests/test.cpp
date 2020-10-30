#ifndef TEST_CPP  
#define TESST_CPP  

#include "../include/A01633021.hpp"

#define num_tests 10
#define altura 1.357
#define brazo 9.3757
#define antebrazo 14.641
#define incx -1.161
#define incy 8.85
#define total brazo+antebrazo

void test_point(A01633021 *robot, float coor1, float coor2, float coor3){
    robot->set_coor_union1(coor1);
    robot->set_coor_union2(coor2);
    robot->set_coor_union3(coor3);
    //robot->set_coor_end_efector(coor);
    robot->calculate_forward_kinematics();
    std::cout << "x: " << robot->get_coor_end_efector().x << " y: " << robot->get_coor_end_efector().y << " z: " << robot->get_coor_end_efector().z << "\n";

}

void test_point_inverse(A01633021 *robot, float x, float y, float z){
    coordinates_t coor = {.x=x, .y=y, .z=z};
    robot->set_coor_end_efector(coor);
    robot->calculate_inverse_kinematics();
    std::cout << "1: " << robot->get_coor_union1() << " 2: " << robot->get_coor_union2() << " 3: " << robot->get_coor_union3() << "\n";
}

void test_inverse(A01633021 *robot){
    robot->calculate_inverse_kinematics();
    std::cout << "1: " << robot->get_coor_union1() << " 2: " << robot->get_coor_union2() << " 3: " << robot->get_coor_union3() << "\n";
}

void test_direct(A01633021 *robot){
    robot->calculate_forward_kinematics();
    std::cout << "x: " << robot->get_coor_end_efector().x << " y: " << robot->get_coor_end_efector().y << " z: " << robot->get_coor_end_efector().z << "\n";   
}

int main(){
    coordinates_t err {.x=0, .y=0, .z=0};
    coordinates_t coor {.x=3.001, .y=4.001, .z=5.001};
    
    float test_x[num_tests] = {0+incx,           0+incx,         0+incx,                 0+incx,            total+incx,       0+incx,                 -total+incx,   7,   7,     -7  };
    float test_y[num_tests] = {0+incy,           total+incy,     total*0.707+incy,       0+incy,            0+incy,           total+incy,             0+incy,        15,  15,    15};
    float test_z[num_tests] = {total+altura,     0+altura,       -total*0.707+altura,    -total+altura,     0+altura,         0*0.707+altura,         0+altura,        7,  -7,   7};
    
    //Tests midiendo como referencia el punto 2 y longitudes de 5 y 5 cm
    //Testeo horizontal
    /*
    float test_x[num_tests] = {10,   5,      0,  -7.07,      -10};
    float test_y[num_tests] = {0,   6.5,   10, 7.07,   0};
    float test_z[num_tests] = {0,  0,   0,  0, 0};
    */
    //Testeo vertical
    /*
    float test_x[num_tests] = {0,   0,      0,  0,      0};
    float test_y[num_tests] = {0,   7.07,   10, 7.07,   0};
    float test_z[num_tests] = {10,  7.07,   0,  -0.707, -10};
    */
    //Testeo diagonal
    /*
    float test_x[num_tests] = {0,   3,      7.07,  -3,      0};
    float test_y[num_tests] = {0,   3,   7.07, 3,   0};
    float test_z[num_tests] = {10,  7.07,   0,  -0.707, -10};
    */
    distance_t w_1 = {.dx=-3.95, .dy=6.44, .dz=0.0304, .dist=7.5549};
    distance_t w_2 = {.dx=-1.161, .dy=8.85, .dz=0.9696, .dist=9.0474};
    distance_t w_3 = {.dx=-1.161, .dy=17.89, .dz=0.9796, .dist=17.989};
    distance_t w_4 = {.dx=3.95, .dy=32.541, .dz=-13.153, .dist=32.8062};
    A01633021 *robot = new A01633021(w_1, w_2, w_3, w_4, altura, brazo, antebrazo);
    
    for (int idx=0; idx<num_tests; idx++){
        std::cout << "Point " << idx << "\n";
        test_point_inverse(robot, test_x[idx], test_y[idx], test_z[idx]);
        test_direct(robot);
        test_inverse(robot);
        err.x += test_x[idx]-robot->get_coor_end_efector().x;
        err.y += test_y[idx]-robot->get_coor_end_efector().y;
        err.z += test_z[idx]-robot->get_coor_end_efector().z;
    }
    
    std::cout << "Error promedio \n";
    std::cout << "x: " << err.x/num_tests << "\n";
    std::cout << "y: " << err.y/num_tests << "\n";
    std::cout << "z: " << err.z/num_tests << "\n";

    return 0;
}

#endif // TEST_CPP  