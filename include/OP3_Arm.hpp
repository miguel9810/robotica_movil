#ifndef OP3_ARM_HPP  
#define OP3_ARM_HPP


//Estructura para almacenar coordenadas cartesianas
typedef struct coordinates_s {
    float x;
    float y;
    float z;
} coordinates_t;

//Estructura para almacenar distancia entre uniones
typedef struct distance_s {
    float dx;
    float dy;
    float dz;
    float dist;
} distance_t;

class OP3_Arm {
    protected:
        float coor_union1;                  //Coordenada angular union 1
        float coor_union2;                  //Coordenada angular union 2   
        float coor_union3;                  //Coordenada angular union 3
        coordinates_t coor_end_efector;     //Coordenada cartesiana efector final
        //Parametros constantes
        distance_t dis_w_p1;                //Distancia del punto w a punto 1
        distance_t dis_w_p2;                //Distancia del punto w a punto 2
        distance_t dis_w_p3;                //Distancia del punto w a punto 3
        distance_t dis_w_p4;                //Distancia del punto w a punto 4
        float altura;                           //Altura de union 2 con respecto a W
        float brazo;                           //Longitud brazo
        float antebrazo;                       //Longitud antebrazo

    public:
        OP3_Arm();      //Constructor default
        OP3_Arm(distance_t _dis_w_p1, distance_t _dis_w_p2, distance_t _dis_w_p3, distance_t _dis_w_p4, float _altura, float _l1, float _l2);  //Constructor indicando parametros constantes
        ~OP3_Arm();     //Destructor
        //Setters parametros constantes
        void set_dis_w_p1(distance_t _dis_w_p1);    
        void set_dis_w_p2(distance_t _dis_w_p2);
        void set_dis_w_p3(distance_t _dis_w_p3);
        void set_dis_w_p4(distance_t _dis_w_p4);
        void set_altura(float _altura);
        void set_brazo(float _brazo);
        void set_antebrazo(float _antebrazo);
        //Getters parametros constantes
        distance_t get_dis_w_p1();
        distance_t get_dis_w_p2();
        distance_t get_dis_w_p3();
        distance_t get_dis_w_p4();
        float get_altura();
        float get_brazo();
        float get_antebrazo();
        void set_coor_end_efector(coordinates_t _coor_end_efector);       //Set coordinadas efector final
        coordinates_t get_coor_end_efector();           //get coordinates efector final
        //Setters coordinadas uniones
        void set_coor_union1(float _coor_union1);
        void set_coor_union2(float _coor_union2);
        void set_coor_union3(float _coor_union3);
        //Getters coordinadas uniones
        float get_coor_union1();
        float get_coor_union2();
        float get_coor_union3();
        //Metodos para ser sobre escritos
        virtual void calculate_forward_kinematics();
        virtual void calculate_inverse_kinematics();

};

#endif //OP3_ARM_HPP