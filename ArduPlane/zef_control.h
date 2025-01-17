#include <math.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Math/AP_Math.h>
#include "Plane.h" //#include "quadplane.h"
#include "zef_k_matrix.h"

//zef_control.h alterado

class ZefControl {
private:
    double antagonist_force = 3.0E+00;
    double comando_minimo = 2.0E-01; 
    double comando_maximo = 1.0E+00;
    double F_max_mot = 2.1E+01; 
    double F_min_mot = 3.0E-01;
    double coeficiente_quadratico_forca = -1.68E-04;
    double coeficiente_linear_forca = 4.19E-02; 
    double coeficiente_nulo_forca = 1.87E-01;  
    int ref_index = 0; 
    double speed_refs[40] = {1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0,10.0,11.0,12.0,13.0,14.0,15.0,16.0,17.0,18.0,19.0,20.0,21.0,22.0,23.0,24.0,25.0,26.0,27.0,28.0,29.0,30.0,31.0,32.0,33.0,34.0,35.0,36.0,37.0,38.0,39.0}; //speeds references to change the active K matrix
    double RHO_air_density = 1.2; 
    
    int gpio_pin = 50;
    int inflator_state = 0;
    int inflator1_state = 0; 
    int inflator2_state = 0;
    
    float k_matrix_all[40][12][12] = {};
    
    void find_matrix(const double longit_speed);
    void set_K_matrix(double matrix[12][9]);
    void add_traction(double (&U_array)[12], double longit_speed);
    
    bool is_manual_mode = false;
    void adjust_for_manual(double (&U_array)[12]);
    double set_angle_range(double last_angle, double v1, double v0);
    double get_engine_command(double power);
    void set_power_and_angles(double (&U)[12]);
    void put_forces_in_range();
    void print_output_data();
    double last_X[12] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,};
    
    int semaforo = 0;
public:
    friend class Parameters;
    friend class ParametersG2;
    friend class Plane;
  
    double J1_cmd_roll, J2_cmd_pitch, J3_cmd_throttle, J4_cmd_yaw, RS_cmd_up_down, LS_cmd_letf_right = 0.0;
    double F1_forca_motor, F2_forca_motor, F3_forca_motor, F4_forca_motor = 0.0;
    double comando_M1, comando_M2, comando_M3, comando_M4 = 0.0;
    double d1_angulo_motor, d2_angulo_motor, d3_angulo_motor, d4_angulo_motor = 0.0;
    double dvu_ang_estab_vert_cima, dvd_ang_estab_vert_baixo, dhr_ang_estab_horiz_direito, dhl_ang_estab_horiz_esquerdo = 0.0;

    void mult_mat(const float matrix[8][12], const double vector[12], double (&results)[12]);
    
    void set_manual(bool state);//alterado
    
	void set_RHO(double rho);//alterado

    void manual_inputs_update(double aileron, double elevator, double rudder, double throttle, double right_switch, double left_switch);
    
    void update( double U_longit_speed,
        double V_lateral_speed, double W_vertical_speed, double P_v_roll, 
        double Q_v_pitch, double R_v_yaw, double Roll, double Pitch, double Yaw,
        double x_position_n_s, double y_position_e_w, double z_position_height);
        
    int get_value_to_pwm_servo(double in_value, int min_pwm, int max_pwm);//alterado
    int get_value_to_pwm_motor(double in_value, int min_pwm, int max_pwm);//alterado
    
    float dead_zone(double input_value, double dead_zone_limit);//alterado
    void operateInflators(AP_Baro *barometer, float min_p, float max_p, int deactivate_unit);
    void stopInflators();
    
    Vector3f rotate_inertial_to_body(float roll, float pitch, float yaw, const Vector3f &inertial_vector);
    void getPositionError(double desired_position_lat, double desired_position_long, double desired_position_alt, double current_position_lat, double current_position_long, double current_position_alt, double azimute, double (&ret_errors)[3]);
    
    int tca_initialized = 0;
    void TCA9548A(uint16_t bus);
    
    ZefControl() {
        memcpy(k_matrix_all, zef_matrix, sizeof(zef_matrix));//Não se é necessário se alterar pois a matriz é de uma pasta/ é necessário altera
															 // a matriz a todo momento.
    }
};

extern ZefControl zefiroControl;

