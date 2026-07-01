#include <iostream> 
#include <array>
#include <numbers>
#define _USE_MATH_DEFINES
#include <cmath> 

int magnitude_thrust_array = 14;

//rocket properties
double mass = 1.01074445935;
double gravity = 9.81;
double center_of_pressure = 0.08775954;
double center_of_gravity = 0.4059174;
double distance_to_thrust_array = 6477;

double I_xx = 0.0249899588;
double I_yy = 0.0249868814;


double dt = 0.00001; 
int sim_time = 3;

std::array<double, 3> force_thrust_rf(double gimbal_angle_x, double gimbal_angle_y, double t);
std::array<double, 4> vector_pure_q(const std::array<double, 3>& vec);
std::array<double, 4> multiply_q_p(const std::array<double, 4>& q, const std::array<double, 4>& p); 
std::array<double, 4> conjugate_q(const std::array<double, 4>&q);
std::array<double, 3> rotate_rf_wf(const std::array<double, 4>& state_quaternion, const std::array<double, 3>& vector_rf); 
double degrees_to_rads(double angle_degrees);

int main(void){     
    double init_gimbal_angle_x_degrees = 2;
    double init_gimbal_angle_y_degrees = 3; 


    //quaterion initaliztion
    std::array<double, 4> state_q = {1.0, 0.0, 0.0, 0.0};

    //vector initalization
    std::array<double, 3> thrust_rf = {0.0, 0.0, 0.0};
    std::array<double, 3> thrust_wf = {0.0, 0.0, 0.0};
    std::array<double, 3> sum_forces_wf = {0.0, 0.0, 0.0};

    for(int i = 0; i < sim_time/dt; i++){
        double t = i * dt; 

        thrust_rf = force_thrust_rf(degrees_to_rads(init_gimbal_angle_x_degrees), degrees_to_rads(init_gimbal_angle_y_degrees), t);
        thrust_wf = rotate_rf_wf(state_q, thrust_rf); 

        sum_forces_wf = {thrust_wf.at(0), thrust_wf.at(1), thrust_wf.at(2)-mass*gravity}; 

        std::cout << "F_x = " << sum_forces_wf[0] << std::endl;
        
    }



    return 0;
}


std::array<double, 3> force_thrust_rf(double gimbal_angle_x, double gimbal_angle_y, double t){

    if(t > 3){
        std::array<double, 3> force_thrust_array_rf = {0.0, 0.0, 0.0}; 
        return force_thrust_array_rf;
    }
    
    else {
        std::array<double, 3> force_thrust_array_rf = {magnitude_thrust_array*std::sin(gimbal_angle_y), magnitude_thrust_array*std::sin(gimbal_angle_x), magnitude_thrust_array*std::cos(gimbal_angle_y)*std::cos(gimbal_angle_x)};   
        return force_thrust_array_rf;
    }
}
std::array<double, 4> vector_pure_q(const std::array<double, 3>& vec){
    std::array<double, 4> vec_q = {0.0, vec[0], vec[1], vec[2]} ;

    return vec_q;
}
std::array<double, 4> multiply_q_p(const std::array<double, 4>& q, const std::array<double, 4>& p){
    double q_p_0 = q[0]*p[0] - q[1]*p[1] - q[2]*p[2] - q[3]*p[3];
    double q_p_1 = q[0]*p[1] + q[1]*p[0] + q[2]*p[3] - q[3]*p[2];
    double q_p_2 = q[0]*p[2] - q[1]*p[3] + q[2]*p[0] + q[3]*p[1];
    double q_p_3 = q[0]*p[3] + q[1]*p[2] - q[2]*p[1] + q[3]*p[0];
    
    
    std::array<double, 4> q_times_p = {q_p_0, q_p_1, q_p_2, q_p_3};

    return q_times_p; 
}
std::array<double, 4> conjugate_q(const std::array<double, 4>&q){
    std::array<double, 4> q_star = {q[0], -q[1], -q[2], -q[3]};

    return q_star;
}
std::array<double, 3> rotate_rf_wf(const std::array<double, 4>& state_quaternion, const std::array<double, 3>& vector_rf){
    std::array<double, 4> array_rf_q = vector_pure_q(vector_rf);
    std::array<double, 4> q_1 = multiply_q_p(state_quaternion, array_rf_q);
    std::array<double, 4> conjugate_state_q = conjugate_q(state_quaternion); 
    std::array<double, 4> vector_wf_q = multiply_q_p(q_1, conjugate_state_q);
    
    std::array<double, 3> vector_wf = {vector_wf_q.at(1), vector_wf_q.at(2), vector_wf_q.at(3)}; 

    return vector_wf;
}
double degrees_to_rads(double angle_degrees){
    return (angle_degrees * (M_PI/180.0));
}