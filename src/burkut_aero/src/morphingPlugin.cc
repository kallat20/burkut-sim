#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <vector>
#include <iostream>

namespace gazebo
{
    // verileri tutacak struct
    struct AeroData{
        double kanat_acikligi;
        double alan;
        double CL0;
        double CLa;
        double CD0;
        double Cemq;
        double Cellp;
        double Cell_ctrl;
    };

    std::vector<AeroData> aero_table = {
        {0.00,  0.200, 0.150,  4.50,  0.025, -10.0, -0.35,  0.015},
        {0.25,  0.225, 0.150,  4.70,  0.026, -10.5, -0.42,  0.022},
        {0.50,  0.250, 0.150,  4.90,  0.027, -11.0, -0.50,  0.033},
        {0.75,  0.275, 0.150,  5.10,  0.028, -11.5, -0.60,  0.048},
        {1.00,  0.300, 0.150,  5.30,  0.029, -12.0, -0.72,  0.070}
    };

    // interpolasyon fonksiyonu
    double interpolasyon(double y1, double y2, double x1, double x2, double x){
        return y1 + (y2 - y1) * (x - x1) / (x2 - x1);
    }
    // plugin classı
    class MorphingPlugin : public ModelPlugin
    {
        //pointerlar
        private: physics::ModelPtr model;
        private: physics::LinkPtr base_link;
        private: physics::JointPtr morphing_joint;
        private: event::ConnectionPtr updateConnection;

        //sabitler
        private: double p_hava = 1.225; // kg/m^3
        
        
    }

};
