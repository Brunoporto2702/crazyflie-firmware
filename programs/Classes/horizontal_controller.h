# ifndef horizontal_controller_h
# define horizontal_controller_h

# include "mbed.h"
# include "src/utils/parameters.h"

// Attitude controller class
class HorizontalController
{
    public:
        // Class constructor
        HorizontalController();

        // Control torques (N.m) given reference angles (rad) and current angles ( rad ) and angular velocities ( rad /s)
        void control(float x, float y, float x_r, float y_r, float vx, float vy);

        // angles out 
        float theta_r;
        float phi_r;

    private:
       
      
};

# endif
