# ifndef vertical_controller_h
# define vertical_controller_h

# include "mbed.h"
# include "src/utils/parameters.h"

// Attitude controller class
class VerticalController
{
    public:
        // Class constructor
        VerticalController();

        // Control torques (N.m) given reference angles (rad) and current angles ( rad ) and angular velocities ( rad /s)
        void control(float z, float w, float r);

        // Torques (N.m)
        float Ft;

    private:
       
        void deploy( );
};

# endif
