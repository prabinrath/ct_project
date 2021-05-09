#include <ct/core/core.h>
#include "my_ct_project/Masspoint.h"
int main(int argc, char** argv)
{
    // a damped oscillator has two states, position and velocity
    const size_t state_dim = Masspoint::STATE_DIM;  // = 2

    // create a state
    ct::core::StateVector<state_dim> x;    
    x(0) = 5;
    x(1) = 0;
    // create our mass point instance
    double m = 1.5;  //Mass in Kg
    double d = -2.5; //Damping ratio
    double k = 80; //Spring constant
    std::shared_ptr<Masspoint> masspoint(new Masspoint(m, k, d));
    
    // create an integrator
    ct::core::Integrator<state_dim> integrator(masspoint, ct::core::IntegrationType::RK4);
    // simulate 1000 steps
    double dt = 0.01;
    ct::core::Time t0 = 0.0;
    size_t nSteps = 500;
    integrator.integrate_n_steps(x, t0, nSteps, dt);
    // print the new state
    std::cout << "state after integration: " << x.transpose() << std::endl;
    return 0;
}