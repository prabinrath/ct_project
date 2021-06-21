#include <ct/optcon/optcon.h> 
#include <cmath>
#include <ros/package.h>
#include "my_ct_project/Cartpole.h"
#include "my_ct_project/CustomController.h"
#include "CartpoleSystemLinearized.h"

int main(int argc, char** argv)
{
    // define the dimensions of the system
    const size_t state_dim = cartpole::CartpoleSystem::STATE_DIM;
    const size_t control_dim = cartpole::CartpoleSystem::CONTROL_DIM;

    // typedefs for the auto-differentiable codegen system
    typedef ct::core::generated::CartpoleSystemLinearized CartpoleSystemLinearized;

    // create nonlinear systems, must remain consistent with the paarameters used for CartpoleSystemLinearized codegen
    double m1 = 5;  // (kg) Cart mass
    double m2 = 1;  // (kg) pole mass
    double l = 2;   // (m) pendulum (pole) length

    std::shared_ptr<CartpoleSystemLinearized> cartpoleLinearized(new CartpoleSystemLinearized());
    // create state, control and time variables
    CartpoleSystemLinearized::state_vector_t x;
    x << 0, M_PI, 0, 0;
    CartpoleSystemLinearized::control_vector_t u;
    u << 0;
    double t = 0;

    // use the auto differentiation linearzier
    auto A = cartpoleLinearized->getDerivativeState(x, u, t);
    auto B = cartpoleLinearized->getDerivativeControl(x, u, t);

    // load the weighting matrices
    // ct::optcon::TermQuadratic<state_dim, control_dim> quadraticCost;
    // quadraticCost.loadConfigFile(ros::package::getPath("my_ct_project")+"/config/lqrCost.info", "termLQR");
    // auto Q = quadraticCost.stateSecondDerivative(x, u, t);    // x, u and t can be arbitrary here
    // auto R = quadraticCost.controlSecondDerivative(x, u, t);  // x, u and t can be arbitrary here

    Eigen::Matrix<double, state_dim, state_dim> Q;
    Q << 10, 0, 0, 0,
         0, 10, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1; 

    Eigen::Matrix<double, control_dim, control_dim> R;
    R << 0.01;

    // design the LQR controller
    ct::optcon::LQR<state_dim, control_dim> lqrSolver;
    ct::core::FeedbackMatrix<state_dim, control_dim> K;
    std::cout << "A: " << std::endl << A << std::endl << std::endl;
    std::cout << "B: " << std::endl << B << std::endl << std::endl;
    std::cout << "Q: " << std::endl << Q << std::endl << std::endl;
    std::cout << "R: " << std::endl << R << std::endl << std::endl;
    lqrSolver.compute(Q, R, A, B, K);
    std::cout << "LQR gain matrix:" << std::endl << K << std::endl;
    
    std::shared_ptr<cartpole::CartpoleSystem> cartpoleSystem(new cartpole::CartpoleSystem(m1, m2, l));

    // create our controller
    CartpoleController::GainVector gainK = K;
    ct::core::StateVector<state_dim> state, setPoint = x;
    state << 1, M_PI+0.7, 0, 0;
    std::shared_ptr<CartpoleController> controller(new CartpoleController(gainK, setPoint));
    // assign our controller
    cartpoleSystem->setController(controller);

    // create an integrator
    ct::core::Integrator<state_dim> integrator(cartpoleSystem, ct::core::IntegrationType::RK4);
    // simulate steps
    double dt = 0.01;
    ct::core::Time t0 = 0.0;
    size_t nSteps = 200;
    integrator.integrate_n_steps(state, t0, nSteps, dt);
    // print the new state
    std::cout << "state after integration: " << state.transpose() << std::endl;

    return 0;
}