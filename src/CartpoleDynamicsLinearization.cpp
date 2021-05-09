#include <ct/core/core.h>
#include <cmath>
#include <cppad/cg.hpp>
#include <cppad/cppad.hpp>
#include <cppad/example/cppad_eigen.hpp>
#include <ros/package.h>
#include "my_ct_project/Cartpole.h"

int main(int argc, char** argv)
{
    // define the dimensions of the system
    const size_t state_dim = cartpole::CartpoleSystem::STATE_DIM;
    const size_t control_dim = cartpole::CartpoleSystem::CONTROL_DIM;

    // typedefs for the auto-differentiable system
    typedef CppAD::AD<double> AD_Scalar;
    typedef cartpole::tpl::CartpoleSystem<AD_Scalar> CartpoleSystemAD;

    // typedefs for the auto-differentiable codegen system
    typedef ct::core::ADCodegenLinearizer<state_dim, control_dim>::ADCGScalar Scalar;
    typedef typename Scalar::value_type AD_ValueType;
    typedef cartpole::tpl::CartpoleSystem<Scalar> CartpoleSystemADCG;    

    // handy typedefs for the Jacobian
    typedef Eigen::Matrix<double, state_dim, state_dim> A_type;
    typedef Eigen::Matrix<double, state_dim, control_dim> B_type;

    // create nonlinear systems
    double m1 = 5;  // (kg) Cart mass
    double m2 = 1;  // (kg) pole mass
    double l = 2;   // (m) pendulum (pole) length
    std::shared_ptr<cartpole::CartpoleSystem> cartpoleSystem(new cartpole::CartpoleSystem(m1, m2, l));
    std::shared_ptr<CartpoleSystemAD> cartpoleSystemAD(new CartpoleSystemAD(AD_Scalar(m1), AD_Scalar(m2), AD_Scalar(l)));
    std::shared_ptr<CartpoleSystemADCG> cartpoleSystemADCG(new CartpoleSystemADCG(AD_ValueType(m1), AD_ValueType(m2), AD_ValueType(l)));

    // create a linearizer that applies numerical differentiation
    ct::core::SystemLinearizer<state_dim, control_dim> systemLinearizer(cartpoleSystem);
    // create a linearizer that uses auto-diff
    ct::core::AutoDiffLinearizer<state_dim, control_dim> adLinearizer(cartpoleSystemAD);
    // create a linearizer that uses auto-diff codegen
    ct::core::ADCodegenLinearizer<state_dim, control_dim> adcgLinearizer(cartpoleSystemADCG);

    std::cout << "compiling..." << std::endl;
    adcgLinearizer.compileJIT("ADCGCodegenLib");
    std::cout << "... done!" << std::endl;

    try
     {
        std::cout << "generating code..." << std::endl;
        // generate code for the Jacobians
        adcgLinearizer.generateCode("CartpoleSystemLinearized", 
            ros::package::getPath("my_ct_project")+"/generated");
        std::cout << "... done!" << std::endl;
     } catch (const std::runtime_error& e)
     {
         std::cout << "code generation failed: " << e.what() << std::endl;
     }

    // create state, control and time variables
    ct::core::StateVector<state_dim> x;
    x << 0, M_PI, 0, 0;
    ct::core::ControlVector<control_dim> u;
    u << 0;
    double t = 0;

    // use the numerical differentiation linearizer
    A_type A_system = systemLinearizer.getDerivativeState(x, u, t);
    B_type B_system = systemLinearizer.getDerivativeControl(x, u, t);
    // use the auto differentiation linearzier
    A_type A_ad = adLinearizer.getDerivativeState(x, u, t);
    B_type B_ad = adLinearizer.getDerivativeControl(x, u, t);
    // use the auto differentiation linearzier
    A_type A_adcg = adcgLinearizer.getDerivativeState(x, u, t);
    B_type B_adcg = adcgLinearizer.getDerivativeControl(x, u, t);

    std::cout << "Numerical Differentiation A: \n";
    std::cout << A_system << std::endl;
    std::cout << "Numerical Differentiation B: \n";
    std::cout << B_system << std::endl << std::endl;
    std::cout << "Automatic Differentiation A: \n";
    std::cout << A_ad << std::endl;
    std::cout << "Automatic Differentiation B: \n";
    std::cout << B_ad << std::endl << std::endl;
    std::cout << "Automatic Differentiation Codegen A: \n";
    std::cout << A_adcg << std::endl;
    std::cout << "Automatic Differentiation Codegen B: \n";
    std::cout << B_adcg << std::endl << std::endl;
    
    return 0;
}