#pragma once
#include <cmath>
#include <memory>
#include <iostream>
#include <ct/core/core.h>  // as usual, include CT

class Masspoint : public ct::core::System<2>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    static const size_t STATE_DIM = 2;
    // constructor
    Masspoint(double mass, double k, double d) : mass_(mass), k_(k), d_(d) {}
    // copy constructor
    Masspoint(const Masspoint& other) : mass_(other.mass_), k_(other.k_), d_(other.d_) {}
    // destructor
    ~Masspoint() = default;
    // clone method for deep copying
    Masspoint* clone() const override
    {
        return new Masspoint(*this);  // calls copy constructor
    }
    // The system dynamics. We override this method which gets called by e.g. the Integrator
    void computeDynamics(const ct::core::StateVector<STATE_DIM>& x,
        const ct::core::Time& t,
        ct::core::StateVector<STATE_DIM>& derivative) override
    {
        // first part of state derivative is the velocity
        derivative(0) = x(1);
        // second part is the acceleration which is caused by damper forces
        derivative(1) = (-k_ / mass_) * x(0) + (-d_ / mass_) * x(1);
    }
private:
    double mass_;
    double k_;
    double d_;
};

class MasspointControlled : public ct::core::ControlledSystem<2, 1>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef ct::core::ControlledSystem<2, 1> Base;
    static const size_t STATE_DIM = 2, CONTROL_DIM = 1;

    // constructor
    MasspointControlled(double mass, double k, double d, std::shared_ptr<ct::core::Controller<2, 1>> controller = nullptr) 
    : mass_(mass), k_(k), d_(d), Base(controller, ct::core::SYSTEM_TYPE::SECOND_ORDER) 
    {
        A << 0, 1, (-k_ / mass_), (-d_ / mass_);
        B << 0, (1 / mass_);
    }
    // copy constructor
    MasspointControlled(const MasspointControlled& other) 
    : mass_(other.mass_), k_(other.k_), d_(other.d_), A(other.A), B(other.B) {}
    // destructor
    ~MasspointControlled() = default;

    // clone method for deep copying
    MasspointControlled* clone() const override
    {
        return new MasspointControlled(*this);  // calls copy constructor
    }
    // The system controlled dynamics. We override this method which gets called by e.g. the Integrator
    void computeControlledDynamics(const ct::core::StateVector<STATE_DIM>& state,
         const time_t& t,
         const ct::core::ControlVector<CONTROL_DIM>& control,
         ct::core::StateVector<STATE_DIM>& derivative) override
     {
        //  derivative(0) = state(1);
        //  derivative(1) = (-k_ / mass_) * state(0) + (-d_ / mass_) * state(1) + (control(0) / mass_);
        derivative = A * state + B * control(0);
     }
 
private:
    double mass_;
    double k_;
    double d_;
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> A;
    Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> B;
};