#pragma once
#include <ct/core/core.h>  // as usual, include CT

class CustomController : public ct::core::Controller<2, 1>
{
public:
    static const size_t state_dim = 2;    // two states
    static const size_t control_dim = 1;  // one control action

    typedef Eigen::Matrix<double, control_dim, state_dim> GainVector;

    CustomController(GainVector gainK, ct::core::StateVector<state_dim> setPoint)
    : gainK_(gainK), setPoint_(setPoint) {}
    ~CustomController() {}
    CustomController(const CustomController& other) : gainK_(other.gainK_), setPoint_(other.setPoint_) {}
    CustomController* clone() const override
    {
        return new CustomController(*this);  // calls copy constructor
    }
    void computeControl(const ct::core::StateVector<state_dim>& state,
        const double& t,
        ct::core::ControlVector<control_dim>& controlAction) override
    {
        controlAction(0) = -gainK_*(state-setPoint_);  //feedback control
    }
private:
    GainVector gainK_;
    ct::core::StateVector<state_dim> setPoint_;
};

class CartpoleController : public ct::core::Controller<4, 1>
{
public:
    static const size_t state_dim = 4;    // two states
    static const size_t control_dim = 1;  // one control action

    typedef Eigen::Matrix<double, control_dim, state_dim> GainVector;

    CartpoleController(GainVector gainK, ct::core::StateVector<state_dim> setPoint)
    : gainK_(gainK), setPoint_(setPoint) {}
    ~CartpoleController() {}
    CartpoleController(const CartpoleController& other) : gainK_(other.gainK_), setPoint_(other.setPoint_) {}
    CartpoleController* clone() const override
    {
        return new CartpoleController(*this);  // calls copy constructor
    }
    void computeControl(const ct::core::StateVector<state_dim>& state,
        const double& t,
        ct::core::ControlVector<control_dim>& controlAction) override
    {
        controlAction(0) = -gainK_*(state-setPoint_);  //feedback control
    }
private:
    GainVector gainK_;
    ct::core::StateVector<state_dim> setPoint_;
};