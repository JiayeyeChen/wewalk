#ifndef JOYSTICK_1_CONTAINER_HPP
#define JOYSTICK_1_CONTAINER_HPP

#include <gui_generated/containers/joystick_1_containerBase.hpp>

class joystick_1_container : public joystick_1_containerBase
{
public:
    joystick_1_container();
    virtual ~joystick_1_container() {}

    virtual void initialize();
protected:
};

#endif // JOYSTICK_1_CONTAINER_HPP
