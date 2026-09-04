# Experimental GNC editor features

Experimental autopilot panels (for example LQR or MPC) belong here as sibling
features. They should own their event/model/controller boundary and use typed
messaging commands. They must not add experimental parameters to the PX4
attitude or TECS feature controllers.
