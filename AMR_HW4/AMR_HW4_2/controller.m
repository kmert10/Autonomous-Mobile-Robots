function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

    u1 = 0;
    u2 = 0;

    % FILL IN YOUR CODE HERE
    Kp_t=30; Kd_t=20;
    z_des_2 = des_state.acc(2);
    error_z_1 = des_state.vel(2) - state.vel(2);
    error_z = des_state.pos(2) - state.pos(2);
    u1= params.mass * (params.gravity + z_des_2 + Kp_t*error_z + Kd_t*error_z_1);

    Kp_y=5;
    Kd_y=18;
    y_des_2 = des_state.acc(1);
    error_y_1 = des_state.vel(1) - state.vel(1);
    error_y = des_state.pos(1) - state.pos(1);
    q_commanded= -1/params.gravity * (y_des_2 + Kp_y*error_y + Kd_y*error_y_1);

    q_commanded_dot=-1/params.gravity * (0 + Kp_y*error_y_1);

    Kp_angle=200;
    Kd_angle=40;
    error_q = q_commanded - state.rot;
    error_q_dot =q_commanded_dot - state.omega;
    u2= params.Ixx*( 0+Kp_angle*error_q + Kd_angle*error_q_dot);

end

