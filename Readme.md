# FCND-Term1-P3-3D-Quadrotor-Controller
Udacity Flying Car Nanodegree - Term 1 - Project 3 - 3D Quadrotor Controller

In this project, we implemented and tuned a cascade PID controller.

# Project description

The controller needs to be implemented with c++.

## Prerequisites

To run this project, you need to have the following software installed:
    Visual Studio 2017
    Simulator
    Codebase supplied from Udacity

### Implemented Controller

#### 1. Implemented body rate control in C++.

**C++ code**
```cpp
V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
    // Calculate a desired 3-axis moment given a desired and current body rate
    // INPUTS:
    //   pqrCmd: desired body rates [rad/s]
    //   pqr: current or estimated body rates [rad/s]
    // OUTPUT:
    //   return a V3F containing the desired moments for each of the 3 axes

    // HINTS:
    //  - you can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
    //  - you'll need parameters for moments of inertia Ixx, Iyy, Izz
    //  - you'll also need the gain parameter kpPQR (it's a V3F)

    V3F momentCmd;

    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    const V3F pqr_dot = kpPQR * (pqrCmd - pqr);
    momentCmd = V3F(Ixx, Iyy, Izz) * pqr_dot;
    /////////////////////////////// END STUDENT CODE ////////////////////////////

    return momentCmd;
}
```

#### 2. Implement roll pitch control in C++.

The roll-pitch controller takes as input the commanded acceleration (NED), the current attitude and the thrust command, and outputs the desired pitch and roll rates in body frame.

First, we obtain the current tilt `b_a_y` and `b_a_y` from the rotation matrix `R`.

Then, we compute the desired tilt `b_c_x` and `b_c_y` by normalizing the desired acceleration by the thrust. Note that we must divide it by the mass of the drone to convert it to acceleration. A negative sign is also needed since thrust is positive upwards but the acceleration is given in NED coordinates (positive down). We constrain it to a maximum and minimum tilt value to prevent the drone from fliping.

Next, a P controller determines the desired roll and pitch rate in the world frame (`b_c_x_dot` and `b_c_y_dot`).

Finally, in order to output the desired roll and pitch rate in the body frame, we apply a non-linear transformation as seen in the lectures, taking into account the rotation matrix as seen in the lectures. Below the full source code is presented:

**C++ code**

```cpp
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
    // Calculate a desired pitch and roll angle rates based on a desired global
    //   lateral acceleration, the current attitude of the quad, and desired
    //   collective thrust command
    // INPUTS:
    //   accelCmd: desired acceleration in global XY coordinates [m/s2]
    //   attitude: current or estimated attitude of the vehicle
    //   collThrustCmd: desired collective thrust of the quad [N]
    // OUTPUT:
    //   return a V3F containing the desired pitch and roll rates. The Z
    //     element of the V3F should be left at its default value (0)

    // HINTS:
    //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
    //  - you'll need the roll/pitch gain kpBank
    //  - collThrustCmd is a force in Newtons! You'll likely want to convert it to acceleration first

    V3F pqrCmd;
    Mat3x3F R = attitude.RotationMatrix_IwrtB();

    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    // Current attitude
    const float b_x_a = R(0,2);
    const float b_y_a = R(1,2);

    // Target attitude
    const float thrust_acceleration = -collThrustCmd / mass;
    const float b_x_c = accelCmd.x / (thrust_acceleration);
    const float b_y_c = accelCmd.y / (thrust_acceleration);

    // Commanded rates in world frame
    const float b_x_c_dot = kpBank * (b_x_c - b_x_a);
    const float b_y_c_dot = kpBank * (b_y_c - b_y_a);

    // Roll and pitch rates
    const float r_33_inv = 1.0F / R(2,2);
    pqrCmd.x =  r_33_inv * (R(1,0)*b_x_c_dot - R(0,0)*b_y_c_dot);
    pqrCmd.y =  r_33_inv * (R(1,1)*b_x_c_dot - R(0,1)*b_y_c_dot);
    pqrCmd.z = 0.0F;  // yaw controller set in YawControl
    /////////////////////////////// END STUDENT CODE ////////////////////////////

    return pqrCmd;
}

```

#### 3. Implement altitude controller in C++.

The only difference in this case is that we are provided with constraints about the ascent and descent rate of the drone, which we take into account by constraining the commanded velocity.

The full source code is shown below:

```cpp
float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ,
                                   Quaternion<float> attitude, float accelZCmd, float dt)
{
    // Calculate desired quad thrust based on altitude setpoint, actual altitude,
    //   vertical velocity setpoint, actual vertical velocity, and a vertical
    //   acceleration feed-forward command
    // INPUTS:
    //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
    //   posZ, velZ: current vertical position and velocity in NED [m]
    //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
    //   dt: the time step of the measurements [seconds]
    // OUTPUT:
    //   return a collective thrust command in [N]

    // HINTS:
    //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
    //  - you'll need the gain parameters kpPosZ and kpVelZ
    //  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
    //  - make sure to return a force, not an acceleration
    //  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER

    Mat3x3F R = attitude.RotationMatrix_IwrtB();
    float thrust = 0;

    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    // Get z component of the thrust
    const float b_z = R(2,2);

    // Constrain commanded velocity (NED, descending means higher Z)
    velZCmd = CONSTRAIN(velZCmd, -maxAscentRate, maxDescentRate);

    // Compute error
    const float error = posZCmd - posZ;
    const float error_dot = velZCmd - velZ;
    integratedAltitudeError += error * dt;

    // Compute desired acceleration
    const float u1_bar = kpPosZ * error + \
                         kpVelZ * error_dot + \
                         KiPosZ * integratedAltitudeError + \
                         accelZCmd;
    float acc_z_desired = (u1_bar - CONST_GRAVITY) / b_z;

    // Compute thrust (positive upwards)
    thrust = -acc_z_desired * mass;
    /////////////////////////////// END STUDENT CODE ////////////////////////////

    return thrust;
}
```

#### 4. Implement lateral position control in C++.

Similar to the altitude controller, the lateral control is a second-order system problem, and thus we need to use a PD controller. The code is rather simple since we simply take as input position and velocities and output desired accelerations, all in NED coordinates.

In the C++ code, we contrain the commanded velocity and acceleration values to the physical limits of the drone.

**C++ code**

```cpp
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmd)
{
    // Calculate a desired horizontal acceleration based on
    // desired lateral position/velocity/acceleration and current pose
    // INPUTS:
    //   posCmd: desired position, in NED [m]
    //   velCmd: desired velocity, in NED [m/s]
    //   pos: current position, NED [m]
    //   vel: current velocity, NED [m/s]
    //   accelCmd: desired acceleration, NED [m/s2]
    // OUTPUT:
    //   return a V3F with desired horizontal accelerations.
    //     the Z component should be 0
    // HINTS:
    //  - use fmodf(foo,b) to constrain float foo to range [0,b]
    //  - use the gain parameters kpPosXY and kpVelXY
    //  - make sure you cap the horizontal velocity and acceleration
    //    to maxSpeedXY and maxAccelXY

    // make sure we don't have any incoming z-component
    accelCmd.z = 0;
    velCmd.z = 0;
    posCmd.z = pos.z;

    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    // Contrain desired velocity
    velCmd.x = CONSTRAIN(velCmd.x, -maxSpeedXY, maxSpeedXY);
    velCmd.y = CONSTRAIN(velCmd.y, -maxSpeedXY, maxSpeedXY);

    // Compute PD controller + feedforward
    const V3F error = posCmd - pos;
    const V3F error_dot = velCmd - vel;

    accelCmd = kpPosXY*error + kpVelXY*error_dot + accelCmd;

    // Constrain desired acceleration
    accelCmd.x = CONSTRAIN(accelCmd.x, -maxAccelXY, maxAccelXY);
    accelCmd.y = CONSTRAIN(accelCmd.y, -maxAccelXY, maxAccelXY);
    accelCmd.z = 0.0F;
    /////////////////////////////// END STUDENT CODE ////////////////////////////

    return accelCmd;
}


```

#### 5. Implement yaw control in C++.

Finally, the yaw controller is a P controller that takes as input the current and commanded yaw, and outputs the desired yaw rate in rad/s.

The only caveat here is that we need to **normalize** the error to account for angle wrap.

The full source code is shown below.

**C++ code**

```cpp
float NormAngle(const float x)
{
    float y = fmodf(x + F_PI, 2.0F*F_PI);

    if (y < 0.0F)
    {
        y += 2.0F*F_PI;
    }

    return y - F_PI;
}

float QuadControl::YawControl(float yawCmd, float yaw)
{
    // Calculate a desired yaw rate to control yaw to yawCmd
    // INPUTS:
    //   yawCmd: commanded yaw [rad]
    //   yaw: current yaw [rad]
    // OUTPUT:
    //   return a desired yaw rate [rad/s]
    // HINTS:
    //  - use fmodf(foo,b) to constrain float foo to range [0,b]
    //  - use the yaw control gain parameter kpYaw

    float yawRateCmd=0;
    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    const float error = NormAngle(yawCmd - yaw);
    yawRateCmd = kpYaw * error;
    /////////////////////////////// END STUDENT CODE ////////////////////////////

    return yawRateCmd;
}
```

#### 6. Implement calculating the motor commands given commanded thrust and moments in C++.

We infer the motor commands following the same procedure as described in the lectures:

1. Determine the physical equations that govern the motion of the quadcopter: 1 equation for thrust, and 3 equations for torque.
2. Write them in an `Ax = b` form and solve for x.

Note however that this exercise was different from the lectures in the following ways:

- Motors 3 and 4 are swapped.
- The motors spin in opposite direction than shown in the lecture.
- The constants `k_m` and `k_f` are not given. Instead, the ration between them, `kappa`, is given.
- The distance `L` is the distance from the center of the quad to one of the rotors.

With these considerations, we solve the linear equation symbolically using Matlab and write the operations directly in C++ to improve computational performance. For example we expand all the operations instead of performing matrix multiplication.

The full source code is shown below:

```cpp
VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
    // Convert a desired 3-axis moment and collective thrust command to
    //     individual motor thrust commands
    // INPUTS:
    //     collThrustCmd: desired collective thrust [N]
    //     momentCmd: desired rotation moment about each axis [N m]
    // OUTPUT:
    //     set class member variable cmd (class variable for graphing) where
    //     cmd.desiredThrustsN[0..3]: motor commands, in [N]

    // HINTS:
    // - you can access parts of desMoment via e.g. desMoment.x
    // You'll need the arm length parameter L, and the drag/thrust ratio kappa

    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    const float l = L * 0.5F * sqrt(2.0F);  // Arm length perpendicular to X-Y axis
    const float l_inv = 1.0F / l;
    const float k_inv = 1.0F / kappa;

    const float f = collThrustCmd;
    const float t_x = momentCmd.x;
    const float t_y = momentCmd.y;
    const float t_z = momentCmd.z;

    // This comes from the matrix equation:
    // [ 1  1  1  1][F1]     [Ft   ]
    // [ L -L  L -L][F2]     [tau_x]
    // [ L  L -L -L][F3] =   [tau_y]
    // [-K  K  K -K][F4]     [tau_z]

    // 1) The motors spin in opposite direction
    // 2) M3 and M4 are swapped

    // The inverse of the 4x4 matrix is:
    // [0.25,  0.25/L,  0.25/L, -0.25/K],
    // [0.25, -0.25/L,  0.25/L,  0.25/K],
    // [0.25,  0.25/L, -0.25/L,  0.25/K],
    // [0.25, -0.25/L, -0.25/L, -0.25/K]]
    cmd.desiredThrustsN[0] = 0.25 * (f + l_inv*t_x + l_inv*t_y - k_inv*t_z); // front left
    cmd.desiredThrustsN[1] = 0.25 * (f - l_inv*t_x + l_inv*t_y + k_inv*t_z); // front right
    cmd.desiredThrustsN[2] = 0.25 * (f + l_inv*t_x - l_inv*t_y + k_inv*t_z); // rear left
    cmd.desiredThrustsN[3] = 0.25 * (f - l_inv*t_x - l_inv*t_y - k_inv*t_z); // rear right

    /////////////////////////////// END STUDENT CODE ////////////////////////////

    return cmd;
}
```

### Flight Evaluation

#### 1. Your python controller is successfully able to fly the provided test trajectory, meeting the minimum flight performance metrics.

#Simulation1 (../config/1_Intro.txt)
PASS: ABS(Quad.PosFollowErr) was less than 0.500000 for at least 0.800000 second

#Simulation2 (../config/2_AttitudeControl.txt)
Simulation #2 (../config/2_AttitudeControl.txt)
PASS: ABS(Quad.Roll) was less than 0.025000 for at least 0.750000 seconds
PASS: ABS(Quad.Omega.X) was less than 2.500000 for at least 0.750000 seconds

#Simulation3 (../config/3_PositionControl.txt)
PASS: ABS(Quad1.Pos.X) was less than 0.100000 for at least 1.250000 seconds
PASS: ABS(Quad2.Pos.X) was less than 0.100000 for at least 1.250000 seconds
PASS: ABS(Quad2.Yaw) was less than 0.100000 for at least 1.000000 seconds

#Simulation4 (../config/4_Nonidealities.txt)
PASS: ABS(Quad1.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
PASS: ABS(Quad2.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
PASS: ABS(Quad3.PosFollowErr) was less than 0.100000 for at least 1.500000 secon

#Simulation5 (../config/5_TrajectoryFollow.txt)
PASS: ABS(Quad2.PosFollowErr) was less than 0.250000 for at least 3.000000 seconds
