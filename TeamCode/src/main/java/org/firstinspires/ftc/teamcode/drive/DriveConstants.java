package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/*
 * Constants shared between multiple drive types.
 */
@Config
public class DriveConstants {

    /*
     * These are motor constants that should be listed online for your motors.
     */
    public static final double TICKS_PER_REV = 354;
    public static final double MAX_RPM = 312;
    public static final double MAX_DISTANCE = 540;


    /*
     * Set RUN_USING_ENCODER to true to enable built-in hub velocity control using drive encoders.
     */
    public static final boolean RUN_USING_ENCODER = false;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
      getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

    /*
     *physical constants
     */
    public static double WHEEL_RADIUS = 1.37795276/2; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 16.3385827; // in

    /*
     * feedforward parameters
     */
    public static double kV = 1 / rpmToVelocity(MAX_RPM);
    public static double kA = 0.2;
    public static double kStatic = 0.01;

    /*
     * values used to generate the trajectories for robot
     */

    public static double MAX_VEL = 52.48291908330528;
    public static double MAX_ACCEL = 52.48291908330528;
    public static double MAX_ANG_VEL = Math.toRadians(187.94061000000002);
    public static double MAX_ANG_ACCEL = Math.toRadians(187.94061000000002);


    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
      // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
      return 32767 / ticksPerSecond;
    }
}