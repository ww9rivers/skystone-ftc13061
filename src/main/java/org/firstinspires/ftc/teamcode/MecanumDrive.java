/**
 * Based on http://team9960.org/mecanum-drive-prototype-1-manual-drive-software/
 * -- Team 9960 Revision 161027.0
 */
package org.firstinspires.ftc.teamcode;

/**
 * Team 13061 version 1.0.0
 * This program provides driver station control of the Team 9960 Mecanum Drive Prototype.
 *
 * This robot uses four VEX Mecanum wheels, each direct driven by Neverest 20 motors.
 * It is designed as a linear op mode, and uses RUN_WITH_ENCODER motor operation.
 *
 * The gamepad1 right joystick is used for translation movement, while the left joystick x-axis controls rotation.
 *
 */

public interface MecanumDrive {

    /* Declare OpMode members. */
    // operational constants
    double joyScale = 0.5;
    double motorMax = 0.6; // Limit motor power to this value for Andymark RUN_USING_ENCODER mode

    public void manual_drive ();
    public void auto_drive(double drivePower, double driveAngle, double turnPower);
}
