/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This is NOT an opmode. The purpose is to centralize all configuration in one spot.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel: Left front drive motor:    "left_front_motor"
 * Motor channel: Right front drive motor:   "right_front_motor"
 * Motor channel: Left rear drive motor:     "left_rear_motor"
 * Motor channel: Right rear drive motor:    "right_rear_motor"
 * Motor channel: Manipulator drive motor:   "left_arm"
 * Servo channel: Servo to open left claw:   "left_hand"
 * Servo channel: Servo to open right claw:  "right_hand"
 * Color Sensor:  REV color/distance sensor: "colro_sencor"
 *
 * Reference: org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot
 */
public class RobotConfig implements MecanumDrive
{
    /* Public OpMode members. */
    public DcMotor          leftFrontMotor  = null;
    public DcMotor          leftRearMotor   = null;
    public DcMotor          rightFrontMotor = null;
    public DcMotor          rightRearMotor  = null;

    public Servo            leftClaw        = null;
    public Servo            rightClaw       = null;
    public ColorSensor      sensorColor     = null;
    public DistanceSensor   sensorDistance  = null;
    public ElapsedTime      runtime         = new ElapsedTime();

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;



    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.823 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    /* local OpMode members. */
    HardwareMap hwMap       = null;
    OpMode app              = null;
    DriveMode driveMode = DriveMode.MANUAL;

    static final double length = 11.5;
    static final double width = 14.5;
    private double perimeter  = 1.414*Math.PI * Math.sqrt(length*length+width*width);


    /* Constructor - This is a singleton class. */
    private static RobotConfig theRobot = null;
    private RobotConfig(OpMode opmode, DriveMode mode) {
        app = opmode;
        opmode.telemetry.addData("Status", "Initialized");
        hwMap = opmode.hardwareMap;
        driveMode  = mode;

        // Define and Initialize Motors
        // These polarities are for the Neverest 20 motors
        leftFrontMotor = get_motor("left_front_motor", DcMotor.Direction.REVERSE);
        rightFrontMotor = get_motor("right_front_motor", DcMotor.Direction.FORWARD);
        leftRearMotor = get_motor("left_rear_motor", DcMotor.Direction.REVERSE);
        rightRearMotor = get_motor("right_rear_motor", DcMotor.Direction.FORWARD);

        if(leftFrontMotor == null){
            app.telemetry.addData("LeftFrontMotor", "LeftFrontMotor is null");
            app.telemetry.update();
            return;
        }


        //leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        //leftClaw  = hwMap.get(Servo.class, "left_hand");
        //rightClaw = hwMap.get(Servo.class, "right_hand");
        //leftClaw.setPosition(MID_SERVO);
        //rightClaw.setPosition(MID_SERVO);

/*        try {
            // The color sensor and the distance sensor are in the same device.
            sensorColor = hwMap.get(ColorSensor.class, "color_sensor");
            sensorDistance = hwMap.get(DistanceSensor.class, "color_sensor");
        } catch (Exception ex) {
            opmode.telemetry.addData("Exception", ex.getMessage());
        }
*/        opmode.telemetry.update();
    }

    private DcMotor get_motor (String name, DcMotor.Direction direction) {
        try {
            DcMotor motor = hwMap.get(DcMotor.class, name);
            motor.setPower(0);
            // Set the drive motor direction:
            // "Reverse" the motor that runs backwards when connected directly to the battery
            motor.setDirection(direction);
            // Set the drive motor run modes:
            // "RUN_USING_ENCODER" causes the motor to try to run at the specified fraction of full velocity
            // Note: We were not able to make this run mode work until we switched Channel A and B encoder wiring into
            // the motor controllers. (Neverest Channel A connects to MR Channel B input, and vice versa.)
            if(driveMode == DriveMode.AUTO) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            else
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            return motor;
        } catch (Exception ex) {
            app.telemetry.addData("status", "Missing motor: "+name);
            app.telemetry.addData("Exception", ex.getMessage());
        }
        return null;
    }

    /** Initialize standard Hardware interfaces
     *
     * @param  opmode    OpMode initializing this robot.
     */
    public static RobotConfig init(OpMode opmode, DriveMode mode) {
        if (theRobot == null) {
            theRobot = new RobotConfig(opmode, mode);
        }
        return theRobot;
    }

    public void reset() {
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    /**
     * Drive the motors by setting specified power levels.
     *
     * @param lf    Power level for leftFrontMotor.
     * @param rf    Power level for rightFrontMotor.
     * @param lr    Power level for leftRearMotor.
     * @param rr    Power level for leftRearMotor.
     */
    public void drive(double lf, double rf, double lr, double rr) {
        // Send values to the motors
        leftFrontMotor.setPower(lf);
        rightFrontMotor.setPower(rf);
        leftRearMotor.setPower(lr);
        rightRearMotor.setPower(rr);
        app.telemetry.addData("LF", "%.3f", lf);
        app.telemetry.addData("RF", "%.3f", rf);
        app.telemetry.addData("LR", "%.3f", lr);
        app.telemetry.addData("RR", "%.3f", rr);
    }

    public void manual_drive () {
/*        if (rightRearMotor == null) {
            app.telemetry.addData("Error", "The robot is missing motor.");
            return;
        }
        // declare motor speed variables
        double RF; double LF; double RR; double LR;
        // declare joystick position variables
        double X1; double Y1; double X2; double Y2;

        // run until the end of the match (driver presses STOP)
        // Reset speed variables
        LF = 0; RF = 0; LR = 0; RR = 0;

        // Get joystick values
        Y1 = -app.gamepad1.right_stick_y * joyScale; // invert so up is positive
        X1 = app.gamepad1.right_stick_x * joyScale;
        Y2 = -app.gamepad1.left_stick_y * joyScale; // Y2 is not used at present
        X2 = app.gamepad1.left_stick_x * joyScale;

        // Forward/back movement
        LF += Y1; RF += Y1; LR += Y1; RR += Y1;

        // Side to side movement
        LF += X1; RF -= X1; LR -= X1; RR += X1;

        // Rotation movement
        LF += X2; RF -= X2; LR += X2; RR -= X2;

        // Clip motor power values to +-motorMax
        LF = Math.max(-motorMax, Math.min(LF, motorMax));
        RF = Math.max(-motorMax, Math.min(RF, motorMax));
        LR = Math.max(-motorMax, Math.min(LR, motorMax));
        RR = Math.max(-motorMax, Math.min(RR, motorMax));
*/

//     Changed joystick operation: use left joystick to control robot translation, use right joystick to control robot turn.

        double leftX = app.gamepad1.left_stick_x;
        double leftY = -app.gamepad1.left_stick_y;
        double r = Math.hypot(leftX, leftY)*motorMax;
        double robotAngle = Math.atan2(leftY, leftX) - Math.PI / 4;
        double rightX = app.gamepad1.right_stick_x;
        final double LF = r * Math.cos(robotAngle) + rightX;
        final double RF = r * Math.sin(robotAngle) - rightX;
        final double LR = r * Math.sin(robotAngle) + rightX;
        final double RR = r * Math.cos(robotAngle) - rightX;
        // Send values to the motors
        drive(LF, RF, LR, RR);
        // Send some useful parameters to the driver station
        app.telemetry.addData("gamepad", "x: (%.2f), y: (%.2f) angle: (%.2f)", leftX, leftY, robotAngle*180/Math.PI);
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double angle,
                             double distanceInInches,
                             double timeoutS) {

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;

    // Ensure that the opmode is still active
        // Determine new target position, and pass to motor controller
        // Turn On RUN_TO_POSITION
        double robotAngle =  Math.toRadians(angle) - Math.PI / 4;

        if(leftFrontMotor == null){
            app.telemetry.addData("LeftFrontMotor", "LeftFrontMotor is null");
            app.telemetry.update();
            return;
        }

        newLeftFrontTarget = leftFrontMotor.getCurrentPosition() - (int)((distanceInInches * COUNTS_PER_INCH) * Math.cos(robotAngle));
        newRightFrontTarget = rightFrontMotor.getCurrentPosition() + (int)((distanceInInches * COUNTS_PER_INCH)* Math.sin(robotAngle));
        newLeftRearTarget = leftFrontMotor.getCurrentPosition() - (int)((distanceInInches * COUNTS_PER_INCH)* Math.sin(robotAngle));
        newRightRearTarget = rightFrontMotor.getCurrentPosition() + (int)((distanceInInches * COUNTS_PER_INCH)* Math.sin(robotAngle));

        leftFrontMotor.setTargetPosition(newLeftFrontTarget);
        rightFrontMotor.setTargetPosition(newRightFrontTarget);
        leftRearMotor.setTargetPosition(newLeftRearTarget);
        rightRearMotor.setTargetPosition(newRightRearTarget);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // reset the timeout time and start motion.
        runtime.reset();
        leftFrontMotor.setPower(Math.abs(speed));
        rightFrontMotor.setPower(Math.abs(speed));
        leftRearMotor.setPower(Math.abs(speed));
        rightRearMotor.setPower(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while ((runtime.seconds() < timeoutS) &&
                (leftFrontMotor.isBusy() && rightFrontMotor.isBusy())) {

            // Display it for the driver.
            app.telemetry.addData("Path1",  "Running to %7d: %7f", (int)distanceInInches,  angle);
            app.telemetry.addData("Path2",  "Running at %7d :%7d",
                    leftFrontMotor.getCurrentPosition(),
                    rightFrontMotor.getCurrentPosition());
            app.telemetry.update();
        }

        // Stop all motion;
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);

        // Turn off RUN_TO_POSITION
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
    }


    public void encoderTurn(double speed,
                            double angle) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;

    // Ensure that the opmode is still active

        // Determine new target position, and pass to motor controller
        // Turn On RUN_TO_POSITION

        newLeftFrontTarget = leftFrontMotor.getCurrentPosition() - (int)(angle*perimeter/360 * COUNTS_PER_INCH);
        newRightFrontTarget = rightFrontMotor.getCurrentPosition() + (int)(-angle*perimeter/360 * COUNTS_PER_INCH);
        newLeftRearTarget = leftFrontMotor.getCurrentPosition() - (int)(angle*perimeter/360 * COUNTS_PER_INCH);
        newRightRearTarget = rightFrontMotor.getCurrentPosition() + (int)(-angle*perimeter/360 * COUNTS_PER_INCH);


        leftFrontMotor.setTargetPosition(newLeftFrontTarget);
        rightFrontMotor.setTargetPosition(newRightFrontTarget);
        leftRearMotor.setTargetPosition(newLeftRearTarget);
        rightRearMotor.setTargetPosition(newRightRearTarget);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // reset the timeout time and start motion.
        runtime.reset();
        leftFrontMotor.setPower(Math.abs(speed));
        rightFrontMotor.setPower(Math.abs(speed));
        leftRearMotor.setPower(Math.abs(speed));
        rightRearMotor.setPower(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while ((leftFrontMotor.isBusy() && rightFrontMotor.isBusy())) {

            // Display it for the driver.
            app.telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
            app.telemetry.addData("Path2",  "Running at %7d :%7d",
                    leftFrontMotor.getCurrentPosition(),
                    rightFrontMotor.getCurrentPosition());
            app.telemetry.update();
        }

        // Stop all motion;
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);

        // Turn off RUN_TO_POSITION
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //  sleep(250);   // optional pause after each move
    }

    /**
     * Show run time.
     */
    public void showtime () {
        app.telemetry.addData("Run Time", runtime.toString());
        app.telemetry.update();
    }

    /**
     * Start the robot - reset the runtime to start counting.
     */
    public void start () {
        runtime.reset();
//        theRobot.reset();
    }

    /**
     * Stop the robot.
     */
    public void stop () {
        drive(0,0,0,0);
    }
}