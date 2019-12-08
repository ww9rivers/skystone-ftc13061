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
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.REVIMU;

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
 * Motor channel: Left rear drive motor:     "left_rear_motor"
 * Motor channel: Right rear drive motor:    "right_rear_motor"
 * Motor channel: Left front drive motor:    "left_front_motor"
 * Motor channel: Right front drive motor:   "right_front_motor"
 * Motor channel: Manipulator drive motor:   "left_arm"
 * Servo channel: Servo to open left claw:   "left_hand"
 * Servo channel: Servo to open right claw:  "right_hand"
 * Color Sensor:  REV color/distance sensor: "colro_sencor"
 *
 * Reference: org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot
 */
public class RobotConfig extends MecanumDrive
{
    /* local OpMode members. */
    HardwareMap hwMap       = null;
    OpMode app              = null;
    DriveMode driveMode = DriveMode.MANUAL;

    static final double length = 11.5;
    static final double width = 14.5;
    private double perimeter  = 1.414*Math.PI * Math.sqrt(length*length+width*width);


    // Hardware components
    //  Device for direction:
    protected REVIMU        imu = null;

    public Servo            leftClaw        = null;
    public Servo            rightClaw       = null;
    public Servo            puller1         = null;
    public Servo            puller2         = null;
    public ColorSensor      colorSensor     = null;
    public DistanceSensor   distanceSensor  = null;
    public DigitalChannel   touchSensor     = null;
    public ElapsedTime      runtime         = new ElapsedTime();

    // NOTE: Different copy-and-pasted code use different names for these constants.
    //       Use both sets, but still trying to keep things understandable.
    // https://www.reddit.com/r/FTC/comments/7s6y32/rev_hd_hex_motor_encoder_cpr/
    private static final int ENCODER_CPR_HEXHD  = 1120;     // CPR of REV Hex HD Motor (REV-41-1301)
    private static final int ENCODER_CPR_TETRIX = 1440;     // CPR of Tetrix DC motor
    private static final int ENCODER_CPR        = ENCODER_CPR_HEXHD;    // We are using REV Hex HD motors
    private static final double PULLER_DOWN     = 0.0;      // Maximum rotational position
    private static final double PULLER_UP       = 0.5;      // Maximum rotational position
    public static final double MID_SERVO        = 0.5 ;
    public static final double ARM_UP_POWER     = 0.45 ;
    public static final double ARM_DOWN_POWER   = -0.45 ;
    static final double COUNTS_PER_MOTOR_REV    = ENCODER_CPR;  // REV Hex HD motor w/ encoder
    static final double DRIVE_GEAR_REDUCTION    = 1.175 ;       // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES   = 4.0 ;         // For figuring circumference
    static final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    private static final double ENCODER_CPI     = COUNTS_PER_INCH;  // encoder counts per inch
    public static final double DRIVE_SPEED      = 0.6;
    public static final double TURN_SPEED       = 0.5;

    public static final String STATUS           = "Status";
    public static final String TOUCH_SENSOR     = "touch_sensor";
    private static final String SERVOS[]        = {
        "servo0", "servo1", "servo2", "servo3", "servo4", "servo5"
    };
    public static Servo servo[] = { null, null, null, null };

    /* Constructor - This is a singleton class. */
    private static RobotConfig theRobot = null;
    private RobotConfig (OpMode opmode, DriveMode mode) {
        app = opmode;
        opmode.telemetry.addData(STATUS, "Initialized");
        hwMap = opmode.hardwareMap;
        driveMode  = mode;

        // Define and Initialize Motors
        // These polarities are for the Neverest 20 motors
        leftFrontMotor = get_motor("left_front_motor", DcMotor.Direction.FORWARD);
        rightFrontMotor = get_motor("right_front_motor", DcMotor.Direction.FORWARD);
        leftRearMotor = get_motor("left_rear_motor", DcMotor.Direction.FORWARD);
        rightRearMotor = get_motor("right_rear_motor", DcMotor.Direction.FORWARD);

        puller1 = hwMap.get(Servo.class, "servo3");
        puller2 = hwMap.get(Servo.class, "servo4");

        if(leftFrontMotor == null){
            app.telemetry.addData("LeftFrontMotor", "LeftFrontMotor is null");
            app.telemetry.update();
            return;
        }

        try {
            touchSensor = hwMap.get(DigitalChannel.class, TOUCH_SENSOR);
            touchSensor.setMode(DigitalChannel.Mode.INPUT);
        } catch (Exception ex) {
            opmode.telemetry.addData("ERROR", "Missing " + TOUCH_SENSOR);
        }

        // Define and initialize ALL installed servos.
        try {
            for (int i = 0; i < SERVOS.length; i++) {
                servo[i] = hwMap.get(Servo.class, SERVOS[i]);
            }
            puller1 = servo[3];
            puller2 = servo[4];
        } catch (Exception ex) {
            opmode.telemetry.addData("ERROR", "Missing servo");
        }

        try {
            // The color sensor and the distance sensor are in the same device.
            colorSensor = hwMap.get(ColorSensor.class, "color_sensor");
            distanceSensor = hwMap.get(DistanceSensor.class, "color_sensor");
        } catch (Exception ex) {
            opmode.telemetry.addData("ERROR", ex.getMessage());
        }
        imu = new REVIMU(this);
        opmode.telemetry.update();
    }

    /**
     * Return true if motor is done turning. When that is true, reset motor mode.
     */
    public boolean done_turning () {
        if (is_motor_busy()) {
            return false;
        }
        drive_using_encoder();
        return true;
    }

    /**
     * Get the current motor encoder positions.
     *
     * @param mp    An array of 4 integers to host the positions.
     */
    // Motor encoder position
    int[] mposition = {0, 0, 0, 0};
    public int[] get_current_position () {
        mposition[0] = leftFrontMotor.getCurrentPosition();
        mposition[1] = rightFrontMotor.getCurrentPosition();
        mposition[2] = leftRearMotor.getCurrentPosition();
        mposition[3] = rightFrontMotor.getCurrentPosition();
        return mposition;
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
            app.telemetry.addData(STATUS, "Missing motor: "+name);
            app.telemetry.addData("ERROR", ex.getMessage());
        }
        return null;
    }

    /** Initialize standard Hardware interfaces
     *
     * @param  opmode    OpMode initializing this robot.
     * @return  The Robot - a singleton.
     */
    public static RobotConfig init(OpMode opmode, DriveMode mode) {
        if (theRobot == null) {
            theRobot = new RobotConfig(opmode, mode);
        }
        return theRobot;
    }

    /**
     * Intialize the robot for manual drive.
     *
     * @param opmode    The OpMode initializing this robot.
     * @return  The Robot - a singleton.
     */
    public static RobotConfig init(OpMode opmode) {
        return init(opmode, DriveMode.MANUAL);
    }

    /**
     * Stop and reset drive motor encoders.
     */
    public void reset_motor_encoder () {
        set_drive_mode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Return the state of the touch sensor.
     */
    public boolean detect_touch () {
        return touchSensor.getState();
    }

    /**
     * Drive the motors by setting specified power levels.
     *
     * Power levels should be between 0 and maxMotor.
     *
     * @param lf    Power level for leftFrontMotor.
     * @param rf    Power level for rightFrontMotor.
     * @param lr    Power level for leftRearMotor.
     * @param rr    Power level for leftRearMotor.
     */
    public void drive(double lf, double rf, double lr, double rr) {
        // Send values to the motors
        try {
            app.telemetry.addData("LF", "%.3f", lf);
            leftFrontMotor.setPower(-lf);
            app.telemetry.addData("RF", "%.3f", rf);
            rightFrontMotor.setPower(rf);
            app.telemetry.addData("LR", "%.3f", lr);
            leftRearMotor.setPower(-lr);
            app.telemetry.addData("RR", "%.3f", rr);
            rightRearMotor.setPower(rr);
        } catch (Exception ex) {
            app.telemetry.addData("ERROR", ex.getMessage());
        }
    }

    /**
     * Drive the robot in the direction specified by robotAngle.
     *
     * @param robotAngle    Angle of the robot movement, in radians;
     * @param speed         Speed of the robot movement, between [0, maxMotor].
     */
    public void drive (double robotAngle, double speed) {
        double xforce = speed*Math.cos(robotAngle);
        double yforce = speed*Math.sin(robotAngle);
        final double LF = yforce + xforce;
        final double RF = yforce - xforce;
        final double LR = yforce - xforce;
        final double RR = yforce + xforce;
        // Send values to the motors
        drive(LF, RF, LR, RR);
        // Send some useful parameters to the driver station
        app.telemetry.addData("power", "x: (%.2f), y: (%.2f) angle: (%.2f)", xforce, yforce, robotAngle*180/Math.PI);
        app.telemetry.update();
    }
    public void drive_reverse() {
        drive(-Math.PI/2, this.motorMax);
    }
    public void drive_forward() {
        drive(Math.PI/2, this.motorMax);
    }

    /**
     * Set the motors up to drive across a distance.
     */
    public void drive_across (double distance) {
        int delta = (int) (distance / ENCODER_CPI);
        int[] mp = get_current_position();
        mp[0] += delta;
        mp[1] += delta;
        mp[2] -= delta;
        mp[3] -= delta;
        drive_to(mp);
    }
    /**
     * Set the motors up to drive over a distance.
     *
     * TBD: This only works for forward and reverse directions. Need to create a generic version
     * that would drive over a distance in any direction.
     *
     * @param inches  Distance to drive over, in inches.
     */
    public void drive_over (double inches) {
        int delta = (int) (inches / ENCODER_CPI);
        int[] mp = get_current_position();
        mp[0] -= delta; // LF
        mp[1] += delta; // RF
        mp[2] -= delta; // LR
        mp[3] += delta; // RR
        drive_to(mp);
    }

    /**
     * Drive to given motor encoder positions.
     *
     * @param morotPosition     Motor encoder positions.
     */
    public void drive_to (int morotPosition[]) {
        // Turn On RUN_TO_POSITION
        set_drive_mode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set targets on all motors
        leftFrontMotor.setTargetPosition(morotPosition[0]);
        rightFrontMotor.setTargetPosition(morotPosition[1]);
        leftRearMotor.setTargetPosition(morotPosition[2]);
        rightRearMotor.setTargetPosition(morotPosition[3]);
    }

    /**
     * Reset RUN_USING_ENCODER on all motors.
     */
    public void drive_using_encoder () {
        set_drive_mode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Check if any of the four motors is busy.
     *
     * @return  True if one or more of the four motors are busy.
     */
    public boolean is_motor_busy () {
        return leftFrontMotor.isBusy()
                || rightFrontMotor.isBusy()
                || leftRearMotor.isBusy()
                || rightRearMotor.isBusy();
    }
    /**
     * Manual drive using gamepad 1:
     */
    public void manual_drive () {
        if (rightRearMotor == null) {
            app.telemetry.addData("ERROR", "The robot is missing motor.");
            return;
        }

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
        app.telemetry.addData("power", "x: (%.2f), y: (%.2f) angle: (%.2f)", leftX, leftY, robotAngle*180/Math.PI);
        app.telemetry.update();
    }

    /**
     * Pause the robot.
     */
    public void pause () {
        drive(0, 0, 0, 0);
    }
    /**
     * Set the pull server position to lower the puller to the foundation.
     */
    public void puller_down() {
        puller1.setPosition(PULLER_DOWN);
        puller2.setPosition(PULLER_DOWN);
    }
    public void puller_up() {
        puller1.setPosition(PULLER_UP);
        puller2.setPosition(PULLER_UP);
    }

    /**
     * Set driver motors to the given drive mode.
     *
     * @param xmode A DcMotor.RunMode run mode constant.
     */
    public void set_drive_mode (DcMotor.RunMode xmode) {
        leftFrontMotor.setMode(xmode);
        rightFrontMotor.setMode(xmode);
        leftRearMotor.setMode(xmode);
        rightRearMotor.setMode(xmode);
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed, double angle, double distanceInInches, double timeoutS) {

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
        newRightRearTarget = rightFrontMotor.getCurrentPosition() + (int)((distanceInInches * COUNTS_PER_INCH)* Math.cos(robotAngle));

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

        // Turn off RUN_TO_POSITION
        robotReset();
            //  sleep(250);   // optional pause after each move
    }

    public void encoderDriveWithTouchSensor(double speed, double angle, double distanceInInches, double timeoutS) {

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;

        // Ensure that the opmode is still active
            // Determine new target position, and pass to motor controller
            // Turn On RUN_TO_POSITION
        double robotAngle =  Math.toRadians(angle) - Math.PI / 4;

        newLeftFrontTarget = leftFrontMotor.getCurrentPosition() - (int)((distanceInInches * COUNTS_PER_INCH) * Math.cos(robotAngle));
        newRightFrontTarget = rightFrontMotor.getCurrentPosition() + (int)((distanceInInches * COUNTS_PER_INCH)* Math.sin(robotAngle));
        newLeftRearTarget = leftFrontMotor.getCurrentPosition() - (int)((distanceInInches * COUNTS_PER_INCH)* Math.sin(robotAngle));
        newRightRearTarget = rightFrontMotor.getCurrentPosition() + (int)((distanceInInches * COUNTS_PER_INCH)* Math.cos(robotAngle));

        leftFrontMotor.setTargetPosition(newLeftFrontTarget);
        rightFrontMotor.setTargetPosition(newRightFrontTarget);
        leftRearMotor.setTargetPosition(newLeftRearTarget);
        rightRearMotor.setTargetPosition(newRightRearTarget);

        set_drive_mode(DcMotor.RunMode.RUN_TO_POSITION);

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
            if (detect_touch()) {
                // Stop all motion;
                robotReset();
                return;
            }

        }

        robotReset();
            //  sleep(250);   // optional pause after each move
    }


    public void encoderTurn(double speed, double angle) {
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

        set_drive_mode(DcMotor.RunMode.RUN_TO_POSITION);

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

        // Turn off RUN_TO_POSITION
        robotReset();

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
    }

    /**
     * Stop the robot.
     */
    public void stop () {
        theRobot = null;
        drive(0,0,0,0);
        set_drive_mode(DcMotor.RunMode.RUN_USING_ENCODER);
        app.telemetry.addData(STATUS, "Robot parked");
        app.telemetry.update();
    }

    /**
     * Turn the robot a given angle: Positive to turn counter-clock wise.
     *
     * @param angle     Angle to turn, in fractions of Math.PI, e.g. 1/4.
     */
    public void turn (double angle) {
        int[] mp = get_current_position();
        int delta = (int) (angle * ENCODER_CPR);   // Fraction of a full turn
        mp[0] += delta;
        mp[1] += delta;
        mp[2] += delta;
        mp[3] += delta;
        drive_to(mp);
    }
    /**
     * Turning the robot left.
     *
     * @param power     Power level (0..maxMotor).
     */
    public void turn_left (double power) {
        drive(-power, power, -power, power);
    }
    public void turn_right (double power) {
        turn_left(-power);
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    private void robotReset () {
        // Stop all motion;
        pause();
        reset_motor_encoder();
        drive_using_encoder();
    }
}