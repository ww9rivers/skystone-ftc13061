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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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
public class RobotConfig implements MecanumDrive
{
    /* Public OpMode members. */
    public DcMotor          leftFrontMotor  = null;
    public DcMotor          leftRearMotor   = null;
    public DcMotor          rightFrontMotor = null;
    public DcMotor          rightRearMotor  = null;

    public Servo            pullerServo     = null;
    public Servo            clawServo       = null;
    public ColorSensor      colorSensor     = null;
    public DistanceSensor   sensorDistance  = null;
    public DigitalChannel   touchSensor     = null;
    public ElapsedTime      runtime         = new ElapsedTime();

    private static final double PULLER_DOWN =  1.0;     // Maximum rotational position
    private static final double PULLER_UP   =  0.5;     // Maximum rotational position
    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hwMap       = null;
    OpMode app              = null;

    /* Constructor - This is a singleton class. */
    private static RobotConfig theRobot = null;
    private RobotConfig(OpMode opmode) {
        app = opmode;
        opmode.telemetry.addData("status", "Initialized");
        hwMap = opmode.hardwareMap;

        // Define and Initialize Motors
        // These polarities are for the Neverest 20 motors
        leftFrontMotor = get_motor("left_front_motor", DcMotor.Direction.FORWARD);
        rightFrontMotor = get_motor("right_front_motor", DcMotor.Direction.FORWARD);
        leftRearMotor = get_motor("left_rear_motor", DcMotor.Direction.FORWARD);
        rightRearMotor = get_motor("right_rear_motor", DcMotor.Direction.FORWARD);
        //leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        touchSensor = hwMap.get(DigitalChannel.class, "touch_sensor");
        if (touchSensor != null) {
            touchSensor.setMode(DigitalChannel.Mode.INPUT);
        }

        // Define and initialize ALL installed servos.
        pullerServo = hwMap.get(Servo.class, "puller_servo");
        pullerServo.setPosition(MID_SERVO);
        //rightClaw = hwMap.get(Servo.class, "right_hand");
        //rightClaw.setPosition(MID_SERVO);

        try {
            // The color sensor and the distance sensor are in the same device.
            colorSensor = hwMap.get(ColorSensor.class, "color_sensor");
            sensorDistance = hwMap.get(DistanceSensor.class, "color_sensor");
        } catch (Exception ex) {
            opmode.telemetry.addData("ERROR", ex.getMessage());
        }
        opmode.telemetry.update();
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
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            return motor;
        } catch (Exception ex) {
            app.telemetry.addData("status", "Missing motor: "+name);
            app.telemetry.addData("ERROR", ex.getMessage());
        }
        return null;
    }

    /** Initialize standard Hardware interfaces
     *
     * @param  opmode    OpMode initializing this robot.
     */
    public static RobotConfig init(OpMode opmode) {
        if (theRobot == null) {
            theRobot = new RobotConfig(opmode);
        }
        return theRobot;
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
     * Set the pull server position to lower the puller to the foundation.
     */
    public void lower_puler() {
        pullerServo.setPosition(0.5);
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
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
}