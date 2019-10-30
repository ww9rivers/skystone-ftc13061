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
 * Motor channel: Left  drive motor:         "left_drive"
 * Motor channel: Right drive motor:         "right_drive"
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

    /* local OpMode members. */
    HardwareMap hwMap       = null;
    OpMode app              = null;

    /* Constructor - This is a singleton class. */
    private static RobotConfig theRobot = null;
    private RobotConfig(OpMode opmode) {
        app = opmode;
        opmode.telemetry.addData("Status", "Initialized");
        hwMap = opmode.hardwareMap;

        // Define and Initialize Motors
        // These polarities are for the Neverest 20 motors
        leftFrontMotor = get_motor("lf_drive", DcMotor.Direction.REVERSE);
        rightFrontMotor = get_motor("rf_drive", DcMotor.Direction.FORWARD);
        leftRearMotor = get_motor("lr_drive", DcMotor.Direction.REVERSE);
        rightRearMotor = get_motor("rr_drive", DcMotor.Direction.FORWARD);
        //leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        //leftClaw  = hwMap.get(Servo.class, "left_hand");
        //rightClaw = hwMap.get(Servo.class, "right_hand");
        //leftClaw.setPosition(MID_SERVO);
        //rightClaw.setPosition(MID_SERVO);

        try {
            // The color sensor and the distance sensor are in the same device.
            sensorColor = hwMap.get(ColorSensor.class, "color_sensor");
            sensorDistance = hwMap.get(DistanceSensor.class, "color_sensor");
        } catch (Exception ex) {
            opmode.telemetry.addData("Exception", ex.getMessage());
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
            app.telemetry.addData("Exception", ex.getMessage());
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
    }
    public void manual_drive () {
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

        // Send values to the motors
        drive(LF, RF, LR, RR);
        // Send some useful parameters to the driver station
        app.telemetry.addData("LF", "%.3f", LF);
        app.telemetry.addData("RF", "%.3f", RF);
        app.telemetry.addData("LR", "%.3f", LR);
        app.telemetry.addData("RR", "%.3f", RR);
    }

    /**
     * Show run time.
     */
    public void showtime () {
        app.telemetry.addData("Status", "Run Time: " + runtime.toString());
        app.telemetry.update();
    }

    public void start () {
        runtime.reset();
    }
}