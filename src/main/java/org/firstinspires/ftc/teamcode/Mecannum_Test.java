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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Mecannum_Test", group="Linear Opmode")
//@Disabled
public class Mecannum_Test extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor leftRearMotor = null;
    private DcMotor rightRearMotor = null;

    static final double INCREMENT   = 0.001;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position
    static final double CLAW_MAX_POS = 0.6;
    static final double TRAVEL_POS1_Load = 0.3;
    static final double TRAVEL_POS1_NoLoad = 0.15;
    static final double TRAVEL_POS_2 = 0.65;


    // Define class members
    Servo claw, arm, elbow, puller1, puller2;
    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean rampUp = true;
    double clawPos = 0;
    double armPos = 1.0;
    double elbowPos = 1.0;
    double pullerPos = 0.0;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontMotor  = hardwareMap.get(DcMotor.class, "left_front_motor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front_motor");
        leftRearMotor  = hardwareMap.get(DcMotor.class, "left_rear_motor");
        rightRearMotor = hardwareMap.get(DcMotor.class, "right_rear_motor");
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm = hardwareMap.get(Servo.class, "servo0");
        elbow = hardwareMap.get(Servo.class, "servo1");
        claw = hardwareMap.get(Servo.class, "servo2");
        puller1 = hardwareMap.get(Servo.class, "servo3");
        puller2 = hardwareMap.get(Servo.class, "servo4");
        puller1.setPosition(0.0);
        puller2.setPosition(0.0);



        float leftX, leftY;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Most robots need the motor on one side to be reversed to drive forward
            // Reverse the motor that runs backwards when connected directly to the battery
            leftX = gamepad1.left_stick_x;
            leftY = -gamepad1.left_stick_y;
            double r = Math.hypot(leftX, leftY);
            double robotAngle = Math.atan2(leftY, leftX) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            leftFrontMotor.setPower(-v1);
            rightFrontMotor.setPower(v2);
            leftRearMotor.setPower(-v3);
            rightRearMotor.setPower(v4);


            if(gamepad2.a)
            {
                armPos = TRAVEL_POS1_Load;
                elbowPos = TRAVEL_POS_2;
                arm.setPosition(armPos);
                elbow.setPosition(elbowPos);
            }

            if(gamepad2.y)
            {
                armPos = TRAVEL_POS1_NoLoad;
                elbowPos = TRAVEL_POS_2;
                arm.setPosition(armPos);
                elbow.setPosition(elbowPos);
            }


            if(gamepad2.dpad_right) {  //01{
                armPos -= INCREMENT;
                if (armPos <= MIN_POS)
                    armPos = MIN_POS;
                arm.setPosition(armPos);
            }

            if(gamepad2.dpad_left) {
                armPos += INCREMENT;
                if (armPos >= MAX_POS)
                    armPos = MAX_POS;
                arm.setPosition(armPos);
            }

            if(gamepad2.dpad_down) {  //01{
                elbowPos -= INCREMENT;
                if (elbowPos <= MIN_POS)
                    elbowPos = MIN_POS;
                elbow.setPosition(elbowPos);
            }

            if(gamepad2.dpad_up) {
                elbowPos += INCREMENT;
                if (elbowPos >= MAX_POS)
                    elbowPos = MAX_POS;
                elbow.setPosition(elbowPos);
            }

            if(gamepad2.left_bumper) {  //01{
                clawPos -= INCREMENT;
                if (clawPos <= MIN_POS)
                    clawPos = MIN_POS;
                claw.setPosition(clawPos);
            }

            if(gamepad2.right_bumper) {
                clawPos += INCREMENT;
                if (clawPos >= CLAW_MAX_POS)
                    clawPos = CLAW_MAX_POS;
                claw.setPosition(clawPos);
            }

            if(gamepad2.right_stick_x >0)
            {
                pullerPos += INCREMENT;
                if(pullerPos > 0.65)
                    pullerPos = 0.65;
                puller1.setPosition(pullerPos);
                puller2.setPosition(pullerPos);
            }

            if(gamepad2.right_stick_x < 0)
            {
                pullerPos -= INCREMENT;
                if(pullerPos <= MIN_POS)
                    pullerPos = MIN_POS;
                puller1.setPosition(pullerPos);
                puller2.setPosition(pullerPos);
            }


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("gamepad", "x: (%.2f), y: (%.2f) angle: (%.2f)", leftX, leftY, robotAngle*180/Math.PI);
            telemetry.addData("ArmPos:", "%5.2f", armPos);
            telemetry.addData("ElbowPos:", "%5.2f", elbowPos);
            telemetry.addData("ClawPos:", "%5.2f", clawPos);
            telemetry.update();
        }
    }
}
