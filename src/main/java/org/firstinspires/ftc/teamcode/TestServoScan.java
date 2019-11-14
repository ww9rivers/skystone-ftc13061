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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This OpMode scans a single servo back and forwards until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo position each cycle
 * CYCLE_MS sets the update period.
 *
 * This code assumes a Servo configured with the name "left_hand" as is found on a pushbot.
 *
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "Test: Scan Servo", group = "Test")
//@Disabled
public class TestServoScan extends LinearOpMode {

    static final double INCREMENT   = 0.001;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    // Define class members
    Servo   claw, arm, elbow, puller;
    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean rampUp = true;
    double clawPos = position;
    double armPos = position;
    double elbowPos = 1.0;
    double pullerPos;

    @Override
    public void runOpMode() {

        // Connect to servo (Assume PushBot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        arm = hardwareMap.get(Servo.class, "servo0");
        elbow = hardwareMap.get(Servo.class, "servo1");
        claw = hardwareMap.get(Servo.class, "servo2");
        puller = hardwareMap.get(Servo.class, "servo5");


        claw.setPosition(clawPos);
       // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        waitForStart();

        // Scan servo till stop pressed.
        while(opModeIsActive()){
            if(gamepad2.dpad_left) {  //01{
                armPos -= INCREMENT;
                if (armPos <= MIN_POS)
                    armPos = MIN_POS;
                arm.setPosition(armPos);
            }

            if(gamepad2.dpad_right) {
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

            clawPos = 1.0 - gamepad2.left_stick_y;
            claw.setPosition(clawPos);

            pullerPos = 1.0 - gamepad2.left_stick_x;
            puller.setPosition(pullerPos);


            // Display the current value
            telemetry.addData("Arm Position", "%5.2f", armPos);
            telemetry.addData("ElbowPos Position", "%5.2f", elbowPos);
            telemetry.addData("ClawPos Position", "%5.2f", clawPos);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
