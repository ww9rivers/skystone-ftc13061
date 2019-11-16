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
@TeleOp(name = "Test: Servo0 Test", group = "Test")
//@Disabled
public class TestServo0 extends LinearOpMode {

    static final double INCREMENT   = 0.001;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    // Define class members
    Servo   servo0,servo1, servo2;
    double  position0 = 1.0; // Start at halfway position
    double  position1 = 1.0; // Start at halfway position
    double  position2 = 0.5; // Start at halfway position


    @Override
    public void runOpMode() {

        // Connect to servo (Assume PushBot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        servo0 = hardwareMap.get(Servo.class, "servo0");
        servo0.setPosition(position0);
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo1.setPosition(position1);

        servo2 = hardwareMap.get(Servo.class, "servo2");
        servo2.setPosition(position2);

       // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        waitForStart();

        // Scan servo till stop pressed.
        while(opModeIsActive()){
            if(gamepad2.dpad_left) {  //01{
                position0 -= INCREMENT;
                if (position0 <= MIN_POS)
                    position0 = MIN_POS;
                servo0.setPosition(position0);
            }

            if(gamepad2.dpad_right) {
                position0 += INCREMENT;
                if (position0 >= MAX_POS)
                    position0 = MAX_POS;
                servo0.setPosition(position0);
            }

            if(gamepad2.dpad_down) {  //01{
                position1 -= INCREMENT;
                if (position1 <= MIN_POS)
                    position1 = MIN_POS;
                servo1.setPosition(position1);
            }

            if(gamepad2.dpad_up) {
                position1 += INCREMENT;
                if (position1 >= MAX_POS)
                    position1 = MAX_POS;
                servo1.setPosition(position1);
            }


            if(gamepad2.left_bumper) {  //01{
                position2 -= INCREMENT;
                if (position2 <= MIN_POS)
                    position2 = MIN_POS;
                servo2.setPosition(position2);
            }

            if(gamepad2.right_bumper) {
                position2 += INCREMENT;
                if (position2 >= MAX_POS)
                    position2 = MAX_POS;
                servo2.setPosition(position2);
            }

            // Display the current value
            telemetry.addData("Position0:", "%5.2f", position0);
            telemetry.addData("Position1:", "%5.2f", position1);
            telemetry.addData("Position2:", "%5.2f", position2);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
