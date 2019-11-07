package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Mecanum auto forward", group="Test")
public class TestMecanumForward extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotConfig robot = RobotConfig.init(this);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot.start();

        // run until the end of the match (driver presses STOP)
        telemetry.addData("Status", "Started!");
        telemetry.update();
        while (opModeIsActive()) {
            // Send values to the motors
            robot.drive(Math.PI/2, robot.motorMax/2);
            robot.showtime();
        }
        robot.stop();
    }
}
