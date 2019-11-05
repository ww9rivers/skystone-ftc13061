package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Mecanum auto SE", group="Test")
public class TestMecanumSE extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotConfig robot = RobotConfig.init(this);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot.start();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Send values to the motors
            robot.drive(Math.PI*7/4, robot.motorMax);
            robot.showtime();
        }
    }
}
