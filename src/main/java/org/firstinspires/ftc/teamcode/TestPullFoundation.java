package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Test: Pull foundation", group="Test")
public class TestPullFoundation extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotConfig robot = RobotConfig.init(this, MecanumDrive.DriveMode.MANUAL);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Press Start to test foundation pull");
        telemetry.update();
        waitForStart();
        robot.start();

        // Send values to the motors
        robot.puller_up();
        robot.showtime();
        robot.drive_over(16.1);
        robot.puller_down();
        robot.drive_over(-16.2);
        robot.puller_up();
        robot.showtime();
        robot.stop();
    }
}
