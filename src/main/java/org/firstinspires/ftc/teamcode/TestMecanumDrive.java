package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(name="Mecanum Proto Manual", group="Test")
public class TestMecanumDrive extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotConfig robot = RobotConfig.init(this, MecanumDrive.DriveMode.MANUAL);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot.start();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Send values to the motors
            robot.manual_drive();
            robot.showtime();
        }
    }
}
