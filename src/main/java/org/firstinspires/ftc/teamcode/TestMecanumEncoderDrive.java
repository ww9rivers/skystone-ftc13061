package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="Mecanum: Encorder Drive", group="Test")
public class TestMecanumEncoderDrive extends LinearOpMode {

    RobotConfig robot = null;
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    public void runOpMode() {
        robot = RobotConfig.init(this, MecanumDrive.DriveMode.AUTO);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot.start();
        // run until the end of the match (driver presses STOP)
        robot.encoderDrive(DRIVE_SPEED, 90, 10, 10);
        robot.encoderDrive(DRIVE_SPEED, -90, 10, 10);
    }
}
