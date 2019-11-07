package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Test motors: LF/RF/LR/RR", group="Test")
public class TestMotors extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotConfig robot = RobotConfig.init(this);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot.start();

        // run until the end of the match (driver presses STOP)
        ElapsedTime t0 = new ElapsedTime();
        boolean power_set = false;
        int mx = 0;                     // Motor #
        double mp = robot.motorMax/4;   // Motor direction
        while (opModeIsActive()) {
            if (power_set && t0.seconds() < 3) { continue; }

            t0.reset();
            robot.drive(
                    (mx == 0) ? mp : 0,
                    (mx == 1) ? mp : 0,
                    (mx == 2) ? mp : 0,
                    (mx == 3) ? mp : 0
            );
            power_set = true;
            mx++;
            if (mx > 3) {
                mx = 0;
                mp = -mp;
            }
            robot.showtime();
        }
        robot.stop();
    }
}
