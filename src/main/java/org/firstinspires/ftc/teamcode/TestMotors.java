package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Test motors: LF/RF/LR/RR", group="Test")
public class TestMotors extends LinearOpMode {

    enum State {
        TEST_INDIVIDUAL_MOTORS,
        TEST_ENCODER_DRIVE,
        WAIT_ENCODER_DRIVE
    };
    State state = State.TEST_INDIVIDUAL_MOTORS;

    int motorPos[];
    private void show_encoder () {
        telemetry.addData("status", "Encoders: " + motorPos[0] + ", " + motorPos[1] + ", " + motorPos[2] + ", " + motorPos[3]);
    }

    @Override
    public void runOpMode() {
        RobotConfig robot = RobotConfig.init(this);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot.start();

        // run until the end of the match (driver presses STOP)
        motorPos = new int[4];
        ElapsedTime t0 = new ElapsedTime();
        boolean power_set = false;
        int mx = 0;                     // Motor #
        double motorPower = robot.motorMax/4;   // Motor direction
        while (opModeIsActive()) {
            if (power_set && t0.seconds() < 3) { continue; }

            switch (state) {
                case TEST_ENCODER_DRIVE:
                    robot.drive_to(motorPos);
                    state = State.WAIT_ENCODER_DRIVE;
                    continue;
                case WAIT_ENCODER_DRIVE:
                    if (robot.is_motor_busy()) {
                        continue; // continue to wait.
                    }
                    robot.drive_using_encoder();
                    state = State.TEST_INDIVIDUAL_MOTORS;
                // Fall through to test individual motors.
            }
            robot.get_current_position(motorPos);
            show_encoder();
            t0.reset();
            robot.drive(
                    (mx == 0) ? motorPower : 0,
                    (mx == 1) ? motorPower : 0,
                    (mx == 2) ? motorPower : 0,
                    (mx == 3) ? motorPower : 0
            );
            power_set = true;
            mx++;
            if (mx > 3) {
                mx = 0;
                motorPower = -motorPower;
                state = State.TEST_ENCODER_DRIVE;
            }
            robot.showtime();
        }
        robot.stop();
    }
}
