package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Aliance;

/** AutonMode1: #1 Position - This is a library, not an OpMode
 *
 * This Autonomous opmode assumes that the robot starts in the Red position 1, on the side of the
 * Foundation, facing towards the Foundation. The tasks are:
 *
 * 1.   Move Foundation to Building Site;
 * 1.1. Drive to Foundation;
 * 1.2. Lower a hand to the Foundation;
 * 1.3. Drive to Building Site;
 * 1.4.
 * 2.   Transport Skystone
 * 3.   Parking
 * 3.1. Determine current position and orientation;
 * 3.2. Driver towards Skybridge - detect color while driving;
 * 3.3. Stop when color is detected.
 *
 * Reference: https://drive.google.com/open?id=1HdyA5MHN3-CSbFCGKsrEqOEmcXNH-F_7
 * Reference: org.firstinspires.ftc.robotcontroller.external.samples.PushbotAutoDrive*
 */
public class AutonMode1 extends OpMode {
    // Declare OpMode members.
    private Aliance.Color aliance = Aliance.Color.RED;
    RobotConfig robot = null;
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    enum State {
        MOVE_FOUNDATION,
        TRANSPORT_STONE,
        PARKING,
        STOP
    };
    private State  robot_state;

    /**
     * Constructor.
     *
     * @param isRed     True, if aliance color is RED.
     */
    public AutonMode1(Boolean isRed) {
        aliance = isRed ? Aliance.Color.RED : Aliance.Color.BLUE;
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot = RobotConfig.init(this);

        // Tell the driver that initialization is complete.
        telemetry.addData(robot.STATUS, "Initialized");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY, to
     * set the initial state of the robot state machine.
     */
    @Override
    public void init_loop() {
        robot_state = State.MOVE_FOUNDATION;
        telemetry.addData(robot.STATUS, "Auton Mode 1: " + (aliance == Aliance.Color.RED ? "Red" : "Blue"));
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        switch (robot_state) {
            case MOVE_FOUNDATION:
                move_foundation();
                break;
            case TRANSPORT_STONE:
                transport_stone();
                break;
            case PARKING:
                park();
                return;
            case STOP:
                stop();
                return;
        }
    }

    /**
     * Robot is in this state right after start. In Position 1, blue or red.
     */
    enum MovingFoundation {
        GOTO_FOUNDATION,
        DETECT_FOUNDATION,
        LOWER_PULLER,
        PULL_FOUNDATION
    }
    MovingFoundation moving_foundation_state = MovingFoundation.GOTO_FOUNDATION;
    double timer, travel;
    private void move_foundation () {
        telemetry.addData(robot.STATUS, "Moving foundation");
        switch (moving_foundation_state) {
            case GOTO_FOUNDATION:
                travel = robot.runtime.milliseconds();
                robot.drive_reverse();
                moving_foundation_state = MovingFoundation.DETECT_FOUNDATION;
                return;
            case DETECT_FOUNDATION:
                // Detect the foundation when the touch sensor is triggered:
                if (robot.detect_touch()) {
                    timer = robot.runtime.milliseconds();
                    travel = timer - travel;
                    timer += 300; // waiting time for the puller to lower
                    robot.puller_down();
                    moving_foundation_state = MovingFoundation.LOWER_PULLER;
                }
                return;
            case LOWER_PULLER:
                if (robot.runtime.milliseconds() < timer) {
                    return;
                }
                robot.drive_forward();
                moving_foundation_state = MovingFoundation.PULL_FOUNDATION;
                travel += robot.runtime.milliseconds() + 5;
                return;
            case PULL_FOUNDATION:
                if (robot.runtime.milliseconds() < travel) {
                    return;
                }
                robot.puller_up();
                robot_state = State.TRANSPORT_STONE;
                moving_foundation_state = MovingFoundation.GOTO_FOUNDATION;
                return;
        }
    }

    /**
     *  Park the robot under Skybridge
     *
     * 1.  Try to look for the navigation marks to locate self;
     * 2.  Drive towards the skybridge.
     * 3.  While driving, try to detect the color using sensor mounted under the robot.
     * 4.  Stop robot if color is detected.
     */
    private void park () {
        telemetry.addData(robot.STATUS, "Parking robot");
        // How to orient self?
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        robot.start();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.stop();
    }

    /**
     * Process to transport stones to the foundation.
     */
    private void transport_stone () {
        telemetry.addData(robot.STATUS, "Transporting Stone");
        telemetry.update();

    }
}