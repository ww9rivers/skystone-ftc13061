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
    RobotConfig robot = RobotConfig.init(this);
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

    /**
     * The old code of loop().
     */
    private void drive () {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        leftPower = Range.clip(drive + turn, -1.0, 1.0);
        rightPower = Range.clip(drive - turn, -1.0, 1.0);

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + robot.runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(this);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        robot_state = State.MOVE_FOUNDATION;
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        switch (robot_state) {
            case MOVE_FOUNDATION:
                moving_foundation_state = MovingFoundation.GOTO_FOUNDATION;
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
    private void move_foundation () {
        telemetry.addData("Status", "Moving foundation");
        switch (moving_foundation_state) {
            case GOTO_FOUNDATION:
                robot.drive_reverse();
                moving_foundation_state = MovingFoundation.DETECT_FOUNDATION;
                return;
            case DETECT_FOUNDATION:
                // Detect the foundation when the touch sensor is triggered:
                if (robot.detect_touch()) {
                    moving_foundation_state = MovingFoundation.LOWER_PULLER;
                    robot.lower_puler();
                }
                return;
            case LOWER_PULLER:
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
        telemetry.addData("Status", "Parking robot");
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

    private void transport_stone () {
        telemetry.addData("Status", "Transporting Stone");
    }
}