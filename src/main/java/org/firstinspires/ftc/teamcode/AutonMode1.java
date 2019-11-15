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
    RobotConfig robot = RobotConfig.init(this, MecanumDrive.DriveMode.AUTO);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

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
        robot.init(this, MecanumDrive.DriveMode.AUTO);

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
    private void move_foundation () {

        robot.encoderDrive(DRIVE_SPEED, 180, 12, 3);
        robot.encoderDrive(DRIVE_SPEED, 270, 18, 3);
        telemetry.addData("Status", "Moving foundation");
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