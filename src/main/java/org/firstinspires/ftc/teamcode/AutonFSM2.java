package org.firstinspires.ftc.teamcode;

/** AutonMode2: #2 Position - This is a library, not an OpMode
 *
 * This Autonomous opmode assumes that the robot starts in the Red position 1, on the side of the
 * Foundation, facing towards the Foundation. The tasks are:
 *
 * 1.   Transport Skystone
 * 2.   Parking
 * 2.1. Determine current position and orientation;
 * 2.2. Driver towards Skybridge - detect color while driving;
 * 2.3. Stop when color is detected.
 *
 * Reference: https://drive.google.com/open?id=1HdyA5MHN3-CSbFCGKsrEqOEmcXNH-F_7
 * Reference: org.firstinspires.ftc.robotcontroller.external.samples.PushbotAutoDrive*
 */
public class AutonFSM2 extends AutonMode {
    enum State {
        DETECT_STONE,
        TRANSPORT_STONE,
        PARKING,
        STOP,
        IDLE
    };
    private State  robot_state;

    /**
     * Constructor.
     *
     * @param isRed     True, if aliance color is RED.
     */
    public AutonFSM2(boolean isRed) {
        super(isRed);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        robot_state = State.DETECT_STONE;
        telemetry.addData(robot.STATUS, "Auton Mode 1: " + (aliance == Aliance.Color.RED ? "Red" : "Blue"));
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        switch (robot_state) {
            case DETECT_STONE:
                // TBD: Refactor - stone detection should be done in parallel to other actions.
                detection_state = detect_stone();
                break;
            case TRANSPORT_STONE:
                transport_state = transport_stone();
                break;
            case PARKING:
                parking_state = park();
                return;
            case STOP:
                stop();
                robot_state = State.IDLE;
                return;
            case IDLE:
                idle();
                break;
        }
    }

    /**
     * Robot is in this state right after start. In Position 1, blue or red.
     */
    enum DetectionState {
        DETECTION_START,
        DETECTION_WAIT,
        DETECTION_FAILED
    };
    private DetectionState detection_state = DetectionState.DETECTION_START;
    private DetectionState detect_stone () {
        switch (detection_state) {
            case DETECTION_START:
                telemetry.addData(robot.STATUS, "Detect stone");
                timer.reset();
                if (detector == null) {
                    // Object detector failed: No detection, drive over and randomly pick up.
                    robot.drive_forward();
                    return DetectionState.DETECTION_FAILED;
                }
                robot.drive_over(24.0);
                return DetectionState.DETECTION_WAIT;
            case DETECTION_FAILED:
                if (timer.milliseconds() < 1200) { break; }
                robot.pause();
                // Fall through to DETECTION_WAIT to continue.
            case DETECTION_WAIT:
                if (robot.is_motor_busy()) { break; }
                transport_state = TransportState.TRANSPORT_START;
                robot_state = State.TRANSPORT_STONE;
                return DetectionState.DETECTION_START;
        }
        return detection_state;
    }

    /**
     *  Park the robot under Skybridge
     *
     * 1.  Try to look for the navigation marks to locate self;
     * 2.  Drive towards the skybridge.
     * 3.  While driving, try to detect the color using sensor mounted under the robot.
     * 4.  Stop robot if color is detected.
     */
    enum ParkingState {
        GO_PARKING
    }
    private ParkingState parking_state = ParkingState.GO_PARKING;
    private ParkingState park () {
        telemetry.addData(robot.STATUS, "Parking robot");
        switch (parking_state) {
            case GO_PARKING:
                break;
        }
        return parking_state;
    }

    enum TransportState {
        TRANSPORT_START,
        TRANSPORT_FIND,
        TRANSPORT_PICKUP,
        TRANSPORT_DELIVER
    };
    TransportState transport_state = TransportState.TRANSPORT_START;
    private TransportState transport_stone () {
        switch (transport_state) {
            case TRANSPORT_START:
                telemetry.addData(robot.STATUS, "Transporting Stone");
                timer.reset();
                robot.drive_forward();
                return TransportState.TRANSPORT_FIND;
            case TRANSPORT_FIND:
                stone = detect();
                if (stone != null) {
                    // Adjust robot position to get the object
                    adjust_robot(stone);
                } else if (timer.milliseconds() < 1200) {
                    // The timer needs to be calibrated to make sure the robot reaches the stones.
                    // Nothing detected, but we'll have to try pick up something.
                    break;
                }
                return TransportState.TRANSPORT_PICKUP;
        }
        return transport_state;
    }
}