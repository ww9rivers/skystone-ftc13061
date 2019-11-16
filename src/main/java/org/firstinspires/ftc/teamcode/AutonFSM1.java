package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

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
public class AutonFSM1 extends AutonMode {

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
    public AutonFSM1(boolean isRed) {
        super(isRed);
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
                moving_foundation_state = move_foundation();
                break;
            case TRANSPORT_STONE:
                transport_state = transport_stone();
                break;
            case PARKING:
                parking_state = park();
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
    private MovingFoundation move_foundation () {
        telemetry.addData(robot.STATUS, "Moving foundation");
        switch (moving_foundation_state) {
            case GOTO_FOUNDATION:
                travel = robot.runtime.milliseconds();
                robot.drive_reverse();
                return MovingFoundation.DETECT_FOUNDATION;
            case DETECT_FOUNDATION:
                // Detect the foundation when the touch sensor is triggered:
                if (robot.detect_touch()) {
                    timer = robot.runtime.milliseconds();
                    travel = timer - travel;
                    timer += 300; // waiting time for the puller to lower
                    robot.puller_down();
                    return MovingFoundation.LOWER_PULLER;
                }
                break;
            case LOWER_PULLER:
                if (robot.runtime.milliseconds() < timer) {
                    break;
                }
                robot.drive_forward();
                travel += robot.runtime.milliseconds() + 5;
                return MovingFoundation.PULL_FOUNDATION;
            case PULL_FOUNDATION:
                if (robot.runtime.milliseconds() < travel) {
                    break;
                }
                robot.puller_up();
                robot_state = State.TRANSPORT_STONE;
                return MovingFoundation.GOTO_FOUNDATION;
        }
        return moving_foundation_state;
    }

    private void linear_move_foundation () {

        robot.encoderDrive(RobotConfig.DRIVE_SPEED,  180, 24,  3);  // S1: Forward 47 Inches with 5 Sec timeout
        //       encoderDriveWithTouchSensor(DRIVE_SPEED,  -90, 16,  15);  // S1: Forward 47 Inches with 5 Sec timeout
        robot.encoderDriveWithTouchSensor(RobotConfig.DRIVE_SPEED,  -90, 35,  15);  // S1: Forward 47 Inches with 5 Sec timeout
        robot.puller1.setPosition(0.5);
        robot.puller2.setPosition(0.5);
        robot.sleep(1000);  // optional pause after each move
        robot.encoderDrive(RobotConfig.DRIVE_SPEED,  90, 30,  15);  // S1: Forward 47 Inches with 5 Sec timeout
        robot.puller1.setPosition(0.);
        robot.puller2.setPosition(0.);
        robot.sleep(1000);   // optional pause after each move
        robot.encoderDrive(RobotConfig.DRIVE_SPEED,  0, 50,   15);
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
    enum ParkingState {
        GOTO_PARK,
        PARKED
    };
    ParkingState parking_state = ParkingState.GOTO_PARK;
    private ParkingState park () {
        telemetry.addData(robot.STATUS, "Parking robot");
        // How to orient self?
        switch (parking_state) {
            case GOTO_PARK:
                // TBD: Where are we?
                break;
        }
        return parking_state;
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
    enum TransportStoneState {
        TURN_SOUTH,
        TURN_SOURTH_WAIT,
        TRAVEL_SOUTH,
        TRAVELING_SOUTH,
        TURN_TO_STONE,
        PICKUP_STONE,
        TURN_NORTH,
        TURNING_NORTH,
        DELIVER_STONE,
        DELIVERING,
        GO_PARKING
    };
    TransportStoneState transport_state = TransportStoneState.TURN_SOUTH;
    List<Recognition> stones = null;
    private TransportStoneState transport_stone () {
        // Give the robot 5 seconds to park.
        if (robot.runtime.seconds() > 25) {
            robot_state = State.PARKING;
            return TransportStoneState.TURN_SOUTH;
        }
        telemetry.addData(robot.STATUS, "Transporting Stone");
        telemetry.update();
        switch (transport_state) {
            case TURN_SOUTH:
                robot.turn(1/4);
                return TransportStoneState.TURN_SOURTH_WAIT;
            case TURN_SOURTH_WAIT:
                if (robot.done_turning()) {
                    return TransportStoneState.TRAVEL_SOUTH;
                }
                break;
            case TRAVEL_SOUTH:
                robot.drive_over(4*24+12);  // 4+1/2 tiles
                return TransportStoneState.TRAVELING_SOUTH;
            case TRAVELING_SOUTH:
                stones = detector.detect();
                if (stones != null) {
                    // TBD:
                    return TransportStoneState.TURN_TO_STONE;
                }
                break;
            case TURN_TO_STONE:
                // TBD: some condition must be met
                return TransportStoneState.PICKUP_STONE;
            case PICKUP_STONE:
                // TBD: if a stone is picked up
                return TransportStoneState.TURN_NORTH;
            case TURN_NORTH:
                robot.turn(1);
            case TURNING_NORTH:
                if (robot.done_turning()) {
                    return TransportStoneState.DELIVER_STONE;
                }
                break;
            case DELIVER_STONE:
                // Driver under alliance bridge
                robot.drive_over(3*24);
                return TransportStoneState.DELIVERING;
            case DELIVERING:
                if (robot.done_turning()) {
                    robot.turn(1);
                    return TransportStoneState.TURNING_NORTH;
                }
                break;
        }
        return transport_state;
    }
}