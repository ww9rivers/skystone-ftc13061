package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

public abstract class AutonMode extends OpMode {
    // Declare OpMode members.
    protected Aliance.Color     aliance = Aliance.Color.RED;
    protected RobotConfig       robot = null;
    protected ObjectDetector    detector = null;
    protected Recognition       stone = null;
    protected ElapsedTime       timer = null;      // timer for being in a state

    /**
     * Constructor.
     *
     * @param isRed     True, if aliance color is RED.
     */
    public AutonMode (boolean isRed) {
        aliance = isRed ? Aliance.Color.RED : Aliance.Color.BLUE;
    }

    /**
     * Adjust robot position to given object.
     */
    public int[] adjust_robot (Recognition xobj) {
        int mp[] = robot.get_current_position();
        // TBD: Compute direction and distance of robot adjustment
        return mp;
    }
    /**
     * Attempt to detect objects using the detector. If multiple objects are detected, one with
     * more points would be selected.
     *
     * @return One object Recognition.
     */
    public Recognition detect () {
        List<Recognition> xobjs;
        xobjs = detector.detect();
        switch (xobjs.size()) {
            case 0: return null;
            case 1: break;
            default:
                // Multiple objects detected - Select one:
                // A Skystone is picked over a Stone, as it has more points;
                for (Recognition xobj : xobjs) {
                    if (detector.is_skystone(xobj)) {
                        return xobj;
                    }
                }
        }
        return xobjs.get(0);
    }

    /**
     * Puts the current thread to sleep for a bit as it has nothing better to do. This allows other
     * threads in the system to run.
     *
     * <p>One can use this method when you have nothing better to do in your code as you await state
     * managed by other threads to change. Calling idle() is entirely optional: it just helps make
     * the system a little more responsive and a little more efficient.</p>
     *
     * <p>{@link #idle()} is conceptually related to waitOneFullHardwareCycle(), but makes no
     * guarantees as to completing any particular number of hardware cycles, if any.</p>
     *
     * @see #com.qualcomm.robotcore.eventloop.opmode.LinearOpMode.idle()
     */
    public final void idle() {
        // Otherwise, yield back our thread scheduling quantum and give other threads at
        // our priority level a chance to run
        Thread.yield();
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init () {
        robot = RobotConfig.init(this, MecanumDrive.DriveMode.AUTO);
        try {
            timer = new ElapsedTime();
            detector = new ObjectDetector(this);
        } catch (Throwable ex) {
            ex.printStackTrace();
        }
        // Tell the driver that initialization is complete.
        telemetry.addData(robot.STATUS, "Initialized");
        telemetry.update();
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
        if (detector != null) {
            try {
                detector.finalize();
            } catch (Throwable throwable) {
                throwable.printStackTrace();
            }
            detector = null;
        }
    }
}