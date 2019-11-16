/*  All rights reserved. Redistribution and use in source and binary forms, with or without modification,
    are permitted (subject to the limitations in the disclaimer below) provided that the following
    conditions are met: Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer. Redistributions in binary form must
    reproduce the above copyright notice, this list of conditions and the following disclaimer
    in the documentation and/or other materials provided with the distribution. Neither the name
    of Robert Atkinson nor the names of his contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS
    OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
    AND FITNESSFOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
    CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
    SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
    ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
    OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import java.util.Locale;
/**
 * This file illustrates the concept of driving a path based on Gyro heading and encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 * otherwise you would use: PushbotAutoDriveByTime;
 *
 * This code ALSO requires that you have a Modern Robotics I2C gyro with the name "gyro"
 * otherwise you would use: PushbotAutoDriveByEncoder;
 *
 * This code requires that the drive Motors have been configured such that a positive
 * power command moves them forward, and causes the encoders to count UP.
 *
 * This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * In order to calibrate the Gyro correctly, the robot must remain stationary during calibration.
 * This is performed when the INIT button is pressed on the Driver Station.
 * This code assumes that the robot is stationary when the INIT button is pressed.
 * If this is not the case, then the INIT should be performed again.
 *
 * Note: in this example, all angles are referenced to the initial coordinate frame set during the
 * the Gyro Calibration process, or whenever the program issues a resetZAxisIntegrator() call on the Gyro.
 *
 * The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 * which means that a Positive rotation is Counter Clock Wise, looking down on the field.
 * This is consistent with the FTC field coordinate conventions set out in the document:
 * ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefi nition.pdf
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 *
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Test: Auto Drive By IMU", group="Test")
@Disabled
public class TestAutoIMU extends LinearOpMode {
    /* Declare OpMode members. */
    RobotConfig robot = null;
    // Use a Pushbot's hardware
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    static final double COUNTS_PER_MOTOR_REV = 1440 ; // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0 ; // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0 ; // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double DRIVE_SPEED = 0.3; // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.5; // Nominal half speed for better accuracy.
    static final double HEADING_THRESHOLD = 1 ; // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1; // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15; // Larger is more responsive, but also less stable
    @Override public void runOpMode() {
        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        robot = RobotConfig.init(this);
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");
        // telemetry.update();
        composeTelemetry();
        telemetry.addData(">", "Robot Ready.");
        // telemetry.update();
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", formatAngle(angles.angleUnit, angles.firstAngle));
            telemetry.update();
            idle();
        }
        imu.startAccelerationIntegration(new Position(), new Velocity(),1000);
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn
        gyroDrive(DRIVE_SPEED, 25.0, 0.0);
        // Drive FWD 48 inches
        gyroTurn( TURN_SPEED, -45.0);
        // Turn CCW to -45 Degrees
        gyroHold( TURN_SPEED, -45.0, 0.5);
        // Hold -45 Deg heading for a 1/2 second
        gyroTurn( TURN_SPEED, 45.0);
        // Turn CW to 45 Degrees
        gyroHold( TURN_SPEED, 45.0, 0.5);
        // Hold 45 Deg heading for a 1/2 second
        gyroTurn( TURN_SPEED, 0.0);
        // Turn CW to 0 Degrees
        gyroHold( TURN_SPEED, 0.0, 1.0);
        // Hold 0 Deg heading for a 1 second
        gyroDrive(DRIVE_SPEED,-25.0, 0.0);
        // Drive REV 48 inches
        gyroHold( TURN_SPEED, 0.0, 0.5);
        // Hold 0 Deg heading for a 1/2 second
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
    /**
     * Method to drive on a fixed compass bearing (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param speed Target speed for forward motion. Should allow for _/- variance for adjusting heading
     * @param distance Distance (in inches) to move from current position. Negative distance means move backwards.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive ( double speed, double distance, double angle) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed; // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newFrontLeftTarget = robot.leftFrontMotor.getCurrentPosition() + moveCounts;
            newFrontRightTarget = robot.rightFrontMotor.getCurrentPosition() + moveCounts;
            newBackLeftTarget = robot.leftRearMotor.getCurrentPosition() + moveCounts;
            newBackRightTarget = robot.rightRearMotor.getCurrentPosition() + moveCounts;
            // Set Target and Turn On RUN_TO_POSITION
            robot.leftFrontMotor.setTargetPosition(newFrontLeftTarget);
            robot.rightFrontMotor.setTargetPosition(newFrontRightTarget);
            robot.leftRearMotor.setTargetPosition(newBackLeftTarget);
            robot.rightRearMotor.setTargetPosition(newBackRightTarget);
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftFrontMotor.setPower(speed);
            robot.rightFrontMotor.setPower(speed);
            robot.leftRearMotor.setPower(speed);
            robot.rightRearMotor.setPower(speed);
            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive()
                    && (robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy() && robot.leftRearMotor.isBusy() && robot.rightRearMotor.isBusy())) {
                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);
                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0) steer *= -1.0;
                leftSpeed = speed - steer;
                rightSpeed = speed + steer;
                // Normalize speeds if any one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }
                robot.leftFrontMotor.setPower(leftSpeed);
                robot.rightFrontMotor.setPower(rightSpeed);
                robot.leftRearMotor.setPower(leftSpeed);
                robot.rightRearMotor.setPower(rightSpeed);
                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Actual", "%7d:%7d",
                        robot.leftFrontMotor.getCurrentPosition(),
                        robot.rightFrontMotor.getCurrentPosition(),
                        robot.leftRearMotor.getCurrentPosition(),
                        robot.rightRearMotor.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.update();
            }
            // Stop all motion;
            robot.leftFrontMotor.setPower(0);
            robot.rightFrontMotor.setPower(0);
            robot.leftRearMotor.setPower(0);
            robot.rightRearMotor.setPower(0);
            // Turn off RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn ( double speed, double angle) {
        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }
    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }
        // Stop all motion;
        robot.leftFrontMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);
        robot.leftRearMotor.setPower(0);
        robot.rightRearMotor.setPower(0);
    }
    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     * If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error ;
        double steer ;
        boolean onTarget = false ;
        double leftSpeed;
        double rightSpeed;
        // determine turn power based on +/- error
        error = getError(angle);
        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff); rightSpeed = speed * steer; leftSpeed = -rightSpeed;
        }
        // Send desired speeds to motors.
        robot.leftFrontMotor.setPower(leftSpeed);
        robot.rightFrontMotor.setPower(rightSpeed);
        robot.leftRearMotor.setPower(leftSpeed);
        robot.rightRearMotor.setPower(rightSpeed);
        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);
        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {
        double robotError; // calculate error in -179 to +180 range
        robotError = targetAngle - imu.getAngularOrientation().firstAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }
    /**
     * returns desired steering force. +/- 1 range. +ve = steer left
     * @param error Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
    void composeTelemetry() {
        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });
        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });
        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });
        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override
                    public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f", Math.sqrt(gravity.xAccel*gravity.xAccel + gravity.yAccel*gravity.yAccel + gravity.zAccel*gravity.zAccel));
                    }
                });
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit , angle));
    }
    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(),"%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}