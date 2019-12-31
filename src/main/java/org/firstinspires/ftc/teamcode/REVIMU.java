package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

public class REVIMU {
    RobotConfig robot = null;
    Telemetry telemetry = null;
    BNO055IMU imu = null;
    Orientation angles;
    Acceleration gravity;

    public REVIMU (RobotConfig bot) {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        robot = bot;
        OpMode app = bot.app;
        telemetry = app.telemetry;
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
        imu = robot.app.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.set_drive_mode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");
        // telemetry.update();
        composeTelemetry();
        telemetry.addData(">", "Robot Ready.");
        // telemetry.update();
//        robot.drive_using_encoder();
        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
    }

    /**
     * Get the first angle from the IMU.
     *
     * @return
     */
    public double get_angle () {
        return imu.getAngularOrientation().firstAngle;
    }
    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {
        double robotError; // calculate error in -179 to +180 range
        robotError = targetAngle - get_angle();
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

    /**
     * Format a given angle into text for display.
     *
     * @param angleUnit
     * @param angle
     * @return
     */
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit , angle));
    }

    /**
     * Formatting the default angle for displaying.
     *
     * @return  The default angle in text
     */
    String formatAngle() {
        return formatAngle(angles.angleUnit, angles.firstAngle);
    }
    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(),"%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    public void startAccelerationIntegration (Position initalPosition, Velocity initialVelocity, int msPollInterval) {
        imu.startAccelerationIntegration(initalPosition, initialVelocity, msPollInterval);
    }
}