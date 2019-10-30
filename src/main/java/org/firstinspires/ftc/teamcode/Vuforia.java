package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class Vuforia {
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    // 2018-04-22 — robotics@annhua.org
    private static final String VUFORIA_KEY =
            "AbPc14P/////AAABmZuZNgh3mUvnsRr1h9FPQO4Y+RlkIDvh83ytAED4SQKx/DkIDT9Qwzje2wZTuoLFym2U1VylmNq0IWYAzGGlHfvydR8MPg/KiYBMnzbgApDJlvoScnXtGH8S0NVrBN3erxZFkmOBhmBsXWOoGu7I2wziurj+XRN0j3pp6p1KwJ++B3xpMWupPGD/j2cCk6wrKXt+l201axgWmBDJ7BfeI8CdptUcbltnBE2ouPl68MH/cU7Xw2n7COfReWNGtNvjpiE6vnZWnMJilnIffDSxnCdcZUk4rBaN4bdGE6yJpkISVzKG3qd8SEJr/DQZ/3S19uLa/D3yAd/aTLOwAq2mjZMk+hoS8ssUNtL1Yab7N1Ge";
    // IMPORTANT:  For Phone Camera, set 1) the camera source and 2) the orientation, based on how your phone is mounted:
    // 1) Camera Source.  Valid choices are:  BACK (behind screen) or FRONT (selfie side)
    // 2) Phone Orientation. Choices are: PHONE_IS_PORTRAIT = true (portrait) or PHONE_IS_PORTRAIT = false (landscape)
    //
    // NOTE: If you are running on a CONTROL HUB, with only one USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    //
    public static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    public static final boolean PHONE_IS_PORTRAIT = false  ;

    // Webcam name in text:
    private static final String WEBCAM = "Webcam 1";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private static VuforiaLocalizer vuforia     = null;
    private static VuforiaLocalizer navigator   = null;
    private static VuforiaLocalizer webcam      = null;

    /* local OpMode members. */
    private static HardwareMap hw                      = null;

    public Vuforia(HardwareMap hwmap) { hw = hwmap; }

    /**
     * Initialize the Vuforia localization engine for object detection.
     *
     * @return  An instance of Vuforia localizer.
     */
    public static VuforiaLocalizer getInstance () {
        if (vuforia != null) { return vuforia; }
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        return vuforia;
        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Get Vuforia localization engine for navigation.
     *
     * @return  An instance of Vuforia localizer.
     */
    public static VuforiaLocalizer getNavigator () {
        if (navigator != null) { return navigator; }

        int cameraMonitorViewId = hw.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hw.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        navigator = ClassFactory.getInstance().createVuforia(parameters);
        return navigator;
    }

    /**
     * Get a Vuforia localization engine for navigation using the webcam.
     *
     * @return  An instance of Vuforia localizer.
     */
    public static VuforiaLocalizer getWebcam () {
        if (webcam != null) { return webcam; }

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hw.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hw.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * We also indicate which camera on the RC we wish to use -- Retrieve the camera to use from hardware map.
         */
        parameters.cameraName = hw.get(WebcamName.class, WEBCAM);

        //  Instantiate the Vuforia engine
        webcam = ClassFactory.getInstance().createVuforia(parameters);
        return webcam;
    }
}
