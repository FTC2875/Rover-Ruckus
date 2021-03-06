package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.Frame;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import edu.spa.ftclib.internal.controller.ErrorTimeThresholdFinishingAlgorithm;
import edu.spa.ftclib.internal.controller.FinishableIntegratedController;
import edu.spa.ftclib.internal.controller.PIDController;
import edu.spa.ftclib.internal.drivetrain.HeadingableMecanumDrivetrain;
import edu.spa.ftclib.internal.sensor.IntegratingGyroscopeSensor;
import vision.VisionController;
import vision.YellowBlockResult;

/**
 * Created by Michaela on 1/3/2018.
 * Demonstrates the use of a HeadingableMecanumDrivetrain to autonomously rotate a robot to specific headings and hold those headings.
 * Tested and found fully functional by Gabriel on 2018-8-4.
 */

/**
 * Author: Darwin
 *
 * Usage: Implements HOMAR FTC library for mecanum drive control and precise turns during autonomous, uses Vuforia to identify yellow cube and move it,
 *          PID controller for aiming towards yellow block
 *
 * Motors: 4 drive motors
 * Sensors: internal REV IMU, External USB webcam
 */

@Autonomous(name = "Mecanum Auto Demo", group = "sample")
public class MecanumGyroAuto extends LinearOpMode {
    public static final String TAG = "AutoOpMode";

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    public HeadingableMecanumDrivetrain drivetrain;

    public FinishableIntegratedController controller;

    public BNO055IMUImpl imu;



    /**
     *     Vision setup stuff
     */
    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName;

    // stores the data about the yellow block location
    private YellowBlockResult result;

    // allows us to interact with vision controller
    private VisionController vision;


    // allows for pivoting if yellow block isn't found
    private boolean firstScan = true;


    /**
     * Override this method and place your code here.
     * <p>
     * Please do not swallow the InterruptedException, as it is used in cases
     * where the op mode needs to be terminated early.
     *
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {   //Notice that this is almost the exact same code as in HeadingableOmniwheelRotationAutonomous.
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");

        imu = hardwareMap.get(BNO055IMUImpl.class, "imu");
        BNO055IMU.Parameters gyroParams = new BNO055IMU.Parameters();
        gyroParams.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        gyroParams.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //Add calibration file?
        gyroParams.loggingEnabled = true;   //For debugging
        gyroParams.loggingTag = "IMU";      //For debugging
        gyroParams.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();  //Figure out why the naive one doesn't have a public constructor
        imu.initialize(gyroParams);
        while (!imu.isGyroCalibrated());


        PIDController pid = new PIDController(0.4, 0.1, 0);
        pid.setMaxErrorForIntegral(0.002);

        controller = new FinishableIntegratedController(new IntegratingGyroscopeSensor(imu), pid, new ErrorTimeThresholdFinishingAlgorithm(Math.PI/50, 1));
        drivetrain = new HeadingableMecanumDrivetrain(new DcMotor[]{frontLeft,frontRight, backLeft, backRight}, controller);

        for (DcMotor motor : drivetrain.motors) telemetry.addData("Motor Direction: ", motor.getDirection());
        telemetry.update();

        for (DcMotor motor : drivetrain.motors) motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//
//        /*
//         * Retrieve the camera we are to use.
//         */
//        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
//
//        /*
//         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
//         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
//         */
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//
//        // OR...  Do Not Activate the Camera Monitor View, to save power
//        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//
//        parameters.vuforiaLicenseKey = "AZF6Ye7/////AAABmSaSJ0vxakcfl8+TXLN7Uh035vWwHJ3VH9oAn6It3Ar7yaNj5jQ6oA578Usqt7z4P11rBqMaZdW2zBvgdgDmHiUxr26Ca4DUBtXyodB5GFVxnRqKcs05pKRG7e2fKcFUYG3Y2CrpLnJo7Ugtcqb6uBLbJVGHcsTav9r+ToFk5tlIhNrWwqPlVV+tB+hKYd5wWPGPChPdtF5ASO68PVjqDuq6fA5/YkYOsEk4orwMW0Tf4K5dMNHPXbw+ccDfhMB44XE/3f+4VC8rkBmZLEyHUtNVgUY3JsSFgrYufEwosrU/8heXUxt/LRuuj8QJUE9ngsP+/wUzM8GxEvdggyP9oeyquOvokWusgMbuAHJIdxws";
//
//        /**
//         * We also indicate which camera on the RC we wish to use.
//         */
//        parameters.cameraName = webcamName;
//
//        /**
//         * Instantiate the Vuforia engine
//         */
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
//
//        /**
//         * Because this opmode processes frames in order to write them to a file, we tell Vuforia
//         * that we want to ensure that certain frame formats are available in the {@link Frame}s we
//         * see.
//         */
//        vuforia.enableConvertFrameToBitmap();
//
//        /**
//         * Load the data sets that for the trackable objects we wish to track. These particular data
//         * sets are stored in the 'assets' part of our application (you'll see them in the Android
//         * Studio 'Project' view over there on the left of the screen). You can make your own datasets
//         * with the Vuforia Target Manager: https://developer.vuforia.com/target-manager. PDFs for the
//         * example "StonesAndChips", datasets can be found in in this project in the
//         * documentation directory.
//         */
//        VuforiaTrackables roverRuckusTargets = vuforia.loadTrackablesFromAsset("RoverRuckus");
//        VuforiaTrackable roverTarget = roverRuckusTargets.get(0);
//        roverTarget.setName("Rover");  // Stones
//
//        VuforiaTrackable footprintTarget  = roverRuckusTargets.get(1);
//        footprintTarget.setName("Footprint");  // Chips
//
//        VuforiaTrackable soilTarget  = roverRuckusTargets.get(2);
//        soilTarget.setName("Soil");  // Chips
//
//        VuforiaTrackable galaxyTarget  = roverRuckusTargets.get(3);
//        galaxyTarget.setName("Galaxy");  // Galaxy
//
//        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
//        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
//        allTrackables.addAll(roverRuckusTargets);
//
//
//        // Beginning of Custom Vuforia Code
//        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
//        vuforia.setFrameQueueCapacity(5);
//
//        /**
//         * We use units of mm here because that's the recommended units of measurement for the
//         * size values specified in the XML for the ImageTarget trackables in data sets. E.g.:
//         *      <ImageTarget name="stones" size="247 173"/>
//         * You don't *have to* use mm here, but the units here and the units used in the XML
//         * target configuration files *must* correspond for the math to work out correctly.
//         */
//        float mmPerInch        = 25.4f;
//        float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
//        float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels
//
//        OpenGLMatrix redTargetLocationOnField = OpenGLMatrix
//                /* Then we translate the target off to the RED WALL. Our translation here
//                is a negative translation in X.*/
//                .translation(-mmFTCFieldWidth/2, 0, 0)
//                .multiplied(Orientation.getRotationMatrix(
//                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
//                        AxesReference.EXTRINSIC, AxesOrder.XZX,
//                        AngleUnit.DEGREES, 90, 90, 0));
//        roverTarget.setLocationFtcFieldFromTarget(redTargetLocationOnField);
//        RobotLog.ii(TAG, "Red Target=%s", format(redTargetLocationOnField));
//
//        /*
//         * To place the Stones Target on the Blue Audience wall:
//         * - First we rotate it 90 around the field's X axis to flip it upright
//         * - Finally, we translate it along the Y axis towards the blue audience wall.
//         */
//        OpenGLMatrix blueTargetLocationOnField = OpenGLMatrix
//                /* Then we translate the target off to the Blue Audience wall.
//                Our translation here is a positive translation in Y.*/
//                .translation(0, mmFTCFieldWidth/2, 0)
//                .multiplied(Orientation.getRotationMatrix(
//                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
//                        AxesReference.EXTRINSIC, AxesOrder.XZX,
//                        AngleUnit.DEGREES, 90, 0, 0));
//        footprintTarget.setLocationFtcFieldFromTarget(blueTargetLocationOnField);
//        RobotLog.ii(TAG, "Blue Target=%s", format(blueTargetLocationOnField));
//
//        OpenGLMatrix robotFromCamera = OpenGLMatrix
//                .translation(mmBotWidth/2,0,0)
//                .multiplied(Orientation.getRotationMatrix(
//                        AxesReference.EXTRINSIC, AxesOrder.XZY,
//                        AngleUnit.DEGREES, 90, 90, 0));
//        RobotLog.ii(TAG, "camera=%s", format(robotFromCamera));
//
//        /**
//         * Let the trackable listeners we care about know where the camera is. We know that each
//         * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
//         * we have not ourselves installed a listener of a different type.
//         */
//        ((VuforiaTrackableDefaultListener)roverTarget.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
//        ((VuforiaTrackableDefaultListener)footprintTarget.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
//        /**
//         *  All setup is complete
//         *
//         */
        waitForStart();

        // start tracking the VuMarks
//        roverRuckusTargets.activate();
//        vision = new VisionController(vuforia, FtcRobotControllerActivity.getDemoView());
//        result = new YellowBlockResult();


//        drivetrain.setTargetHeading(Math.PI/2);
//        while (drivetrain.isRotating() && opModeIsActive()) {
//            drivetrain.updateHeading();
//            doTelemetry();
//        }
//
//        sleep(1000);
//
//        drivetrain.setTargetHeading(-Math.PI/2);
//        while (drivetrain.isRotating()) {
//            drivetrain.updateHeading();
//            doTelemetry();
//        }
//
//        sleep(1000);
//
//
//        drivetrain.setTargetHeading(0);
//        sleep(1000);

        drivetrain.setTargetPosition(500);
        drivetrain.setVelocity(1.5);

        while (opModeIsActive()) {

        }
    }

    boolean firstIterationofScan = true;
    private int orientYellowBlockI = 0;
//    private void orientYellowBlock() {
//        vision.processFrame();
//        result = vision.getResult();
//
//        if (!opModeIsActive()) return;
//
//        if (!result.isFoundBlock() || result.getPoint() == null || result.getBlockArea() > 23000)  { // if block is no longer detected or is too large, we're close
//            if (firstScan) {
//
//                if (firstIterationofScan) {
//                    moveForward(0.2, 500);
//                    firstIterationofScan = false;
//                }
//
//
//                telemetry.addData("area: ", result.getBlockArea());
//                telemetry.update();
//
//                if (orientYellowBlockI % 2 == 0)
//                    pivotLeft(0.2, 500 + (orientYellowBlockI * 50)); // increases duration of pivot every single iteration
//                else
//                    pivotRight(0.2, 500 + (orientYellowBlockI * 50));
//
//                sleep(500);
//                orientYellowBlockI++;
//                orientYellowBlock();
//            } else { // we're right in front of the block, this is good
//                return;
//            }
//        }
//
////        if (result.getBlockArea() > 20000) return; // in front of block
//
//
//        telemetry.addData("center: ", result.getPoint().x);
//        telemetry.addData("area: ", result.getBlockArea());
//
//
//        double kP = 0.0060;
//
//        double error = result.getPoint().x - 462; // subtract the x coordinate from the "center"
//        double motorGain = error * kP;
//
//        double motorPower = 0.3;
//
//        double leftMotorPower = motorPower + (motorPower * motorGain);
//
//        frontRight.setPower(motorPower); // theoretically pivot on the spot towards the yellow block
//        frontLeft.setPower(leftMotorPower);
//
//        telemetry.addData("Error: ", error);
//        telemetry.addData("Gain: ", motorGain);
//        telemetry.addData("Right Motor: ", motorPower);
//        telemetry.addData("Left Motor: ", leftMotorPower);
//        telemetry.update();
//
//        firstScan = false;
//        orientYellowBlock();
//
//    }

    void doTelemetry() {
        PIDController pid = (PIDController) drivetrain.controller.algorithm;
        telemetry.addData("heading, target", drivetrain.controller.getSensorValue()+","+pid.getTarget());
        telemetry.addData("KP", pid.getKP());
        telemetry.addData("KI", pid.getKI());
        telemetry.addData("KD", pid.getKD());
        telemetry.addData("error", pid.getError());
        telemetry.addData("integral", pid.getIntegral());
        telemetry.addData("derivative", pid.getDerivative());
        telemetry.update();
    }

    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }
}
