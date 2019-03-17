package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import edu.spa.ftclib.internal.drivetrain.MecanumDrivetrain;

/**
 * Created by Michaela on 1/3/2018.
 *
 *  ^^ Thanks Michaela - 2875
 */

/**
 * Author: Darwin
 *
 * Usage: Implements HOMAR FTC library for mecanum drive control
 *
 * Motors: 4 drive motors
 * Sensors: internal REV IMU
 */


@TeleOp(name = "Mecanum Robot Tele-op")
public class MecanumRobotTeleop extends OpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private Servo markerServo;
//    private DcMotor liftRight;
//    private DcMotor liftLeft;

    private DcMotor[] motors;

//    private DcMotorEx armSwivel;

    private MecanumDrivetrain drivetrain;

    private DcMotor leftLift = null;
    private DcMotor rightLift = null;

    private final int LIFT_MAX = 20000;
    private final int LIFT_MIN = 200;
    
    private final int minLiftTicks = 0;
    private final int maxLiftTicks = 100;

    private final double FAST_FACTOR = 3; // max: 0.75
    boolean motorsOn;
    double startTime;

    private boolean stopToggle = true;
    private boolean buttonToggle = true;


    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    @Override
    public void init() {
        markerServo= hardwareMap.get(Servo.class, "depotDropper");
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");
//        liftRight = hardwareMap.get(DcMotor.class, "lr");
//        liftLeft = hardwareMap.get(DcMotor.class, "ll");


        rightLift = hardwareMap.dcMotor.get("rightlift");
        leftLift = hardwareMap.dcMotor.get("leftlift");

    //    armSwivel = (DcMotorEx) hardwareMap.get(DcMotor.class, "swivel");

        motors = new DcMotor[]{frontLeft, frontRight, backLeft, backRight};

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        liftRight.setDirection(DcMotorSimple.Direction.FORWARD);
//        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightLift.setDirection(DcMotor.Direction.FORWARD);
        leftLift.setDirection(DcMotor.Direction.REVERSE);

     //   armSwivel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armSwivel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
     //   armSwivel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
     //   armSwivel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Direction Key: ", "Front left, Front Right, Back Left, Back Right, Lift");
        for (DcMotor motor : motors) {
            telemetry.addData("Motor Direction: ", motor.getDirection());
        }

        drivetrain = new MecanumDrivetrain(motors);

        telemetry.update();
        startTime = 0;
    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    @Override
    public void loop() {
        double course = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI/2;
        double rotation = -gamepad1.left_stick_x;
        double velocity = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);

        //Servo Code
        if(gamepad1.a) {
            markerServo.setPosition(1);
        }else{
            markerServo.setPosition(0);
        }
        /**
         * Motor driving commands, this shouldn't be relevant in terms of testing out the swivel arm
         */

        // limit the velocity and rotational velocity since its so STRONG
        if (velocity > 0.75)
            velocity = 0.75;

        if (rotation < -0.75)
            rotation = -0.75;
        else if (rotation > 0.75)
            rotation = 0.75;

        // this is dangerous
        if (gamepad1.left_bumper) {
            velocity *= FAST_FACTOR;
            rotation *= FAST_FACTOR;
        }


        if (gamepad1.dpad_up) {
            rightLift.setPower(0.75);
            leftLift.setPower(0.75);

        } else if (gamepad1.dpad_down) {
            rightLift.setPower(-0.75);
            leftLift.setPower(-0.75);
        } else {
            rightLift.setPower(0);
            leftLift.setPower(0);
        }


        // normal control mode, dpad-up to bring bar up and dpad-down to bring it down
        //if (gamepad1.dpad_up) {
        //    armSwivel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //    armSwivel.setPower(0.35);
        //} else if (gamepad1.dpad_down) {
        //    armSwivel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //    armSwivel.setPower(-0.35);
        //} else {
        //    if (stopToggle)
        //        armSwivel.setPower(0);
        //}

//        if(gamepad1.y && gamepad1.a)
//            if (gamepad1.y) {
//                liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                liftRight.setPower(0.35);
//            } else if (gamepad1.a) {
//                while(liftRight.getCurrentPosition() > 0)
//                liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                liftRight.setPower(-0.35);
//        }

        // run to positon mode, pressing A should bring the bar up to the approximately the halfway point
        //if (gamepad1.a) {
        //    armSwivel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //    armSwivel.setTargetPosition(1200);
        //    armSwivel.setPower(0.15);
        //}


        // this toggle will explicity set the swivel motor to zero power or not (turn it off to test out the run to position mode)
        //if (gamepad1.b && !buttonToggle) {
        //    stopToggle = !stopToggle;
        //}

        // makes sure that we don't double press a button
        buttonToggle = (gamepad1.a || gamepad1.b || gamepad1.y || gamepad1.x);



        drivetrain.setCourse(course);
        drivetrain.setVelocity(velocity);
        drivetrain.setRotation(rotation);

        telemetry.addData("Stop Toggle: ", stopToggle);

    //    PIDFCoefficients coefficients = armSwivel.getPIDFCoefficients(armSwivel.getMode());
    //    telemetry.addData("P I D F: ",coefficients.toString());

     //   telemetry.addData("Arm Velocity (rad/s): ", armSwivel.getVelocity(AngleUnit.RADIANS));
     //   telemetry.addData("Arm Pos: ", armSwivel.getCurrentPosition());

        telemetry.addData("course", course);
        telemetry.addData("velocity", velocity);
        telemetry.addData("rotation", rotation);
        telemetry.update();
    }
}

