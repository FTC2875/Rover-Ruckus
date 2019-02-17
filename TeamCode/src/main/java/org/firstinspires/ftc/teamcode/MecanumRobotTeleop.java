package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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
    private DcMotor liftRight;
    private DcMotor liftLeft;

    private DcMotor[] motors;

//    private DcMotorEx armSwivel;

    private MecanumDrivetrain drivetrain;
    
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
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");
        liftRight = hardwareMap.get(DcMotor.class, "lr");
        liftLeft = hardwareMap.get(DcMotor.class, "ll");

    //    armSwivel = (DcMotorEx) hardwareMap.get(DcMotor.class, "swivel");

        motors = new DcMotor[]{frontLeft, frontRight, backLeft, backRight, liftRight, liftLeft};

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        liftRight.setDirection(DcMotorSimple.Direction.FORWARD);
        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);

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


        /**
         * Motor driving commands, this shouldn't be relevant in terms of testing out the swivel arm
         */

        // limit the velocity and rotational velocity since its so STRONG
        if (velocity > 0.25)
            velocity = 0.25;

        if (rotation < -0.25)
            rotation = -0.25;
        else if (rotation > 0.25)
            rotation = 0.25;

        // this is dangerous
        if (gamepad1.left_bumper) {
            velocity *= FAST_FACTOR;
            rotation *= FAST_FACTOR;
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

        if(gamepad1.y && gamepad1.a)
            if (gamepad1.y) {
                liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                liftRight.setPower(0.35);
            } else if (gamepad1.a) {
                while(liftRight.getCurrentPosition() > 0)
                liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                liftRight.setPower(-0.35);
        }

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

        frontRight.setPower(gamepad1.right_stick_y);
        backRight.setPower(gamepad1.right_stick_y);

        frontLeft.setPower(gamepad1.left_stick_y);
        backLeft.setPower(gamepad1.left_stick_y);


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

