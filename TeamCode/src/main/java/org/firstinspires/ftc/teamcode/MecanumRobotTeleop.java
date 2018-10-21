package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import edu.spa.ftclib.internal.drivetrain.MecanumDrivetrain;

/**
 * Created by Michaela on 1/3/2018.
 *
 *  ^^ Thanks Michaela - 2875
 */


@TeleOp(name = "Mecanum Robot Tele-op")

public class MecanumRobotTeleop extends OpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private DcMotor[] motors;

    private MecanumDrivetrain drivetrain;

    private final double FAST_FACTOR = 3;

    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "flMotor");
        frontRight = hardwareMap.get(DcMotor.class, "frMotor");
        backLeft = hardwareMap.get(DcMotor.class, "blMotor");
        backRight = hardwareMap.get(DcMotor.class, "brMotor");

        motors = new DcMotor[]{frontLeft, frontRight, backLeft, backRight};

        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Direction Key: ", "Front left, Front Right, Back Left, Back Right");
        for (DcMotor motor : motors) {
            telemetry.addData("Motor Direction: ", motor.getDirection());
        }

        drivetrain = new MecanumDrivetrain(motors);

        telemetry.update();
    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    @Override
    public void loop() {
        double course = Math.atan2(-gamepad1.right_stick_y, -gamepad1.right_stick_x) - Math.PI/2;
        double rotation = -gamepad1.left_stick_x;
        double velocity = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);

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




        drivetrain.setCourse(course);
        drivetrain.setVelocity(velocity);
        drivetrain.setRotation(rotation);

        telemetry.addData("course", course);
        telemetry.addData("velocity", velocity);
        telemetry.addData("rotation", rotation);
        telemetry.update();
    }
}

