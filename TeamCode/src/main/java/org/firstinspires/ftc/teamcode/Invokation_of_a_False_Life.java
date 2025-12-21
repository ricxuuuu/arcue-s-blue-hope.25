package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


public class Invokation_of_a_False_Life {
    //create motor, IMU, servo, etc. objects
    public DcMotor frontLeft, frontRight, backLeft, backRight, intake;
    public DcMotorEx flywheel;
    public Servo hood, flicker;
    GoBildaPinpointDriver pinpoint;

    //create static variables to represent the strings, ease of change.
    private static final String FRONT_LEFT = "frontLeft";
    private static final String FRONT_RIGHT = "frontRight";         //change these for config file
    private static final String BACK_LEFT = "backLeft";
    private static final String BACK_RIGHT = "backRight";
    private static final String INTAKE = "intake";
    private static final String FLYWHEEL = "flywheel";
    private static final String HOOD = "hood";
    private static final String FLICKER = "flicker";
    private static final String PINPOINT = "pinpoint";

    public void init(HardwareMap hwMap){
        //drivetrain
        frontLeft = hwMap.get(DcMotor.class, FRONT_LEFT);
        frontRight = hwMap.get(DcMotor.class, FRONT_RIGHT);
        backLeft   = hwMap.get(DcMotor.class, BACK_LEFT);
        backRight  = hwMap.get(DcMotor.class, BACK_RIGHT);
        //flywheel and intake
        flywheel = hwMap.get(DcMotorEx.class, FLYWHEEL);
        intake = hwMap.get(DcMotor.class, INTAKE);
        //servos
        hood = hwMap.get(Servo.class, HOOD);
        flicker = hwMap.get(Servo.class, FLICKER);

        //IMU
        pinpoint= hwMap.get(GoBildaPinpointDriver.class, PINPOINT);
        configurePinpoint();
        pinpoint.setPosition(new Pose2D(DistanceUnit. INCH, 0, 0, AngleUnit. DEGREES, 0));

        //motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);

        //zero power behaviour
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    public void drive(double axial, double lateral, double yaw) {

        // imperfect strafe compensation
        lateral *= 1.3;

        // mecanum calculations
        double fl = (axial + lateral + yaw);
        double bl = (axial - lateral + yaw);
        double fr = (axial + lateral - yaw);
        double br = (axial - lateral - yaw);

        // Normalize wheel powers
        double max = Math.max(1.0,
                Math.max(Math.abs(fl),
                        Math.max(Math.abs(fr),
                                Math.max(Math.abs(bl), Math.abs(br)))));

        frontLeft.setPower(fl / max);
        backLeft.setPower(bl / max);
        frontRight.setPower(fr / max);
        backRight.setPower(br / max);
    }

    //--------------------------------------------------------------------------------------------//

    public void configurePinpoint() {
        //x-off = how left the forward pod is from the tracking point
        //y-off = how forward the strafe pod is from the tracking point
        pinpoint.setOffsets(45.166, 64.957, DistanceUnit.MM);

        //set the encoder type to the gobilda 4-arm pods used on #3.
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        //set the directions of the encoders, first the x encoder then the y encoder
        pinpoint.setEncoderDirections
                (GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        //recalibrate, see 'SensorGoBildaPinpoint' for reasoning.
        pinpoint.resetPosAndIMU();
    }

}

