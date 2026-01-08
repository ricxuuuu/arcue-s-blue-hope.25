package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.sun.tools.javac.code.Attribute;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

//--------------------------------------------------imports and packages

public class Invokation_of_a_False_Life {
    //create motor, IMU, servo, etc. objects
    public DcMotor frontLeft, frontRight, backLeft, backRight;
    public DcMotorEx flywheel, flywheel2, intake;
    public Servo hood, flicker;
    GoBildaPinpointDriver pinpoint;

    public Follower follower;

    //create static variables to represent the config strings
    private static final String FRONT_LEFT = "frontLeft";
    private static final String FRONT_RIGHT = "frontRight";         //change these for config file
    private static final String BACK_LEFT = "backLeft";
    private static final String BACK_RIGHT = "backRight";
    private static final String INTAKE = "intake";
    private static final String FLYWHEEL = "flywheel";
    private static final String FLYWHEEL2 = "flywheel2";
    private static final String HOOD = "hood";
    private static final String FLICKER = "flicker";
    private static final String PINPOINT = "pinpoint";

    //create Enum for flicker servo, and assign position
    public enum flickStates {START,UPWARDS,DOWNWARDS}
    flickStates flickState = flickStates.START;

    Pose2D startingPose = new Pose2D(DistanceUnit. INCH, 0, 0, AngleUnit. DEGREES, 0);
    Pose f_startingPose = new Pose(0, 0, Math.toRadians(0));


    //---------------------- class creation ↑ --- methods ↓ -------

    //initialization methods
    public void init(@NonNull HardwareMap hwMap){
        //drivetrain
        frontLeft = hwMap.get(DcMotor.class, FRONT_LEFT);
        frontRight = hwMap.get(DcMotor.class, FRONT_RIGHT);
        backLeft   = hwMap.get(DcMotor.class, BACK_LEFT);
        backRight  = hwMap.get(DcMotor.class, BACK_RIGHT);
        //flywheel and intake
        flywheel = hwMap.get(DcMotorEx.class, FLYWHEEL);
        flywheel2 = hwMap.get(DcMotorEx.class, FLYWHEEL2);
        intake = hwMap.get(DcMotorEx.class, INTAKE);
        //servos
        hood = hwMap.get(Servo.class, HOOD);
        flicker = hwMap.get(Servo.class, FLICKER);

        //IMU
        pinpoint= hwMap.get(GoBildaPinpointDriver.class, PINPOINT);
        configurePinpoint();
        pinpoint.setPosition(startingPose);

        //motor/servo directions/position
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        flicker.setPosition(0);

        //zero power behaviour
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        follower = Constants.createFollower(hwMap);
        follower.setStartingPose(f_startingPose);
        follower.updatePose();

    }
    public void configurePinpoint() {
        //x-off = how left the forward pod is from the tracking point
        //y-off = how forward the strafe pod is from the tracking point
        pinpoint.setOffsets(27.854, 119.227, DistanceUnit.MM);

        //set the encoder type to the gobilda 4-arm pods used on #3.
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        //set the directions of the encoders, first the x encoder then the y encoder
        pinpoint.setEncoderDirections
                (GoBildaPinpointDriver.EncoderDirection.REVERSED,
                        GoBildaPinpointDriver.EncoderDirection.REVERSED);

        //recalibrate, see 'SensorGoBildaPinpoint' for reasoning.
        pinpoint.resetPosAndIMU();
    }

    //action methods
    public void drive(double axial, double lateral, double yaw) {

        // imperfect strafe compensation
        lateral *= 1.3;

        // mecanum calculations
        double fl = (axial + lateral + yaw);
        double bl = (axial - lateral + yaw);
        double fr = (axial - lateral - yaw);
        double br = (axial + lateral - yaw);

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

    //information acquisition and output methods
    public double getFlywheelRPM() {
        double ticksPer = 28; //adj for REV ultraplanetary ticks
        double velocity = flywheel.getVelocity(); //ticks per second
        return (velocity / ticksPer) * 60.0; //return rotations per minute
    }

    public double findIdealLaunchAngle(boolean is_blue_alliance) {
        double hypotenuse = 0;
        pinpoint.update();

        if (is_blue_alliance) {
            hypotenuse = Math.hypot(Math.abs(-61 -pinpoint.getPosX(DistanceUnit.INCH)), Math.abs(58 - pinpoint.getPosY(DistanceUnit.INCH)));
        } else {
            hypotenuse = Math.hypot(Math.abs(-61 -pinpoint.getPosX(DistanceUnit.INCH)), Math.abs(-58 - pinpoint.getPosY(DistanceUnit.INCH)));
        }

        //ADD THE MATH TO FIND IDEAL LAUNCH ANGLE HERE, NEED TESTING TO BE DONE FIRST.

        return hypotenuse;
    }

    public double findIdealGoalAngle(boolean is_blue_alliance) {
        double angle = 0;
        pinpoint.update();

        //find angle to point @goal
        if (is_blue_alliance) {
            angle = Math.atan2(58 - pinpoint.getPosY(DistanceUnit.INCH), -61 - pinpoint.getPosX(DistanceUnit.INCH));
        } else {
            angle = Math.atan2(-58 - pinpoint.getPosY(DistanceUnit.INCH), -61 - pinpoint.getPosX(DistanceUnit.INCH));
        }

        //find angle robot needs to move
        angle = angle - pinpoint.getHeading(AngleUnit.RADIANS);

        //normalize angle (e.g. 358deg rotation simplifies to 2deg)
        angle = Math.atan2(Math.sin(angle), Math.cos(angle));

        return angle;
    }

}

