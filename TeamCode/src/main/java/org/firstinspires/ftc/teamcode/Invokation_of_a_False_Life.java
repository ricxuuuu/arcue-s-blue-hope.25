package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


//--------------------------------------------------imports and packages

public class Invokation_of_a_False_Life {
    //create motor, IMU, servo, etc. objects
    public DcMotor frontLeft, frontRight, backLeft, backRight;
    public DcMotorEx flywheel, flywheel2, intake;
    public Servo hood, flicker;
    GoBildaPinpointDriver pinpoint;

    public double hallucination;

    public Follower follower;
    public AprilTagProcessor aprilTPR;
    private VisionPortal visionPortal;

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

    public enum hoodStates {NEAR, RASPBERRY, MID, FAR}
    hoodStates hoodState = hoodStates.NEAR;

    public double dream_of_flight = 1;

    Pose2D startingPose = new Pose2D(DistanceUnit. INCH, 72, 72, AngleUnit. DEGREES, -90);
    Pose f_startingPose = PoseConverter.pose2DToPose(startingPose, InvertedFTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    private final Position cameraPosition = new Position(DistanceUnit.MM, 133.550, 83.102, 256.042, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, 72.029, 0, 0);

    //---------------------- class creation ↑ --- methods ↓ -------

    //initialization methods
    public void init(HardwareMap hwMap){
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
        flicker.setPosition(0);
        hood.setPosition(0);

        //zero power behaviour
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        follower = Constants.createFollower(hwMap);
        follower.setStartingPose(f_startingPose);
        follower.updatePose();

        //vision things
        aprilTPR = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                //.setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.MM, AngleUnit.DEGREES)
                .build();
        aprilTPR.setDecimation(2);
        VisionPortal.Builder VPbuilder = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "webcam"))
                //.setCameraResolution(new Size(640, 360))
                .enableLiveView(false)

                .addProcessor(aprilTPR);
        visionPortal = VPbuilder.build();
    }

    private void configurePinpoint() {
        //x-off = how left the forward pod is from the tracking point
        //y-off = how forward the strafe pod is from the tracking point
        pinpoint.setOffsets(-119.227, 27.854, DistanceUnit.MM);

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
        lateral *= 1;

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

    public void setFlywheelPower(double power) {
        flywheel.setPower(power);
        flywheel2.setPower(power);
    }

    public void setHoodPos(hoodStates hoodState) {
        if (hoodState == hoodStates.NEAR) {
            hood.setPosition(0);
        }
        if (hoodState == hoodStates.RASPBERRY) {
            hood.setPosition(0.17);
        }
        if (hoodState == hoodStates.MID) {
            hood.setPosition(0.4);
        }
        if (hoodState == hoodStates.FAR) {
            hood.setPosition(1);
        }
    }

    public void setIdealHoodState(boolean is_blue_alliance) {
        double hypotenuse = findHypotenuseFromGoal(is_blue_alliance);

        if (hypotenuse <= 42) {
            hoodState = hoodStates.NEAR;
        } else if (hypotenuse > 42 && hypotenuse <= 79) {
            hoodState = hoodStates.RASPBERRY;
        } else if (hypotenuse >68 && hypotenuse <= 118) {
            hoodState = hoodStates.FAR;
        } else if (hypotenuse > 118) {
            hoodState = hoodStates.FAR;
        }
    }

    public void findIdealFlightSpeed(boolean is_blue_alliance) {
        double hypotenuse = findHypotenuseFromGoal(is_blue_alliance);

        if (hypotenuse <= 42) {
            dream_of_flight = 0.84;
        } else if (hypotenuse > 42 && hypotenuse <= 79) {
            dream_of_flight = 0.84;
        } else if (hypotenuse >68 && hypotenuse <= 118) {
            dream_of_flight = 0.84;
        } else if (hypotenuse > 118) {
            dream_of_flight = 1;
        }
    }


    //information acquisition and output methods
    public double getFlywheelRPM() {
        double ticksPer = 28; //adj for REV ultraplanetary ticks
        double velocity = flywheel.getVelocity(); //ticks per second
        return (velocity / ticksPer) * 60.0; //return rotations per minute
    }

    public double findHypotenuseFromGoal (boolean is_blue_alliance) {
        double hypotenuse = 0;
        pinpoint.update();

        if (is_blue_alliance) {
            hypotenuse = Math.hypot(Math.abs(-72 -pinpoint.getPosX(DistanceUnit.INCH)), Math.abs(-72 - pinpoint.getPosY(DistanceUnit.INCH)));
        } else {
            hypotenuse = Math.hypot(Math.abs(-72 -pinpoint.getPosX(DistanceUnit.INCH)), Math.abs(72 - pinpoint.getPosY(DistanceUnit.INCH)));
        }

        return hypotenuse;
    }

    public double findIdealGoalAngle(boolean is_blue_alliance) {
        double angle = 0;
        pinpoint.update();

        //find angle to point @goal
        if (is_blue_alliance) {
            angle = Math.atan2(-72 - pinpoint.getPosY(DistanceUnit.INCH), -72 - pinpoint.getPosX(DistanceUnit.INCH));
        } else {
            angle = Math.atan2(72 - pinpoint.getPosY(DistanceUnit.INCH), -72 - pinpoint.getPosX(DistanceUnit.INCH));
        }

        return angle;
    }

    public void localizeViaApril() {
        List<AprilTagDetection> currentDetections = aprilTPR.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 24) {
                hallucination = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
                if (hallucination < 90 && hallucination > 0) {
                    pinpoint.setPosition(new Pose2D(
                            DistanceUnit.MM,
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            AngleUnit.DEGREES,
                            translateTCHeading(true, hallucination)));
                }
            }
            if (detection.id == 20) {
                hallucination = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
                if (hallucination < 0 && hallucination > -90) {
                    pinpoint.setPosition(new Pose2D(
                            DistanceUnit.MM,
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            AngleUnit.DEGREES,
                            translateTCHeading(false, hallucination)));
                }
            }
        }
    }

    private double translateTCHeading(boolean red, double angle_given) {
        if (red) {
            angle_given += 90;
        } else {
            angle_given -= 90;
        } //adj for team goal heading default difference

        angle_given = angle_given % 360;
        angle_given = (angle_given + 360) % 360;
        if (angle_given > 180) {
            angle_given -= 360;
        } //normalize

        return angle_given; //heading 0 facing audience / north if X is vert.
    }

}

