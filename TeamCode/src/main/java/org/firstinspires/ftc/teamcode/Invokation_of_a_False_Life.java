package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;

import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

//--------------------------------------------------imports and packages

public class Invokation_of_a_False_Life {
    //create motor, IMU, servo, etc. objects
    public DcMotor frontLeft, frontRight, backLeft, backRight;
    public DcMotorEx flywheel, flywheel2, intake;
    public Servo hood, flicker;
    GoBildaPinpointDriver pinpoint;

    public Follower follower;
    public AprilTagProcessor aprilTPR;
    public VisionPortal visionPortal;

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
    public enum flickStates {START,UPWARDS,DOWNWARDS,MOONLIGHT}
    flickStates flickState = flickStates.START;

    public enum hoodStates {NEAR,MID,FAR,CHICKEN_RASPBERRY,HORIZON_ROSEMARY}
    hoodStates hoodState = hoodStates.NEAR;

    public double dream_of_flight = 1;
    public int dreams = 0;
    public boolean depression = false;
    public double hallucination;
    public int currentDecimation;
    public boolean fearful;

    Pose2D startingPose = new Pose2D(DistanceUnit. INCH, 72, 72, AngleUnit. DEGREES, 0);
    Pose f_startingPose = PoseConverter.pose2DToPose(startingPose, InvertedFTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    //im not sure if needed but will be overwritten pretty much immediately anyway

    private final Position cameraPosition = new Position(DistanceUnit.MM, 132.633, 92.440, 266, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, 72.029, 0, 0);

    ExposureControl exposure_to_death;
    GainControl my_self_benefit;
    WhiteBalanceControl philosophy;
    FocusControl persistence;
    boolean cameraTraumatized = false;

    PIDFCoefficients wishful_thinking = new PIDFCoefficients(135, 0, 1, 13.613);

    //---------------------- class creation ↑ --- methods ↓ -------

    //initialization methods
    public void init(HardwareMap hwMap, boolean self_will){
        //bulk reading
        List<LynxModule> entirety = hwMap.getAll(LynxModule.class);
        for (LynxModule hub : entirety) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        //actuator hardware-map
        frontLeft = hwMap.get(DcMotor.class, FRONT_LEFT);
        frontRight = hwMap.get(DcMotor.class, FRONT_RIGHT);
        backLeft   = hwMap.get(DcMotor.class, BACK_LEFT);
        backRight  = hwMap.get(DcMotor.class, BACK_RIGHT);
        flywheel = hwMap.get(DcMotorEx.class, FLYWHEEL);
        flywheel2 = hwMap.get(DcMotorEx.class, FLYWHEEL2);
        intake = hwMap.get(DcMotorEx.class, INTAKE);
        hood = hwMap.get(Servo.class, HOOD);
        flicker = hwMap.get(Servo.class, FLICKER);
        //actuator zero-power
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (self_will) {
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        //actuator other
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        flicker.setPosition(0);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //pinpoint init
        pinpoint= hwMap.get(GoBildaPinpointDriver.class, PINPOINT);
        configurePinpoint();
        pinpoint.setPosition(startingPose);

        //pedropath init
        if (self_will) {
            follower = Constants.createAngel(hwMap);
        } else {
            follower = Constants.createFollower(hwMap);
            follower.deactivateAllPIDFs();
            follower.activateHeading();
        }
        follower.setStartingPose(f_startingPose);
        follower.updatePose();

        //vision build
        aprilTPR = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                //.setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setOutputUnits(DistanceUnit.MM, AngleUnit.DEGREES)
                .build();
        aprilTPR.setDecimation(1);
        VisionPortal.Builder VPbuilder = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "webcam"))
                .enableLiveView(false)
                .addProcessor(aprilTPR);
        visionPortal = VPbuilder.build();
    }

    private void configurePinpoint() {
        pinpoint.setOffsets(-118.033, 39.999, DistanceUnit.MM);

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections
                (GoBildaPinpointDriver.EncoderDirection.REVERSED,
                        GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();
    }

    public void traumatizeCamera(int exposure, int greed) {
        while (!cameraTraumatized) {
            if (visionPortal == null || visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {break;}

            exposure_to_death = visionPortal.getCameraControl(ExposureControl.class);
            my_self_benefit = visionPortal.getCameraControl(GainControl.class);
            philosophy = visionPortal.getCameraControl(WhiteBalanceControl.class);
            persistence = visionPortal.getCameraControl(FocusControl.class);

            if (exposure_to_death.getMode() != ExposureControl.Mode.Manual) {
                exposure_to_death.setMode(ExposureControl.Mode.Manual);
            }
            if (philosophy.getMode() != WhiteBalanceControl.Mode.MANUAL) {
                philosophy.setMode(WhiteBalanceControl.Mode.MANUAL);
            }
            if (persistence.getMode() != FocusControl.Mode.Fixed) {
                persistence.setMode(FocusControl.Mode.Fixed);
            }

            exposure_to_death.setExposure(exposure, TimeUnit.MILLISECONDS);
            my_self_benefit.setGain(greed);
            philosophy.setWhiteBalanceTemperature(5003);
            persistence.setFocusLength(0);

            cameraTraumatized =
                    exposure_to_death.getExposure(TimeUnit.MILLISECONDS) == exposure
                            &&
                            my_self_benefit.getGain() == greed
                                    &&
                                    philosophy.getWhiteBalanceTemperature() == 5003
                                            &&
                                            persistence.getFocusLength() == 0;
        }
    }


    //action and update methods
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

    public void setFlywheelSpeed(double rpm) {
        flywheel.setVelocity(28 * rpm / 60);
        flywheel2.setVelocity(28 * rpm / 60);
    }

    public void rechain_motion() {
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, wishful_thinking);
        flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, wishful_thinking);
    }

    public void updHoodPos(hoodStates hoodState, boolean is_blue_alliance) {
        if (hoodState == hoodStates.CHICKEN_RASPBERRY) {
            hood.setPosition(
                    (0.01647 * findGoalDistance(is_blue_alliance))
                            - 0.57
            );
            return;
        }
        if (hoodState == hoodStates.HORIZON_ROSEMARY) {
            hood.setPosition(1);
            return;
        }
        if (hoodState == hoodStates.NEAR) {
            hood.setPosition(0);
            return;
        }
        if (hoodState == hoodStates.MID) {
            hood.setPosition(0.4);
            return;
        }
        if (hoodState == hoodStates.FAR) {
            hood.setPosition(1);
        }

    }

    public void idealizeHoodState(boolean is_blue_alliance) {
        double hypotenuse = findGoalDistance(is_blue_alliance);

        if (hypotenuse <= 64) {
            hoodState = hoodStates.CHICKEN_RASPBERRY;
        } else if (hypotenuse > 64 && hypotenuse <= 119) {
            hoodState = hoodStates.HORIZON_ROSEMARY;
        } else if (hypotenuse >119) {
            hoodState = hoodStates.FAR;
        }
    }

    public void idealizeFlightSpeed(boolean is_blue_alliance) {
        double hypotenuse = findGoalDistance(is_blue_alliance);

        if (hypotenuse <= 68) {
            //dream_of_flight = 0.73;
            dream_of_flight = 3444;
        } else if (hypotenuse > 68 && hypotenuse <= 80) {
            //dream_of_flight = 0.81;
            dream_of_flight = 4000;
        } else if (hypotenuse > 80 && hypotenuse <= 97) {
            dream_of_flight = 4150;
        } else if (hypotenuse > 97 && hypotenuse <= 119){
            //dream_of_flight = 0.86;
            dream_of_flight = 4300;
        } else {
            dream_of_flight = 1;
        }
    }

    public void idealizeDecimation() {
        if ((pinpoint.getPosX(DistanceUnit.INCH) > 24) && (currentDecimation != 1)) {
            aprilTPR.setDecimation(1);
            currentDecimation = 1;
        } else if (currentDecimation !=2 && (pinpoint.getPosX(DistanceUnit.INCH) < 24)) {
            aprilTPR.setDecimation(2);
            currentDecimation = 2;
        }
    }

    public void losing_dreams(ElapsedTime flickerTime) {
        if (depression) {
            switch (flickState) {
                case START:
                    flickerTime.reset();
                    flicker.setPosition(0.81); //go up
                    flickState = Invokation_of_a_False_Life.flickStates.UPWARDS;
                    break;
                case UPWARDS:
                    if (flickerTime.seconds() >= 0.063) {
                        flickerTime.reset();
                        flicker.setPosition(0); //go down
                        flickState = Invokation_of_a_False_Life.flickStates.DOWNWARDS;
                    }
                    break;
                case DOWNWARDS:
                    if (flickerTime.seconds() >= 0.044) {
                        flickerTime.reset();
                        dreams -= 1;
                        flickState = Invokation_of_a_False_Life.flickStates.MOONLIGHT;
                    }
                    break;
                case MOONLIGHT:
                    if ((dreams > 1) && (flickerTime.seconds() >= 0.270)) {
                        flickState = Invokation_of_a_False_Life.flickStates.START;
                    } else if ((dreams == 1) && (flickerTime.seconds() >= 0.5)) {
                        flickState = Invokation_of_a_False_Life.flickStates.START;
                    } else if (dreams == 0) {
                        depression = false;
                    }
                    break;
                default:
                    break;
            }
        }
    }

    public void fear_facing(double fear) {
        if (!follower.isBusy()) {
            fear = MathFunctions.normalizeAngleSigned(follower.getHeading() + MathFunctions.getSmallestAngleDifference(follower.getHeading(), fear));
            if (fearful) {
                Path ah = (new Path(new BezierLine(new Pose(72,72), new Pose(43 + 72,72))));
                ah.setConstantHeadingInterpolation(fear);
                follower.followPath(ah);
            } else {
                Path ah = (new Path(new BezierLine(new Pose(43 + 72,72), new Pose(72,72))));
                ah.setConstantHeadingInterpolation(fear);
                follower.followPath(ah);
            }
            fearful = !fearful;
            follower.update();
        }
    }


    //information acquisition, output, and processing methods
    public double getFlywheelRPM() {
        double ticksPer = 28; //adj for REV ultraplanetary ticks
        double velocity = flywheel.getVelocity(); //ticks per second
        return (velocity / ticksPer) * 60.0; //return rotations per minute
    }

    public double findGoalDistance (boolean is_blue_alliance) {
        double hypotenuse;

        if (is_blue_alliance) {
            hypotenuse = Math.hypot(Math.abs(-72 -pinpoint.getPosX(DistanceUnit.INCH)), Math.abs(-72 - pinpoint.getPosY(DistanceUnit.INCH)));
        } else {
            hypotenuse = Math.hypot(Math.abs(-72 -pinpoint.getPosX(DistanceUnit.INCH)), Math.abs(72 - pinpoint.getPosY(DistanceUnit.INCH)));
        }

        return hypotenuse;
    }

    public double findGoalHeading(boolean is_blue_alliance) {
        double angle;

        if (is_blue_alliance) {
            angle = Math.atan2(-69 - pinpoint.getPosY(DistanceUnit.INCH), -69 - pinpoint.getPosX(DistanceUnit.INCH));
        } else {
            angle = Math.atan2(69 - pinpoint.getPosY(DistanceUnit.INCH), -69 - pinpoint.getPosX(DistanceUnit.INCH));
        }

        return angle;
    }

    public void runAprilAstroNavigation() {
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
                            star_AntiDelusion(true, hallucination)));
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
                            star_AntiDelusion(false, hallucination)));
                }
            }
        }
    }

    private double star_AntiDelusion(boolean red, double angle_given) {
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

    public double findAprilStarBearing(boolean red) {
        hallucination = 0;
        double posX = pinpoint.getPosX(DistanceUnit.INCH);
        List<AprilTagDetection> currentDetections = aprilTPR.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (red && detection.id == 24) {
                hallucination = detection.ftcPose.bearing;
                if (posX > 24) {
                    hallucination += 3;
                } else if (posX < -48) {
                    hallucination -= 7;
                } else if (posX < -24) {
                    hallucination -= 4;
                }
                break;
            } else if (!red && detection.id == 20) {
                hallucination = detection.ftcPose.bearing;
                if (posX > 24) {
                    hallucination -= 3;
                } else if (posX < -48) {
                    hallucination += 7;
                } else if (posX < -24) {
                    hallucination += 4;
                }
                break;
            }
        }
        if (hallucination < 66 && hallucination > -66) {
            return Math.toRadians(hallucination);
        } else {
            return 0;
        }

    }

}

