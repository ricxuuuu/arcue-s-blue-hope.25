package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierCurve;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "AUTON//blue neg(x) ୨୧", group = "Linear Op-mode")
public class AUTON_BF extends LinearOpMode {

    private final Invokation_of_a_False_Life robot = new Invokation_of_a_False_Life();

    //-----------------------------------------------poses
    private final Pose startPose = new Pose(22.3,125.8, Math.toRadians(143.5));
    private final Pose shootPose = new Pose(48,96,Math.toRadians(135));
    private final Pose approachTPose = new Pose(48,87,Math.toRadians(180));
    private final Pose pickupTPose = new Pose(16.3,87,Math.toRadians(180));
    private final Pose approachMPose = new Pose(48,60,Math.toRadians(180));
    private final Pose pickupMPose = new Pose(9.3,60,Math.toRadians(180));
    private final Pose PMcontrolPose = new Pose(60, 60, Math.toRadians(160));
    private final Pose approachBPose = new Pose(48,36,Math.toRadians(180));
    private final Pose pickupBPose = new Pose(9.3,36,Math.toRadians(180));
    private final Pose leave = new Pose(48, 130, Math.toRadians(180));

    private PathChain scorePre ,scoreT, scoreM, scoreB, runAway;
    //-----------------------------------------------poses

    int pathState;

    public void runOpMode() {

        robot.init(hardwareMap);
        robot.hood.setPosition(0.6);

        robot.follower.setPose(startPose);
        buildPaths();
        pathState = 0;

        telemetry.addData("BLUE GOALTOUCH", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            robot.follower.update();
            robot.pinpoint.update();

            switch (pathState) {
                case 0:
                    robot.intake.setPower(1);
                    robot.setFlywheelPower(0.65);
                    robot.follower.followPath(scorePre, true);
                    pathState = 1;
                    break;
                case 1:
                    if (!robot.follower.isBusy()) {
                        pulseFlywheel();
                        tripleShot();
                        robot.setFlywheelPower(0);
                        robot.intake.setPower(1);
                        robot.follower.followPath(scoreT, true);
                        pathState = 2;
                    }
                    break;
                case 2:
                    if (!robot.follower.isBusy()) {
                        robot.setFlywheelPower(0.65);
                        sleep(1900);
                        tripleShot();
                        robot.setFlywheelPower(0);
                        robot.intake.setPower(1);
                        robot.follower.followPath(scoreM, true);
                        pathState = 3;
                    }
                    break;
                case 3:
                    if (!robot.follower.isBusy()) {
                        robot.setFlywheelPower(0.65);
                        sleep(1900);
                        tripleShot();
                        robot.setFlywheelPower(0);
                        robot.intake.setPower(1);
                        robot.follower.followPath(scoreB, true);
                        pathState = 4;
                    }
                    break;
                case 4:
                    if (!robot.follower.isBusy()) {
                        robot.setFlywheelPower(0.65);
                        sleep(1900);
                        tripleShot();
                        robot.setFlywheelPower(0);
                        robot.intake.setPower(1);
                        robot.follower.followPath(runAway, true);
                        pathState = 5;
                        robot.intake.setPower(0);
                        robot.setFlywheelPower(0);
                    }
                    break;
                case 5:
                    if (!robot.follower.isBusy()) {
                        pathState = -999;
                    }
                    break;
                default:
                    robot.intake.setPower(-1);
                    break;

            }

            telemetry.addData("---------------------------//BLUE", "omg");
            telemetry.addData("pathState", pathState);
            telemetry.addData("x", robot.follower.getPose().getX());
            telemetry.addData("y", robot.follower.getPose().getY());
            telemetry.addData("h", robot.follower.getHeading());
            telemetry.addData("state", pathState);
            telemetry.update();
        }
    }

    private void buildPaths() {
        scorePre = robot.follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        scoreT = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootPose, approachTPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), approachTPose.getHeading())
                .addPath(new BezierLine(approachTPose, pickupTPose))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(pickupTPose, shootPose))
                .setLinearHeadingInterpolation(pickupTPose.getHeading(), shootPose.getHeading())
                .build();

        scoreM = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootPose, approachMPose))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(approachMPose, pickupMPose))
                .setTangentHeadingInterpolation()
                .addPath(new BezierCurve(pickupMPose, PMcontrolPose, shootPose))
                .setLinearHeadingInterpolation(pickupMPose.getHeading(), shootPose.getHeading())
                .build();

        scoreB = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootPose, approachBPose))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(approachBPose, pickupBPose))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(pickupBPose, shootPose))
                .setLinearHeadingInterpolation(pickupBPose.getHeading(), shootPose.getHeading())
                .build();

        runAway = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootPose, leave))
                .setTangentHeadingInterpolation()
                .build();
    }

    private void oneShot() { //no need for FSM since robot won't be movin n shootin
        robot.flicker.setPosition(0.7);
        sleep(200);
        robot.flicker.setPosition(0);
        sleep(200);
    }

    private void tripleShot() {
        robot.hood.setPosition(0.6);
        robot.intake.setPower(0);
        oneShot();
        sleep(100);
        robot.intake.setPower(1);
        sleep(555);
        robot.intake.setPower(0);
        oneShot();
        sleep(100);
        robot.intake.setPower(1);
        sleep(555);
        robot.intake.setPower(0.5);
        oneShot();
        robot.intake.setPower(0);
    }

    private void pulseFlywheel() {
        robot.setFlywheelPower(0);
        robot.setFlywheelPower(0.65);
    }

}
