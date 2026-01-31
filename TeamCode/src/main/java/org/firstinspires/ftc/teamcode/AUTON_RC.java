package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AUTON//red pos(x) ୨୧", group = "Linear Op-mode")
public class AUTON_RC extends LinearOpMode {

    private final Invokation_of_a_False_Life robot = new Invokation_of_a_False_Life();

    //-----------------------------------------------poses
    private final Pose startPose = new Pose(88.75,8.8, Math.toRadians(90));
    private final Pose shootPose = new Pose(88,15,Math.toRadians(64.54)); //56 129
    private final Pose approachMPose = new Pose(96,60,Math.toRadians(0));
    private final Pose pickupMPose = new Pose(134.7,63,Math.toRadians(0));
    private final Pose approachBPose = new Pose(96,36,Math.toRadians(0));
    private final Pose pickupBPose = new Pose(134.7,36,Math.toRadians(0));
    private final Pose leave = new Pose(120, 24, Math.toRadians(0));

    private PathChain scorePre ,scoreB, scoreM, runAway;
    //-----------------------------------------------poses

    int pathState;

    public void runOpMode() {

        robot.init(hardwareMap);

        robot.follower.setPose(startPose);
        buildPaths();
        pathState = 0;

        telemetry.addData("RED AUDIENCE", "Initialized");
        telemetry.update();
        waitForStart();
        robot.hood.setPosition(1);
        sleep(800);

        while (opModeIsActive()) {

            robot.hood.setPosition(1);
            robot.follower.update();
            robot.pinpoint.update();

            switch (pathState) {
                case 0:
                    robot.intake.setPower(1);
                    robot.setFlywheelPower(1);
                    robot.follower.followPath(scorePre, true);
                    pathState = 1;
                    break;
                case 1:
                    if (!robot.follower.isBusy()) {
                        pulseFlywheel();
                        tripleShot();
                        robot.intake.setPower(1);
                        robot.setFlywheelPower(0);
                        robot.follower.followPath(scoreB, true);
                        pathState = 2;
                    }
                    break;
                case 2:
                    if (!robot.follower.isBusy()) {
                        robot.follower.turnTo(shootPose.getHeading());
                        robot.setFlywheelPower(1);
                        sleep(1900);
                        tripleShot();
                        robot.intake.setPower(1);
                        robot.setFlywheelPower(0);
                        robot.follower.followPath(scoreM, true);
                        pathState = 3;
                    }
                    break;
                case 3:
                    if (!robot.follower.isBusy()) {
                        robot.follower.turnTo(shootPose.getHeading());
                        robot.setFlywheelPower(1);
                        sleep(1900);
                        tripleShot();
                        robot.intake.setPower(1);
                        robot.setFlywheelPower(0);
                        robot.follower.followPath(runAway, true);
                        pathState = 4;
                    }
                    break;
                case 4:
                    if (!robot.follower.isBusy()) {
                        robot.intake.setPower(0);
                        pathState = -999;
                    }
                    break;
                default:
                    robot.intake.setPower(0);
                    break;

            }

            telemetry.addData("---------------------------//RED", "omg");
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

        scoreB = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootPose, approachBPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), approachBPose.getHeading())
                .addPath(new BezierLine(approachBPose, pickupBPose))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(pickupBPose, shootPose))
                .setLinearHeadingInterpolation(pickupBPose.getHeading(), shootPose.getHeading())
                .build();

        scoreM = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootPose, approachMPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), approachMPose.getHeading())
                .addPath(new BezierLine(approachMPose, pickupMPose))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(pickupMPose, shootPose))
                .setLinearHeadingInterpolation(pickupMPose.getHeading(), shootPose.getHeading())
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
        robot.hood.setPosition(1);
        robot.intake.setPower(0);
        oneShot();
        sleep(100);
        robot.intake.setPower(1);
        sleep(800);
        robot.intake.setPower(0);
        oneShot();
        sleep(100);
        robot.intake.setPower(1);
        sleep(800);
        robot.intake.setPower(0.5);
        oneShot();
        robot.intake.setPower(0);
    }

    private void pulseFlywheel() {
        robot.setFlywheelPower(0);
        robot.setFlywheelPower(1);
    }

}
