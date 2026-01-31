package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AUTON//red neg(x) ୨୧", group = "Linear Op-mode")
public class AUTON_RF extends LinearOpMode {

    private final Invokation_of_a_False_Life robot = new Invokation_of_a_False_Life();

    //-----------------------------------------------poses
    private final Pose startPose = new Pose(121.7,125.8, Math.toRadians(36.2));
    private final Pose shootPose = new Pose(96,96,Math.toRadians(45));
    private final Pose approachTPose = new Pose(96,87,Math.toRadians(0));
    private final Pose pickupTPose = new Pose(127.7,87,Math.toRadians(0));
    private final Pose approachMPose = new Pose(96,60,Math.toRadians(0));
    private final Pose pickupMPose = new Pose(134.7,60,Math.toRadians(0));
    private final Pose PMcontrolPose = new Pose(84, 60, Math.toRadians(20));
    private final Pose approachBPose = new Pose(96,36,Math.toRadians(0));
    private final Pose pickupBPose = new Pose(134.7,36,Math.toRadians(0));
    private final Pose leave = new Pose(96, 130, Math.toRadians(0));

    private PathChain scorePre ,scoreT, scoreM, scoreB, runAway;
    //-----------------------------------------------poses

    int pathState;

    public void runOpMode() {

        robot.init(hardwareMap);
        robot.hood.setPosition(0.3);

        robot.follower.setPose(startPose);
        buildPaths();
        pathState = 0;

        telemetry.addData("RED GOALTOUCH", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            robot.follower.update();
            robot.pinpoint.update();

            switch (pathState) {
                case 0:
                    robot.intake.setPower(1);
                    robot.setFlywheelPower(0.53);
                    robot.follower.followPath(scorePre, true);
                    pathState = 1;
                    break;
                case 1:
                    if (!robot.follower.isBusy()) {
                        pulseFlywheel();
                        tripleShot();
                        robot.intake.setPower(1);
                        robot.setFlywheelPower(0);
                        robot.follower.followPath(scoreT, true);
                        pathState = 2;
                    }
                    break;
                case 2:
                    if (!robot.follower.isBusy()) {
                        robot.setFlywheelPower(0.53);
                        sleep(888);
                        tripleShot();
                        robot.intake.setPower(1);
                        robot.setFlywheelPower(0);
                        robot.follower.followPath(scoreM, true);
                        pathState = 3;
                    }
                    break;
                case 3:
                    if (!robot.follower.isBusy()) {
                        robot.setFlywheelPower(0.53);
                        sleep(888);
                        tripleShot();
                        robot.intake.setPower(1);
                        robot.setFlywheelPower(0);
                        robot.follower.followPath(scoreB, true);
                        pathState = 4;
                    }
                    break;
                case 4:
                    if (!robot.follower.isBusy()) {
                        robot.setFlywheelPower(0.53);
                        sleep(888);
                        tripleShot();
                        robot.intake.setPower(1);
                        robot.setFlywheelPower(0);
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
        robot.hood.setPosition(0.3);
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
        robot.setFlywheelPower(0.53);
    }

}
