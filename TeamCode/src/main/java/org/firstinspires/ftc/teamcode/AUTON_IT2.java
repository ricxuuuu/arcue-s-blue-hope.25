package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "AUTON IT2B // OUTOFCOMMISSION", group = "Linear Op-mode")
public class AUTON_IT2 extends LinearOpMode {

    private final Invokation_of_a_False_Life robot = new Invokation_of_a_False_Life();

    private final Pose startPose = new Pose(126.4,123.2,2.2080);
    private final Pose shootPose = new Pose(96,96,2.356);
    private final Pose approachTPose = new Pose(96,84,1.57);
    private final Pose pickupTPose = new Pose(132,84,1.57);
    private final Pose approachMPose = new Pose(96,60,1.57);
    private final Pose pickupMPose = new Pose(132,60,1.57);
    private  final Pose approachBPose = new Pose(96,36,1.57);
    private final Pose pickupBPose = new Pose(132,36,1.57);
    private final Pose leave = new Pose(132, 96, 1.57);

    private PathChain scorePre, runAway,scoreT, scoreM, scoreB;

    int pathState;


    public void runOpMode() {

        robot.init(hardwareMap);
        robot.follower.setStartingPose(startPose);

        buildPaths();
        pathState = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            robot.localizeViaApril();
            robot.follower.updatePose();
            robot.pinpoint.update();

            switch (pathState) {
                case 0:
                    makeLoudNoises();
                    follower.followPath(scorePre);
                    pathState = 1;
                    break;
                case 1:
                    if (!follower.isBusy()) {
                        tripleShot();
                        sleep(222);
                        follower.followPath(scoreT);
                        pathState = 2;
                    }
                    makeLoudNoises();
                    break;
                case 2:
                    if (!follower.isBusy()) {
                        tripleShot();
                        sleep(222);
                        follower.followPath(scoreM);
                        pathState = 3;
                    }
                    makeLoudNoises();
                    break;
                case 3:
                    if (!follower.isBusy()) {
                        tripleShot();
                        sleep(222);
                        follower.followPath(scoreB);
                        pathState = 4;
                    }
                    makeLoudNoises();
                    break;
                case 4:
                    if (!follower.isBusy()) {
                        tripleShot();
                        sleep(222);
                        follower.followPath(runAway);
                        pathState = 5;
                    }
                    makeLoudNoises();
                    break;
                case 5:
                    if (!follower.isBusy()) {
                        robot.intake.setPower(0);
                        robot.setFlywheelPower(0);
                        pathState = -999;
                    }

            }

            telemetry.addData("---------------------------//", "omg");
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
                .setLinearHeadingInterpolation(approachTPose.getHeading(), pickupTPose.getHeading())
                .addPath(new BezierLine(pickupTPose, approachTPose))
                .setLinearHeadingInterpolation(pickupTPose.getHeading(), approachTPose.getHeading())
                .addPath(new BezierLine(approachTPose, shootPose))
                .setLinearHeadingInterpolation(approachTPose.getHeading(), shootPose.getHeading())
                .build();

        scoreM = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootPose, approachMPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), approachMPose.getHeading())
                .addPath(new BezierLine(approachMPose, pickupMPose))
                .setLinearHeadingInterpolation(approachMPose.getHeading(), pickupMPose.getHeading())
                .addPath(new BezierLine(pickupMPose, approachMPose))
                .setLinearHeadingInterpolation(pickupMPose.getHeading(), approachMPose.getHeading())
                .addPath(new BezierLine(approachMPose, shootPose))
                .setLinearHeadingInterpolation(approachMPose.getHeading(), shootPose.getHeading())
                .build();

        scoreB = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootPose, approachBPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), approachBPose.getHeading())
                .addPath(new BezierLine(approachBPose, pickupBPose))
                .setLinearHeadingInterpolation(approachBPose.getHeading(), pickupBPose.getHeading())
                .addPath(new BezierLine(pickupBPose, approachBPose))
                .setLinearHeadingInterpolation(pickupBPose.getHeading(), approachBPose.getHeading())
                .addPath(new BezierLine(approachBPose, shootPose))
                .setLinearHeadingInterpolation(approachBPose.getHeading(), shootPose.getHeading())
                .build();

        runAway = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootPose, leave))
                .setLinearHeadingInterpolation(shootPose.getHeading(), leave.getHeading())
                .build();
    }

    private void oneShot() { //no need for FSM since robot won't be movin n shootin
        robot.flicker.setPosition(1);
        sleep(300);
        robot.flicker.setPosition(0);
        sleep(200);
    }

    private void tripleShot() {
        oneShot();
        robot.hood.setPosition(0.3);
        sleep(1000);
        oneShot();
        robot.hood.setPosition(0.3);
        sleep(1000);
        oneShot();
        robot.hood.setPosition(0.3);
    }

    private void makeLoudNoises() {
        robot.intake.setPower(1);
        robot.setFlywheelSpeed(4777);
    }

}
