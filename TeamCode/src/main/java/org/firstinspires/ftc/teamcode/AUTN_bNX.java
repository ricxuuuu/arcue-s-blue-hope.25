package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AUTON//blue neg(x) ୨୧", group = "Linear Op-mode")
public class AUTN_bNX extends LinearOpMode {

    private final Invokation_of_a_False_Life robot = new Invokation_of_a_False_Life();
    //|||||||||||||||||||||||||||||||// ✧ >.<  //summon a false life to do our bidding

    //-----------------------------------------------poses
    private final Pose startPose = new Pose(22.3,125.8, Math.toRadians(143.5));
    private final Pose shootPose = new Pose(48,96,Math.toRadians(135));
    private final Pose grabT = new Pose(17,84,Math.toRadians(180));
    private final Pose CTRLgT = new Pose(90,75,Math.toRadians(180));
    private final Pose grabM = new Pose(11,57,Math.toRadians(180));
    private final Pose CTRLgM = new Pose(90,53.1,Math.toRadians(180));
    private final Pose CTRLrM = new Pose(33,55,Math.toRadians(133));
    private final Pose leave = new Pose(48, 120, Math.toRadians(155));

    private PathChain scoreT, scoreM, runAway;
    int pathState = 0;
    int shotsFired = 0;
    boolean in = false;
    double intuition = 0;
    //-----------------------------------------------poses

    public void runOpMode() {

        //||||||||||||||||||||||||||||||||||||||//
        telemetry.addData("01/", "NOTICE ///////////// >.<");
        telemetry.addData("02/", "#3.scratch is not yet initialized.");
        telemetry.addData("03/", "Selected Performance - 🔵Blue Outlying");
        telemetry.update();
        //||||||||||||||||||||||||||||||||||||||//
        //-----------------------------------------------PREP

        robot.init(hardwareMap);
        robot.follower.setPose(startPose);
        buildPaths();
        ElapsedTime flickerTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        ElapsedTime revTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        //-----------------------------------------------PREP
        //||||||||||||||||||||||||||||||||||||||//
        telemetry.addData("🔵A//... ", "sparkles of the night ✦");
        telemetry.update();
        //||||||||||||||||||||||||||||||||||||||//

        waitForStart();

        //||||||||||||||||||||||||||||||||||||||//
        robot.hood.setPosition(0);  //CLOSE ZONE HOOD ADJUST
        robot.setFlywheelPower(0.7343);
        revTime.reset();
        sleep(888);      //CLOSE ZONE HOOD ADJUST
        //||||||||||||||||||||||||||||||||||||||//

        //----------------------------------------------------------------------------------------
        while (opModeIsActive()) {

            switch (pathState) {
                case 0:
                    in = true;
                    shootToKill(flickerTime, revTime);
                    if (shotsFired >= 4) {
                        robot.follower.followPath(scoreT, true);
                        pathState = 1;
                        reload();
                    }
                    break;
                case 1:
                    if (!robot.follower.isBusy()) {
                        shootToKill(flickerTime, revTime);
                        if (shotsFired >= 4) {
                            robot.follower.followPath(scoreM, true);
                            pathState = 2;
                            reload();
                        }
                    }
                    break;
                case 2:
                    if (!robot.follower.isBusy()) {
                        shootToKill(flickerTime, revTime);
                        if (shotsFired >= 3) {
                            robot.follower.followPath(runAway, true);
                            pathState = 3;
                        }
                    }
                    break;
                case 3:
                    if (!robot.follower.isBusy()) {
                        pathState = -999;
                    }
                    break;
                default:
                    robot.intake.setPower(0);
                    break;

            }

            //-----------------------------------------------UPDATES
            manageCalories();
            flightEnergyConservation(revTime);
            robot.follower.update();
            robot.pinpoint.update();
            //-----------------------------------------------UPDATES


            //-----------------------------------------------TELEMETRY
            telemetry.addData("---------------------//STATUS-BLUE", "hunting...");
            //telemetry.addData("INTUITION", intuition);
            telemetry.addData("FIRED", shotsFired);
            telemetry.addData("F_STATE/POS", "%s / %.2f", robot.flickState, robot.flicker.getPosition());
            telemetry.addData("F_TIME", flickerTime.seconds());
            telemetry.addData("CASE", pathState);
            telemetry.addData("X.IN", robot.follower.getPose().getX());
            telemetry.addData("Y.IN", robot.follower.getPose().getY());
            telemetry.addData("H.RD", robot.follower.getHeading());
            telemetry.update();
            //-----------------------------------------------TELEMETRY

        }
    }

    /*----------------------------------------------------------------------------------------------
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣬⢟⡛⠍⠉⠉⠙⠻⢵⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣤⣖⣶⣲⣠⡄⣀⠚⠛⠻⢷⣾⡀⠀⠀⠀⠘⢆⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣄⠟⡋⠅⠀⠀⠀⠀⠉⠑⠲⢥⣲⡄⠙⣝⣆⠀⠀⠀⠈⡿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⢀⣄⣠⣀⣄⣠⣀⣄⣠⢄⣠⡄⣤⣠⢤⡤⣤⡤⢤⡤⠤⠤⢤⣤⣤⣾⠥⠖⠒⠋⠉⠉⠉⠙⠒⠦⣄⠀⠈⠛⣗⡿⣺⡀⠀⠀⠀⡟⠧⠤⠤⠤⠤⠤⠤⠤⢤⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡠⠔⡋⠁⠀⠀⣀⠠⠤⠄⠒⠒⠀⠀⠀⠀⠙⢦⡀⠈⢧⡽⠀⠀⣀⡼⠃⠀⠀⠀⠀⠀⠀⠀⠀⢸⡀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⠔⠉⢀⠌⣠⠔⠊⠉⠀⠀⡀⠄⠂⠉⠀⠀⠀⠀⠀⠀⠑⠄⢈⠠⠔⣚⠓⠶⣤⡀⠀⠀⠀⠀⠀⠀⠀⢈⡇⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⢔⠕⠀⢰⡥⠊⠀⠀⠀⠀⠀⠈⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠁⡀⠀⠈⠊⠐⠢⢍⠳⣄⠀⠀⠀⠀⠀⠈⡇⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠂⡀⠀⠀⠀⠀⡠⡣⢊⠐⡼⠣⠀⠀⠀⠀⡠⠊⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠐⡀⠀⠀⠈⠢⠀⠑⢆⠙⠦⣤⣠⣤⡴⣷⡀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⠘⠄⠀⠀⡸⢣⢣⢘⡼⠀⠀⠀⠀⠀⢠⠃⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⠀⠀⠀⠀⠃⠀⠀⠣⡀⠀⠀⠀⠀⣼⡃⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⣿⢿⡦⣄⡴⡡⢇⢣⡞⠀⠀⠀⠀⡐⢡⠃⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠀⠀⠀⠀⠀⠀⠀⠀⠑⡀⠢⡀⣴⡗⠁⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣯⡗⣯⣳⢴⣙⡮⠏⠀⠀⠀⠀⠀⢀⠇⠀⠀⠀⠀⠀⠀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡇⠀⠀⠀⠀⠀⠀⠀⠀⠰⡀⠹⣽⡀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢿⡩⢉⡉⠉⠁⠀⠀⠀⠀⠀⠀⠀⡸⠀⠀⠀⠀⠀⠀⠔⠀⠀⠀⠀⠀⠀⠀⡄⡆⠀⠀⠀⠀⠀⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⢣⠀⢳⢃⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠳⣄⠀⡉⢉⡴⠂⠀⠀⠀⠀⠀⡇⠀⠀⠀⠀⠀⠌⠀⠀⠰⠀⠀⠀⠀⠐⢰⠃⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠘⡄⠈⣿⡄⠀⠀
⠀⠀⠀⠀⠀⠀⠀⢀⣀⣀⣀⣠⠴⣫⣿⠖⠉⠀⠀⠀⠀⠀⠀⢀⡇⠀⠀⠀⠀⠀⠀⠀⠀⡇⠀⠀⠀⠀⠀⣼⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢣⠀⢹⡇⠀⠀
⠀⠀⠀⠀⠀⠀⠀⢧⢰⡺⠶⠒⠚⠉⠀⠀⠀⠀⠀⠀⠠⣀⣠⠞⡇⠀⠀⠀⠀⠀⠀⠀⠸⠃⠀⠀⠀⠀⣴⡇⡆⠀⠀⠀⠀⠀⢀⠀⠀⠀⠀⠀⠀⢸⠀⠀⠀⠈⢆⢸⡕⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠈⠻⢷⣌⡁⠒⠂⠤⠤⠤⠤⠤⢒⣬⡾⠋⠒⢽⡀⠀⠀⠀⠀⠀⠀⣯⠀⠀⠀⡠⣪⠏⣧⢡⠀⠀⠀⠀⠀⢸⠀⠀⠀⠀⠀⠀⠀⣆⠀⠀⠀⠈⢾⣧⣄⡀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣉⠽⡟⠒⠶⠤⠴⢒⣿⡫⠋⠀⠀⠀⠀⢣⠀⠀⠀⠀⠀⢰⢸⠀⢀⢔⡵⠏⠄⠼⣆⣆⣀⣀⡀⠀⠀⡀⠀⠀⠀⠀⠀⠀⠰⢉⠒⠠⠄⠀⠈⣹⡿
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠸⢥⣾⡇⠀⣀⠶⠒⠉⡿⣀⡀⠀⠀⠀⠤⠈⢳⣀⠀⠀⠀⢸⢀⢖⡵⠋⠀⠀⠀⠀⠹⣜⢄⠀⠀⠀⠀⢧⡀⠀⠀⠠⠀⠀⠀⢣⣌⡐⣀⣴⡾⠟⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡇⢠⠇⠀⠀⠀⢳⡿⠿⣿⣷⣶⣤⣄⣠⠷⣕⠢⢀⡸⢸⡋⠀⠀⢀⣀⠀⠀⠀⠘⢮⡢⡀⠀⠀⠸⡐⢄⠀⠀⢂⠀⠀⠈⢯⡽⠁⡏⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢹⢸⠈⢄⠀⠀⠘⡆⠀⣷⡹⣞⡭⣿⠏⠛⠒⠛⠶⢤⣘⡇⠀⢀⣤⣬⣀⣀⣀⣀⠀⠑⣞⢄⡀⠀⢳⡀⠑⠢⣀⠢⡀⠀⠈⠳⣤⣥⣀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⡴⡀⠘⡇⡆⠀⢦⣄⣀⡙⣄⢳⡽⣎⣷⠟⠀⠀⠀⠀⠀⠀⠈⠉⠀⠈⣿⣛⣟⡻⣿⣿⠿⣝⡾⣉⡚⠑⡒⠓⠦⣀⠀⠁⠊⠂⠄⣀⠀⣨⡿⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⡑⠩⠑⢻⡇⠀⠀⣇⠈⠉⠁⠀⠀⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⠵⣎⢷⣣⠇⢀⡞⠀⠀⠀⢈⠍⠑⠢⣈⣽⠳⡗⠾⠶⢒⠛⡗⠁⠀⠀
⠀⣤⣠⢤⡤⣤⢤⣤⠠⠈⢳⢤⠂⠘⢿⡄⠀⠘⢆⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠙⠚⠓⢁⡔⠋⣀⣀⢤⠖⠁⠀⠀⠀⣰⠃⣸⡇⠀⠀⠸⠐⡅⠀⠀⠀
⠠⠷⣭⢷⣹⢮⡟⠁⢀⠔⡡⠊⠠⢁⠀⢙⣦⡀⠘⣍⠢⣄⡀⠀⠀⠀⠀⠀⢀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠉⣹⡣⠊⠀⠀⠀⠀⣰⣣⡼⠋⠀⠀⠀⠀⠀⠃⠀⠀⠀
⠐⣶⣤⡌⢡⢹⠃⣶⠁⠊⠐⠀⠁⢠⣴⠋⠀⠈⣵⠚⣭⣥⣯⣷⣦⣤⠀⠀⠀⠀⠀⠀⠀⣤⣤⣤⣶⣴⣦⣤⡄⠀⣼⠋⠀⠀⠀⠀⠀⡜⠋⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠴⣉⠻⡵⢎⡀⠈⠙⠓⠛⠛⠋⠉⡴⢲⠴⣪⢴⣛⡶⣝⡾⣾⡽⣿⢿⣷⣶⡶⣶⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⡞⠁⠀⠀⠀⢀⡠⠚⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡅⠀⠀⠀
⠀⢓⡌⠳⣌⢣⠱⡀⠀⠀⢀⣀⠤⠒⠋⠉⠉⠛⠛⠓⠛⠛⠿⠷⠿⠿⣿⠏⠀⣧⠸⣿⣿⣻⢟⢿⣿⣿⣿⣿⣏⣀⣀⣀⡤⠖⠋⠀⠀⠀⠀⠀⠀⠂⠀⠀⠀⠀⠀⠄⠀⠀⠀
⠀⢣⠜⡱⢌⢆⢣⡑⣠⣞⡣⣤⣀⣀⣀⣀⣀⣀⠀⠀⠀⠀⠀⠀⠀⠀⢸⡀⠀⡼⡆⢿⣫⣵⣿⢸⣿⣿⡿⣣⡿⠋⠀⠀⢀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡁⠀⠀⠀
⠀⡱⢊⡕⢪⢜⠢⡔⣿⣄⠉⠹⢿⣿⣿⣍⣽⣿⣿⣿⣿⠶⠶⣒⣒⣲⢫⠛⢢⣵⠿⡌⣿⣿⣿⡿⠿⢻⣺⠁⠀⠀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡄⠀⠀⠀
⠀⡱⢣⠜⣡⢎⡱⢌⡟⢉⡤⣤⣤⢂⠭⠭⠭⢍⠉⠉⠁⠀⠄⣴⣶⡎⠎⢀⣾⣿⡆⠈⠁⠀⠀⠀⢠⡿⠛⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⡆⠀⠀⠀
⠀⡱⢃⡞⣡⢎⡴⣂⢧⣥⣞⣛⣓⣶⠦⠤⢥⣼⣿⣿⣯⡳⣄⡈⠻⠵⢀⣾⣟⢾⢃⠀⣀⣀⠠⢔⠿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣤⢴⣭⡇⠀⠀⠀
⠀⡱⢋⡴⢣⠞⣴⢫⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣶⣶⣶⣯⣶⣭⣭⣭⣽⣿⠈⣿⣥⣤⣤⣶⣶⣯⣤⡤⠀⠀⠀⢀⣀⠀⠀⠀⠀⢀⣀⣤⣤⣶⢶⣿⢟⣫⣾⣻⢳⡇⠀⠀⠀
⠀⠑⠉⠲⠉⠚⠤⠋⠬⠧⠽⠽⠛⠟⠿⠟⣛⡟⠛⣿⣿⢿⡻⣟⣻⣻⣿⣿⡷⠿⠿⠿⠏⠙⠒⠓⠒⠄⠀⠀⠒⠛⠒⠶⠛⠿⠿⠿⠝⠚⠓⠚⠳⠲⠯⠷⠛⠶⠿⠀⠀⠀⠀
 -------------------------------------------------------------------------------------------------*/

    private void buildPaths() {
        scoreT = robot.follower.pathBuilder()
                .addPath(new BezierCurve(startPose, CTRLgT, grabT))
                .setLinearHeadingInterpolation(startPose.getHeading(), grabT.getHeading())
                .addPath(new BezierLine(grabT, shootPose))
                .setLinearHeadingInterpolation(grabT.getHeading(), shootPose.getHeading())
                .build();

        scoreM = robot.follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, CTRLgM, grabM))
                .setLinearHeadingInterpolation(shootPose.getHeading(), grabM.getHeading())
                .addPath(new BezierCurve(grabM, CTRLrM, shootPose))
                .setLinearHeadingInterpolation(grabM.getHeading(), shootPose.getHeading())
                .build();

        runAway = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootPose, leave))
                .setLinearHeadingInterpolation(shootPose.getHeading(), leave.getHeading())
                .build();
    }

    private void flightEnergyConservation(ElapsedTime revTime) {
        if (robot.follower.isBusy()) {
            robot.setFlywheelPower(0);
        } else if (robot.flywheel.getPower() == 0){
            if (pathState == 0) {
                robot.setFlywheelPower(0.7343);
            } else {
                robot.setFlywheelPower(0.81);
            }
            revTime.reset();
        }
        if (pathState == 0) {
            robot.hood.setPosition(0);
        } else {
            robot.hood.setPosition(0.6);
        }
    }

    private void manageCalories() {
        if (in && (robot.flickState != (Invokation_of_a_False_Life.flickStates.UPWARDS) && (robot.flickState != Invokation_of_a_False_Life.flickStates.DOWNWARDS))) {
            robot.intake.setPower(0.8);
        } else if (in && (robot.flickState == Invokation_of_a_False_Life.flickStates.DOWNWARDS)) {
            robot.intake.setPower(0.4);
        } else {
            robot.intake.setPower(0);
        }
    }

    private void actOnTheGut() {
        intuition = robot.findAprilStarBearing(true);
        robot.follower.turnTo(intuition + robot.follower.getHeading());
    } //experimental

    private void shootToKill(ElapsedTime flickerTime, ElapsedTime revTime) {
        switch (robot.flickState) {
            case START:
                if (shotsFired < 4 && shotsFired != 0 && (revTime.seconds() > 1.7)) {
                    flickerTime.reset();
                    robot.flicker.setPosition(0.73); //go up
                    robot.flickState = Invokation_of_a_False_Life.flickStates.UPWARDS;
                } else if (shotsFired <= 0 && (revTime.seconds() > 2.3)) {
                    flickerTime.reset();
                    robot.flicker.setPosition(0.73); //go up
                    robot.flickState = Invokation_of_a_False_Life.flickStates.UPWARDS;
                }
                break;
            case UPWARDS:
                if (flickerTime.seconds() >= 0.177) {
                    flickerTime.reset();
                    robot.flicker.setPosition(0); //go down
                    robot.flickState = Invokation_of_a_False_Life.flickStates.DOWNWARDS;
                }
                break;
            case DOWNWARDS:
                if (flickerTime.seconds() >= 0.133) {
                    flickerTime.reset();
                    shotsFired += 1;
                    robot.flickState = Invokation_of_a_False_Life.flickStates.MOONLIGHT;
                }
                break;
            case MOONLIGHT:
                if ((shotsFired < 2) && (flickerTime.seconds() >= 0.888)) {
                    robot.flickState = Invokation_of_a_False_Life.flickStates.START;
                } else if ((shotsFired >= 2) && (flickerTime.seconds() >= 1.3)) {
                    robot.flickState = Invokation_of_a_False_Life.flickStates.START;
                }
                break;
            default:
                break;
        }
    }

    private void reload() {
        shotsFired = 0;
    }

}
