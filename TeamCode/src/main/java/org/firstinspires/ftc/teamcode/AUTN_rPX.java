package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AUTON//red pos(x) ⸙", group = "Linear Op-mode")
public class AUTN_rPX extends LinearOpMode {

    private final Invokation_of_a_False_Life robot = new Invokation_of_a_False_Life();
    //|||||||||||||||||||||||||||||||// ✧ >.<  //summon a false life to do our bidding

    //-----------------------------------------------poses
    private final Pose startPose = new Pose(88.56,8.85, Math.toRadians(90));
    private final Pose shootPose = new Pose(86.6,15.7,Math.toRadians(67));
    private final Pose approachMPose = new Pose(96,53,Math.toRadians(0));
    private final Pose pickupMPose = new Pose(131,51,Math.toRadians(0));
    private final Pose approachBPose = new Pose(96,34.7,Math.toRadians(0));
    private final Pose pickupBPose = new Pose(131,34.7,Math.toRadians(0));
    private final Pose pickupHPose = new Pose(134, 13, Math.toRadians(0));
    private final Pose leave = new Pose(120, 24, Math.toRadians(0));

    private PathChain scorePre ,scoreB, scoreM, scoreH, runAway;
    int pathState = 0;
    int shotsFired = 0;
    boolean in = false;
    double intuition = 0;
    //-----------------------------------------------poses

    public void runOpMode() {

        //||||||||||||||||||||||||||||||||||||||//
        telemetry.addData("01/", "NOTICE ///////////// >.<");
        telemetry.addData("02/", "#3.scratch is not yet initialized.");
        telemetry.addData("03/", "Selected Performance - 🔴Red Audience");
        telemetry.update();
        //||||||||||||||||||||||||||||||||||||||//
        //-----------------------------------------------PREP

        robot.init(hardwareMap, true);
        robot.follower.setPose(startPose);
        buildPaths();
        ElapsedTime flickerTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        ElapsedTime revTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        //-----------------------------------------------PREP
        //||||||||||||||||||||||||||||||||||||||//
        telemetry.addData("🔴A//... ", "let the hunt begin ~ ⚔!");
        telemetry.update();
        //||||||||||||||||||||||||||||||||||||||//

        waitForStart();

        //||||||||||||||||||||||||||||||||||||||//
        robot.hood.setPosition(1);  //FAR ZONE HOOD ADJUST
        //robot.setFlywheelPower(1);
        sleep(888);      //FAR ZONE HOOD ADJUST
        //||||||||||||||||||||||||||||||||||||||//

        //----------------------------------------------------------------------------------------
        while (opModeIsActive()) {

            switch (pathState) {
                case 0:
                    robot.follower.followPath(scorePre, true);
                    in = true;
                    pathState = 1;
                    break;
                case 1:
                    if (robot.follower.isBusy()) {
                        revTime.reset();
                    }
                    if (!robot.follower.isBusy()) {
                        actOnTheGut(revTime);
                        shootToKill(flickerTime, revTime);
                        if (shotsFired >= 4) {
                            robot.follower.followPath(scoreB, true);
                            pathState = 2;
                            reload();
                        }
                    }
                    break;
                case 2:
                    if (!robot.follower.isBusy()) {
                        actOnTheGut(revTime);
                        shootToKill(flickerTime, revTime);
                        if (shotsFired >= 4) {
                            robot.follower.followPath(scoreM, true);
                            pathState = 3;
                            reload();
                        }
                    }
                    break;
                case 3:
                    if (!robot.follower.isBusy()) {
                        actOnTheGut(revTime);
                        shootToKill(flickerTime, revTime);
                        if (shotsFired >= 4) {
                            robot.follower.followPath(scoreH, true);
                            pathState = 4;
                            reload();
                        }
                    }
                    break;
                case 4:
                    if (!robot.follower.isBusy()) {
                        actOnTheGut(revTime);
                        shootToKill(flickerTime, revTime);
                        if (shotsFired >= 2) {
                            robot.follower.followPath(runAway, true);
                            pathState = 5;
                        }
                    }
                    break;
                case 5:
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
            telemetry.addData("---------------------//STATUS-RED", "hunting...");
            telemetry.addData("INTUITION", intuition);
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
⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠁⡇⠀⠀⠀⠀⠀⠀⠀⠸⣿⡇⠀⠚⠉⣸⣿⣅⣀⠀⠀
⠙⠻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⢸⠀⠀⠀⠀⠀⠀⠀⠀⢠⣿⣷⣾⠿⠿⠿⠿⠛⠉⠀⠀
⠀⠀⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡇⡎⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀
⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⠿⠟⠛⠛⠛⠛⠛⠛⢛⠛⠛⠻⠿⠿⠿⠿⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣦⣀⡀⠀⠀⠀⠀⢠⡿⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⣰
⣿⣿⣿⣿⣿⣿⠿⠛⠛⠉⠉⢀⣤⠞⠋⠀⠀⠀⠀⢀⡤⢀⡤⠚⠁⠀⠀⠀⠀⠀⠀⣠⠴⠂⠀⠈⠉⠉⠛⠻⠿⣿⣿⣿⣿⣿⣿⣿⣶⣤⡀⢀⡟⠁⠀⠀⠀⠀⠀⠀⠀⠀⢀⣼⣿
⣿⣿⣿⣿⠏⣠⠀⠀⠀⠀⣰⠟⠁⠀⠀⠀⠀⣠⠖⠉⠀⠀⠀⠀⠀⠀⠀⠀⢀⡴⠚⠁⠀⠀⠀⠀⠀⠀⠀⠀⢀⡟⠈⠉⠛⠿⣿⣿⣿⣿⣿⣿⣄⡀⠀⠀⠀⠀⠀⠀⠀⣠⣾⣿⣿
⣿⣿⣿⢏⡴⠃⠀⠀⢠⡞⠁⠀⣠⠔⠀⣠⢞⣡⠞⠁⠀⠀⠀⠀⠀⠀⢀⡴⠋⠀⠀⠀⠀⠀⠀⠀⣠⠄⠀⡠⢻⠁⠀⠀⠀⠀⠀⠙⠻⢿⣿⣿⣿⣿⣦⡀⠀⠀⠀⢀⣴⣿⣿⠉⠉
⣿⣿⡷⠋⠀⠀⠀⣰⠋⣀⠴⠋⠀⣠⠞⢡⠟⠁⠀⠀⠀⠀⠀⠀⠀⣠⠞⠁⠀⠀⠀⠀⠀⣀⡴⠊⢁⡴⠊⢁⡏⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠻⣿⣿⣿⣿⣦⡀⣠⣾⣿⡟⠉⠀⢀
⡿⠏⡀⠀⢀⣠⣾⠷⠋⠁⠀⣠⠞⠁⢠⠏⠀⠀⠀⠀⠀⠀⠀⢀⡼⠃⠀⠀⠀⠀⣀⡴⠚⠁⣠⠔⠋⠀⠀⣸⠁⠀⠀⠀⠀⠀⠀⠀⠀⣀⠀⣸⠁⠙⢿⣿⣿⣿⣿⣿⣿⡀⠀⠀⠃
⣁⣠⠵⠞⠛⠉⠀⠀⠀⣠⡾⣡⠄⢠⡟⠀⠀⠀⠀⠀⠀⠀⢠⡞⠀⠀⢀⣠⠴⠚⢁⣤⣶⣋⡁⠀⠀⠀⢀⡇⠀⠀⠀⠀⠀⠀⠀⠀⡼⠁⢠⡇⠀⠀⠀⠙⢿⣿⣿⣿⣿⣷⣦⡤⠂
⠁⠀⠀⠀⠀⢀⣠⢴⡾⣫⣾⠃⢠⣿⠁⠀⠀⠀⠀⠀⡀⢠⣏⡠⠴⢚⣉⠤⠖⠛⠁⠀⠀⠀⠈⠙⠳⠦⣼⠁⠀⠀⠀⠀⠀⠀⠀⡜⠁⢠⢿⡇⠀⡜⢠⠀⠀⠙⢿⣿⣿⣿⣿⠀⠀
⠀⠀⠀⠀⢠⠞⢁⣞⡽⢡⡏⠀⡞⡞⠀⠀⠀⠀⠀⠀⢷⣿⠵⠒⣯⡉⠀⢠⡄⠤⠤⠤⣀⣀⠀⠀⠀⠀⡾⠀⠀⠀⠀⠀⠀⠀⠀⠀⣰⡟⢸⠁⠀⠀⡟⢠⢀⠀⡀⠻⣿⣿⣿⣷⣾
⠀⠀⠀⠀⠀⢀⣾⡿⠁⣼⠀⢸⢡⡇⠀⠀⠀⠀⠀⠀⡞⣿⣿⡿⠿⠿⠿⠿⠿⣿⣿⣶⣤⣄⠉⠳⣄⡀⡇⠀⠀⠀⠀⠀⠀⠀⡼⣲⠋⡇⢸⠀⠀⢰⠃⡆⢸⢠⢹⠀⠙⣿⣿⣿⡏
⠀⠀⢀⣠⠴⠛⡿⣧⣴⡏⠀⡏⢸⠇⠀⠀⠀⠀⠀⢸⠁⢹⡇⠀⠀⣠⠖⢻⣟⠲⢮⡙⢿⣿⡗⢆⠘⡅⡇⠀⠀⠀⠀⠀⢀⡞⣰⠇⠀⣷⢸⠀⠀⣾⠀⢧⠈⣿⢸⢠⠀⠸⣿⣿⣿
⣶⣚⣉⠤⠤⠾⣧⠘⣇⣷⠀⠃⢸⡀⠀⠀⠀⠀⠀⣿⠀⠘⡇⠀⡾⠥⠞⠉⠁⠹⣾⣷⠀⠘⢿⡌⠀⢹⡇⠀⠀⠀⠀⠀⡼⢠⠏⠉⠛⢿⣼⠀⢰⣇⠀⠘⠦⣿⠀⢿⠀⠀⢿⣿⣿
⠀⠀⠀⠀⠀⢀⢿⣦⡈⠻⣇⠀⠸⡇⠀⠀⠀⠀⠀⣿⠀⠀⠀⠀⡇⠀⠀⠀⠀⠀⢸⣼⠀⠀⠈⠧⠀⠀⡇⠀⠀⠀⠀⢸⢣⡏⠀⠀⠀⢸⣿⠀⢸⢻⠀⠀⢸⣿⡀⠈⠐⡆⢸⣿⣿
⠀⠀⠀⠀⠀⠁⢈⣯⢳⣄⠘⡄⠀⢷⠀⠀⠀⢸⡀⢹⡀⠀⠀⠀⠹⠤⣀⡀⠀⠀⢀⡟⠀⠀⠀⠀⠀⠀⣷⠀⠀⢰⠂⡏⡾⠀⠀⣀⠀⠀⣿⡇⡾⣾⡄⠀⢸⠃⠃⠀⠀⣧⢸⣿⣿
⠀⠀⠀⠀⠀⣠⠞⢈⣿⡉⠛⣿⣆⠘⣆⠀⠀⠀⢳⡀⢷⡀⠀⠀⠀⠀⠀⠉⠓⠚⠉⠀⠀⠀⠀⠀⠀⠀⢸⡄⠀⢸⣶⣿⠇⣀⡬⢭⣉⠲⣼⣇⡇⠘⡇⢐⡿⠀⠀⠀⢰⡟⢸⣿⣿
⢦⣀⠀⣠⣾⠏⢀⣾⣿⡇⠀⣿⢯⣧⣘⢦⡀⠀⠀⢻⣦⣳⣄⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠘⣧⠀⢸⢻⣿⣴⣾⣿⡿⢿⣷⣌⢿⡇⠀⣧⣼⠁⠀⠀⠀⣼⡇⣸⣿⣿
⡄⠈⠛⠳⠧⣤⣾⠿⠋⠀⠀⡿⠈⢣⡘⣿⠿⠦⣤⣬⡄⠈⠙⠛⠛⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢹⡆⢿⠈⣿⡯⠿⠳⣝⣦⠙⣿⡾⠃⠀⣿⠃⠀⠀⠀⢠⡿⢡⣿⣿⣿
⠛⠷⣤⡀⠀⠀⠀⠀⠀⠀⣰⠇⡇⠀⢳⡘⢦⠀⠀⠀⣧⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢳⣸⡾⠁⠱⠄⠀⢻⣿⠀⠸⣿⣖⡾⠃⠀⠀⠀⠀⡼⢣⣾⣿⣿⣿
⠀⠀⠈⠛⢶⣤⣤⣤⠤⢾⣿⡄⢡⠀⠀⠙⣎⢳⡀⠀⢸⡄⠀⠀⠀⠀⢀⣀⣀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢻⣿⣄⠀⢀⡐⢻⡟⠀⢠⣿⡿⠷⠄⠀⢀⣠⢞⣵⠿⣿⣟⢻⣿
⣦⡀⠀⠀⠀⢻⣿⣦⣴⠋⡏⣇⠘⢧⡀⠀⣏⣷⣽⣦⣄⠹⣄⠀⠀⢀⡟⠃⠈⠉⠹⠗⠢⣄⡀⠀⠀⠀⠀⠀⠀⠀⠙⠻⠷⢤⠤⠊⢀⣴⡿⠋⢀⡼⠒⠚⠿⠚⢋⡇⠘⣿⠻⣿⣿
⢹⣿⣦⡀⠀⠀⠹⣿⣿⠀⠹⣼⡄⠈⠛⡄⢹⣼⡈⢿⠈⠙⠛⠓⠒⠘⡇⠀⠀⠀⠀⠀⠈⠲⡍⠳⢄⡀⠀⠀⠀⠀⠀⠀⠀⠠⢄⣤⣖⣋⣤⢶⡿⠁⠀⠀⢀⡾⣸⠀⠀⣏⠀⠙⣟
⠀⠈⢿⡷⣄⠀⠀⢿⣿⣧⠀⠹⣧⠀⠀⠃⠆⢿⣧⠈⠀⠀⠀⠀⠀⠀⢣⠀⠀⠀⠀⠀⠀⠀⠈⠆⠀⠹⡄⠀⠀⠀⠀⠀⠀⠀⠀⣰⢿⣟⡵⠋⠀⠀⠀⢀⣾⠁⡟⠀⠀⣿⡀⠀⢈
⠀⢀⣽⣧⠈⢷⡀⠸⡎⢻⣷⡀⠘⢧⡀⠀⠀⠸⣿⣆⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⡼⠁⠀⠀⠀⢠⣀⠀⢀⣾⣿⠿⠋⠀⠀⠀⠀⣠⣟⣽⣸⠁⠀⠀⣿⡇⠀⠀
⡁⠊⢿⣿⠀⠀⠙⡄⢷⠀⠙⣿⣦⡈⠛⣦⡀⠀⠹⣿⢧⡀⠀⠀⠀⠀⠀⠀⠀⠀⠠⣄⣀⣠⠤⠚⠉⠀⠀⠀⠀⠀⠀⠙⣿⣭⡉⠀⠀⠀⢀⣠⣴⣿⣿⣿⣿⡏⠀⠀⢸⣿⡇⠀⠐
⣶⣿⣿⣿⡆⠀⠀⡀⢸⡆⠀⠈⢻⣿⣦⡀⠙⢦⣀⢻⡀⠹⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣠⢾⠁⢸⢹⠙⠛⢉⠹⣿⣿⣿⣿⣿⣿⣆⠀⠀⣾⡇⢹⠀⠀
⣿⣿⣿⣿⡇⠀⠀⣷⠀⡇⠀⢀⡀⢿⣿⣿⣦⡀⠙⠻⣿⣆⠈⢷⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣀⣀⣤⠴⠚⠋⠀⢸⠀⣿⠘⡆⠀⢠⢸⣿⢿⣿⣿⣿⠿⣿⣆⣾⡟⠀⢸⠀⡇
⣿⣿⣿⣿⡇⠀⠀⡟⠀⣷⠀⡾⠀⠘⣿⣿⣿⣷⡄⠀⠈⠻⣧⡀⣹⡷⢤⣀⣤⡤⠶⠶⠞⠛⠋⠉⠉⠁⢀⠀⠀⠀⠀⢸⠀⣿⠀⢱⡀⣘⢸⢘⣿⣿⣿⡋⠄⠿⢿⣯⠀⠀⣼⡐⢳
⣿⣿⣿⣿⡇⠀⢀⡇⠀⢺⠀⡇⠀⠀⢻⣿⣿⣿⣿⡀⠀⠀⠈⠻⣿⣷⣾⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⡀⠀⠀⠀⢸⠀⡇⠀⠀⢳⡈⢹⡉⠸⡽⣜⣧⠀⠀⠀⠙⢷⡀⠻⠒⠋
⣿⣿⣿⣿⣧⠀⣸⠁⠀⢸⢸⠃⠀⠀⢸⣿⣿⣿⣿⣷⠀⠀⠀⢀⡘⣿⣿⡿⠳⣦⠀⠀⠀⠀⠀⠀⠀⠀⠀⠃⠀⠀⠀⠈⡇⣧⠀⠀⠀⠹⣌⢧⠀⢳⠙⣎⢷⡀⠀⠀⠈⠻⣄⠀⠀
⣿⣿⣿⣿⡿⢀⡏⠀⠀⣼⣾⠀⠀⠀⠈⣿⣿⣿⣿⣿⣇⠀⠀⠀⠙⠞⢞⢿⣖⣿⣷⣤⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢳⢻⡀⠀⠀⠀⠈⠻⣷⣤⣧⠘⢯⣳⣄⠀⠀⠀⠙⢧⡀
 -------------------------------------------------------------------------------------------------*/

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

        scoreH = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pickupHPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickupHPose.getHeading())
                .addPath(new BezierLine(pickupHPose, shootPose))
                .setLinearHeadingInterpolation(pickupHPose.getHeading(), shootPose.getHeading())
                .build();

        runAway = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootPose, leave))
                .setTangentHeadingInterpolation()
                .build();
    }

    private void flightEnergyConservation(ElapsedTime revTime) {
        if (robot.follower.isBusy()) {
            robot.setFlywheelPower(0.13);
        } else if (robot.flywheel.getPower( ) == 0.13 && pathState != -999){
            robot.setFlywheelPower(1);
            revTime.reset();
        }
    }

    private void manageCalories() {
        if (in && (robot.flickState != (Invokation_of_a_False_Life.flickStates.UPWARDS) && (robot.flickState != Invokation_of_a_False_Life.flickStates.DOWNWARDS))) {
            robot.intake.setPower(1);
        } else if (in && (robot.flickState == Invokation_of_a_False_Life.flickStates.DOWNWARDS)) {
            robot.intake.setPower(0.2);
        } else if (in) {
            robot.intake.setPower(0);
        } else {
            robot.intake.setPower(0);
        }
    }

    private void actOnTheGut(ElapsedTime revTime) {
        if (!robot.follower.isBusy()) {
            if (robot.flywheel.getPower() != 1) {
                robot.setFlywheelPower(1);
                revTime.reset();
            }
            intuition = robot.findAprilStarBearing(false);
            if (Math.abs(intuition) > 4) {
                robot.follower.turn(intuition, true);
            }
        }
    } //experimental

    private void shootToKill(ElapsedTime flickerTime, ElapsedTime revTime) {
        switch (robot.flickState) {
            case START:
                if (shotsFired < 4 && shotsFired != 0 && (revTime.seconds() > 0.5)) {
                    flickerTime.reset();
                    robot.flicker.setPosition(0.6); //go up
                    robot.flickState = Invokation_of_a_False_Life.flickStates.UPWARDS;
                } else if (shotsFired <= 0 && (revTime.seconds() > 1)) {
                    flickerTime.reset();
                    robot.flicker.setPosition(0.5); //go up
                    robot.flickState = Invokation_of_a_False_Life.flickStates.UPWARDS;
                }
                break;
            case UPWARDS:
                if (flickerTime.seconds() >= 0.144 && shotsFired > 0 || flickerTime.seconds() >= 0.3 && shotsFired == 0) {
                    flickerTime.reset();
                    robot.flicker.setPosition(0); //go down
                    robot.flickState = Invokation_of_a_False_Life.flickStates.DOWNWARDS;
                }
                break;
            case DOWNWARDS:
                if (flickerTime.seconds() >= 0.075) {
                    flickerTime.reset();
                    shotsFired += 1;
                    robot.flickState = Invokation_of_a_False_Life.flickStates.MOONLIGHT;
                }
                break;
            case MOONLIGHT:
                if ((shotsFired < 2) && (flickerTime.seconds() >= 0.333)) {
                    robot.flickState = Invokation_of_a_False_Life.flickStates.START;
                } else if ((shotsFired >= 2) && (flickerTime.seconds() >= 0.8)) {
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
