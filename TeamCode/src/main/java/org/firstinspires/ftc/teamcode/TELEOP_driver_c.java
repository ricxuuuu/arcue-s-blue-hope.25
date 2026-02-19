package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.internal.camera.CameraState;
import org.firstinspires.ftc.vision.VisionPortal;

//⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⢻⣿⣿⣿⡿⠙⠉⣉⡉⠉⠉⠉⠉⠉⠉⣉⡉⠉⠛⢯⣍⠉⠉⠉⠙⢟⡋⢉⣽⣿⣿⣏⠉⠉⠉⠉⢉⣉⣉⣉⣉⣉⡉⠭⠭⠭⠭
//⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠳⡄⠀⠀⠀⠀⠀⢸⡼⠟⠁⠀⣠⣾⡿⠀⢀⣤⡀⠀⠀⢶⣿⣿⣷⣄⠀⠀⠀⠀⠀⠀⠀⠙⢿⣿⡿⠃⠙⢷⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
//⠀⠀⢀⠀⠀⠠⢤⣀⠀⣶⢹⢀⡇⠀⠀⠀⠀⣠⠞⠀⠀⠀⠘⠿⠋⠀⠀⠋⠀⠉⠀⠀⠀⠈⠛⠛⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⢧⣀⠀⠀⠙⣆⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
//⠀⠀⠀⠀⠢⢄⡀⠈⠉⢙⣾⠮⠍⠙⠓⠲⣦⠏⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠙⣦⠀⠀⠘⣧⠀⠀⠀⠀⠀⠀⠀⣀⣀⣀⣀
//⡀⢐⠐⣲⣤⣀⠉⠓⠂⡞⠀⡰⠚⠙⠓⠲⢼⡦⣀⠀⠀⠈⠛⠲⢤⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⠀⠀⠀⠘⣇⠀⠀⣈⣀⠀⢀⣔⣶⡖⠒
//⠁⠀⡄⠀⠉⠛⠿⣶⢰⠃⣰⠁⠀⠀⣀⡴⢋⣥⠿⢓⣲⣾⠿⢍⣉⠐⣻⡤⢠⡀⠓⠦⣄⣀⠀⠀⠀⠀⠀⠀⠀⣀⡤⠂⠀⠀⠀⠀⠀⢸⡇⠀⠀⠀⢹⢖⣋⠤⠼⠯⡻⣿⡖⠖⢻
//⠭⠥⠷⠶⡆⣀⡀⠀⡞⢠⠇⠀⢠⡾⣋⡔⠉⠀⣠⡾⠋⠀⠀⣠⣾⡫⠕⠻⣼⠹⡤⢀⣀⠈⠉⢯⠓⠒⠒⠲⣾⣳⡶⠶⠒⠲⣄⣀⣤⠞⠶⣄⡀⣴⡿⠋⠀⠀⠀⠀⠙⡞⣧⡀⠸
//⠍⠋⠛⣄⣳⠈⠙⢳⡇⡸⠀⣠⢎⢴⠏⠀⠀⣼⠋⠀⠀⢀⣼⠟⠁⠀⠀⠀⠹⣧⢱⡀⠀⠉⠁⠘⢳⣄⠀⠉⠈⢿⡌⠉⠒⠢⡨⣳⡍⠑⠦⡈⢿⡋⠀⢀⡠⠴⠒⢦⡀⠸⣽⡩⠭
//⠶⠾⠿⠟⠫⣤⣀⣼⡇⡇⣰⣣⢫⠃⠀⢠⡾⠁⠀⠀⣠⠋⠀⠀⠀⠀⠀⠀⠀⠘⢇⠱⡄⠀⠀⠀⠀⠉⢆⠀⠀⠈⢷⠀⠀⠀⠹⡜⢽⡗⠦⡈⠪⣳⣔⠋⠀⣀⣴⠚⠃⡄⢻⣇⠀
//⣉⣈⣨⣷⡄⠀⠉⣻⡿⡽⡱⢁⠇⠀⢠⡟⠁⠀⢀⡜⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠳⡙⢆⠀⠀⠀⠀⠈⢆⠀⠀⠘⣇⠀⠀⠀⢣⠈⢿⡄⠈⠢⡈⡙⢦⡖⠁⠀⠹⡄⠱⡘⣿⠤
//⠶⢆⡲⣿⣦⠾⠷⢾⣷⡳⠁⡎⠀⢀⡿⠁⠀⣀⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠙⢮⡳⠀⡀⠀⠀⠈⢆⠀⠀⢨⡄⠀⠀⠸⠀⠘⣧⡤⠤⠛⢇⡎⣇⠀⢀⠀⡇⠀⢣⢿⡍
//⣄⣈⠇⣻⡟⣾⠀⡼⡱⠁⡸⠀⠀⣼⠃⠀⡴⠃⢀⡤⠄⣄⠀⠀⠀⠀⠀⠀⠀⠀⢀⡴⠚⠓⠛⢦⡙⠳⡄⠀⠈⢆⠀⠀⡆⠀⠀⠀⠀⠀⠸⡄⢀⡤⠞⡁⡿⡄⢸⠀⡇⠀⠘⡜⡇
//⠓⠒⠀⠉⠛⢿⢺⡳⠁⢠⡇⠀⢠⡇⢀⡜⠡⣞⣁⣀⣀⡸⠇⠀⠀⠀⠀⠀⠀⠀⠈⠑⠦⠤⠤⠤⠟⢦⣀⠀⠀⠈⢆⠀⡇⠀⠀⠀⠀⠀⢠⢻⠉⠀⢠⢣⣿⣷⣸⠀⡇⠀⠀⡇⢘
//⡄⠤⠴⠶⠖⣺⢷⢃⠀⣸⠀⠀⣸⢳⠞⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⡀⠀⠀⠀⠀⠙⠳⣤⡀⠘⣦⡇⠀⠀⢸⠀⠀⢸⡼⡶⠔⢁⣾⣻⣿⡿⢸⠁⠀⠀⠀⢸
//⠀⠠⠤⠤⠴⣟⡏⡎⠀⡏⠀⠀⡽⡏⠀⠀⠀⣀⣤⠤⠖⠚⡃⠀⠀⠀⠀⠀⠀⠀⠀⢈⡛⠚⠳⠤⣄⡀⠀⠀⠈⠛⠯⣷⠇⠀⠀⡞⠀⠀⢸⡇⣿⢠⡾⢃⡇⣿⡇⡼⠀⠀⠀⡀⢸
//⠀⠀⠀⠀⢰⢹⢠⠁⠀⢷⠀⠀⣿⢻⢀⡀⣀⣡⣤⣶⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠈⣽⣿⣶⣦⣄⣀⠀⠀⠀⠀⠀⡟⠀⠀⢠⠇⠀⠀⢸⠃⢸⠋⢠⠞⣼⡟⢱⠇⠀⠀⠀⡇⢸
//⡲⣄⡀⠀⡟⡇⠀⠀⠀⢸⡂⢰⢸⡞⡟⠛⣿⣿⡿⠿⠛⠉⠁⠀⠲⠀⠀⠀⠀⠀⠀⠀⠙⠻⠿⢿⣿⣿⣿⣷⣦⡄⢠⠀⠀⠀⠈⠀⠀⠀⡎⠀⢸⡒⠚⠚⠛⠓⠛⠶⠦⢤⣈⣁⢸
//⠉⠳⢭⡳⡇⡇⢠⠀⠀⠀⣧⠈⣧⢻⡇⢀⠌⡅⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠉⠛⠃⠀⣠⠃⠀⠀⢀⡀⠀⠀⡸⠁⡄⢸⠣⠤⠖⠒⠒⠒⠦⠤⡄⠀⠉⠙
//⠀⠀⠰⢯⣇⣧⢸⡄⢠⠀⠈⢦⡈⢿⣄⠈⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡴⠃⣰⠀⢀⠞⢠⠀⡴⠃⣸⠀⢸⠀⠀⠀⠀⠀⢀⡠⠊⠁⠀⠀⠉
//⠀⠀⣠⢞⣿⣜⣤⢷⣸⠳⣄⠈⠻⣖⠙⠛⠃⠀⠀⠀⠀⠀⠀⠀⢠⣦⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣞⡴⣺⢃⣴⡏⣠⢏⡴⣡⢶⠇⠀⡟⠛⠓⠆⠐⠒⠁⠀⠀⠀⣀⣠⠴
//⠒⠞⣟⣛⠓⠚⠿⠬⣿⡇⠈⠙⡖⣾⠃⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣀⣑⣋⣾⠾⠷⠾⣏⣱⣏⢀⣼⣁⡀⠀⣴⠒⣲⡤⣴⠒⠋⠁⠀⠀
//⡛⠵⢖⣦⢭⣑⠢⠤⣀⡈⠙⠒⠾⣿⣷⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⡠⠤⠔⣒⣛⡭⠉⢁⣀⡠⠤⠤⠐⢒⣒⣋⠭⠭⣿⣗⣻⣭⣭⣿⡽⡄⡇⠀⠀⠀⠀⠀
//⡝⠛⠶⣮⣭⣓⡫⢕⣲⠭⡑⢢⣤⣀⠉⠛⠷⣄⣀⣀⣤⣀⣀⣀⣀⣀⣀⣀⡤⠖⣛⡩⠴⣒⡪⠭⠔⢒⣋⡉⠥⣤⣒⣲⡭⢽⣗⣒⣾⡯⢽⣿⠛⠿⡛⢿⣿⡏⣿⡇⠀⠀⠀⢠⠀
//⢤⢌⣀⢀⣜⠙⡻⡿⢾⣭⣟⡲⠭⣟⠟⡂⠀⢮⡙⢦⣠⡇⣯⣿⣉⡿⢋⠥⠖⡩⠔⡂⠭⣔⣒⡮⣽⣗⣲⡿⢽⡿⠲⢟⠙⠛⡏⣠⣀⢌⠉⢀⡀⠀⢉⢸⣿⣭⡽⠃⠀⠀⠀⡞⠀
//⡇⢀⠀⠈⠀⠀⠀⠀⢥⡂⠉⠛⠿⣷⣶⣍⣽⣂⣷⠤⢥⣤⠴⠶⣓⣤⣭⣭⣖⣻⣭⣭⠿⠷⠛⡋⢋⠅⠉⠑⠖⠉⠛⠀⠀⠀⠈⠉⠀⠀⠑⠁⠀⡀⢠⣞⡟⠛⢷⣄⠀⠀⣰⠃⠀
//⣅⠀⠁⢀⠠⠀⠀⠀⠀⠀⠢⡀⠀⢹⣿⣿⣛⠿⠿⢯⣭⣭⣿⠿⠿⠻⣿⣿⡟⣩⠁⠀⠀⠐⣈⢄⣶⢠⡄⠀⠀⠀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠸⣿⡧⣄⣈⠻⣷⣤⡟⠀⠀
//⡇⢀⠀⠀⢅⢀⠀⠐⠀⠀⠀⠓⠀⠤⠋⢹⡿⠃⡉⠉⠀⠁⠈⣧⡐⠀⢿⣿⡇⠅⠠⠆⠞⠉⠁⠀⠘⢂⠁⠀⢐⠀⡡⢄⠀⣄⣀⠀⠀⠀⠠⠀⠀⢠⡇⣿⣏⠙⠚⠿⣾⣿⣧⡀⠀
//⡇⠀⠀⠄⣨⣆⠢⠀⢀⠀⠀⠀⠀⠀⢄⣸⡗⡲⠤⠁⣀⠀⠄⠠⠀⡀⢰⠿⣇⠀⠀⡆⠀⠠⣄⣀⠀⠀⠈⠆⠰⡁⠊⢀⣄⢱⠆⠰⠀⠈⠁⠠⣶⡆⣧⣿⣏⡉⠒⢤⣄⡙⢛⣿⣤
//⢶⣄⠀⠀⠀⠈⠀⢧⡀⠒⡐⠄⠠⡆⡟⣿⡇⠃⠀⠀⠄⠀⠀⠀⠀⠈⢸⠰⣿⣦⡄⠃⠀⠀⠀⠀⡀⠀⠀⢁⠂⠀⠉⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⣇⣿⣿⣟⡉⠘⠳⢾⡻⢶⣍⣬
//⠀⣽⠀⠀⠀⠀⠀⠀⠀⠁⠀⠀⠀⠳⣧⣾⡆⠀⠏⠀⠀⡀⠀⢀⠀⠀⢸⣤⣿⡇⠀⠀⠀⠀⠃⠈⠀⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⡾⢋⣄⠀⠀⠙⣶⡄⠈⣟⢆⢻⣿
//⢀⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⢸⣿⣇⢄⠀⠘⠦⣀⠀⠀⠀⠀⠈⣿⢻⠃⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠹⣕⠋⠒⣉⣀⠀⡿⣷⠀⢹⠘⡆⣯
//⢿⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡈⣿⣿⣘⡖⠶⠒⠶⠦⠔⠂⣶⠰⣿⢸⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢽⣦⡬⢤⣒⣪⣇⣇⣴⣻⠦⠿⣾
//⣿⣿⣷⣄⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀⣇⣝⣿⣇⣷⣀⣀⣐⣈⣂⣁⣸⣀⣿⣍⣀⣄⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀⣾⣛⣛⣉⣩⣭⣥⣶⣒⣒⣚⣋⣲⣶⣐
// hopefully one day we may soar the skies much like the birds of flight

@TeleOp(name = "PURPLE // * TELEOP", group = "Linear Op-mode")
public class TELEOP_driver_c extends LinearOpMode {

    private final Invokation_of_a_False_Life robot = new Invokation_of_a_False_Life();
    //|||||||||||||||||||||||||||||||// ✧ >.<  //summon a false life to do our bidding

    @Override
    public void runOpMode() {

        //||||||||||||||||||||||||||||||||||||||//
        telemetry.addData("01/", "NOTICE ///////////// >.<");
        telemetry.addData("02/", "#3.scratch is not yet initialized.");
        telemetry.update();
        //||||||||||||||||||||||||||||||||||||||//
        //-----------------------------------------------PREP
        robot.init(hardwareMap, false);

        boolean fly = false;
        boolean in = false;
        boolean out = false;
        boolean ultima_ratio = false;
        boolean is_blue_alliance = false;
        boolean turningB = false;
        boolean turningR = false;
        boolean follower_control;

        //start up a timer for flicker use
        ElapsedTime flickerTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        //-----------------------------------------------PREP
        //||||||||||||||||||||||||||||||||||||||//
        telemetry.addData("01/", "#Configuring Camera...");
        telemetry.update();
        //||||||||||||||||||||||||||||||||||||||//
        robot.traumatizeCamera(7, 177);
        //-----------------------------------------------PREP
        //||||||||||||||||||||||||||||||||||||||//
        telemetry.addData("01/", "ごめん あまない 俺は今お前のために怒ってない");
        telemetry.addData("02/", "誰も憎んじゃいない");
        telemetry.addData("03/", "今はただただ この世界が心地いい");
        telemetry.addData("04/", "天上天下 唯我独尊");
        telemetry.addData("05/", "代々伝わる送電の術式のメリットは取説があること");
        telemetry.addData("06/", "デメリットは術式の情報が漏れやすいこと");
        telemetry.addData("07/", "あんた全員家の人間だろう");
        telemetry.addData("08/", "無限呪術のことはよく知ってるわけだない");
        telemetry.addData("09/", "だがこれは 五条家の中でもごく一部の人間しか知ら");
        telemetry.addData("10/", "順転と反転");
        telemetry.addData("11/", "それぞれの無限を衝突させることで生成される仮想の質量を押し出す");
        telemetry.addData("12/", "虚式 「茈」");
        telemetry.addData("13/", "🔵 + 🔴 = (🟣)");
        telemetry.update();
        waitForStart();
        //||||||||||||||||||||||||||||||||||||||//

        robot.rechain_motion();

        //------------------------------------------------------------------------------------------
        while (opModeIsActive()) {


            //-----------------------------------------------DRIVETRAIN
            if (gamepad1.leftBumperWasPressed()) {
                turningB = true;
            } else if (gamepad1.rightBumperWasPressed()) {
                turningR = true;
            }
            if (gamepad1.leftBumperWasReleased()) {
                turningB = false;
            } else if (gamepad1.rightBumperWasReleased()) {
                turningR = false;
            }
            follower_control = (gamepad1.left_trigger > 0.13 ^ gamepad1.right_trigger > 0.13 ^ turningB ^ turningR);

            if (!follower_control) {
                //---------------------------------MANUAL CONTROL
                double axial = -gamepad1.left_stick_y; // forward/back
                double lateral = gamepad1.left_stick_x; // strafe
                double yaw = gamepad1.right_stick_x; // turn

                //drivetrain joystick movement
                robot.drive(axial, lateral, yaw);
                //---------------------------------MANUAL CONTROL
            } else {
                //---------------------------------BOT HOLD ADJ GOAL
                if (gamepad1.left_trigger > 0.13 && gamepad1.right_trigger < 0.13) {
                    is_blue_alliance = true;
                    robot.follower.turnTo(robot.findGoalHeading(is_blue_alliance));
                }
                if (gamepad1.right_trigger > 0.13 && gamepad1.left_trigger < 0.13) {
                    is_blue_alliance = false;
                    robot.follower.turnTo(robot.findGoalHeading(is_blue_alliance));
                }
                if (turningB && gamepad1.left_trigger < 0.13 && gamepad1.right_trigger < 0.13) {
                    is_blue_alliance = true;
                    robot.follower.turn(robot.findAprilStarBearing(false), true);
                } else if (turningR && gamepad1.left_trigger < 0.13 && gamepad1.right_trigger < 0.13) {
                    is_blue_alliance = false;
                    robot.follower.turn(robot.findAprilStarBearing(true), true);
                }
                robot.follower.update();
                //---------------------------------BOT HOLD ADJ GOAL
            }
            //-----------------------------------------------DRIVETRAIN



            //-----------------------------------------------PARTIAL_IDEALIZE
            if (!ultima_ratio) {
                robot.idealizeFlightSpeed(is_blue_alliance);
                robot.idealizeHoodState(is_blue_alliance);
                gamepad1.stopRumble();
            } else {
                if (gamepad1.dpadLeftWasPressed()) {
                    robot.hoodState = Invokation_of_a_False_Life.hoodStates.LUNAR_L1_NEAR;
                } else if (gamepad1.dpadUpWasPressed()) {
                    robot.hoodState = Invokation_of_a_False_Life.hoodStates.LUNAR_L3_MID;
                } else if (gamepad1.dpadRightWasPressed()) {
                    robot.hoodState = Invokation_of_a_False_Life.hoodStates.LUNAR_L2_FAR;
                }
                robot.where_are_those_who_share_the_memory();
            }
            //-----------------------------------------------PARTIAL_IDEALIZE



            //-----------------------------------------------INTAKE/FLY/STUFF
            //set status
            if(gamepad1.shareWasPressed()) {
                ultima_ratio = !ultima_ratio;
            }
            if (gamepad1.yWasPressed()) {
                fly = !fly;
            }
            if (gamepad1.bWasPressed()) {
                in = !in;
                out = false;
            }
            if (gamepad1.aWasPressed()) {
                out = !out;
                in = false;
            }
            if (gamepad1.rightStickButtonWasPressed() || gamepad1.leftStickButtonWasPressed()) {
                in = true;
                out = false;
                robot.depression = true;
                robot.dreams = 3;
            }
            if (gamepad1.psWasPressed()) {
                if (is_blue_alliance) {
                    robot.pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 64.7, 63.1, AngleUnit.DEGREES, -90));
                } else {
                    robot.pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 64.7, -63.1, AngleUnit.DEGREES, 90));
                }
                robot.visionPortal.stopStreaming();
                robot.just_a_cog = true;
                gamepad1.rumble(0.7, 0.7, 333);
            }
            if (gamepad1.optionsWasPressed()) {
                robot.the_light_at_the_end_of_the_tunnel = !robot.the_light_at_the_end_of_the_tunnel;
            }
            if (gamepad1.touchpadWasPressed()) {
                if (robot.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                    robot.visionPortal.resumeStreaming();
                    gamepad1.rumble(0.43, 0.13, 2222);
                    robot.just_a_cog = false;
                }
            }

            //control based on status
            if (fly) {
                robot.setFlywheelSpeed(robot.dream_of_flight);
            } else {
                robot.setFlywheelPower(0);
            }
            if (in && (robot.flickState != (Invokation_of_a_False_Life.flickStates.UPWARDS) && (robot.flickState != Invokation_of_a_False_Life.flickStates.DOWNWARDS))) {
                robot.intake.setPower(1);
            } else if (in && (robot.flickState == Invokation_of_a_False_Life.flickStates.UPWARDS)) {
                robot.intake.setPower(-0.1);
            } else if (out) {
                robot.intake.setPower(-1);
            } else {
                robot.intake.setPower(0);
            }
            //-----------------------------------------------INTAKE/FLY/STUFF



            //-----------------------------------------------FLICKER FSM
            if (!robot.depression) {
                switch (robot.flickState) {
                    case START:
                        if (gamepad1.xWasPressed() && flickerTime.seconds() >= 0.2) {
                            flickerTime.reset();
                            robot.flicker.setPosition(0.81); //go up
                            robot.flickState = Invokation_of_a_False_Life.flickStates.UPWARDS;
                        }
                        break;
                    case UPWARDS:
                        if (flickerTime.seconds() >= 0.135 ) {
                            flickerTime.reset();
                            robot.flicker.setPosition(0); //go down
                            robot.flickState = Invokation_of_a_False_Life.flickStates.DOWNWARDS;
                        }
                        break;
                    case DOWNWARDS:
                        if (flickerTime.seconds() >= 0.075) {
                            flickerTime.reset();
                            robot.flickState = Invokation_of_a_False_Life.flickStates.START;
                        }
                        break;
                    default:
                        robot.flickState = Invokation_of_a_False_Life.flickStates.START;
                }
            }

            //restart if button is re-pressed
            //if (gamepad1.xWasPressed() && (robot.flickState != Invokation_of_a_False_Life.flickStates.START || robot.depression)) {
            //reset for triple shot of dreams
            if (robot.depression) {
                robot.flickState = Invokation_of_a_False_Life.flickStates.START;
                robot.flicker.setPosition(0);
                robot.depression = false;
                robot.dreams = 0;
            }
            //-----------------------------------------------FLICKER FSM



            //-----------------------------------------------DRIVER FEEDBACK
            if (ultima_ratio) {
                if (!gamepad1.isRumbling()) {
                    gamepad1.rumble(0.2, 0.2, 1333);
                }
            }
            if (robot.the_light_at_the_end_of_the_tunnel) {
                gamepad1.setLedColor(255,255,255, 133);
            } else {
                if (is_blue_alliance) {
                    gamepad1.setLedColor(38, 103, 255, 133);
                }
                if (!is_blue_alliance) {
                    gamepad1.setLedColor(255, 67, 101, 133);
                }
            }
            if (robot.flickState == Invokation_of_a_False_Life.flickStates.UPWARDS) {
                gamepad1.rumble(0.33,0.33,135);
            }
            //-----------------------------------------------DRIVER FEEDBACK



            //-----------------------------------------------UPDATES
            robot.idealizeDecimation();
            robot.runAprilAstroNavigation();
            robot.pinpoint.update();
            robot.follower.updatePose();
            robot.updHoodPos(robot.hoodState, is_blue_alliance);
            robot.losing_dreams(flickerTime);
            //-----------------------------------------------UPDATES


            //-----------------------------------------------TELEMETRY
            telemetry.addData("----------------˖/ᐠ˵- ⩊ -˵マ----//", "---*");
            telemetry.addData(">>> || sometimes I get a craving for fruit that makes me reminisce", "what~*?");
            telemetry.addData("| FL/FR/BL/BR DriveTrain-PWR", "%.2f / %.2f / %.2f / %.2f", robot.frontLeft.getPower(), robot.frontRight.getPower(), robot.backLeft.getPower(), robot.backRight.getPower());
            telemetry.addData("| FLY > RPM/90D IA/STR IA + INT IA", "%.2f / %.2f / %.2f + %.2f", robot.getFlywheelRPM(), robot.flywheel.getCurrent(CurrentUnit.AMPS), robot.flywheel2.getCurrent(CurrentUnit.AMPS), robot.intake.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("| PPT > IMU-H-R/IMU-H-D/X.Pos/Y.Pos", "%.1f/ %.1f/ %.1f/ %.1f", robot.pinpoint.getHeading(AngleUnit.RADIANS), robot.pinpoint.getHeading(AngleUnit.DEGREES), robot.pinpoint.getPosX(DistanceUnit.INCH), robot.pinpoint.getPosY(DistanceUnit.INCH));
            telemetry.addData("| FLCKR > POS/STATE, HOOD > POS", "%.1f / %s, %.1f", robot.flicker.getPosition(), robot.flickState, robot.hood.getPosition());
            telemetry.addData("| VISION > SEEN/DECIMATION/FPS", "%d / %d / %.1f", robot.aprilTPR.getDetections().size(), robot.currentDecimation, robot.visionPortal.getFps());
            telemetry.addData(">>> || I was wrong. You're not greedy... You're bat-shit insane!", "omelettes!");
            telemetry.addData("| ALLIANCE / HYPT FROM / GOAL ∠D / RAW ∠D", "%s / %.1f / %.1f / %.1f", is_blue_alliance ? "BLUE" : "RED", robot.findGoalDistance(is_blue_alliance), robot.findGoalHeading(is_blue_alliance), robot.hallucination);
            telemetry.update();
            //-----------------------------------------------TELEMETRY
        }
    }
}
