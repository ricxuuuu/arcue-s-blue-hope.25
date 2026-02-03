package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

        //-----------------------------------------------PREP
        robot.init(hardwareMap);
        robot.localizeViaApril();

        boolean fly = false;
        boolean in = false;
        boolean out = false;
        boolean follower_control;
        boolean ultima_ratio = false;
        boolean is_blue_alliance = false;

        //start up a timer for flicker use
        ElapsedTime flickerTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
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
        telemetry.addData("13/", "🟣 + 🟣 = (🟣)");
        telemetry.update();
        waitForStart();
        //||||||||||||||||||||||||||||||||||||||//

        //------------------------------------------------------------------------------------------
        while (opModeIsActive()) {
            robot.follower.updatePose();
            follower_control = gamepad1.left_trigger > 0.13 ^ gamepad1.right_trigger > 0.13;


            //-----------------------------------------------DRIVETRAIN
            if (!follower_control) {

                //-----------------------------------------------MANUAL CONTROL
                double axial = -gamepad1.left_stick_y; // forward/back
                double lateral = gamepad1.left_stick_x; // strafe
                double yaw = gamepad1.right_stick_x; // turn

                //drivetrain joystick movement
                robot.drive(axial, lateral, yaw);
                //-----------------------------------------------MANUAL CONTROL

            } else {

                //-----------------------------------------------BOT HOLD ADJ GOAL
                if (gamepad1.left_trigger > 0.13 && gamepad1.right_trigger < 0.13) {
                    is_blue_alliance = true;
                    robot.follower.turnTo(robot.findIdealGoalAngle(is_blue_alliance));

                }
                if (gamepad1.right_trigger > 0.13 && gamepad1.left_trigger < 0.13) {
                    is_blue_alliance = false;
                    robot.follower.turnTo(robot.findIdealGoalAngle(is_blue_alliance));
                }
                robot.follower.update();
                //-----------------------------------------------BOT HOLD ADJ GOAL
            }
            //-----------------------------------------------DRIVETRAIN


            //-----------------------------------------------THE LAST RESORT
            if (!ultima_ratio) {
                gamepad1.stopRumble();
                robot.findIdealFlightSpeed(is_blue_alliance);
                robot.setIdealHoodState(is_blue_alliance);
            } else {
                if (!gamepad1.isRumbling()) {
                    gamepad1.rumble(0.2, 0.2, 1333);
                }
            }
            //-----------------------------------------------THE LAST RESORT


            //-----------------------------------------------INTAKE/FLY
            //player manual control of variables
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

            if (gamepad1.dpadLeftWasPressed() && ultima_ratio) {
                robot.hoodState = Invokation_of_a_False_Life.hoodStates.NEAR;
            } else if (gamepad1.dpadDownWasPressed()) {
                robot.hoodState = Invokation_of_a_False_Life.hoodStates.RASPBERRY;
            } else if (gamepad1.dpadUpWasPressed()) {
                robot.hoodState = Invokation_of_a_False_Life.hoodStates.MID;
            } else if (gamepad1.dpadRightWasPressed()) {
                robot.hoodState = Invokation_of_a_False_Life.hoodStates.FAR;
            }

            //control based off variables
            if (fly && ultima_ratio) {
                robot.setFlywheelPower(1);
            } else if (fly) {
                robot.setFlywheelPower(robot.dream_of_flight);
            } else {
                robot.setFlywheelPower(0);
            }
            if (in) {
                robot.intake.setPower(1);
            } else if (out) {
                robot.intake.setPower(-1);
            } else {
                robot.intake.setPower(0);
            }

            //-----------------------------------------------INTAKE/FLY


            //-----------------------------------------------FLICK SERVO FSM
            switch (robot.flickState) {
                case START:
                    if (gamepad1.xWasPressed()) {
                        flickerTime.reset();
                        robot.flicker.setPosition(0.66); //go up
                        robot.flickState = Invokation_of_a_False_Life.flickStates.UPWARDS;
                    }
                    break;
                case UPWARDS:
                    if (flickerTime.seconds() >= 0.133) {
                        flickerTime.reset();
                        robot.flicker.setPosition(0); //go down
                        robot.flickState = Invokation_of_a_False_Life.flickStates.DOWNWARDS;
                    }
                    break;
                case DOWNWARDS:
                    if (flickerTime.seconds() >= 0.133) {
                        robot.flickState = Invokation_of_a_False_Life.flickStates.START;
                    }
                    break;
                default:
                    robot.flickState = Invokation_of_a_False_Life.flickStates.START;
            }

            //restart if button is re-pressed
            if (gamepad1.xWasPressed() && robot.flickState != Invokation_of_a_False_Life.flickStates.START) {
                robot.flickState = Invokation_of_a_False_Life.flickStates.START;
                robot.flicker.setPosition(0);
            }
            //-----------------------------------------------FLICK SERVO FSM


            //-----------------------------------------------UPDATES
            robot.setHoodPos(robot.hoodState, is_blue_alliance);
            robot.adjustDecimation();
            robot.localizeViaApril();
            robot.pinpoint.update();
            //-----------------------------------------------UPDATES


            //-----------------------------------------------TELEMETRY
            telemetry.addData("----------------˖/ᐠ˵- ⩊ -˵マ----//", "---*");
            telemetry.addData(">>> || sometimes I get a craving for fruit that makes me reminisce", "what~*?");
            telemetry.addData("| FL/FR/BL/BR DriveTrain-PWR", "%2f / %2f / %2f / %2f", robot.frontLeft.getPower(), robot.frontRight.getPower(), robot.backLeft.getPower(), robot.backRight.getPower());
            telemetry.addData("| FLY > RPM/90D IA/STR IA + INT IA", "%2f / %2f / %2f + %2f", robot.getFlywheelRPM(), robot.flywheel.getCurrent(CurrentUnit.AMPS), robot.flywheel2.getCurrent(CurrentUnit.AMPS), robot.intake.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("| PPT > IMU-H-R/IMU-H-D/X.Pos/Y.Pos", "%1f/ %1f/ %1f/ %1f", robot.pinpoint.getHeading(AngleUnit.RADIANS), robot.pinpoint.getHeading(AngleUnit.DEGREES), robot.pinpoint.getPosX(DistanceUnit.INCH), robot.pinpoint.getPosY(DistanceUnit.INCH));
            telemetry.addData("| FLCKR > POS/STATE, HOOD > POS", "%1f / %s, $1f", robot.flicker.getPosition(), robot.flickState, robot.hood.getPosition());
            telemetry.addData("| VISION > SEEN / DECIMATION / FPS", "%d / %d / $f", robot.aprilTPR.getDetections().size(), robot.currentDecimation, robot.visionPortal.getFps());
            telemetry.addData(">>> || I was wrong. You're not greedy... You're bat-shit insane!", "omelettes!");
            telemetry.addData("| ALLIANCE / HYPT FROM / GOAL ∠D / RAW ∠D", "%s / %1f / %1f / %1f", is_blue_alliance ? "BLUE" : "RED", robot.findHypotenuseFromGoal(is_blue_alliance), robot.findIdealGoalAngle(is_blue_alliance), robot.hallucination);
            telemetry.update();
            //-----------------------------------------------TELEMETRY
        }
    }
}
