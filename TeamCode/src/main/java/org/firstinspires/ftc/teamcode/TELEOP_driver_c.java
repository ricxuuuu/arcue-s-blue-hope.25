package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

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


@TeleOp(name = "PURPLE // * TELEOP driver-c", group = "Linear Op-mode")
public class TELEOP_driver_c extends LinearOpMode {

    private final Invokation_of_a_False_Life robot = new Invokation_of_a_False_Life();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap); // Initialize robot hardware
        boolean is_blue_alliance = false; //to know which alliance you are on for auto speed/fly

        //-----------------------------------------------INTAKE/FLY PREP
        boolean fly = false;
        boolean in = false;
        boolean out = false;
        boolean follower_control;
        boolean ultima_ratio = false;

        //start up a timer for flicker use
        ElapsedTime flickerTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        //-----------------------------------------------INTAKE/FLY PREP

        //get ready to start
        robot.pinpoint.setHeading(0, AngleUnit.RADIANS);
        robot.localizeViaApril();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        // runs until the stop button is pressed ---------------------------------------------------
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

            if (!ultima_ratio) {
                gamepad1.stopRumble();
                robot.findIdealFlightSpeed(is_blue_alliance);
                robot.setIdealHoodState(is_blue_alliance);
            } else {
                if (!gamepad1.isRumbling()) {
                    gamepad1.rumble(0.2, 0.2, 1333);
                }
            }

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

            if (gamepad1.dpadLeftWasPressed()) {
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
                        robot.flicker.setPosition(1); //go up
                        robot.flickState = Invokation_of_a_False_Life.flickStates.UPWARDS;
                    }
                    break;
                case UPWARDS:
                    if (flickerTime.seconds() >= 0.2) {
                        flickerTime.reset();
                        robot.flicker.setPosition(0); //go down
                        robot.flickState = Invokation_of_a_False_Life.flickStates.DOWNWARDS;
                    }
                    break;
                case DOWNWARDS:
                    if (flickerTime.seconds() >= 0.2) {
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
            robot.setHoodPos(robot.hoodState);
            robot.localizeViaApril();
            robot.pinpoint.update();
            //-----------------------------------------------UPDATES


            //-----------------------------------------------TELEMETRY
            telemetry.addData("---------------------------//", "---*");
            telemetry.addData("sometimes I get a craving for fruit that", "what~*");
            telemetry.addData("FL_DT-Power", robot.frontLeft.getPower());
            telemetry.addData("FR_DT-Power", robot.frontRight.getPower());
            telemetry.addData("BL_DT-Power", robot.backLeft.getPower());
            telemetry.addData("BR_DT-Power", robot.backRight.getPower());
            telemetry.addData("RA_FLY-RPM", robot.getFlywheelRPM());
            telemetry.addData("RA_FLY-Current", robot.flywheel.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("STR_FLY-Current", robot.flywheel2.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("INT-Current", robot.intake.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("PPT_IMU-Heading, FTC-COORD-R", robot.pinpoint.getHeading(AngleUnit.RADIANS));
            telemetry.addData("PPT_IMU-heading, FTC-COORD-D", robot.pinpoint.getHeading(AngleUnit.DEGREES));
            telemetry.addData("follower-heading", robot.follower.getHeading());
            telemetry.addData("PPT-X.pos, FTC-COORD", robot.pinpoint.getPosX(DistanceUnit.INCH));
            telemetry.addData("PPT-Y.pos, FTC-COORD", robot.pinpoint.getPosY(DistanceUnit.INCH));
            telemetry.addData("FLICKER-POS", robot.flicker.getPosition());
            telemetry.addData("FLICKER-STATE", robot.flickState);
            telemetry.addData("HOOD-POS", robot.hood.getPosition());
            telemetry.addData("ALLIANCE BLUE?", is_blue_alliance);
            telemetry.addData("HYPT-FROM", robot.findHypotenuseFromGoal(is_blue_alliance));
            telemetry.addData("G-Goal Angle", robot.findIdealGoalAngle(is_blue_alliance));
            telemetry.addData("RAW-g_angle", robot.hallucination);
            telemetry.update();
            //-----------------------------------------------TELEMETRY
        }
    }
}
