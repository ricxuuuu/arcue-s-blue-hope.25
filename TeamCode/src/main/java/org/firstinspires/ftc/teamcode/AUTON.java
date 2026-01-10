//meant to be used with bot facing goal

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "RED // * Auton", group = "Linear Op-mode")
public class AUTON extends LinearOpMode {
    private final Invokation_of_a_False_Life robot = new Invokation_of_a_False_Life();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap); // Initialize robot hardware

        ElapsedTime this_will_do_something = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        ElapsedTime flickerTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        boolean imaginary_x_button = false;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        this_will_do_something.reset();
        robot.flickState = Invokation_of_a_False_Life.flickStates.START;

        while(opModeIsActive()) {
            if (this_will_do_something.seconds() <= 0.5) {
                robot.setFlywheelPower(1);
                robot.intake.setPower(1);
                robot.setHoodPos(Invokation_of_a_False_Life.hoodStates.NEAR);
                robot.drive(-1,0, 0);
            }
            if (this_will_do_something.seconds() >= 0.5 && this_will_do_something.seconds() <= 2.5) {
                robot.drive(0,0,0);
            }
            if (this_will_do_something.seconds() >= 1.5 && this_will_do_something.seconds() <= 1.8) {
                imaginary_x_button = true;
            }
            if (this_will_do_something.seconds() >= 1.8 && this_will_do_something.seconds() <= 4.8) {
                imaginary_x_button = false;
            }
            if (this_will_do_something.seconds() >= 4.8 && this_will_do_something.seconds() <= 5.1) {
                imaginary_x_button = true;
            }
            if (this_will_do_something.seconds() >= 5.1 && this_will_do_something.seconds() <= 8.1) {
                imaginary_x_button = false;
            }
            if (this_will_do_something.seconds() >= 8.1 && this_will_do_something.seconds() <= 8.4) {
                imaginary_x_button = true;
            }
            if (this_will_do_something.seconds() >= 8.4 && this_will_do_something.seconds() <= 9) {
                imaginary_x_button = false;
                robot.intake.setPower(0);
                robot.setFlywheelPower(0);
                robot.setHoodPos(Invokation_of_a_False_Life.hoodStates.MID);
            }
            if (this_will_do_something.seconds() >= 9 && this_will_do_something.seconds() <= 12) {
                robot.drive(0,1,0);
            }
            if (this_will_do_something.seconds() >= 12) {
                robot.drive(0,0,0);
            }


            switch (robot.flickState) {
                case START:
                    if (imaginary_x_button) {
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

            robot.setHoodPos(robot.hoodState);
        }

    }
}
