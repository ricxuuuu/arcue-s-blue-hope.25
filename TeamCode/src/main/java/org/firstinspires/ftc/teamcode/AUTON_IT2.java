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

@Autonomous(name = "AUTON IT2 // OUTOFCOMMISSION", group = "Linear Op-mode")
public class AUTON_IT2 extends LinearOpMode {

    private final Invokation_of_a_False_Life robot = new Invokation_of_a_False_Life();

    public void runOpMode() {

        robot.init(hardwareMap);

        int pathState;

        final Pose startPose = new Pose(126.4,123.2,2.2080);
        final Pose shootPose = new Pose(0,0,0);
        final Pose approachTPose = new Pose(0,0,0);
        final Pose pickupTPose = new Pose(0,0,0);
        final Pose approachMPose = new Pose(0,0,0);
        final Pose pickupMPose = new Pose(0,0,0);
        final Pose approachBPose = new Pose(0,0,0);
        final Pose pickupBPose = new Pose(0,0,0);

    }
}
