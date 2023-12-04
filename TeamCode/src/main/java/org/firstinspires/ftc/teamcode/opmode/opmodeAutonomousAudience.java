package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.system.sysDrivetrainOdometryMecanum;
import org.firstinspires.ftc.teamcode.system.sysIntakeArm;
import org.firstinspires.ftc.teamcode.system.sysLighting;
import org.firstinspires.ftc.teamcode.system.sysVision;
import org.firstinspires.ftc.teamcode.utility.utilRobotConstants;

@Config
@Autonomous(name = "Autonomous - Audience", group = "_auto", preselectTeleOp = "Teleop Main")
@Disabled
public class opmodeAutonomousAudience extends LinearOpMode {
    // ------------------------------------------------------------
    // System(s) - Define system and create instance of each system
    // ------------------------------------------------------------
    // -- Robot Initializtion

    // -- Lighting System
    sysLighting sysLighting = new sysLighting(this);

    // -- Drivetrain System
//    sysDrivetrainMecanum sysDrivetrain = new sysDrivetrainMecanum(this);

    // Vision System
    sysVision sysVision = new sysVision(this);

    // Intake / Arm
    sysIntakeArm sysIntakeArm = new sysIntakeArm(this);

    // ------------------------------------------------------------
    // Misc
    // ------------------------------------------------------------
    // -- Command Runtime
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        // ------------------------------------------------------------
        // Configure Telemetry
        // ------------------------------------------------------------
        // Set telemetry mode to append
        telemetry.setAutoClear(false);
        telemetry.clearAll();

        // ------------------------------------------------------------
        // Initialize System(s) - set different light mode between each system init
        // ------------------------------------------------------------

        // Initialize System(s)
        sysLighting.init(utilRobotConstants.Configuration.ENABLE_LIGHTING);
        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_LIGHTING);

//        sysDrivetrain.init();
        sysDrivetrainOdometryMecanum sysDrivetrainOdometryMecanum = new sysDrivetrainOdometryMecanum(hardwareMap);
        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_DRIVETRAIN);

        sysVision.init();
        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_VISION);

        sysIntakeArm.init();
        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_INTAKEARM);

        // ------------------------------------------------------------
        // Configure drivetrain for Autonomous Mode
        // -- Set to run without encoders for timed drive mode
        // ------------------------------------------------------------
//        sysDrivetrain.setDriveMotorRunMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // ------------------------------------------------------------
        // Variables for OpMode
        // ------------------------------------------------------------
        boolean isImageFound = false;
        int targetZone = 0, movementAmount = 0;

        HuskyLens.Block[] targetObjectList = null;
        HuskyLens.Block targetObject = null;

        // ------------------------------------------------------------
        // Send telemetry message to signify robot completed initialization and waiting to start;
        // ------------------------------------------------------------
        telemetry.addData(">", "------------------------------------");
        telemetry.addData(">", "All Systems Ready - Waiting to Start");
        telemetry.update();

        // Reset runtime clock
        runtime.reset();

        // System Initialization Complete
        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_SYSTEM_INIT_COMPLETE);

        // Repeat while in initialize state (disable if using waitForStart)

        // Set AI Camera Mode
        sysVision.setAICameraMode(utilRobotConstants.Vision.AI_Camera.AI_CAMERA_MODE_OBJECT_TRACKING);

        // ------------------------------------------------------------
        // Configure Telemetry
        // ------------------------------------------------------------
        // Set telemetry mode to auto-clear
        telemetry.setAutoClear(true);
        telemetry.clearAll();

        while(opModeInInit() && !isStopRequested()) {

            // Look for Team Element position
            targetObjectList = sysVision.getCameraObjectList();

            if(targetObjectList.length > 0) {

                targetObject = sysVision.getCameraObject(targetObjectList, 1);

                if(targetObject != null) {

                    if (targetObject.x < utilRobotConstants.Vision.RANDOM_TARGET_ZONE_ONE_X) {

                        // Set Zone based on Alliance
                        if(sysVision.getDetectedAllianceTagColor() == "blue") {
                            targetZone = 1; // left
                            sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_ONE);
                        }
                        else {
                            targetZone = 3; // left
                            sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_THREE);
                        }
                    } else if (targetObject.x > utilRobotConstants.Vision.RANDOM_TARGET_ZONE_ONE_X && targetObject.x < utilRobotConstants.Vision.RANDOM_TARGET_ZONE_TWO_X) {
                        targetZone = 2; // center
                        sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_TWO);
                    } else if (targetObject.x > utilRobotConstants.Vision.RANDOM_TARGET_ZONE_TWO_X) {

                        // Set Zone based on Alliance
                        if(sysVision.getDetectedAllianceTagColor() == "blue") {
                            targetZone = 3; // left
                            sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_THREE);
                        }
                        else {
                            targetZone = 1; // left
                            sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_ONE);
                        }
                    } else {

                        // Set Zone based on Alliance
                        if(sysVision.getDetectedAllianceTagColor() == "blue") {
                            targetZone = 3; // left
                            sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_THREE);
                        }
                        else {
                            targetZone = 1; // left
                            sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_ONE);
                        }
                    }

                }
            }
            else {

                // Set Zone based on Alliance
                if(sysVision.getDetectedAllianceTagColor() == "blue") {
                    targetZone = 3; // left
                    sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_THREE);
                }
                else {
                    targetZone = 1; // left
                    sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_ONE);
                }
            }

            // ------------------------------------------------------------
            // Driver Hub Feedback
            // ------------------------------------------------------------
            telemetry.addData("Init Time", runtime.toString());
            // ------------------------------------------------------------
            // - Vision telemetry
            // ------------------------------------------------------------
            telemetry.addData("-", "------------------------------");
            telemetry.addData("-", "-- Vision");
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Alliance Color", sysVision.getDetectedAllianceTagColor());
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Camera Block Count", sysVision.getCameraObjectList().length);
            telemetry.addData("-", "------------------------------");
            telemetry.addData("Randomized Zone", targetZone);
            if (targetObject != null) {
                telemetry.addData("-", "------------------------------");
                telemetry.addData("-", "-- Target Object");
                telemetry.addData("-", "------------------------------");
                telemetry.addData("Target x:", targetObject.x);
                telemetry.addData("Target y:", targetObject.y);
                telemetry.addData("Target width:", targetObject.width);
                telemetry.addData("Target height:", targetObject.height);
                telemetry.addData("Target top:", targetObject.top);
                telemetry.addData("Target left:", targetObject.left);
            }

            // ------------------------------------------------------------
            // - send telemetry to driver hub
            // ------------------------------------------------------------
            telemetry.update();
        }

        // Road Runner -- need to implement in system
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        // Set Starting Pose for Blue Station Two
        Pose2d startPose = new Pose2d(-66, -36, Math.toRadians(0));
        sysDrivetrainOdometryMecanum.setPoseEstimate(startPose);

        // Wait for Start state (disable if using opModeInInit)
        waitForStart();

        if (isStopRequested()) return;

        // ----------------------------------------------------
        // Defined Trajectories
        // ----------------------------------------------------
        // ----------------------
        // Same for ALL
        // ----------------------
        // Move to Random Zone
        Trajectory trajRandomZone = sysDrivetrainOdometryMecanum.trajectoryBuilder(startPose, false)
                .forward(15)
                .build();

        // Determine Zone Angle
        switch (targetZone){
            case 1:
                // Zone 1
                movementAmount = -5;

                // Set Color to zone 1
                sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_ONE);
                break;
            case 2:
                // Zone 2
                movementAmount = -5;

                // Set Color to zone 2
                sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_TWO);
                break;
            case 3:
                // Zone 3
                movementAmount = -2;

                // Set Color to zone 3
                sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_THREE);
                break;
            default:
                // Default
                movementAmount = -5;

                // Set Color to invalid when default
                sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_INVALID);
                break;
        }

        // Clear Pixel
        Trajectory trajClearPixel = sysDrivetrainOdometryMecanum.trajectoryBuilder(trajRandomZone.end(), false)
                .forward(movementAmount)
                .build();

        // ----------------------
        // Alliance Movement
        // ----------------------
        // Strafe out of random zone
        // Strafe based on Alliance and Zone

        // Determine Zone Angle
        switch (targetZone){
            case 1:
                // Zone 1
                // Set Turn Angle based on Alliance
                if(sysVision.getDetectedAllianceTagColor() == "blue") {
                    movementAmount = 12;
                }
                else {
                    movementAmount = -12;
                }

                // Set Color to zone 1
                sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_ONE);
                break;
            case 2:
                // Zone 2
                // Set Turn Angle based on Alliance
                if(sysVision.getDetectedAllianceTagColor() == "blue") {
                    movementAmount = 12;
                }
                else {
                    movementAmount = -12;
                }

                // Set Color to zone 2
                sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_TWO);
                break;
            case 3:
                // Zone 3
                // Set Turn Angle based on Alliance
                if(sysVision.getDetectedAllianceTagColor() == "blue") {
                    movementAmount = 1;
                }
                else {
                    movementAmount = -1;
                }

                // Set Color to zone 3
                sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_THREE);
                break;
            default:
                // Default
                // Set Turn Angle based on Alliance
                if(sysVision.getDetectedAllianceTagColor() == "blue") {
                    movementAmount = 12;
                }
                else {
                    movementAmount = -12;
                }

                // Set Color to invalid when default
                sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_INVALID);
                break;
        }

        Trajectory trajClearRandomZone = sysDrivetrainOdometryMecanum.trajectoryBuilder(trajClearPixel.end(), false)
                .strafeRight(movementAmount)
                .build();

        // ----------------------
        // Same for ALL
        // ----------------------
        // Move into cycle lane - from lower level

        // Determine Zone Angle
        switch (targetZone){
            case 1:
                // Zone 1
                movementAmount = 12;

                // Set Color to zone 1
                sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_ONE);
                break;
            case 2:
                // Zone 2
                movementAmount = 15;

                // Set Color to zone 2
                sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_TWO);
                break;
            case 3:
                // Zone 3
                movementAmount = 12;

                // Set Color to zone 3
                sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_THREE);
                break;
            default:
                // Default
                movementAmount = 12;

                // Set Color to invalid when default
                sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_INVALID);
                break;
        }
        Trajectory trajEnterCycleLaneLower = sysDrivetrainOdometryMecanum.trajectoryBuilder(trajClearRandomZone.end(), false)
                .forward(movementAmount)
                .build();

        // Move to stage
        Trajectory trajMovetoStage = sysDrivetrainOdometryMecanum.trajectoryBuilder(trajEnterCycleLaneLower.end(), false)
                .forward(62) //92
                .build();

        // Set Turn Angle based on Alliance
        if(sysVision.getDetectedAllianceTagColor() == "blue") {
            movementAmount = -12;
        }
        else {
            movementAmount = 12;
        }

        // Move to park
        Trajectory trajStrafeToPark = sysDrivetrainOdometryMecanum.trajectoryBuilder(trajMovetoStage.end())
                .strafeRight(movementAmount)
                .build();

        // ----------------------------------------------------
        // Movement Action(s)
        // ----------------------------------------------------
        // ----------------------
        // Same for ALL
        // ----------------------
        // Move to Random Zone
        sysDrivetrainOdometryMecanum.followTrajectory(trajRandomZone);

        // Determine Zone Angle
        switch (targetZone){
            case 1:
                // Zone 1
                // Set Turn Angle based on Alliance
                if(sysVision.getDetectedAllianceTagColor() == "blue") {
                    movementAmount = 74;
                }
                else {
                    movementAmount = -74;
                }

                // Set Color to zone 1
                sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_ONE);
                break;
            case 2:
                // Zone 2
                // Set Turn Angle based on Alliance
                if(sysVision.getDetectedAllianceTagColor() == "blue") {
                    movementAmount = 0;
                }
                else {
                    movementAmount = 0;
                }

                // Set Color to zone 2
                sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_TWO);
                break;
            case 3:
                // Zone 3
                // Set Turn Angle based on Alliance
                if(sysVision.getDetectedAllianceTagColor() == "blue") {
                    movementAmount = -74;
                }
                else {
                    movementAmount = 74;
                }

                // Set Color to zone 3
                sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_THREE);
                break;
            default:
                // Default
                // Set Turn Angle based on Alliance
                if(sysVision.getDetectedAllianceTagColor() == "blue") {
                    movementAmount = 74;
                }
                else {
                    movementAmount = -74;
                }

                // Set Color to invalid when default
                sysLighting.setLightPattern(utilRobotConstants.Lighting.LIGHT_PATTERN_AUTONOMOUS_ZONE_ID_INVALID);
                break;
        }

        // Turn to Zone
        sysDrivetrainOdometryMecanum.turn(Math.toRadians(movementAmount));

        // Place Purple Pixel
        sysIntakeArm.setIntakeServoPosition(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_ONE, utilRobotConstants.IntakeArm.SERVO_SLOTONE_SETPOINT_OPEN);
        sleep(300);

        // Clear the Pixel
        sysDrivetrainOdometryMecanum.followTrajectory(trajClearPixel);

        // Zone Counter-Turn
        sysDrivetrainOdometryMecanum.turn(Math.toRadians(-(movementAmount)));

        // Clear the Random Zone
        sysDrivetrainOdometryMecanum.followTrajectory(trajClearRandomZone);

//        // Move into Cycle Lane
//        sysDrivetrainOdometryMecanum.followTrajectory(trajEnterCycleLaneLower);
//
//        // Turn to Stage
//        // Set Turn Angle based on Alliance
//        if(sysVision.getDetectedAllianceTagColor() == "blue") {
//            movementAmount = 70; //54 too far
//        }
//        else {
//            movementAmount = -78;
//        }
//
//        sysDrivetrainOdometryMecanum.turn(Math.toRadians(movementAmount));

        // ----------------------
        // Same for ALL
        // ----------------------
//        // Move to stage
//        sysDrivetrainOdometryMecanum.followTrajectory(trajMovetoStage);
//
//        // Final Turn - In Park
//        // Set Turn Angle based on Alliance
//        if(sysVision.getDetectedAllianceTagColor() == "blue") {
//            movementAmount = -72;
//        }
//        else {
//            movementAmount = 72;
//        }
//
//        sysDrivetrainOdometryMecanum.turn(Math.toRadians(movementAmount));
//
//        // Move to Park
//        sysDrivetrainOdometryMecanum.followTrajectory(trajStrafeToPark);

        Pose2d poseEstimate = sysDrivetrainOdometryMecanum.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive());

    }


}