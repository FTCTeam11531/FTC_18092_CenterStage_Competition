package org.firstinspires.ftc.teamcode.system;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.utilRobotConstants;

import java.util.Arrays;
import java.util.List;

/**
 * <h2>System - Intake and Arm</h2>
 * <hr>
 * <b>Author:</b> {@value utilRobotConstants.About#COMMENT_AUTHOR_NAME}<br>
 * <b>Season:</b> {@value utilRobotConstants.About#COMMENT_SEASON_PERIOD}<br>
 * <hr>
 * <p>
 * System that contains attributes and methods for:
 * intake and armature control systems.
 * Endgame controls are also included in this system (drone launch)
 * </p>
 * <br>
 */
public class sysIntakeArm {

    private LinearOpMode sysOpMode = null;

    private DcMotorEx stageOneIntake, stageTwoIntake, leftSideArm, rightSideArm;
    private List<DcMotorEx> listMotorsIntake, listMotorsArm;

    private Servo slotOneIntakeServo, slotTwoIntakeServo, pivotIntakeServo, droneLaunchServo;
    private CRServo sweeperIntakeServo;

    private DistanceSensor limitSlotOneSensor, limitSlotTwoSensor;

    public sysIntakeArm(LinearOpMode inOpMode) {
        sysOpMode = inOpMode;
    }

    public void init() {

        // Intake
        stageOneIntake = sysOpMode.hardwareMap.get(DcMotorEx.class, utilRobotConstants.Configuration.LABEL_INTAKE_MOTOR_STAGE_ONE);
        stageTwoIntake = sysOpMode.hardwareMap.get(DcMotorEx.class, utilRobotConstants.Configuration.LABEL_INTAKE_MOTOR_STAGE_TWO);

        // Add Intake motors to array
        listMotorsIntake = Arrays.asList(stageOneIntake, stageTwoIntake);

        // Intake Servo
        sweeperIntakeServo = sysOpMode.hardwareMap.get(CRServo.class, utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SWEEPER);
        slotOneIntakeServo = sysOpMode.hardwareMap.get(Servo.class, utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_ONE);
        slotTwoIntakeServo = sysOpMode.hardwareMap.get(Servo.class, utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_TWO);
        pivotIntakeServo = sysOpMode.hardwareMap.get(Servo.class, utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_PIVOT);
        droneLaunchServo = sysOpMode.hardwareMap.get(Servo.class, utilRobotConstants.Configuration.LABEL_DRONE_LAUNCH_SERVO_MAIN);

//        limitSlotOneSensor = sysOpMode.hardwareMap.get(DistanceSensor.class, utilRobotConstants.Configuration.LABEL_INTAKE_SENSOR_SLOT_ONE);
//        limitSlotTwoSensor = sysOpMode.hardwareMap.get(DistanceSensor.class, utilRobotConstants.Configuration.LABEL_INTAKE_SENSOR_SLOT_TWO);

        // Arm
        leftSideArm = sysOpMode.hardwareMap.get(DcMotorEx.class, utilRobotConstants.Configuration.LABEL_ARM_MOTOR_LEFT_SIDE);
        rightSideArm = sysOpMode.hardwareMap.get(DcMotorEx.class, utilRobotConstants.Configuration.LABEL_ARM_MOTOR_RIGHT_SIDE);

        // Add linear arm motors to array
        listMotorsArm = Arrays.asList(leftSideArm, rightSideArm);

        // Configuration / Initialize Hardware
        stageOneIntake.setDirection(DcMotorEx.Direction.REVERSE);
        stageTwoIntake.setDirection(DcMotorEx.Direction.REVERSE);

        leftSideArm.setDirection(DcMotorEx.Direction.FORWARD);
        rightSideArm.setDirection(DcMotorEx.Direction.REVERSE);

//        setIntakeMotorZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        setArmMotorZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        resetArm();

        sweeperIntakeServo.setPower(utilRobotConstants.IntakeArm.SERVO_INTAKE_SWEEPER_SETPOINT_INIT);
        pivotIntakeServo.setPosition(utilRobotConstants.IntakeArm.SERVO_PIVOT_SETPOINT_HOME);
        slotOneIntakeServo.setPosition(utilRobotConstants.IntakeArm.SERVO_SLOTONE_SETPOINT_INIT);
        slotTwoIntakeServo.setPosition(utilRobotConstants.IntakeArm.SERVO_SLOTTWO_SETPOINT_INIT);

        droneLaunchServo.setPosition(utilRobotConstants.IntakeArm.SERVO_DRONE_LAUNCH_SETPOINT_INIT);

        // Display telemetry
        sysOpMode.telemetry.addData(">", "------------------------------------");
        sysOpMode.telemetry.addData(">", " System: Intake/Arm Initialized");
        sysOpMode.telemetry.update();

    }

    public void activateIntake(double inIntakePower) {

        // Power Intake Motors
        setIntakeMotorPower(inIntakePower);
    }

    public void deactivateIntake() {

        // Stop Intake Motors
        setIntakeMotorPower(0);
    }

    public void resetArm() {

        // Reset encoder
        setArmMotorRunMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        setArmMotorPower(0);
    }

    public void moveArmToTarget(int inTargetSetPoint, double inMaxOutputPowerPercent) {

        // Configure Motor Target Set Point and Set motor as Run to Position
        setArmTargetPosition(inTargetSetPoint);
        setArmMotorRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Move linear slide within min/max limits
        setArmMotorPower(inMaxOutputPowerPercent);

    }

    public void moveArmManually(double inAppliedPower, double inMaxOutputPowerPercent) {

        // Configure Motor Target Set Point and Set motor as Run to Position
        setArmMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Move linear slide within min/max limits
        setArmMotorPower((inAppliedPower * inMaxOutputPowerPercent));
    }

    public double getIntakeServoPosition(String inIntakeServoName) {
        double outPosition;

        switch(inIntakeServoName) {
            case(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_ONE):
                outPosition = slotOneIntakeServo.getPosition();
                break;
            case(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_TWO):
                outPosition = slotTwoIntakeServo.getPosition();
                break;
            case(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_PIVOT):
                outPosition = pivotIntakeServo.getPosition();
                break;
            case(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SWEEPER):
                outPosition = sweeperIntakeServo.getPower();
                break;
            default:
                outPosition = 0;
        }

        return outPosition;
    }

    public double getSensorDistance(String inRangeSensorName) {
        double outDistance;

        switch(inRangeSensorName) {
            case(utilRobotConstants.Configuration.LABEL_INTAKE_SENSOR_SLOT_ONE):
                outDistance = limitSlotOneSensor.getDistance(utilRobotConstants.IntakeArm.LIMIT_SENSOR_DISTANCE_UNIT);
                break;
            case(utilRobotConstants.Configuration.LABEL_INTAKE_SENSOR_SLOT_TWO):
                outDistance = limitSlotTwoSensor.getDistance(utilRobotConstants.IntakeArm.LIMIT_SENSOR_DISTANCE_UNIT);
                break;
            default:
                outDistance = 0;
        }

        return outDistance;
    }

    public int getArmCurrentPosition(String inMotorLabel) {
        // Variable for output Power value for drivetrain motor(s)
        int outEncoderPosition;

        // Get value for motor specified in method call
        switch (inMotorLabel) {
            // Linear Slide Motor - Left
            case utilRobotConstants.Configuration.LABEL_ARM_MOTOR_LEFT_SIDE:
                outEncoderPosition = leftSideArm.getCurrentPosition();
                break;
            // Linear Slide Motor - Right
            case utilRobotConstants.Configuration.LABEL_ARM_MOTOR_RIGHT_SIDE:
                outEncoderPosition = rightSideArm.getCurrentPosition();
                break;
            // Default - No match
            default:
                outEncoderPosition = 0;
        }

        // Return value
        return outEncoderPosition;
    }

    public void setIntakeServoPosition(String inIntakeServoName, double inTargetPosition) {

        switch(inIntakeServoName) {
            case(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_ONE):
                slotOneIntakeServo.setPosition(inTargetPosition);
                break;
            case(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SLOT_TWO):
                slotTwoIntakeServo.setPosition(inTargetPosition);
                break;
            case(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_PIVOT):
                pivotIntakeServo.setPosition(inTargetPosition);
                break;
            case(utilRobotConstants.Configuration.LABEL_INTAKE_SERVO_SWEEPER):
                // Set Continuous Servo direction
                if(inTargetPosition < 0) {
                    sweeperIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
                }
                else {
                    sweeperIntakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
                }
                sweeperIntakeServo.setPower(Math.abs(inTargetPosition));
                break;
        }

    }

    public void setArmTargetPosition(int inTargetPosition) {
        for (DcMotorEx itemMotor: listMotorsArm) {
            itemMotor.setTargetPosition(inTargetPosition);
        }
    }

    public void setArmMotorRunMode(DcMotorEx.RunMode inRunMode) {
        for (DcMotorEx itemMotor: listMotorsArm) {
            itemMotor.setMode(inRunMode);
        }
    }

    public void setArmMotorZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior inZeroPowerBehavior) {
        for (DcMotorEx itemMotor : listMotorsArm) {
            itemMotor.setZeroPowerBehavior(inZeroPowerBehavior);
        }
    }

    public void setArmMotorPower(double inOutputPower) {
        for (DcMotorEx itemMotor : listMotorsArm) {
            itemMotor.setPower(inOutputPower);
        }

    }

    public void setIntakeMotorZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior inZeroPowerBehavior) {
        for (DcMotorEx itemMotor : listMotorsIntake) {
            itemMotor.setZeroPowerBehavior(inZeroPowerBehavior);
        }
    }

    public void setIntakeMotorPower(double inOutputPower) {
        for (DcMotorEx itemMotor : listMotorsIntake) {
            itemMotor.setPower(inOutputPower);
        }

    }
}
