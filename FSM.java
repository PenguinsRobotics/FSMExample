package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="wow teleop")
public class FSMClass extends OpMode {
    //Setting Lift States
    public enum LiftState{
        GRIPING,
        LIFTING,
        RELEASE,
        LOWER
    }

    LiftState liftState = LiftState.GRIPING;

    double gripClose = 0.38;
    double gripOpen = 0.13;
    int slideUp = 1600;
    int slideDown = 200;

    double gripTime;
    double releaseTime;

    ElapsedTime gripingTimer = new ElapsedTime();
    ElapsedTime releaseTimer = new ElapsedTime();

    double slideVelocity;

    //Declaring Servos and Motors
    Servo gripper = hardwareMap.servo.get("claw");
    DcMotorEx slide = hardwareMap.get(DcMotorEx.class, "slidemotor");

    public void init() {
        gripingTimer.reset();
    }

    public void loop() {
        switch (liftState) {
            case GRIPING:
                // Waiting for some input
                if (gamepad2.x) {
                    gripper.setPosition(gripClose);
                    gripingTimer.reset();
                    liftState = LiftState.LIFTING;
                }
                break;
            case LIFTING:
                if (gripingTimer.seconds() >= gripTime) {
                    slide.setTargetPosition(slideUp);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setVelocity(slideVelocity);
                    liftState = LiftState.RELEASE;
                }
                break;
            case RELEASE:
                if (gamepad2.b){
                    //release cone
                    //reset release timer
                    //set state to lower
                }
                break;
            case LOWER:
                //check if release time has been reached
                //lower slides
                //set state to lifting
                break;
            default:
                liftState = LiftState.GRIPING;
        }
        telemetry.addData("GripperPos", gripper.getPosition());
        telemetry.addData("SlidePos", slide.getCurrentPosition());
        telemetry.update();
    }
}

