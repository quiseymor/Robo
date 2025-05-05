package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.gamepad.OI;
import frc.robot.subsystems.Training;

public class Drive extends CommandBase
{
    /**
     * Bring in Subsystem and Gamepad code
     */
    private static final Training train = RobotContainer.train;
    private static final OI oi = RobotContainer.oi;

    /**
     * Joystick inputs
     */
    double inputLeftY = 0;
    double inputLeftX = 0;
    double inputRightY = 0;
    double inputRightX = 0;
    boolean inputDriveRightTrigger;
    boolean inputDriveRightBumper;
    boolean inputDriveLeftBumper;
    boolean inputDriveLeftTrigger;
    boolean inputDriveDPadX;
    boolean inputDriveDPadY;
    boolean inputDriveDPadB;
    boolean inputDriveDPadA;
    int i = 0;
    // boolean inputDriveLeftButton;

    /**
     * Ramp Constants
     */
    double deltaLeftY = 0;
    double deltaLeftX = 0;
    double deltaRightY = 0;
    double deltaRightX = 0;
    double prevLeftY = 0;
    double prevLeftX = 0;
    double prevRightY = 0;
    double prevRightX = 0;

    /**
     * Ramp up Constant
     */
    private static final double RAMP_UP     = 0.05;

    /**
     * Ramp down Constant
     */
    private static final double RAMP_DOWN   = 0.05;

    /**
     * Delta Limit
     */
    private static final double DELTA_LIMIT = 0.075;

    /**
     * Constructor
     */
    public Drive()
    {
        addRequirements(train); //add the traning subsystem as a requirement 
    }

    

    /**
     * Code here will run once when the command is called for the first time
     */
    @Override
    public void initialize()
    {
        train.setAngleServo(3);
        // train.setServoUSpeed(0.0);
    }

    /**
     * Code here will run continously every robot loop until the command is stopped
     */
    @Override
    public void execute()
    {
        inputLeftX = oi.getLeftDriveX();
        inputLeftY = oi.getLeftDriveY();
        inputRightX = oi.getRightDriveX();
        inputRightY = oi.getRightDriveY();
        inputDriveRightTrigger = oi.getDriveRightTrigger();
        inputDriveRightBumper = oi.getDriveRightBumper();
        inputDriveLeftBumper = oi.getDriveLeftBumper();
        inputDriveLeftTrigger = oi.getDriveLeftTrigger();
        inputDriveDPadX = oi.getDriveXButton();
        inputDriveDPadY = oi.getDriveYButton();
        inputDriveDPadB = oi.getDriveBButton();
        inputDriveDPadA = oi.getDriveAButton();
        if (Math.abs(inputLeftY) > 0.1){
            train.setMotorLSpeed(-inputLeftY);
            train.setMotorRSpeed(inputLeftY);
        }
        else if(Math.abs(inputRightX) > 0.1){
            train.setMotorLSpeed(-inputRightX/3);
            train.setMotorRSpeed(-inputRightX/3);
            train.setMotorBSpeed(inputRightX);
        }
        else if(inputDriveRightTrigger){
            train.setMotorLSpeed(0.5);
            train.setMotorRSpeed(0.5);
            train.setMotorBSpeed(0.5);
        }
        else if(inputDriveLeftTrigger){
            train.setMotorLSpeed(-0.5);
            train.setMotorRSpeed(-0.5);
            train.setMotorBSpeed(-0.5);
        }
        else if (inputDriveDPadX){
            train.setAngleServo(10);
        }
        else if (inputDriveDPadY){
            train.setAngleServo(70);
        }
        else if (inputDriveRightBumper){
            train.setServoUSpeed(2);
            // train.ledSetGreen(true);
        }
        else if (inputDriveLeftBumper){
            train.setServoUSpeed(-2);
            // train.ledSetRed(true);
        }
        else if (inputDriveDPadA){      
            if(i<=10){
                train.ledSetRed(true);
                train.ledSetGreen(false);
                i++;
            }
            else if(i<=20 && i>10){
                train.ledSetRed(true);
                train.ledSetGreen(true);
                i++;
            }
            else if(i<=25 && i>20){
                train.ledSetRed(false);
                train.ledSetGreen(false);
            }
        }       
        else{
            train.setMotorLSpeed(0);
            train.setMotorRSpeed(0);
            train.setMotorBSpeed(0);
            train.setServoUStop();
        }

    }

    /**
     * When the comamnd is stopped or interrupted this code is run
     * <p>
     * Good place to stop motors in case of an error
     */
    @Override
    public void end(boolean interrupted)
    {
        train.setMotorLSpeed(0);
        train.setMotorRSpeed(0);
        train.setMotorBSpeed(0);
    }

    /**
     * Check to see if command is finished
     */
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
