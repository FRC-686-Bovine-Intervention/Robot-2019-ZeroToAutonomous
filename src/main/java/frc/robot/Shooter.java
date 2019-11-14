package frc.robot;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.lib.util.DataLogger;
import frc.robot.loops.Loop;

public class Shooter implements Loop
{
	// singleton class
    private static Shooter instance = null;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    //====================================================
    // Members
    //====================================================
    public TalonSRX shooterMotor;

    //====================================================
    // Constants
    //====================================================
    
    public final double kIntakePercentOutput  = +0.6;          
    public final double kOuttakePercentOutput = -1.0;   // full power outtake
    
    public final int kSlotIdx = 0;

    public final double kCalMaxEncoderPulsePer100ms = 33300;	// velocity at a max throttle (measured using Phoenix Tuner)
    public final double kCalMaxPercentOutput 		= 1.0;	// percent output of motor at above throttle (using Phoenix Tuner)

    public final double kCruiseVelocity = 0.80 * kCalMaxEncoderPulsePer100ms;		// cruise below top speed
    public final double kTimeToCruiseVelocity = 0.1;				// seconds to reach cruise velocity
    public final double kAccel = kCruiseVelocity / kTimeToCruiseVelocity; 
    
	public final double kKf = kCalMaxPercentOutput * 1023.0 / kCalMaxEncoderPulsePer100ms;
	public final double kKp = 0.025;	   
	public final double kKd = 9.0;	// to resolve any overshoot, start at 10*Kp 
	public final double kKi = 0.0;    

	public static double kQuadEncoderGain = 60.0/12.0;			// Arm is on 60t sprocket, Encoder is on 12t sprocket.  
	public static double kQuadEncoderUnitsPerRev = 4*64;
	public static double kEncoderUnitsPerDeg = kQuadEncoderUnitsPerRev * kQuadEncoderGain / 360.0; 

    public final int    kAllowableError = (int)(0.25 * kEncoderUnitsPerDeg);
    public final double kspinIntakeAngleDeg = 5; //0

    public final int kPeakCurrentLimit = 50;
    public final int kPeakCurrentDuration = 200;
    public final int kContinuousCurrentLimit = 30;

    public Shooter() 
    {
        shooterMotor = new TalonSRX(1);

        //====================================================
        // Configure Deploy Motors
        //====================================================

        // Factory default hardware to prevent unexpected behavior
        shooterMotor.configFactoryDefault();

		// configure encoder
		shooterMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kTalonPidIdx, Constants.kTalonTimeoutMs);
		shooterMotor.setSensorPhase(true); // set so that positive motor input results in positive change in sensor value
		shooterMotor.setInverted(true);   // set to have green LEDs when driving forward
		
		// set relevant frame periods to be at least as fast as periodic rate
		shooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,      (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		shooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,    (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		shooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		shooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0,  (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		
		// configure velocity loop PID 
        shooterMotor.selectProfileSlot(kSlotIdx, Constants.kTalonPidIdx); 
        shooterMotor.config_kF(kSlotIdx, kKf, Constants.kTalonTimeoutMs); 
        shooterMotor.config_kP(kSlotIdx, kKp, Constants.kTalonTimeoutMs); 
        shooterMotor.config_kI(kSlotIdx, kKi, Constants.kTalonTimeoutMs); 
        shooterMotor.config_kD(kSlotIdx, kKd, Constants.kTalonTimeoutMs);
        shooterMotor.configAllowableClosedloopError(kSlotIdx, kAllowableError, Constants.kTalonTimeoutMs);
        
        // current limits
        shooterMotor.configPeakCurrentLimit(kPeakCurrentLimit, Constants.kTalonTimeoutMs);
        shooterMotor.configPeakCurrentDuration(kPeakCurrentDuration, Constants.kTalonTimeoutMs);
        shooterMotor.configContinuousCurrentLimit(kContinuousCurrentLimit, Constants.kTalonTimeoutMs);
        shooterMotor.enableCurrentLimit(true);
    }
    
	@Override
	public void onStart() 
	{
    }

    
	@Override
	public void onStop() 
	{
        // stop all motors
        shooterMotor.set(ControlMode.PercentOutput, 0.0);
    }
    
	@Override
	public void onLoop()
    {

    }   

    public void setTarget(double RPM)
    {
    }
    

    
	private final DataLogger logger = new DataLogger()
	{
		@Override
		public void log()
		{
            // put("Shooter/Deploy/targetPosition", targetPosition.toString());

		}
	}; 
    
	public DataLogger getLogger()
	{
		return logger;
	}    
}