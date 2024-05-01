package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

public class ExecuteTimeCommand extends Command {
    private Supplier<Command> mCommandSupplier;
    private Command mCommand;

    public ExecuteTimeCommand(Supplier<Command> commandSupplier)
    {
        mCommandSupplier = commandSupplier;
    }
    
    @Override
    public void initialize() {
        mCommand = mCommandSupplier.get();
        mCommand.schedule();
    }


    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        if(interrupted)
        {
            mCommand.cancel();
        }
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return mCommand.isFinished();
    }    
}
