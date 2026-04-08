package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.lioncore.tasks.Task;
import org.firstinspires.ftc.teamcode.systems.Indicator;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Limelight;
import org.firstinspires.ftc.teamcode.systems.Shooter;

public class IndicatorManager extends Task {

    private Indicator indicator;
    private Intake intake;
    private Shooter shooter;
    private Limelight limelight;

    public IndicatorManager(Indicator indicator, Intake intake, Shooter shooter, Limelight limelight) {
        this.indicator = indicator;
        this.intake = intake;
        this.shooter = shooter;
        this.limelight = limelight;
    }

    @Override
    public void init() {
        indicator.set(Indicator.Colour.Off);
    }

    @Override
    public void run() {
        Indicator.Colour colour;

        if (this.shooter.validEncoder) {
            if (this.limelight.running) {
                if (this.limelight.isValid) colour = Indicator.Colour.Green;
                else colour = Indicator.Colour.Orange;
            } else {
                if (this.intake.getState() == Intake.State.IntakingEmpty) {
                    colour = Indicator.Colour.Purple;
                } else if (this.intake.getState() == Intake.State.IntakeOnly) {
                    colour = Indicator.Colour.Blue;
                } else {
                    if (this.shooter.onTarget) {
                        colour = Indicator.Colour.Green;
                    } else {
                        colour = Indicator.Colour.Yellow;
                    }
                }
            }
        } else {
            colour = Indicator.Colour.Red;
        }

        this.indicator.set(colour);
    }
}
