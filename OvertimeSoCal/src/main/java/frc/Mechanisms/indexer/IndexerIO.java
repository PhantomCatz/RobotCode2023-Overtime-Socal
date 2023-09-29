package frc.Mechanisms.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO 
{
    @AutoLog
    public class IndexerIOInputs
    {
        public boolean indexerBeamBreakOpen;
    }

    public default void updateInputs(IndexerIOInputs inputs) {}

    public default void runIndexerPercentIO(double pwr) {}
}
