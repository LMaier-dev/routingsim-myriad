
package routingsim.mobility;

simple AntennaBase {
    parameters:
        double minSignalPower @unit(db) = default(10db);
        double maxSignalPower @unit(db) = default(12db);
        double minSendGain @unit(db) = default(0db);
        double maxSendGain @unit(db) = default(0db);
        double minRecvGain @unit(db) = default(0db);
        double maxRecvGain @unit(db) = default(0db);
        double minFreq @unit(MHz) = default(2400MHz);
        double maxFreq @unit(MHz) = default(2400MHz);
        double systemNoise @unit(db) = default(-172db);
}

