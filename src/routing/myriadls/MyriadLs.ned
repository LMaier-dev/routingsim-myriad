package routingsim.routing.myriadls;

simple myriadls extends routingsim.foundation.Protocol
{
    parameters:
        @class(MyriadLS);

        // protocol
        double helloInterval @unit(s) = default(10s);
        double deadInterval @unit(s) = default(40s);
        double rxmtInterval @unit(s) = default(5s);

        // logging: DEBUG | INFO | WARN | ERROR
        string logLevel = default("DEBUG");
        // ANSI-color the log prefix by level (Cmdenv terminal only; off for log files / Qtenv)
        bool colorLog = default(false);
}