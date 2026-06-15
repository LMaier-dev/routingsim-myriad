package routingsim.routing.MYRIAD;

simple myriadgp extends routingsim.foundation.Protocol
{
    parameters:
        @class(MyriadGP);

        // protocol — same hello/dead/rxmt knobs as MyriadLS so the shared
        // MyriadHelloProto can read them.
        double helloInterval @unit(s) = default(10s);
        double deadInterval @unit(s) = default(40s);
        double rxmtInterval @unit(s) = default(5s);

        // logging: DEBUG | INFO | WARN | ERROR
        string logLevel = default("DEBUG");
        // ANSI-color the log prefix by level (Cmdenv terminal only; off for log files / Qtenv)
        bool colorLog = default(false);
}
