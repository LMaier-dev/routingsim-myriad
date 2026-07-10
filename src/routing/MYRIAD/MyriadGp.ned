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

        // king re-originates a COLLECT every refreshInterval; members run a
        // watchdog of 2*refreshInterval + uniform(0, refreshJitter).
        double refreshInterval @unit(s) = default(30s);
        double refreshJitter @unit(s) = default(5s);

        // Ring membership override, e.g. "A,B". Empty = use the built-in
        // per-id table. Set per node via **.NodeN.*.ringLabel or a topology
        // node property.
        string ringLabel = default("");

        // logging: DEBUG | INFO | WARN | ERROR
        string logLevel = default("DEBUG");
        // ANSI-color the log prefix by level (Cmdenv terminal only; off for log files / Qtenv)
        bool colorLog = default(false);
}
