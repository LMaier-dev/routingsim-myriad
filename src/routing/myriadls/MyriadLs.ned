package routingsim.routing.MyriadLs;

simple myriadls extends routingsim.foundation.Protocol
{
    parameters:
        @class(MyriadLS);

        // protocol
        double helloInterval @unit(s) = default(10s);
        double deadInterval @unit(s) = default(40s);
}