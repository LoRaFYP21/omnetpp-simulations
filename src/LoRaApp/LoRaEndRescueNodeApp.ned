package loranetwork.LoRaApp;

// Rescue node application module. Currently identical to LoRaEndNodeApp; separate NED type allows
// distinct parameterization in ini files.
// You can extend with extra parameters later if needed.

// Use 'extends' to inherit parameters and gates from the concrete simple module LoRaEndNodeApp.
// 'like' only works with module interfaces, hence the previous error.
simple LoRaEndRescueNodeApp extends LoRaEndNodeApp
{
    parameters:
        @class(inet::LoRaEndRescueNodeApp);
}
