using Cxx = import "/include/c++.capnp";
$Cxx.namespace("cereal");

@0xb526ba661d550a59;

# custom.capnp: a home for empty structs reserved for custom forks
# These structs are guaranteed to remain reserved and empty in mainline
# cereal, so use these if you want custom events in your fork.

# DO rename the structs
# DON'T change the identifier (e.g. @0x81c2f05a394cf4af)

struct CustomReserved0 @0x81c2f05a394cf4af {
}

struct CustomReserved1 @0xaedffd8f31e7b55d {
}

struct CustomReserved2 @0xf35cc4560bbf6ec2 {
}

struct TurboSteerAssist @0xda96579883444c35 {
  active @0 :Bool;
  requestedSteeringAngleDeg @1 :Float32;
  wheelSteeringAngleDeg @2 :Float32;
  baseModelSteeringAngleDeg @3 :Float32;
  sequence @4 :UInt32;
  baseModelLogMonoTime @5 :UInt64;
}

struct TurboTeleopCommand @0x80ae746ee2596b11 {
  command @0 :Command;

  enum Command {
    cruiseEnable @0;
    cruiseCancel @1;
    headlightsOn @2;
    headlightsOff @3;
  }
}

struct TurboSteerAssistState @0xa5cd762cd951a455 {
  applied @0 :Bool;
  status @1 :Text;
  targetAvailable @2 :Bool;
  requestedSteeringAngleDeg @3 :Float32;
  modelSteeringAngleDeg @4 :Float32;
  finalSteeringAngleDeg @5 :Float32;
  sourceSequence @6 :UInt32;
  sourceBaseModelLogMonoTime @7 :UInt64;
}

struct TurboIntentRequest @0xf98d843bfd7004a3 {
  protocolVersion @0 :UInt16;
  operatorId @1 :Text;
  requestId @2 :UInt64;
  sessionId @3 :Text;
  epoch @4 :Text;
  action @5 :Action;
  direction @6 :Direction;
  baseFeedbackLogMonoTime @7 :UInt64;
  ready @8 :Bool;
  reverse @9 :Bool;
  localStatus @10 :Text;
  createdMonoTime @11 :UInt64;

  enum Action { none @0; request @1; cancel @2; }
  enum Direction { none @0; left @1; right @2; }
}

struct TurboIntentState @0xb86e6369214c01c8 {
  protocolVersion @0 :UInt16;
  sessionId @1 :Text;
  epoch @2 :Text;
  operatorId @3 :Text;
  requestId @4 :UInt64;
  direction @5 :TurboIntentRequest.Direction;
  status @6 :Text;
  reason @7 :Text;
  mode @8 :Text;
  available @9 :Bool;
  minSpeed @10 :Float32;
  maxSpeed @11 :Float32;
  pulseFrameId @12 :UInt32;
  pulseMonoTime @13 :UInt64;
  laneChangeProbability @14 :Float32;
  operatorOverride @15 :Bool;
}

struct TurboIntentLinkState @0xf416ec09499d9d19 {
  # Local UGV bridge heartbeat; never accepted from the network.
  sessionId @0 :Text;
  connected @1 :Bool;
}

struct CustomReserved9 @0xa1680744031fdb2d {
}

struct CustomReserved10 @0xcb9fd56c7057593a {
}

struct CustomReserved11 @0xc2243c65e0340384 {
}

struct CustomReserved12 @0x9ccdc8676701b412 {
}

struct CustomReserved13 @0xcd96dafb67a082d0 {
}

struct CustomReserved14 @0xb057204d7deadf3f {
}

struct CustomReserved15 @0xbd443b539493bc68 {
}

struct CustomReserved16 @0xfc6241ed8877b611 {
}

struct CustomReserved17 @0xa30662f84033036c {
}

struct CustomReserved18 @0xc86a3d38d13eb3ef {
}

struct CustomReserved19 @0xa4f1eb3323f5f582 {
}
