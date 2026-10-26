uint8 LANDED_UNKNOWN=0
uint8 LANDED_ON_GROUND=1
uint8 LANDED_IN_AIR=2
uint8 LANDED_TAKEOFF=3
uint8 LANDED_LANDING=4

bool state_valid
bool connected
bool armed
string mode

bool position_valid
geometry_msgs/PoseStamped local_pose
bool heading_valid
bool velocity_valid
# Local-frame linear velocity (ENU/map); angular velocity is not part of this contract.
geometry_msgs/Vector3Stamped local_velocity

# PX4 DDS reports required VehicleLocalPosition validity flags and dead-reckoning state.
# This is a navigation-estimate check, not a full estimator-status summary.
bool estimator_health_valid
bool estimator_healthy

# MAVROS does not expose these states through the inputs used by this backend.
bool failsafe_state_valid
bool failsafe_active
bool preflight_check_state_valid
bool preflight_checks_passed

bool landed_state_valid
uint8 landed_state

# Physical state can recover while DDS control still requires a ground reset.
# Both backends report their actual control inhibition here.
bool control_inhibited
string control_message

# Backend result (MAVROS service reply or final PX4 DDS ACK).
# Acceptance is separate from actual mode/arming/landing state confirmation.
uint32 last_command_id
bool command_completed
bool command_accepted
string command_message
