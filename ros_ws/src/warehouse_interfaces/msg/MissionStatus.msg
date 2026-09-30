# Authoritative mission snapshot published by the supervisor after every loop iteration.

std_msgs/Header header
uint32 revision                # Increases on every accepted state change
bool paused                    # True exactly when hold is non-empty
Hold[<=1] hold
bool language_pending          # A request is being interpreted: cargo and dispatch are fenced
string pending_request_id      # e.g. R3; empty when none
float64 x                      # Robot pose in the map frame
float64 y
float64 yaw
string priority                # Prioritized parcel ID; empty when none
string[] onboard_first         # Parcels whose drops go before any new pickup
Stop[<=1] active               # The one Nav2 goal in progress, if any
Stop[] remaining_stops         # Planned stops after the active one; empty while a replan is pending
bool plan_pending
bool cancel_requested          # A cancel was sent; waiting for Nav2's terminal result
bool finished                  # No active goal and no parcel still owed
bool closed                    # The supervisor session has ended
Parcel[] parcels
