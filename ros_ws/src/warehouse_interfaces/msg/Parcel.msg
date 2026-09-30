# One parcel. Physical state and mission disposition are independent: cancelling or deferring never moves cargo.

string STATE_AWAITING_PICKUP=awaiting_pickup
string STATE_ONBOARD=onboard
string STATE_DELIVERED=delivered

string DISPOSITION_INACTIVE=inactive
string DISPOSITION_ACTIVE=active
string DISPOSITION_CANCELLED=cancelled
string DISPOSITION_DEFERRED=deferred

string id
string pickup        # Station where the parcel waits
string drop          # Station it must be delivered to
string state         # One of the STATE_* values
string disposition   # One of the DISPOSITION_* values
string reason        # Why it was deferred; empty otherwise
