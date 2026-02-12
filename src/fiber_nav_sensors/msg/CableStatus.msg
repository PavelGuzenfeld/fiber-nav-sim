# Cable dynamics status
std_msgs/Header header
float32 tension             # Total cable tension at drone attachment (N)
float32 deployed_length     # Total fiber deployed from spool (m)
float32 airborne_length     # Fiber length suspended in air (m)
float32 drag_force          # Aerodynamic drag magnitude (N)
float32 weight_force        # Cable weight force magnitude (N)
float32 spool_friction      # Spool friction force magnitude (N)
bool is_broken              # True if tension exceeded breaking strength
