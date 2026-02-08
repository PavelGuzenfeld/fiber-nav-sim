# Fiber spool status with accumulated length
std_msgs/Header header
float32 velocity          # Payout rate (m/s), non-negative
float32 total_length      # Total unrolled fiber length (m)
bool is_moving            # True if velocity > threshold
