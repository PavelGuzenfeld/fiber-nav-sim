# Gazebo Wrench Lessons Learned

Hard-won lessons from implementing wrench-based flight control in Gazebo Harmonic.

## 1. Persistent Wrench Accumulation

**Problem:** `ApplyLinkWrench` (the persistent `/world/.../wrench/persistent` topic) uses `push_back()` internally. Every publish adds a **new** wrench entry. All entries are applied every physics step. Publishing at 50Hz causes force to grow without bound — the vehicle launches into orbit.

**Solution:** Use the one-time wrench topic (`/world/<name>/wrench`) instead. One-time wrenches are cleared after each physics step. Scale the force by the number of physics steps since last publish to maintain the correct impulse.

```cpp
// ONE-TIME wrench (correct)
wrench_topic_ = "/world/" + world_name_ + "/wrench";

// PERSISTENT wrench (WRONG for repeated publishing)
// wrench_topic_ = "/world/" + world_name_ + "/wrench/persistent";
```

## 2. MODEL vs LINK Entity Type

**Problem:** Applying wrench to a LINK entity (`gz::msgs::Entity::LINK`) causes DART physics solver divergence in multi-link models with fixed joints. The solver computes conflicting constraint forces between the links, leading to NaN velocities within seconds.

**Solution:** Apply wrench to the MODEL entity (`gz::msgs::Entity::MODEL`). Gazebo distributes the force correctly across the model's link structure.

```cpp
entity->set_type(gz::msgs::Entity::MODEL);  // Correct
// entity->set_type(gz::msgs::Entity::LINK);  // Causes DART divergence
```

## 3. Wall Clock vs Sim Time

**Problem:** Headless Gazebo runs at variable speed (often 2-10x faster than real-time). Using `rclcpp::Clock::now()` or wall-clock timers for control loop timing produces incorrect forces because the control rate doesn't match the physics rate.

**Solution:** Use odometry timestamps (sim time) for all timing calculations. The odometry message header contains the Gazebo simulation timestamp.

```cpp
// Use sim time from odometry (correct)
double sim_time = odom_.header.stamp.sec + odom_.header.stamp.nanosec * 1e-9;
double sim_dt = sim_time - last_publish_sim_time_;

// Wall-clock time (WRONG for force scaling)
// auto now = this->now();
```

## 4. Impulse Scaling for One-Time Wrenches

**Problem:** One-time wrenches apply for exactly 1 physics step (`physics_step_size`, typically 0.001s). If the control loop runs at a lower rate than the physics engine, forces are only applied for a fraction of the time between control updates, resulting in insufficient thrust.

**Solution:** Scale the one-time wrench by `sim_dt / physics_step_size` — the number of physics steps since the last publish. This ensures the impulse equals what a continuous force would deliver.

```cpp
double scale = clamp(sim_dt / physics_step_, 1.0, 500.0);
publish_wrench(fx * scale, fy * scale, fz * scale,
               tx * scale, ty * scale, tz * scale);
```

The upper clamp (500) prevents absurd forces if the controller stalls for a long time.

## Summary

| Lesson | Wrong | Right |
|--------|-------|-------|
| Wrench topic | `/wrench/persistent` | `/wrench` (one-time) |
| Entity type | `LINK` | `MODEL` |
| Time source | `rclcpp::Clock::now()` | Odometry header stamp |
| Force scaling | Raw force value | `force * (sim_dt / physics_step)` |
