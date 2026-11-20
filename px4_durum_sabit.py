"""PX4 vehicle_status nav_state sabitleri (msg/versioned/VehicleStatus.msg, v2). rclpy içe aktarmaz."""

NAV_NAMES = {0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "AUTO_MISSION", 4: "AUTO_LOITER", 5: "AUTO_RTL",
             6: "POSITION_SLOW", 8: "ALTITUDE_CRUISE", 10: "ACRO", 12: "DESCEND", 13: "TERMINATION",
             14: "OFFBOARD", 15: "STAB", 17: "AUTO_TAKEOFF", 18: "AUTO_LAND", 19: "AUTO_FOLLOW_TARGET",
             20: "AUTO_PRECLAND", 21: "ORBIT", 22: "AUTO_VTOL_TAKEOFF"}
AUTO_LOITER, AUTO_RTL, OFFBOARD = 4, 5, 14
