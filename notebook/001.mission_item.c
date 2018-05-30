int main()
{
	struct mission_item_s mission_item =
	{ };

	mission_item.nav_cmd = NAV_CMD_WAYPOINT;
	mission_item.lat = 41.9420254664;
	mission_item.lon = 123.3875214835;
	mission_item.altitude = 100;
	mission_item.altitude_is_relative = false;
	mission_item.yaw = NAN;
	mission_item.acceptance_radius = 10.0;
	mission_item.time_inside = 0.0f;
	mission_item.autocontinue = true;
	mission_item.origin = ORIGIN_MAVLINK;

	dm_write(DM_KEY_WAYPOINTS_OFFBOARD_0, 0, DM_PERSIST_POWER_ON_RESET, &mission_item, sizeof(struct mission_item_s));

	mission_item.lat = 41.9420254664;
	mission_item.lon = 123.3865214835;
	dm_write(DM_KEY_WAYPOINTS_OFFBOARD_0, 1, DM_PERSIST_POWER_ON_RESET, &mission_item, sizeof(struct mission_item_s));

	mission_item.lat = 41.9430254664;
	mission_item.lon = 123.3865214835;
	dm_write(DM_KEY_WAYPOINTS_OFFBOARD_0, 2, DM_PERSIST_POWER_ON_RESET, &mission_item, sizeof(struct mission_item_s));

	mission_s mission;
	mission.timestamp = hrt_absolute_time();
	mission.dataman_id = 0;
	mission.count = 3;
	mission.current_seq = 0;
	dm_write(DM_KEY_MISSION_STATE, 0, DM_PERSIST_POWER_ON_RESET, &mission, sizeof(mission_s));

	mission_stats_entry_s stats;
	stats.num_items = 0;
	stats.update_counter = 0;
	dm_write(DM_KEY_FENCE_POINTS, 0, DM_PERSIST_POWER_ON_RESET, &stats, sizeof(mission_stats_entry_s));
}
