t2 = cos(yaw);
t3 = sin(roll);
t4 = sin(yaw);
t5 = cos(roll);
t6 = sin(pitch);
t7 = cos(pitch);
A0[0][0] = t2 * t7;
A0[0][1] = -t4 * t5 + t2 * t3 * t6;
A0[0][2] = t3 * t4 + t2 * t5 * t6;
A0[1][0] = t4 * t7;
A0[1][1] = t2 * t5 + t3 * t4 * t6;
A0[1][2] = -t2 * t3 + t4 * t5 * t6;
A0[2][0] = -t6;
A0[2][1] = t3 * t7;
A0[2][2] = t5 * t7;
