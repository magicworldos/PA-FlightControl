t2 = 1.0 / range;
A0[0][0] = t2 * (q1 * vd * 2.0 + q0 * ve * 2.0 - q3 * vn * 2.0);
A0[0][1] = t2 * (q0 * vd * 2.0 - q1 * ve * 2.0 + q2 * vn * 2.0);
A0[0][2] = t2 * (q3 * vd * 2.0 + q2 * ve * 2.0 + q1 * vn * 2.0);
A0[0][3] = -t2 * (q2 * vd * -2.0 + q3 * ve * 2.0 + q0 * vn * 2.0);
A0[0][4] = -t2 * (q0 * q3 * 2.0 - q1 * q2 * 2.0);
A0[0][5] = t2 * (q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3);
A0[0][6] = t2 * (q0 * q1 * 2.0 + q2 * q3 * 2.0);
