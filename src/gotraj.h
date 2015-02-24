#ifndef GOTRAJ_H
#define GOTRAJ_H

#include "gotypes.h"		/* go_result, go_real */

#ifdef __cplusplus
extern "C" {
#endif
#if 0
}
#endif

typedef struct {
  go_real at0;			/*< accel for Phase I and III */
  go_real t1;			/*< cumulative time at end of Phase I */
  go_real dt1;			/*< cumulative distance at end of Phase I */
  go_real vt1;			/*< speed at end of Phase I/in Phase II */
  go_real t2;			/*< cumulative time at end of Phase II */
  go_real dt2;			/*< cumulative distance at end of Phase II */
  go_real tend;			/*< total time for motion */
  go_real dtend;		/*< total distance for motion */
  go_real invd;			/*< inverse of total distance */
} go_traj_ca_spec;

typedef struct {
  go_real jt0;			/*< jerk in Phases I, III, V, VII */
  go_real t1;			/*< cumulative time at end of Phase I */
  go_real dt1;			/*< cumulative distance at end of Phase I */
  go_real vt1;			/*< speed at end of Phase I */
  go_real at1;			/*< accel at end of Phase I */
  go_real t2;			/*< cumulative time at end of Phase II */
  go_real dt2;			/*< cumulative distance at end of Phase II */
  go_real vt2;			/*< speed at end of Phase II */
  go_real t3;			/*< cumulative time at end of Phase III */
  go_real dt3;			/*< cumulative distance at end of Phase III */
  go_real vt3;			/*< speed at end of Phase III/in Phase IV */
  go_real t4;			/*< cumulative time at end of Phase IV */
  go_real dt4;			/*< cumulative distance at end of Phase IV */
  go_real t5;			/*< cumulative time at end of Phase V */
  go_real dt5;			/*< cumulative distance at end of Phase V */
  go_real t6;			/*< cumulative time at end of Phase VI */
  go_real dt6;			/*< cumulative distance at end of Phase VI */
  go_real tend;			/*< total time for motion */
  go_real dtend;		/*< total distance for motion */
  go_real invd;			/*< inverse of total distance */
} go_traj_cj_spec;

typedef struct {
  go_real s, d, v, a, j;
} go_traj_interp_spec;

/*
  go_traj_ca_generate() takes an accel value, and intervals for the accel period
  and cruise period, and fills in the go_traj_ca_spec with the
  interval parameters. This is useful for generating test cases.
*/

extern go_result go_traj_ca_generate(go_real acc, go_real deltacc,
				     go_real deltvel, go_traj_ca_spec * pts);

/*
  go_traj_ca_compute() takes values for distance 'd' to move, max velocity 'v' to
  limit if necessary, and constant accel 'a', and fills in the go_traj_ca_spec
  with the interval parameters.
*/

extern go_result go_traj_ca_compute(go_real d, go_real v, go_real a,
				    go_traj_ca_spec * pts);

/*
  go_traj_ca_scale() takes a time 't' for the desired time of the motion,
  and scales the times and motion params so that the total distance
  remains the same and everything else is in proportion 
*/

extern go_result go_traj_ca_scale(const go_traj_ca_spec * ts, go_real t,
				  go_traj_ca_spec * pts);

/*
  go_traj_ca_stop() takes a time 't' for the desired time to begin
  stopping, and recomputes the times so that the move will stop as
  soon as possible during subsequent interps.
*/

extern go_result go_traj_ca_stop(const go_traj_ca_spec * ts, go_real t,
				 go_traj_ca_spec * pts);

/*
  go_traj_ca_extend() takes a time 't' for the desired time to finish
  the motion, and extends the constant-speed section so that it stops
  then. The time must be shorter than the original time, and longer
  than the fastest stopping time.
*/

extern go_result go_traj_ca_extend(const go_traj_ca_spec * ts, go_real t,
				   go_traj_ca_spec * pts);

/*
  go_traj_ca_interp() takes a go_traj_ca_spec and interpolates the d-v-a
  values for the given time t, storing the d-v-a values in ti.
*/

extern go_result go_traj_ca_interp(const go_traj_ca_spec * ts, go_real t,
				   go_traj_interp_spec * ti);

/*
  go_traj_cj_generate() takes a jerk value, and intervals for the jerk period,
  accel period and cruise period, and fills in the go_traj_cj_spec with the
  interval parameters. This is useful for generating test cases.
*/

extern go_result go_traj_cj_generate(go_real jrk, go_real deltjrk,
				     go_real deltacc, go_real deltvel,
				     go_traj_cj_spec * pts);

/*
  go_traj_cj_compute() takes values for distance 'd' to move, max velocity 'v' to
  limit if necessary, max accel 'a' to limit if necessary, and constant
  jerk 'j', and fills in the go_traj_cj_spec with the interval parameters.
*/

extern go_result go_traj_cj_compute(go_real d, go_real v, go_real a,
				    go_real j, go_traj_cj_spec * pts);

/*
  go_traj_cj_scale() takes a time 't' for the desired time of the motion,
  and scales the times and motion params so that the total distance
  remains the same and everything else is in proportion 
*/

extern go_result go_traj_cj_scale(const go_traj_cj_spec * ts, go_real t,
				  go_traj_cj_spec * pts);

/*
  go_traj_cj_stop() takes a time 't' for the desired time to begin
  stopping, and recomputes the times so that the move will stop as
  soon as possible during subsequent interps.
*/

extern go_result go_traj_cj_stop(const go_traj_cj_spec * ts, go_real t,
				 go_traj_cj_spec * pts);

/*
  go_traj_cj_extend() takes a time 't' for the desired time to finish
  the motion, and extends the constant-speed section so that it stops
  then. The time must be shorter than the original time, and longer
  than the fastest stopping time.
*/

extern go_result go_traj_cj_extend(const go_traj_cj_spec * ts, go_real t,
				   go_traj_cj_spec * pts);

/*
  go_traj_cj_interp() takes a go_traj_cj_spec and interpolates the d-v-a-j
  values for the given time t, storing the d-v-a-j values in ti.
*/

extern go_result go_traj_cj_interp(const go_traj_cj_spec * ts, go_real t,
				   go_traj_interp_spec * ti);

#if 0
{
#endif
#ifdef __cplusplus
}
#endif

#endif /* GOTRAJ_H */
