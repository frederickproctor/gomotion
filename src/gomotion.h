/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I. 
*/

/*!
  \file gomotion.h

  \brief Declarations for motion queue manipulation
*/

#ifndef GOMOTION_H
#define GOMOTION_H

#include "gotypes.h"		/* go_result, go_count */
#include "gomath.h"		/* go_pose */
#include "gotraj.h"		/* go_traj_cv,a,j_spec */

#ifdef __cplusplus
extern "C" {
#endif
#if 0
}
#endif

#define GO_MOTION_JOINT_NUM 8	/* number of joints supported */

enum {
  /* types of motion queueing done */
  GO_MOTION_NONE,
  GO_MOTION_JOINT,
  GO_MOTION_UJOINT,
  GO_MOTION_WORLD,

  /* these are used to further specify types of world motion */
  GO_MOTION_LINEAR,
  GO_MOTION_CIRCULAR,
  GO_MOTION_WRAPPED /* where rotation is considered a wrapped linear axis */
};

/*
  Depending upon whether you are doing joint interpolation or
  world coordinate interpolation (as specified by your call to
  go_motion_queue_set_type()), fill in joint[] or pose accordingly.
*/
typedef struct {
  union {
    go_real joint[GO_MOTION_JOINT_NUM];
    go_pose pose;
  } u;
} go_position;

extern void go_position_zero_joints(go_position * position);
extern void go_position_zero_pose(go_position * position);

typedef struct {
  go_real vel;			/*< max vel for each motion */
  go_real acc;			/*< max accel for each motion */
  go_real jerk;			/*< max jerk for each motion */
} go_motion_params;

/*
  In the following comments, LIN, CIR and ALL refer to linear moves,
  circular moves and both, respectively. If they are not in parens,
  e.g., ALL:, you need to provide them for that type of motion.
  If they are in parens, e.g., (CIR), they are computed for you.
*/

typedef struct {
  go_cart uvec;			/* (LIN) unit vector along line */
  go_quat uquat;		/* (LIN) unit quaternion along rotation */
} go_motion_linear_params;

typedef struct {
  /*! The vector to the circle center. */
  go_cart center;

  /*! The normal vector that defines the plane of the circle. */
  go_cart normal;

  /*! The normal vector expressed as a unit rotation. */
  go_quat qnormal;

  /*! The unit vector from center to start, projected onto the normal plane. */
  go_cart urcsp;

  /*! The starting radius. */
  go_real rstart;

  /*! The vector from the normal plane to the start. */
  go_cart zstart;

  /*! The signed total angular displacement around normal vector */
  go_real thtot;

  /*! The signed displacement from start to end projected radii */
  go_real rtot;

  /*! The signed displacement from start to end z off-normals  */
  go_real ztot;

  /*! The inverse of total approximate arc length. If negative, no
    translation motion is taking place. */
  go_real stotinv;

  /*! The number of turns in the circle. 0 means partial CCW, -1 means
    partial CW, otherwise more turns are added in each direction. */
  go_integer turns;
} go_motion_circular_params;

typedef struct {
  go_flag type;			/* ALL: GO_MOTION_JOINT,LINEAR,CIRCULAR */
  go_integer id;		/* ALL: id echoed as current move */
  go_real totalt;		/* (ALL) total planned time for the motion */
  go_position start;		/* (ALL)  start pose wrt world; prev end */
  go_position end;		/* ALL: target position, joints or pose */
  go_quat uquat;		/* (ALL) unit rotation from start to end */
  /* For joint moves, each pertains to the indexed joint. For world moves,
     [0] pertains to trans, [1] pertains to rot, and the [2]-[*] are unused. */
  go_motion_params par[GO_MOTION_JOINT_NUM];
  /* specific params for linear and circular moves */
  union {
    go_motion_linear_params lpar;	/* (LIN) linear params */
    go_motion_circular_params cpar;	/* CIR: some circular params */
  } u;
  /* (ALL) times for the various tran phases of CV, CA and CJ profiles */
  /* For world motion [0] is for tran, [1] is for rot, rest unused */
  go_traj_cj_spec cj[GO_MOTION_JOINT_NUM];
} go_motion_spec;

extern go_result go_motion_spec_init(go_motion_spec * spec);

extern go_result go_motion_spec_set_type(go_motion_spec * spec, go_flag type);
extern go_flag go_motion_spec_get_type(go_motion_spec * spec);

extern go_result go_motion_spec_set_id(go_motion_spec * spec, go_integer id);
extern go_integer go_motion_spec_get_id(go_motion_spec * spec);

extern go_result go_motion_spec_set_jpar(go_motion_spec * spec, go_integer i, go_real vel, go_real acc, go_real jerk);
extern go_result go_motion_spec_set_tpar(go_motion_spec * spec, go_real vel, go_real acc, go_real jerk);
extern go_result go_motion_spec_set_rpar(go_motion_spec * spec, go_real vel, go_real acc, go_real jerk);
extern go_result go_motion_spec_set_cpar(go_motion_spec * spec, go_cart * center, go_cart * normal, go_integer turns);
extern go_result go_motion_spec_set_time(go_motion_spec * spec, go_real time);
extern go_result go_motion_spec_set_end_position(go_motion_spec * spec, go_position * end);
extern go_result go_motion_spec_set_end_pose(go_motion_spec * spec, go_pose * end);

typedef struct {
  go_traj_ca_spec scale_spec;
  go_flag scaling;		/*< non-zero means we're scaling time */
  go_flag scale_dir;		/*< non-zero means add scale to scale_b  */
  go_flag scale_isneg;		/*< non-zero means negative scale  */
  go_real scale_b;		/*< original base scale factor */
  go_real scale;		/*< current scale factor */
  go_real scale_next;		/*< pending scale factor */
  go_real scale_v_next;		/*< pending d(scale)/dt */
  go_real scale_a_next;		/*< pending d^2(scale)/dt^2 */
  go_real scale_t;		/*< time into scaling */
} go_scale_spec;

extern go_result go_scale_init(go_scale_spec * spec, go_real scale);
extern go_result go_scale_set(go_scale_spec * spec, 
			      go_real scale,
			      go_real scale_v,
			      go_real scale_a);
extern go_result go_scale_eval(go_scale_spec * spec,
			       go_real deltat, 
			       go_real * scale);

typedef struct {
  go_flag type;			/*< go_MOTION_JOINT for joint, otherwise world */
  go_position here;		/*< initial starting point */
  go_position there;		/*< where the end is */
  go_motion_spec *startptr;	/*< ptr to original start */
  go_motion_spec *endptr;	/*< the original end */
  go_motion_spec *start;	/*< ptr to first entry of the queue */
  go_motion_spec *end;		/*< ptr to last (empty) entry */
  go_integer size;		/*< size of whole queue */
  go_integer joint_num;		/*< number of joints to be interpolated */
  go_integer number;		/*< number of motions on queue */
  go_integer last_id;		/*< id of last motion appended */
  go_real deltat;		/*< cycle time */
  go_real time;			/*< time into the current spec */
  go_scale_spec timescale;	/*< walked-in time scale factor */
} go_motion_queue;

extern go_result go_motion_queue_init(go_motion_queue * queue,
				      go_motion_spec * space,
				      go_integer size,
				      go_real deltat);

extern go_result go_motion_queue_reset(go_motion_queue * queue);

extern go_result go_motion_queue_set_type(go_motion_queue * queue,
					  go_flag type);

extern go_flag go_motion_queue_get_type(go_motion_queue * queue);

extern go_result go_motion_queue_set_joint_number(go_motion_queue * queue,
						  go_integer joints);

extern go_integer go_motion_queue_get_joint_number(go_motion_queue * queue);

extern go_result go_motion_queue_set_here(go_motion_queue * queue,
					  const go_position * here);

extern go_result go_motion_queue_set_cycle_time(go_motion_queue * queue,
						go_real deltat);

extern go_result go_motion_queue_set_scale(go_motion_queue * queue,
					   go_real scale,
					   go_real scale_v,
					   go_real scale_a);

extern go_result go_motion_queue_append(go_motion_queue * queue,
					const go_motion_spec * motion);

extern go_result go_motion_queue_number(const go_motion_queue * queue,
					go_integer * number);

extern go_result go_motion_queue_size(const go_motion_queue * queue,
				      go_integer * size);

extern go_result go_motion_queue_head(const go_motion_queue * queue,
				      go_motion_spec * motion);

extern go_result go_motion_queue_here(const go_motion_queue * queue,
				      go_position * position);

extern go_result go_motion_queue_there(const go_motion_queue * queue, 
				       go_position * position);

extern go_result go_motion_queue_interp(go_motion_queue * queue,
					go_position * position);

extern go_result go_motion_queue_stop(go_motion_queue * queue);

extern go_result go_motion_queue_set_id(go_motion_queue * queue, go_integer id);

extern go_integer go_motion_queue_last_id(const go_motion_queue * queue);

extern go_flag go_motion_queue_is_empty(const go_motion_queue * queue);

extern go_result go_motion_queue_delete(go_motion_queue * queue);

extern go_result go_motion_queue_drop_pending(go_motion_queue * queue);

#if 0
{
#endif
#ifdef __cplusplus
}
#endif

#endif /* GOMOTION_H */
