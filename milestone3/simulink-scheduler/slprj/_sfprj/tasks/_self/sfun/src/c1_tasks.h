#ifndef __c1_tasks_h__
#define __c1_tasks_h__

/* Type Definitions */
#ifndef typedef_SFc1_tasksInstanceStruct
#define typedef_SFc1_tasksInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c1_sfEvent;
  boolean_T c1_doneDoubleBufferReInit;
  uint8_T c1_is_active_c1_tasks;
  void *c1_fEmlrtCtx;
  real_T *c1_finished_prev_task;
  real_T *c1_current_task_id;
  real_T *c1_task_to_add_id;
} SFc1_tasksInstanceStruct;

#endif                                 /*typedef_SFc1_tasksInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c1_tasks_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c1_tasks_get_check_sum(mxArray *plhs[]);
extern void c1_tasks_method_dispatcher(SimStruct *S, int_T method, void *data);

#endif
