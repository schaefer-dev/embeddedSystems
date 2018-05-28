#ifndef __c3_tasks_h__
#define __c3_tasks_h__

/* Type Definitions */
#ifndef typedef_SFc3_tasksInstanceStruct
#define typedef_SFc3_tasksInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c3_sfEvent;
  boolean_T c3_doneDoubleBufferReInit;
  uint8_T c3_is_active_c3_tasks;
  void *c3_fEmlrtCtx;
  real_T *c3_clock;
  real_T *c3_running_task_c;
  real_T *c3_finished_prev_task_c;
  real_T *c3_current_task_id_c;
  real_T *c3_task_to_add_id_c;
} SFc3_tasksInstanceStruct;

#endif                                 /*typedef_SFc3_tasksInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c3_tasks_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c3_tasks_get_check_sum(mxArray *plhs[]);
extern void c3_tasks_method_dispatcher(SimStruct *S, int_T method, void *data);

#endif
