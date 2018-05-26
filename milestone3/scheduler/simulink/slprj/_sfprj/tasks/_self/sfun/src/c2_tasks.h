#ifndef __c2_tasks_h__
#define __c2_tasks_h__

/* Type Definitions */
#ifndef typedef_SFc2_tasksInstanceStruct
#define typedef_SFc2_tasksInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c2_sfEvent;
  boolean_T c2_doneDoubleBufferReInit;
  uint8_T c2_is_active_c2_tasks;
  void *c2_fEmlrtCtx;
  real_T *c2_y;
  real_T *c2_a;
  real_T *c2_b;
} SFc2_tasksInstanceStruct;

#endif                                 /*typedef_SFc2_tasksInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c2_tasks_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c2_tasks_get_check_sum(mxArray *plhs[]);
extern void c2_tasks_method_dispatcher(SimStruct *S, int_T method, void *data);

#endif
