enum ScheduleStrategy {SCHED_STATIC = 1, SCHED_DYNAMIC = 2, SCHED_GUIDED = 3};

void cppad_cg_set_thread_pool_disabled(int disabled) {
}

int cppad_cg_is_thread_pool_disabled() {
   return 1;
}

void cppad_cg_set_thread_number(unsigned int n) {
}

unsigned int cppad_cg_get_thread_number() {
   return 1;
}

void cppad_cg_thpool_set_scheduler_strategy(enum ScheduleStrategy s) {
}

enum ScheduleStrategy cppad_cg_thpool_get_scheduler_strategy() {
   return SCHED_STATIC;
}

void cppad_cg_thpool_set_verbose(int v) {
}

int cppad_cg_thpool_is_verbose() {
   return 0;
}

void cppad_cg_thpool_set_guided_maxgroupwork(float v) {
}

float cppad_cg_thpool_get_guided_maxgroupwork() {
   return 1.0;
}

void cppad_cg_thpool_set_number_of_time_meas(unsigned int n) {
}

unsigned int cppad_cg_thpool_get_number_of_time_meas() {
   return 0;
}

