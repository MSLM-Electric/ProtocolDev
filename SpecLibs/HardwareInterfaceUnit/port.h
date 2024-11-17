#include <stdint.h>

#define SLAVE_ISR(name) void SlaveMCUInterruptRoutineImmitation_##name##(void)
#define SLAVE_CallISR(name) SlaveMCUInterruptRoutineImmitation_##name##()
#define MASTER_ISR(name) void MasterMCUInterruptRoutineImmitation_##name##(void)
#define MASTER_CallISR(name) MasterMCUInterruptRoutineImmitation_##name##()